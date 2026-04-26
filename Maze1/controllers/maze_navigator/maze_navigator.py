"""Webots entry point for the ROSbot maze navigator.

Pipeline per tick
-----------------
    1. Sensor read       (robot_io)
    2. Pose update       (odometry — wheel + IMU yaw)
    3. Mapping update    (grid_2d:    lidar log-odds
                                       + depth-cam aux obstacle layer
                                       + green-mask poison layer)
    4. Perception        (HSV blue/yellow/green; world-position memory)
    5. FSM tick          (transitions only)
    6. Action            (planning -> path follower; recovery; initial scan)
    7. Debug dump        (~periodic PNGs)

Architecture: planning + path following (A* + pure pursuit) with mission FSM,
fallback to frontier exploration when the active target is unknown, and
deterministic recovery on stuck. Poison is a HARD lethal constraint via the
costmap, never erased. Goal handling uses a standoff approach — the planner
never plans into the pillar centre.
"""
import os
import sys
import math

import numpy as np

import config as C
from robot_io import RobotIO
from odometry import Odometry
from lidar_scan import ScanProcessor
from grid_2d import Grid2D
from perception import PillarDetector
from clearance import ClearanceChecker
from explorer import Explorer
from planning import plan_to_world, plan_to_pillar
from local_control import PathFollower
from fsm import State, MissionFSM
from recovery import StuckMonitor, Recovery
from debug_viz import dump_grid, dump_costmap, dump_camera_masks
from mapping_export import export_ply, export_final_png


class MazeNavigator:
    def __init__(self):
        self.io = RobotIO()
        self.dt = self.io.timestep * 1e-3

        # Modules
        self.odom = Odometry()
        self.scan = ScanProcessor(self.io.lidar) if self.io.lidar is not None else None
        self.grid = Grid2D()
        self.perc = PillarDetector(self.io.camera, self.io.depth) \
            if self.io.camera is not None else None
        self.clear = ClearanceChecker(self.io.depth, self.io.camera)
        self.explore = Explorer(self.grid)
        self.follower = PathFollower()
        self.fsm = MissionFSM(log_fn=self._fsm_log)
        self.stuck = StuckMonitor()
        self.recovery = Recovery()

        # Mission state
        self.sim_time = 0.0
        self.tick = 0
        self.theta_spin_accum = 0.0
        self._last_theta = None
        self.blue_reached = False
        self.t_blue_reached = None
        self.t_yellow_reached = None
        self.frontier_blacklist = []   # world-points to avoid as frontiers
        self.cur_frontier = None
        self.cur_standoff = None
        self.cur_target_color = None   # "blue" / "yellow" / None
        self.t_last_plan = -math.inf
        self.t_last_pillar_attempt = -math.inf
        self._last_pillar_target = {"blue": None, "yellow": None}
        self.last_costmap = None

        # Pillar world-position smoothing (deque of (x, y))
        self._pillar_obs = {"blue": [], "yellow": []}
        self.pillar_world_pos = {"blue": None, "yellow": None}

        # Camera helper for area-based "very close" reach test
        self._pillar_area_thresh = None
        if self.io.camera is not None:
            w = self.io.camera.getWidth()
            h = self.io.camera.getHeight()
            self._pillar_area_thresh = C.PILLAR_AREA_FRAC * w * h * 255

        # Output (Maze2 redirects this via env var so dumps don't collide)
        default_dir = os.path.join(os.path.dirname(__file__), "maps")
        self.png_dir = os.environ.get("MAZE_NAVIGATOR_MAPS_DIR", default_dir)
        os.makedirs(self.png_dir, exist_ok=True)
        self._last_png_t = -math.inf
        self._last_dbg_t = -math.inf

        # 3D point cloud accumulator for the final PLY export
        self.cloud_chunks = []
        self._cloud_every_s = 0.5
        self._last_cloud_t = -math.inf
        self.pose_history = []
        self._export_done = False

    # ----------------------------- helpers ------------------------------

    def _fsm_log(self, msg):
        print(msg, flush=True)

    def _log(self, extra=""):
        if self.tick % C.LOG_EVERY_TICKS != 0:
            return
        x, y, th = self.odom.pose()
        print(
            f"[t={self.sim_time:6.2f}s state={self.fsm.state:14s}] "
            f"pose=({x:+.2f}, {y:+.2f}, {th:+.2f}) {extra}",
            flush=True,
        )

    def _track_spin(self):
        _, _, th = self.odom.pose()
        if self._last_theta is None:
            self._last_theta = th
            return
        d = th - self._last_theta
        if d > math.pi:
            d -= 2 * math.pi
        elif d < -math.pi:
            d += 2 * math.pi
        self.theta_spin_accum += abs(d)
        self._last_theta = th

    def _smooth_pillar(self, name, pdet):
        rng = pdet.get("range", float("nan"))
        bearing = pdet["bearing"]
        if not np.isfinite(rng) or rng <= 0.0 or rng > 6.0:
            return
        rx, ry, rth = self.odom.pose()
        gx = rx + rng * math.cos(rth + bearing)
        gy = ry + rng * math.sin(rth + bearing)
        buf = self._pillar_obs[name]
        buf.append((gx, gy))
        if len(buf) > C.PILLAR_OBS_AVG_N:
            buf.pop(0)
        if len(buf) >= 2:
            xs = np.array([p[0] for p in buf])
            ys = np.array([p[1] for p in buf])
            self.pillar_world_pos[name] = (float(xs.mean()), float(ys.mean()))

    def _project_green_to_grid(self, gdet):
        """Project the green floor-mask centroid to the world plane and
        permanently mark a poison disc in the lethal layer."""
        if gdet is None or self.perc is None:
            return None
        rng = gdet.get("range", float("nan"))
        v_pix = gdet.get("v", self.perc.cy)
        bearing = gdet.get("bearing", 0.0)
        v_below = v_pix - self.perc.cy
        if np.isfinite(rng) and rng > 0.15:
            d_floor = float(rng)
        elif v_below > 5:
            d_floor = C.CAMERA_MOUNT_Z * self.perc.fx / v_below
            if not (0.1 < d_floor < 4.5):
                return None
        else:
            return None

        x_r = d_floor
        y_r = -math.tan(bearing) * d_floor
        rx, ry, rth = self.odom.pose()
        c, s = math.cos(rth), math.sin(rth)
        wx = rx + c * x_r - s * y_r
        wy = ry + s * x_r + c * y_r
        if not self.grid.in_bounds(*self.grid.w2i(wx, wy)):
            return None
        was_marked = self.grid.is_poison_world(wx, wy)
        self.grid.mark_poison(wx, wy, radius=C.POISON_DETECT_RADIUS)
        if not was_marked:
            print(f"[mission] GREEN poison detected at ({wx:+.2f}, {wy:+.2f})",
                  flush=True)
        return (wx, wy)

    def _project_aux_obstacles(self, pose, max_range=2.5):
        """Use the depth cam to mark chassis-height obstacles into the
        aux_obstacle layer. Lightweight — runs every tick."""
        if not self.clear.ok:
            return
        d = self.clear._depth_image()
        if d is None:
            return
        # Only project depths inside [LIDAR_MOUNT_Z, ROBOT_HEIGHT + 5cm]
        x_r, y_r, z_r = self.clear._robot_frame(d)
        valid = (np.isfinite(d) & (d > self.clear.depth_min)
                 & (d < max_range)
                 & (z_r > C.LIDAR_MOUNT_Z) & (z_r < C.ROBOT_HEIGHT + 0.05))
        if not valid.any():
            return
        xr = x_r[valid]
        yr = y_r[valid]
        rx, ry, rth = pose
        c, s = math.cos(rth), math.sin(rth)
        wx = rx + c * xr - s * yr
        wy = ry + s * xr + c * yr
        self.grid.mark_aux_obstacle_points(np.stack([wx, wy], axis=1))

    def _accumulate_cloud(self):
        if not self.clear.ok:
            return
        if self.sim_time - self._last_cloud_t < self._cloud_every_s:
            return
        pts = self.clear.depth_to_world_cloud(self.odom.pose())
        if pts.shape[0] > 0:
            self.cloud_chunks.append(pts)
        self._last_cloud_t = self.sim_time

    def _aux_clear(self):
        return self.clear.min_forward_at_chassis_height()

    def _carve_free_around_robot(self):
        """Guarantee the robot's own footprint is free in the grid so the
        planner never refuses to start. Uses the chassis half-extent."""
        rx, ry, _ = self.odom.pose()
        self.grid.mark_free_disc(rx, ry, radius=C.ROBOT_CHASSIS_HALF_LENGTH * 0.85)

    def _reactive_drive(self, ranges, dets):
        """Fallback when the planner cannot produce a path.

        Drive forward if the lidar forward cone is clear; otherwise rotate
        toward the freer side. This keeps the robot moving so the map fills
        in and a future plan attempt succeeds.
        """
        ranges_arr = ranges["ranges"]
        angles_arr = ranges["angles"]
        from local_control import _forward_clear, _side_min  # local import
        fwd = _forward_clear(ranges_arr, angles_arr)
        aux = self._aux_clear()
        if np.isfinite(aux):
            fwd = min(fwd, float(aux))
        sl = _side_min(ranges_arr, angles_arr, "left")
        sr = _side_min(ranges_arr, angles_arr, "right")

        # Avoid driving toward poison: bias rotation away from any poison
        # cell within ~0.5 m in front.
        rx, ry, rth = self.odom.pose()
        front_x = rx + 0.4 * math.cos(rth)
        front_y = ry + 0.4 * math.sin(rth)
        if self.grid.is_poison_world(front_x, front_y):
            return 0.0, C.W_MAX * 0.6 if sl > sr else -C.W_MAX * 0.6

        if fwd > C.PF_FWD_SLOW_DIST:
            return min(0.25, C.V_MAX * 0.6), 0.0
        if fwd > C.PF_FWD_BRAKE_DIST:
            scale = (fwd - C.PF_FWD_BRAKE_DIST) / max(
                C.PF_FWD_SLOW_DIST - C.PF_FWD_BRAKE_DIST, 1e-3
            )
            return 0.10 * max(0.3, scale), (0.4 if sl > sr else -0.4)
        # blocked -> spin toward freer side
        return 0.0, (0.9 if sl > sr else -0.9)

    # ----------------------------- planning glue ------------------------

    def _replan_to_pillar(self, color, force=False):
        target_xy = self.pillar_world_pos[color]
        if target_xy is None:
            return False, None
        # Throttle: only retry when target moved or after replan period.
        last = self._last_pillar_target.get(color)
        moved = last is None or math.hypot(
            target_xy[0] - last[0], target_xy[1] - last[1]
        ) > C.PILLAR_REPLAN_MOVE_M
        elapsed = self.sim_time - self.t_last_pillar_attempt
        if not force and not moved and elapsed < C.ASTAR_REPLAN_EVERY_S:
            return False, None
        self.t_last_pillar_attempt = self.sim_time
        self._last_pillar_target[color] = target_xy

        path, idx, standoff, cost = plan_to_pillar(
            self.grid, self.odom.pose(), target_xy,
            keep_arc_to=(self.odom.pose()[0], self.odom.pose()[1]),
        )
        self.last_costmap = cost
        if path is None:
            print(f"[planner] FAILED to plan to {color} pillar @ {target_xy}",
                  flush=True)
            return False, None
        self.follower.set_path(path, final_tol=C.WAYPOINT_REACH_TOL)
        self.cur_standoff = standoff
        self.t_last_plan = self.sim_time
        print(f"[planner] {color} plan: standoff={standoff} "
              f"len={len(path)} via {idx[0]}->{idx[-1]}",
              flush=True)
        return True, path

    def _replan_to_frontier(self):
        f = self.explore.find_frontier(
            self.odom.pose(), blacklist=self.frontier_blacklist
        )
        if f is None:
            return False
        path, idx, cost = plan_to_world(self.grid, self.odom.pose(), f)
        self.last_costmap = cost
        if path is None:
            # blacklist this frontier so we don't keep retrying it
            print(f"[planner] frontier unreachable, blacklisting {f}",
                  flush=True)
            self.frontier_blacklist.append(f)
            self.frontier_blacklist = self.frontier_blacklist[-15:]
            return False
        self.follower.set_path(path, final_tol=0.30)
        self.cur_frontier = f
        self.cur_standoff = None
        self.t_last_plan = self.sim_time
        print(f"[planner] frontier plan -> {f} len={len(path)}",
              flush=True)
        return True

    def _path_invalid(self):
        """True if the current path has any lethal cell on it."""
        if not self.follower.path:
            return True
        lethal = self.grid.lethal_mask()
        from utils import line_clear
        cells = lethal.shape[0]
        prev = None
        for wx, wy in self.follower.path:
            ix, iy = self.grid.w2i(wx, wy)
            if not (0 <= ix < cells and 0 <= iy < cells):
                return True
            if lethal[ix, iy]:
                return True
            if prev is not None:
                pix, piy = prev
                if not line_clear(lethal, pix, piy, ix, iy):
                    return True
            prev = (ix, iy)
        return False

    # ----------------------------- pillar reach -------------------------

    def _pillar_reached(self, color, pdet):
        """Visual + geometric arrival test."""
        target_xy = self.pillar_world_pos.get(color)
        if target_xy is None:
            return False
        rx, ry, _ = self.odom.pose()
        d = math.hypot(target_xy[0] - rx, target_xy[1] - ry)
        # Need to be close in odometry AND see the pillar centred-ish.
        if d <= C.PILLAR_REACH_TOL:
            if pdet is None:
                return True   # very close on map; vision occlusion ok
            if abs(pdet["bearing"]) < C.PILLAR_BEARING_TOL:
                return True
        # Belt-and-braces: image area proxy
        if pdet is not None and self._pillar_area_thresh is not None:
            if (pdet.get("area", 0.0) > self._pillar_area_thresh
                    and abs(pdet["bearing"]) < C.PILLAR_BEARING_TOL):
                rng = pdet.get("range", float("nan"))
                if (np.isfinite(rng) and rng < C.PILLAR_REACH_TOL + 0.2) \
                        or not np.isfinite(rng):
                    return True
        return False

    # ----------------------------- main step ----------------------------

    def _drive(self, v, w):
        self.io.set_cmd(v, w)
        self.stuck.update(self.odom.pose(), w, self.sim_time)

    def _enter_recovery(self, reason=""):
        if self.fsm.recovery_chain >= C.RECOVERY_MAX_CHAIN:
            print(f"[recovery] chain exhausted ({reason}); blacklisting & "
                  f"resetting plan", flush=True)
            # blacklist roughly in front of the robot to avoid the same trap
            rx, ry, rth = self.odom.pose()
            self.frontier_blacklist.append(
                (rx + 0.4 * math.cos(rth), ry + 0.4 * math.sin(rth))
            )
            self.fsm.reset_recovery_chain()
        print(f"[recovery] BEGIN ({reason})", flush=True)
        self.recovery.begin(self.sim_time, prefer_spin_dir=None)
        self.fsm.transition(State.RECOVERY, self.sim_time)
        self.follower.set_path([])
        self.stuck.reset(self.odom.pose(), self.sim_time)

    def _post_recovery_state(self):
        """Choose next state after a recovery cycle."""
        if self.blue_reached:
            return State.EXPLORE_YELLOW
        return State.EXPLORE_BLUE

    def _maybe_save_png(self, force=False):
        if force or (self.sim_time - self._last_png_t) >= C.PNG_DUMP_PERIOD_S:
            stem = f"grid_{int(self.sim_time):04d}.png"
            text = (f"t={self.sim_time:6.2f}s  state={self.fsm.state}\n"
                    f"blue={self.pillar_world_pos['blue']}\n"
                    f"yellow={self.pillar_world_pos['yellow']}")
            dump_grid(
                os.path.join(self.png_dir, stem),
                self.grid,
                pose=self.odom.pose(),
                path_world=self.follower.path or None,
                frontier=self.cur_frontier,
                standoff_world=self.cur_standoff,
                state_text=text,
            )
            self._last_png_t = self.sim_time

    def _maybe_debug_dump(self):
        if (self.sim_time - self._last_dbg_t) < C.DEBUG_DUMP_PERIOD_S:
            return
        self._last_dbg_t = self.sim_time
        if self.last_costmap is not None:
            dump_costmap(
                os.path.join(self.png_dir, f"cost_{int(self.sim_time):04d}.png"),
                self.grid, self.last_costmap,
            )
        dump_camera_masks(
            os.path.join(self.png_dir, f"cam_{int(self.sim_time):04d}"),
            self.perc, self.io,
        )

    # ----------------------------- per-state logic ----------------------

    def _state_init(self):
        if self.fsm.time_in_state(self.sim_time) > 0.25:
            self._last_theta = self.odom.pose()[2]
            self.theta_spin_accum = 0.0
            self.fsm.transition(State.INIT_SCAN, self.sim_time)

    def _state_init_scan(self):
        self.io.set_cmd(0.0, 0.9)
        self._track_spin()
        if self.theta_spin_accum >= 2 * math.pi * C.INITIAL_SCAN_REVS:
            self.io.stop()
            self.fsm.transition(State.EXPLORE_BLUE, self.sim_time)
            self.follower.set_path([])
            self.cur_frontier = None
            self.stuck.reset(self.odom.pose(), self.sim_time)

    def _state_explore(self, color, ranges, dets):
        """Common explore logic. Returns the (v, w) command."""
        # If the pillar of the current colour has been detected with a world
        # position, try to plan an approach. Throttled inside _replan_to_pillar
        # so a stuck target doesn't spam log lines.
        if self.pillar_world_pos.get(color) is not None:
            ok, _ = self._replan_to_pillar(color)
            if ok:
                print(f"[mission] {color.upper()} approach plan locked in",
                      flush=True)
                self.cur_target_color = color
                self.fsm.transition(
                    State.GO_BLUE if color == "blue" else State.GO_YELLOW,
                    self.sim_time,
                )
                self.stuck.reset(self.odom.pose(), self.sim_time)
                return 0.0, 0.0
            # planning failed or skipped; fall through to frontier exploration

        # Periodic frontier (re)plan if no path or path stale/invalid
        need_plan = (
            not self.follower.has_path()
            or self._path_invalid()
            or (self.sim_time - self.t_last_plan) > 6.0
        )
        if need_plan:
            ok = self._replan_to_frontier()
            if not ok:
                # No reachable frontier — fall back to a reactive driver so
                # the robot keeps moving and the map keeps growing.
                if self.fsm.time_in_state(self.sim_time) > 25.0 \
                        and not any(self.grid.unknown_mask().ravel()[::32]):
                    print("[explore] map saturated — entering DONE", flush=True)
                    self.fsm.transition(State.DONE, self.sim_time)
                    return 0.0, 0.0
                return self._reactive_drive(ranges, dets)

        # follow the current plan
        v, w, status = self.follower.step(
            self.odom.pose(), ranges["ranges"], ranges["angles"],
            aux_clear=self._aux_clear(),
        )
        if status == "arrived":
            self.cur_frontier = None
            self.follower.set_path([])
            return 0.0, 0.0
        if status == "blocked":
            # The path is locally blocked; replan next tick.
            self.t_last_plan = -math.inf
            # While waiting for the replan, drive reactively rather than
            # freezing, so we keep gathering information.
            return self._reactive_drive(ranges, dets)
        return v, w

    def _state_go_pillar(self, color, ranges, dets):
        # Visual reach test (most reliable when the pillar is in view)
        pdet = dets.get(color)
        if self._pillar_reached(color, pdet):
            self._handle_pillar_arrival(color)
            return 0.0, 0.0

        # If we have lost sight AND drifted far from the recorded pos, the
        # observation may have been noisy: refine it from current detections
        # and replan.
        if pdet is not None:
            self._smooth_pillar(color, pdet)

        # Replan periodically or if the path is invalid
        need_plan = (
            not self.follower.has_path()
            or self._path_invalid()
            or (self.sim_time - self.t_last_plan) > C.ASTAR_REPLAN_EVERY_S
        )
        if need_plan:
            ok, _ = self._replan_to_pillar(color)
            if not ok:
                # Give it a couple of seconds before giving up
                if self.fsm.time_in_state(self.sim_time) > 6.0:
                    print(f"[mission] cannot plan to {color}, back to explore",
                          flush=True)
                    self.fsm.transition(
                        State.EXPLORE_BLUE if color == "blue"
                        else State.EXPLORE_YELLOW, self.sim_time,
                    )
                    self.follower.set_path([])
                # While waiting, drive reactively instead of just spinning so
                # we keep filling in the map.
                return self._reactive_drive(ranges, dets)

        # Stuck on the way?
        if self.stuck.stuck(self.sim_time, timeout_s=C.STUCK_TIMEOUT_S) \
                or self.stuck.oscillating():
            self._enter_recovery(f"stuck-going-to-{color}")
            return 0.0, 0.0

        v, w, status = self.follower.step(
            self.odom.pose(), ranges["ranges"], ranges["angles"],
            aux_clear=self._aux_clear(),
        )
        if status == "arrived":
            # Standoff reached but reach test didn't fire yet — try again
            # next tick with the visual test.
            self.follower.set_path([])
            self.t_last_plan = -math.inf
            return 0.0, 0.0
        if status == "blocked":
            self.t_last_plan = -math.inf
            return self._reactive_drive(ranges, dets)
        return v, w

    def _handle_pillar_arrival(self, color):
        if color == "blue":
            self.blue_reached = True
            self.t_blue_reached = self.sim_time
            print(f"[mission] BLUE PILLAR REACHED @ t={self.sim_time:.2f}s",
                  flush=True)
            self.follower.set_path([])
            self.fsm.transition(State.EXPLORE_YELLOW, self.sim_time)
            self.fsm.reset_recovery_chain()
            self.stuck.reset(self.odom.pose(), self.sim_time)
        else:
            self.t_yellow_reached = self.sim_time
            blue_to_yellow = self.sim_time - (self.t_blue_reached or 0.0)
            print(f"[mission] YELLOW PILLAR REACHED @ t={self.sim_time:.2f}s "
                  f"(blue->yellow leg = {blue_to_yellow:.2f}s)", flush=True)
            self.follower.set_path([])
            self.fsm.transition(State.DONE, self.sim_time)

    def _state_recovery(self):
        v, w, done = self.recovery.step(self.sim_time)
        self.io.set_cmd(v, w)
        if done:
            nxt = self._post_recovery_state()
            print(f"[recovery] END -> {nxt}", flush=True)
            self.fsm.transition(nxt, self.sim_time)
            self.stuck.reset(self.odom.pose(), self.sim_time)
            self.t_last_plan = -math.inf

    def _state_done(self):
        self.io.stop()
        if not self._export_done:
            self._finalize_exports()

    # ----------------------------- exports ------------------------------

    def _finalize_exports(self):
        if self._export_done:
            return
        self._export_done = True
        try:
            export_final_png(
                os.path.join(self.png_dir, "final_map.png"),
                self.grid,
                pose_history=self.pose_history,
                pillar_positions={
                    "blue": self.pillar_world_pos.get("blue"),
                    "yellow": self.pillar_world_pos.get("yellow"),
                },
            )
            if self.cloud_chunks:
                all_pts = np.concatenate(self.cloud_chunks, axis=0)
                export_ply(
                    os.path.join(self.png_dir, "scene.ply"),
                    all_pts, voxel=0.04,
                )
        except Exception as e:
            print(f"[export] failed: {e}", flush=True)

    # ----------------------------- run ----------------------------------

    def run(self):
        try:
            while self.io.step() != -1:
                self.sim_time += self.dt
                self.tick += 1

                # 1. odometry --------------------------------------------
                wl, wr = self.io.read_encoders()
                if wl is not None:
                    yaw = self.io.read_yaw()
                    self.odom.update(wl, wr, yaw, self.dt)
                pose = self.odom.pose()
                if self.tick % 5 == 0:
                    self.pose_history.append(pose)

                # 2. mapping ---------------------------------------------
                ranges_arr = np.zeros(0)
                angles_arr = np.zeros(0)
                if self.scan is not None:
                    pts, ranges_arr = self.scan.scan_world(pose)
                    self.grid.integrate_scan(pose, pts)
                    angles_arr = self.scan._angles
                self._project_aux_obstacles(pose)
                # Guarantee the robot's footprint is plannable. Done AFTER
                # mapping so spurious aux-obstacle hits inside the chassis
                # are erased before A* sees them.
                self._carve_free_around_robot()

                # 3. perception ------------------------------------------
                dets = self.perc.detect() if self.perc is not None \
                    else {"blue": None, "yellow": None, "green": None}
                if dets.get("blue") is not None:
                    self._smooth_pillar("blue", dets["blue"])
                if dets.get("yellow") is not None:
                    self._smooth_pillar("yellow", dets["yellow"])
                self._project_green_to_grid(dets.get("green"))

                # 4. cloud accumulation ----------------------------------
                self._accumulate_cloud()

                # 5. FSM tick --------------------------------------------
                ranges = {"ranges": ranges_arr, "angles": angles_arr}
                state = self.fsm.state
                if state == State.INIT:
                    self._state_init()
                elif state == State.INIT_SCAN:
                    self._state_init_scan()
                elif state == State.EXPLORE_BLUE:
                    v, w = self._state_explore("blue", ranges, dets)
                    self._drive(v, w)
                    if self.stuck.stuck(self.sim_time, timeout_s=C.STUCK_TIMEOUT_S) \
                            or self.stuck.oscillating():
                        self._enter_recovery("stuck-explore-blue")
                elif state == State.GO_BLUE:
                    v, w = self._state_go_pillar("blue", ranges, dets)
                    self._drive(v, w)
                elif state == State.EXPLORE_YELLOW:
                    v, w = self._state_explore("yellow", ranges, dets)
                    self._drive(v, w)
                    if self.stuck.stuck(self.sim_time, timeout_s=C.STUCK_TIMEOUT_S) \
                            or self.stuck.oscillating():
                        self._enter_recovery("stuck-explore-yellow")
                elif state == State.GO_YELLOW:
                    v, w = self._state_go_pillar("yellow", ranges, dets)
                    self._drive(v, w)
                elif state == State.RECOVERY:
                    self._state_recovery()
                elif state == State.DONE:
                    self._state_done()

                # 6. dumps -----------------------------------------------
                self._maybe_save_png()
                self._maybe_debug_dump()
                self._log()
        except KeyboardInterrupt:
            print("[main] interrupted", flush=True)
        finally:
            self._finalize_exports()


def main():
    try:
        nav = MazeNavigator()
        nav.run()
    except Exception as e:
        print("[maze_navigator] fatal:", e, file=sys.stderr)
        raise


if __name__ == "__main__":
    main()
