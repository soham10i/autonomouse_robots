"""mak_02_controller — Maze4 frontier-exploration navigation for the ROSbot.

Mission (from Modularbeit.pdf): drive from the start to the BLUE pillar, then to
the YELLOW pillar, in the least simulation time, without touching walls or the
green poison floor.  No Webots Supervisor is used; the robot localises with
wheel + IMU odometry refined by a light lidar scan matcher, maps with a 2-D
lidar log-odds grid, explores with visual frontiers, plans with A*, and drives
with a DWA local planner on the live lidar (so wall clearance is drift-immune).

Maze4-specific addition vs. the Maze5 baseline: 4 low wall panels are partially
or fully invisible to the single-plane 2-D lidar (see config.py).  A
depth-camera-derived "aux" obstacle layer (mapping.OccupancyGrid.aux, built via
raytrace-clearing, NOT the sticky/decay scheme that caused bugs on Maze1) and
the chassis IR proximity sensors (close-range hard-stop backstop) cover them.

FSM:  INIT_SCAN -> EXPLORE_BLUE -> GO_BLUE -> EXPLORE_YELLOW -> GO_YELLOW -> DONE
      (RECOVERY is reachable from any driving state when progress stalls.)

Run: set the Rosbot node's ``controller`` field to ``mak_02_controller`` (already
set in Maze4.wbt).  Press 'Q' in the sim to finalise outputs early.
"""
from __future__ import annotations

import math
import os

import numpy as np

import config as C
import astar
import frontier as FR
import local_planner as LP
from depth_model import DepthModel
from geometry import (compose_pose, relative_pose, wrap_angle, pose_distance,
                      transform_points, inverse_transform_points)
from mapping import LidarModel, OccupancyGrid
from odometry import Odometry
from perception import Perception
from robot_io import RobotIO
from viz import Visualizer


class Mission:
    INIT_SCAN = "INIT_SCAN"
    EXPLORE_BLUE = "EXPLORE_BLUE"
    GO_BLUE = "GO_BLUE"
    EXPLORE_YELLOW = "EXPLORE_YELLOW"
    GO_YELLOW = "GO_YELLOW"
    RECOVERY = "RECOVERY"
    DONE = "DONE"


class Mak02Controller:
    def __init__(self):
        self.io = RobotIO()
        self.dt = self.io.dt

        specs = self.io.lidar_specs()
        if specs is not None:
            n_beams, fov, r_min, r_max = specs
            self.lidar = LidarModel(n_beams, fov, r_min, r_max)
        else:
            self.lidar = None
            print("[mak_02] WARNING: no lidar found")

        self.grid = OccupancyGrid()
        self.odom = Odometry()

        cam = self.io.camera_specs()
        if cam is not None:
            self.percep = Perception(cam[0], cam[1], cam[2])
        else:
            self.percep = None
            print("[mak_02] WARNING: no camera found")

        dspecs = self.io.depth_specs()
        if dspecs is not None:
            dw, dh, dfov, dmin, dmax = dspecs
            self.depth_model = DepthModel(dw, dh, dfov, depth_min=dmin, depth_max=dmax)
        else:
            self.depth_model = None
            print("[mak_02] WARNING: no depth camera found — low floating panels "
                  "will rely on the IR bumper only")

        self.dwa = LP.DWAPlanner(ctrl_dt=self.dt)
        self.viz = Visualizer(self.grid)

        # pose belief (odom frame; absolute world frame is unknown w/o supervisor)
        self.pose = (0.0, 0.0, 0.0)
        self._last_raw = None
        self.start_xy = (0.0, 0.0)

        # mapping bookkeeping
        self.scan_body = np.empty((0, 2))
        self.scan_ranges = np.empty((0,))
        self._pose_at_last_integrate = None
        self.cost = None
        self.lethal = None
        self._last_costmap_t = -1e9

        # mission state
        self.state = Mission.INIT_SCAN
        self.spin_accum = 0.0
        self._prev_theta_for_spin = None
        self.path = []                 # current global path (world points)
        self.goal_world = None
        self.carrot = None
        self.frontier_mask = None
        self.blacklist = []            # failed frontier goals (world)
        self.go_fail_until = 0.0
        self.no_frontier_until = 0.0
        self.cur_v = 0.0

        # stuck / recovery
        self._progress_ref = None
        self._progress_ref_t = 0.0
        self._frozen_ref = None
        self._frozen_t = 0.0
        self.recovery_phase = None
        self.recovery_t0 = 0.0
        self.recovery_return = Mission.EXPLORE_BLUE
        self.recovery_chain = 0

        # timing table
        self.t_start = None
        self.t_blue = None
        self.t_yellow = None

        self.tick = 0
        self._last_snapshot_t = -1e9
        self.green_block = False
        self.ir_block = False
        self.outdir = os.path.join(os.path.dirname(os.path.abspath(__file__)),
                                   C.OUTPUT_DIRNAME)
        os.makedirs(self.outdir, exist_ok=True)

    # ===================================================================== #
    #  Sensing & localisation
    # ===================================================================== #
    def _sense(self):
        wl, wr = self.io.read_encoders()
        yaw = self.io.read_yaw()
        self.odom.update(wl, wr, yaw, self.dt)
        raw = self.odom.pose()
        if self._last_raw is None:
            self.pose = raw
            self._last_raw = raw
        else:
            inc = relative_pose(self._last_raw, raw)
            self.pose = compose_pose(self.pose, inc)
            self._last_raw = raw

        if self.lidar is not None:
            ranges = self.io.read_lidar_ranges()
            if ranges is not None:
                self.scan_body, self.scan_ranges = self.lidar.ranges_to_body(ranges)

    def _moved_enough(self):
        if self._pose_at_last_integrate is None:
            return True
        dx, dy, dth = relative_pose(self._pose_at_last_integrate, self.pose)
        return (math.hypot(dx, dy) > C.SM_MIN_TRAVEL_M or
                abs(dth) > C.SM_MIN_TURN_RAD)

    def _slam_step(self):
        if self.scan_body.shape[0] == 0 or not self._moved_enough():
            return
        if C.SM_ENABLED:
            corrected, hit = self.grid.scan_match(self.pose, self.scan_body)
            self.pose = corrected
        self.grid.integrate_scan(self.pose, self.scan_body, self.scan_ranges)
        self.grid.mark_free_disc(self.pose[0], self.pose[1], C.ROBOT_RADIUS * 0.8)
        self._pose_at_last_integrate = self.pose

    def _perception_step(self):
        if self.percep is None:
            return
        bgr = self.io.read_rgb_bgr()
        if bgr is None:
            return
        depth = self.io.read_depth()
        self.percep.update_pillars(bgr, depth, self.pose)
        green_world = self.percep.green_floor_world(bgr, self.pose)
        self.grid.add_poison_points(green_world)
        self.green_block = self.percep.green_reflex(bgr)

    def _depth_aux_step(self):
        """Thin-scan the depth image and raytrace-update the aux obstacle layer.

        See mapping.OccupancyGrid.integrate_aux + the module docstring for why
        this is hit/clear only (no sticky counters, no decay) — that is the
        deliberate fix for the bug history documented in Maze1's HANDOFF.md.
        """
        if self.depth_model is None:
            return
        depth = self.io.read_depth()
        if depth is None:
            return
        bearings, hit_ranges, hit_mask, clear_mask = self.depth_model.thin_scan(depth)
        self.grid.integrate_aux(self.pose, bearings, hit_ranges, hit_mask, clear_mask,
                                self.depth_model.depth_min)

    def _ir_step(self):
        """Close-range chassis-IR hard-stop, independent of mapping (last resort
        for the lowest panel, WallShort(15), which sits below where the depth
        camera reliably resolves at very close range)."""
        if not C.IR_BUMPER_ENABLED:
            self.ir_block = False
            return
        ir = self.io.read_ir_m()
        if not ir:
            self.ir_block = False
            return
        vals = [v for v in ir.values() if np.isfinite(v)]
        self.ir_block = bool(vals) and min(vals) < C.IR_BUMPER_STOP_DIST

    # ===================================================================== #
    #  Planning
    # ===================================================================== #
    def _ensure_costmap(self, t, force=False):
        if force or self.cost is None or (t - self._last_costmap_t) >= C.REPLAN_PERIOD_S:
            self.cost, self.lethal = self.grid.build_costmap()
            self._last_costmap_t = t

    def _plan_to(self, goal_world):
        if self.cost is None:
            return None
        start = self.grid.world_to_grid(self.pose[0], self.pose[1])
        goal = self.grid.world_to_grid(goal_world[0], goal_world[1])
        cells = astar.plan(self.cost, self.lethal, start, goal)
        if cells is None:
            return None
        cells = astar.simplify(cells, self.lethal, self.cost)
        return [self.grid.grid_to_world(ix, iy) for (ix, iy) in cells]

    def _blacklisted(self, wxy):
        for bx, by in self.blacklist:
            if math.hypot(wxy[0] - bx, wxy[1] - by) < C.FRONTIER_BLACKLIST_R:
                return True
        return False

    def _select_frontier(self):
        """Pick the best reachable frontier; return (path, goal_world) or (None,None)."""
        self.frontier_mask = FR.detect_frontier_cells(self.grid, self.lethal)
        cands = FR.find_frontiers(self.grid, self.lethal, self.pose[:2], self.start_xy)
        cands = [c for c in cands if not self._blacklisted(c["world"])]
        if not cands:
            return None, None
        # cheapest first: evaluate the closest handful with real A* path length
        rx, ry = self.pose[0], self.pose[1]
        cands.sort(key=lambda c: math.hypot(c["world"][0] - rx, c["world"][1] - ry))
        best = None
        for c in cands[:7]:
            path = self._plan_to(c["world"])
            if path is None:
                continue
            plen = self._path_length(path)
            util = C.UTIL_INFO_W * c["size"] - C.UTIL_DIST_W * plen
            if best is None or util > best[0]:
                best = (util, path, c["world"])
        if best is None:
            return None, None
        return best[1], best[2]

    @staticmethod
    def _path_length(path):
        return sum(math.hypot(path[i + 1][0] - path[i][0], path[i + 1][1] - path[i][1])
                   for i in range(len(path) - 1))

    def _pillar_standoff(self, pillar_world):
        """A reachable point ~PILLAR_STANDOFF short of the pillar centre."""
        px, py = pillar_world
        d = math.hypot(px - self.pose[0], py - self.pose[1])
        if d < 1e-3:
            return pillar_world
        ux, uy = (px - self.pose[0]) / d, (py - self.pose[1]) / d
        s = max(0.0, d - C.PILLAR_STANDOFF)
        return (self.pose[0] + ux * s, self.pose[1] + uy * s)

    # ===================================================================== #
    #  Driving
    # ===================================================================== #
    def _extra_obstacles_body(self):
        """Mapped poison + aux (low-panel) points near the robot, body frame.

        Folded into one array for the DWA local planner — both are static,
        mapped obstacle sources the live lidar can't (always) see directly.
        """
        near_r = C.DWA_SLOWDOWN_DIST + 0.4
        pw = self.grid.poison_points_near(self.pose[0], self.pose[1], near_r)
        aw = self.grid.aux_points_near(self.pose[0], self.pose[1], near_r)
        parts = [a for a in (pw, aw) if a.shape[0] > 0]
        if not parts:
            return np.empty((0, 2))
        world = np.concatenate(parts, axis=0)
        return inverse_transform_points(world, self.pose[0], self.pose[1], self.pose[2])

    def _drive_to(self, path, v_cap=None):
        """Carrot + DWA toward the end of ``path``; returns (v, w, near_goal)."""
        if not path:
            return 0.0, 0.0, True
        carrot, near_goal = LP.choose_carrot(path, self.pose, self.cur_v)
        self.carrot = carrot
        extra_b = self._extra_obstacles_body()
        v, w = self.dwa.compute(self.pose, carrot, self.scan_body, extra_b,
                                green_block=self.green_block, v_cap=v_cap)
        return v, w, near_goal

    # ===================================================================== #
    #  Stuck detection & recovery
    # ===================================================================== #
    def _reset_progress(self, t):
        self._progress_ref = (self.pose[0], self.pose[1])
        self._progress_ref_t = t

    def _is_stuck(self, t):
        """Pure POSITION watchdog: no translation for STUCK_TIMEOUT_S in a
        driving state means stuck, regardless of what is currently commanded.

        Maze1's mak_02_controller documents this exact bug under the same
        name: its no-reachable-frontier branch used to "spin to look around"
        WITHOUT running the stuck-check, "the permanent freeze" -- because a
        v=0 in-place rescan spin never satisfies a forward-command gate, and
        the heading is visibly changing every tick so a frozen-pose backstop
        keyed on heading ALSO never fires.  Two watchdogs, both blind to the
        one failure mode that actually happens (spin-only deadlock).  Maze1's
        fix was to drop the forward-command requirement and key stuck-ness on
        translation alone; ported here unchanged.
        """
        if self._progress_ref is None:
            self._reset_progress(t)
            return False
        moved = math.hypot(self.pose[0] - self._progress_ref[0],
                           self.pose[1] - self._progress_ref[1])
        if moved > C.STUCK_PROGRESS_MIN_M:
            self._reset_progress(t)
            self.recovery_chain = 0
            return False
        return (t - self._progress_ref_t) > C.STUCK_TIMEOUT_S

    def _is_frozen(self, t):
        """Backstop: True if NEITHER position NOR heading has changed for too long
        in a driving state (catches any v=0,w=0 deadlock the stuck-check misses
        because it requires forward command)."""
        p = (self.pose[0], self.pose[1], self.pose[2])
        if self._frozen_ref is None:
            self._frozen_ref, self._frozen_t = p, t
            return False
        moved = math.hypot(p[0] - self._frozen_ref[0], p[1] - self._frozen_ref[1])
        turned = abs(wrap_angle(p[2] - self._frozen_ref[2]))
        if moved > 0.03 or turned > 0.05:
            self._frozen_ref, self._frozen_t = p, t
            return False
        return (t - self._frozen_t) > C.FROZEN_TIMEOUT_S

    def _enter_recovery(self, t, return_state):
        self.recovery_return = return_state
        self.state = Mission.RECOVERY
        self.recovery_phase = "reverse"
        self.recovery_t0 = t
        self.recovery_chain += 1
        self._frozen_ref = None   # reset the frozen watchdog on entering recovery
        self.dwa.reset()
        rear = LP.rear_clearance(self.scan_body, 0.18)
        if rear < C.RECOVERY_REAR_MIN_CLEAR:
            self.recovery_phase = "spin"   # no room behind -> spin instead
        print(f"[recovery] enter ({self.recovery_phase}); rear={rear:.2f}m chain={self.recovery_chain}")

    def _run_recovery(self, t):
        if self.recovery_phase == "reverse":
            if (t - self.recovery_t0) < C.RECOVERY_REVERSE_T and \
               LP.rear_clearance(self.scan_body, 0.18) > C.RECOVERY_REAR_MIN_CLEAR:
                return -C.RECOVERY_REVERSE_V, 0.0
            self.recovery_phase = "spin"
            self.recovery_t0 = t
        # spin toward the freer side
        side = LP.freer_side(self.scan_body)
        if (t - self.recovery_t0) < C.RECOVERY_SPIN_T:
            return 0.0, side * C.RECOVERY_SPIN_W
        # done
        if self.recovery_chain >= C.RECOVERY_MAX_CHAIN and self.goal_world is not None:
            self.blacklist.append(self.goal_world)   # give up on this goal
            self.recovery_chain = 0
        self.path = []
        self.goal_world = None
        self._reset_progress(t)
        self.state = self.recovery_return
        return 0.0, 0.0

    # ===================================================================== #
    #  Mission state machine
    # ===================================================================== #
    def _maybe_go(self, target, go_state, t):
        """Switch EXPLORE->GO iff the pillar is known AND an A* path exists now."""
        pw = self.percep.pillar_world.get(target) if self.percep else None
        if pw is None or t < self.go_fail_until:
            return False
        # throttle the reachability A* so we don't run it every 32 ms tick
        if (t - getattr(self, "_last_go_try", -1e9)) < 0.4:
            return False
        self._last_go_try = t
        goal = self._pillar_standoff(pw)
        path = self._plan_to(goal)
        if path is None:
            return False
        self.path = path
        self.goal_world = goal
        self.state = go_state
        self._reset_progress(t)
        self.dwa.reset()
        print(f"[mission] {target} pillar known at "
              f"({pw[0]:.2f},{pw[1]:.2f}) -> {go_state}")
        return True

    def _explore(self, target, go_state, t):
        if self._maybe_go(target, go_state, t):
            return self._drive_to(self.path)[:2]
        # (re)select a frontier goal when needed
        need_new = (not self.path or self.goal_world is None or
                    pose_distance(self.pose, self.goal_world) < C.GOAL_REACH_TOL or
                    (t - self._last_plan_t()) > C.REPLAN_PERIOD_S)
        if need_new:
            path, goal = self._select_frontier()
            if path is not None:
                self.path, self.goal_world = path, goal
                self._plan_stamp = t
            else:
                # nothing to explore: rescan in place for a while
                if t < self.no_frontier_until:
                    return 0.0, C.INIT_SPIN_W
                self.no_frontier_until = t + C.NO_FRONTIER_SPIN_S
                self.path, self.goal_world = [], None
                return 0.0, C.INIT_SPIN_W
        v, w, _ = self._drive_to(self.path)
        return v, w

    def _last_plan_t(self):
        return getattr(self, "_plan_stamp", -1e9)

    def _go(self, target, next_state, t, reached_cb):
        pw = self.percep.pillar_world.get(target) if self.percep else None
        if pw is None:
            self.state = self.recovery_return = (Mission.EXPLORE_BLUE
                if target == "blue" else Mission.EXPLORE_YELLOW)
            return 0.0, 0.0
        # arrival check
        if pose_distance(self.pose, pw) <= C.PILLAR_REACH_DIST:
            reached_cb(t)
            self.io.stop()
            self.path = []
            self.dwa.reset()
            self._reset_progress(t)   # don't inherit stale stuck timer into next state
            self.state = next_state
            return 0.0, 0.0
        # keep the goal/standoff fresh as the estimate refines
        goal = self._pillar_standoff(pw)
        if (self.goal_world is None or pose_distance(goal, self.goal_world) > 0.15
                or not self.path or (t - self._last_plan_t()) > C.REPLAN_PERIOD_S):
            path = self._plan_to(goal)
            if path is None:
                # Force costmap rebuild and retry once before giving up
                self._ensure_costmap(t, force=True)
                path = self._plan_to(goal)
            if path is None:
                # truly blocked: fall back to exploring to open a route
                self.go_fail_until = t + C.GO_FAIL_COOLDOWN_S
                self.state = (Mission.EXPLORE_BLUE if target == "blue"
                              else Mission.EXPLORE_YELLOW)
                print(f"[mission] no path to {target}; exploring to open route")
                return 0.0, 0.0
            self.path, self.goal_world = path, goal
            self._plan_stamp = t
        v, w, near = self._drive_to(self.path, v_cap=C.V_CRUISE)
        return v, w

    def _reached_blue(self, t):
        self.t_blue = t
        print(f"\n*** BLUE pillar reached at t = {t:.2f} s "
              f"(start->blue = {t - self.t_start:.2f} s) ***\n")

    def _reached_yellow(self, t):
        self.t_yellow = t
        print(f"\n*** YELLOW pillar reached at t = {t:.2f} s "
              f"(blue->yellow = {t - self.t_blue:.2f} s) ***\n")

    def _step_fsm(self, t):
        st = self.state
        if st == Mission.INIT_SCAN:
            if self._prev_theta_for_spin is None:
                self._prev_theta_for_spin = self.pose[2]
            self.spin_accum += abs(wrap_angle(self.pose[2] - self._prev_theta_for_spin))
            self._prev_theta_for_spin = self.pose[2]
            if self.spin_accum >= C.INITIAL_SCAN_REVS * 2.0 * math.pi:
                self.state = Mission.EXPLORE_BLUE
                self._reset_progress(t)
                print("[mission] initial scan complete -> EXPLORE_BLUE")
                return 0.0, 0.0
            return 0.0, C.INIT_SPIN_W

        if st == Mission.EXPLORE_BLUE:
            return self._explore("blue", Mission.GO_BLUE, t)
        if st == Mission.GO_BLUE:
            return self._go("blue", Mission.EXPLORE_YELLOW, t, self._reached_blue)
        if st == Mission.EXPLORE_YELLOW:
            return self._explore("yellow", Mission.GO_YELLOW, t)
        if st == Mission.GO_YELLOW:
            return self._go("yellow", Mission.DONE, t, self._reached_yellow)
        if st == Mission.RECOVERY:
            return self._run_recovery(t)
        # DONE
        return 0.0, 0.0

    # ===================================================================== #
    #  Visualisation / output
    # ===================================================================== #
    def _visualize(self, t, force=False):
        snap = force or (t - self._last_snapshot_t) >= C.SNAPSHOT_PERIOD_S
        # render live at ~10 Hz (every 3rd tick); always render when snapping
        if not (snap or force or self.tick % 3 == 0):
            return
        pillars = {k: v for k, v in (self.percep.pillar_world.items()
                                     if self.percep else [])}
        img = self.viz.render(self.pose, self.frontier_mask, self.path,
                              self.carrot, pillars, self.state, t)
        self.viz.show(img)
        if snap:
            self.viz.save(img, os.path.join(self.outdir, "live_map.png"))
            self._last_snapshot_t = t

    def _print_timing(self):
        print("\n================ TIMING TABLE ================")
        if self.t_blue is not None:
            print(f"  start  -> blue   : {self.t_blue - self.t_start:7.2f} s")
        else:
            print("  start  -> blue   :   (not reached)")
        if self.t_yellow is not None and self.t_blue is not None:
            print(f"  blue   -> yellow : {self.t_yellow - self.t_blue:7.2f} s")
            print(f"  TOTAL  (start->yellow): {self.t_yellow - self.t_start:7.2f} s")
        else:
            print("  blue   -> yellow :   (not reached)")
        print("=============================================\n")

    def _finalize(self):
        self.io.stop()
        self.io.step()
        self._ensure_costmap(self.io.time(), force=True)
        self._visualize(self.io.time(), force=True)
        try:
            self.viz.save(self.viz.render(self.pose, self.frontier_mask, self.path,
                          self.carrot, {k: v for k, v in (self.percep.pillar_world.items()
                          if self.percep else [])}, self.state, self.io.time()),
                          os.path.join(self.outdir, "final_map.png"))
            np.savez_compressed(os.path.join(self.outdir, "map.npz"),
                                L=self.grid.L, poison=self.grid.poison, aux=self.grid.aux)
        except Exception as e:
            print("[mak_02] finalize save error:", e)
        self._print_timing()
        self.viz.close()

    # ===================================================================== #
    #  Main loop
    # ===================================================================== #
    def run(self):
        # prime sensors
        if self.io.step() == -1:
            return
        self._sense()
        self.start_xy = (self.pose[0], self.pose[1])
        self.t_start = self.io.time()
        print(f"[mak_02] start pose (odom) = ({self.pose[0]:.2f},{self.pose[1]:.2f},"
              f"{math.degrees(self.pose[2]):.1f} deg)")

        while self.io.step() != -1:
            self.tick += 1
            t = self.io.time()

            keys = self.io.poll_key()
            if ord('Q') in keys or ord('q') in keys:
                print("[mak_02] Q pressed -> finalize")
                break

            self._sense()
            self._slam_step()
            if self.tick % C.PERCEPTION_EVERY_TICKS == 0:
                self._perception_step()
            if self.tick % C.DEPTH_AUX_EVERY_TICKS == 0:
                self._depth_aux_step()
            self._ir_step()
            self._ensure_costmap(t)
            # keep the robot's own cell plannable in the costmap snapshot
            self.grid.mark_free_disc(self.pose[0], self.pose[1], C.ROBOT_RADIUS * 0.8)

            v, w = self._step_fsm(t)

            # position watchdog applies in every driving state, REGARDLESS of
            # what is currently commanded (see _is_stuck docstring) -- this is
            # what catches the no-reachable-frontier rescan spin (v=0, w!=0).
            driving = self.state in (Mission.EXPLORE_BLUE, Mission.GO_BLUE,
                                     Mission.EXPLORE_YELLOW, Mission.GO_YELLOW)
            if driving and (self._is_stuck(t) or self._is_frozen(t)):
                self._enter_recovery(t, self.state)
                v, w = 0.0, 0.0

            if self.green_block and v > 0.0:
                v = 0.0  # hard poison reflex safety net
            if self.ir_block and v > 0.0:
                v = 0.0  # hard chassis-IR bumper safety net (low floating panels)

            self.io.set_cmd(v, w)
            self.cur_v = v

            self._visualize(t)
            if self.tick % C.LOG_EVERY_TICKS == 0:
                self._log(t)

            if self.state == Mission.DONE:
                # linger briefly so the final frame is visible, then stop
                if not hasattr(self, "_done_t"):
                    self._done_t = t
                if t - self._done_t > C.DONE_LINGER_S:
                    break

        self._finalize()

    def _log(self, t):
        occ = int(self.grid.occupied_mask().sum())
        pois = int(self.grid.poison.sum())
        aux = int(self.grid.aux.sum())
        pb = self.percep.pillar_world.get("blue") if self.percep else None
        py = self.percep.pillar_world.get("yellow") if self.percep else None
        print(f"[t={t:6.1f}] {self.state:13s} "
              f"pose=({self.pose[0]:+.2f},{self.pose[1]:+.2f},{math.degrees(self.pose[2]):+6.1f}) "
              f"v={self.cur_v:.2f} occ={occ} pois={pois} aux={aux} "
              f"blue={'Y' if pb else '-'} yellow={'Y' if py else '-'} "
              f"green_block={'Y' if self.green_block else '-'} "
              f"ir_block={'Y' if self.ir_block else '-'}")


def main():
    ctrl = Mak02Controller()
    try:
        ctrl.run()
    except Exception:
        import traceback
        traceback.print_exc()
        try:
            ctrl.io.stop()
        except Exception:
            pass


if __name__ == "__main__":
    main()
