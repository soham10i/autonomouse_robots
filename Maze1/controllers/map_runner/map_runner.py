"""Phase 3 — map_runner: SLAM-corrected autonomous BLUE→YELLOW navigation.

Builds its own map from scratch using ICP-corrected odometry, then
navigates autonomously to the blue and yellow pillars.

Architecture
------------
1. **Mapping from scratch**: LiDAR + depth-camera build a live occupancy
   grid.  No pre-loaded ``map.npz`` is required.
2. **ICP scan-to-scan matching**: Every ``ICP_EVERY_N`` scans, the
   ``IcpLocalizer`` aligns the current LiDAR cloud against an
   accumulated reference cloud to correct odometry drift.
3. **Perception**: HSV pillar detection (blue + yellow) with smoothed
   running-mean world positions.  Per-pixel green-floor projection
   for the poison layer.
4. **FSM**:
       INIT_SCAN  →  EXPLORE  →  GO_BLUE  →  AT_BLUE
                                            ↓
                     DONE  ←  AT_YELLOW  ←  GO_YELLOW
                                            ↓
                                          RECOVERY
5. **Navigation**: A* global path + DWA local planner from the shared
   ``maze_navigator`` modules.
6. **Logging**: Every tick is logged to ``maps/run_log.jsonl``; a
   human-readable summary is written on shutdown.

Controls
--------
    (fully autonomous — no keyboard input needed)
    The robot starts mapping immediately, explores using frontier-based
    exploration, then navigates BLUE → YELLOW once both pillars are found.

Module reuse
------------
Imports from ``maze_navigator/``: config, robot_io, odometry, lidar_scan,
clearance, grid_2d, perception, planning, dwa_planner, recovery,
mapping_export, explorer.

Local modules: icp_localizer, run_logger.
"""
import os
import sys
import math

import numpy as np

# ── Make maze_navigator modules importable ──
_HERE = os.path.dirname(os.path.abspath(__file__))
_NAV = os.path.normpath(os.path.join(_HERE, "..", "maze_navigator"))
if os.path.isdir(_NAV) and _NAV not in sys.path:
    sys.path.insert(0, _NAV)

import config as C
from robot_io import RobotIO
from odometry import Odometry
from lidar_scan import ScanProcessor
from clearance import ClearanceChecker
from grid_2d import Grid2D
from perception import PillarDetector
from planning import plan_to_pillar
from dwa_planner import DWAPlanner
from recovery import StuckMonitor, Recovery
from mapping_export import export_ply, export_final_png
from explorer import Explorer

# Local modules
from icp_localizer import IcpLocalizer
from run_logger import RunLogger


# ── Tuning constants ─────────────────────────────────────────────────
ICP_EVERY_N = 3             # run ICP every N LiDAR scans
LIDAR_MAX_RANGE_GRID = 4.0  # clamp LiDAR range for grid integration (reduces long-range smear)
L_FREE_LIVE = -0.2          # reduced free-space decay (slower wall erasure)

MISSION_REPLAN_PERIOD_S = 2.0
MISSION_REACH_DIST_M = 0.50
MISSION_DWELL_AT_PILLAR_S = 1.0

PNG_PERIOD_S = 3.0
CLOUD_PERIOD_S = 0.4
LOG_EVERY_TICKS = 4         # log to JSONL every N ticks

INIT_SCAN_REVS = 1.1        # full rotations at start for initial map
INIT_SCAN_W = 1.0            # rad/s during initial scan

EXPLORE_REPLAN_S = 3.0       # frontier re-planning cadence
EXPLORE_TIMEOUT_S = 180.0    # max time in exploration before giving up

PILLAR_GRACE_AFTER_EXPLORE_S = 15.0  # extra time to find pillars after frontiers exhausted


# ── FSM States ───────────────────────────────────────────────────────
class State:
    INIT_SCAN = "INIT_SCAN"       # initial 360° scan for first map
    EXPLORE = "EXPLORE"            # frontier-based autonomous exploration
    GO_BLUE = "GO_BLUE"
    AT_BLUE = "AT_BLUE"
    GO_YELLOW = "GO_YELLOW"
    AT_YELLOW = "AT_YELLOW"
    RECOVERY = "RECOVERY"
    DONE = "DONE"
    FAILED = "FAILED"


class MapRunner:
    """SLAM-corrected autonomous maze runner with comprehensive logging."""

    def __init__(self):
        # ── Hardware ──
        self.io = RobotIO()
        self.dt = self.io.timestep * 1e-3
        self.sim_time = 0.0
        self.tick = 0

        # ── Sensors + mapping ──
        self.odom = Odometry()
        self.scan = ScanProcessor(self.io.lidar) if self.io.lidar else None
        self.clear = ClearanceChecker(self.io.depth, self.io.camera)
        self.grid = Grid2D()
        self.perc = (
            PillarDetector(self.io.camera, self.io.depth)
            if self.io.camera else None
        )

        # ── ICP localizer ──
        self.icp = IcpLocalizer()
        self._scan_count = 0
        self._corrected_pose = (0.0, 0.0, 0.0)

        # ── Navigation ──
        self.driver = DWAPlanner()
        self.stuck = StuckMonitor()
        self.recovery = Recovery()
        self.recovery_chain = 0
        self._prev_state = None     # for recovery resume
        self.explorer = Explorer(self.grid)

        # ── FSM ──
        self.state = State.INIT_SCAN
        self.t_state = 0.0
        self._init_scan_start_th = None
        self._init_scan_total = 0.0

        # ── Pillar tracking ──
        self._pillar_obs = {"blue": [], "yellow": []}
        self.pillar_world_pos = {"blue": None, "yellow": None}
        self._announced = {"blue": False, "yellow": False}
        self._poison_announced = False

        # ── Mission state ──
        self.cur_target_color = None
        self.cur_standoff = None
        self.t_last_plan = -math.inf
        self.t_at_pillar_start = None
        self.last_path_world = None
        self.t_mission_start = None
        self.t_blue_reached = None
        self.t_yellow_reached = None
        self.path_length_blue_m = 0.0
        self.path_length_yellow_m = 0.0
        self.v_cmd = 0.0
        self.w_cmd = 0.0

        # ── Exploration state ──
        self.t_explore_start = None
        self._explore_last_plan_t = -math.inf
        self._explore_frontier = None
        self._explore_path = None
        self._frontiers_exhausted_t = None

        # ── Outputs ──
        self.maps_dir = os.path.join(_HERE, "maps")
        os.makedirs(self.maps_dir, exist_ok=True)
        self.cloud_chunks = []
        self.pose_history = []
        self._last_cloud_t = -math.inf
        self._last_png_t = -math.inf

        # ── Logger ──
        self.logger = RunLogger(self.maps_dir)

        self._print_banner()

    # ─── Banner ──────────────────────────────────────────────────────
    def _print_banner(self):
        print("=" * 64, flush=True)
        print("MAP RUNNER — SLAM-corrected autonomous BLUE→YELLOW", flush=True)
        print("Builds map from scratch.  ICP corrects odometry drift.", flush=True)
        print(f"Outputs → {self.maps_dir}", flush=True)
        print("=" * 64, flush=True)

    # ─── State transition ────────────────────────────────────────────
    def _transition(self, new_state):
        if new_state == self.state:
            return
        print(f"[fsm] {self.state} → {new_state} @ t={self.sim_time:.2f}s",
              flush=True)
        self.state = new_state
        self.t_state = self.sim_time

    # ─── Pose management ─────────────────────────────────────────────
    def _update_pose(self):
        """Update odometry from wheel encoders + IMU."""
        wl, wr = self.io.read_encoders()
        if wl is not None:
            yaw = self.io.read_yaw()
            self.odom.update(wl, wr, yaw, self.dt)

    def _get_pose(self):
        """Return the best-estimate pose (ICP-corrected or raw odom)."""
        return self._corrected_pose

    def _run_icp(self):
        """Run ICP scan matching if it's time."""
        if self.scan is None:
            return
        self._scan_count += 1
        if self._scan_count % ICP_EVERY_N != 0:
            return

        # Get current scan in local (robot) frame
        ranges = np.asarray(self.scan.lidar.getRangeImage(), dtype=np.float32)
        if ranges.size != self.scan.n:
            return
        valid = (np.isfinite(ranges)
                 & (ranges > self.scan.r_min)
                 & (ranges < self.scan.r_max))
        if valid.sum() < 15:
            return
        a = self.scan._angles[valid]
        r = ranges[valid]
        scan_local = np.stack([r * np.cos(a), r * np.sin(a)], axis=1)

        # Run ICP correction
        odom_pose = self.odom.pose()
        corrected, info = self.icp.correct(odom_pose, scan_local)

        # Update the corrected pose
        self._corrected_pose = corrected

        # If ICP applied a correction, also nudge odometry so future
        # dead-reckoning starts from the corrected position
        if info["applied"]:
            self.odom.x = corrected[0]
            self.odom.y = corrected[1]
            self.odom.theta = corrected[2]

            if self.tick % 64 == 0:
                dx, dy, dth = info["correction"]
                print(f"[icp] correction applied: "
                      f"Δ=({dx:+.4f}, {dy:+.4f}, {math.degrees(dth):+.2f}°) "
                      f"err={info['mean_error']:.4f}m "
                      f"matched={info['n_matched']}/{scan_local.shape[0]} pts "
                      f"ref={info['ref_cloud_size']} pts",
                      flush=True)
        else:
            self._corrected_pose = odom_pose

    # ─── Grid integration ────────────────────────────────────────────
    def _update_grid(self, pose):
        """Integrate LiDAR + depth camera into the occupancy grid."""
        # 1. LiDAR log-odds (with clamped range to reduce long-range smear)
        if self.scan is not None:
            pts, ranges = self.scan.scan_world(pose)
            
            # Map hits (walls)
            if pts.shape[0] > 0:
                # Use reduced L_FREE to slow wall erasure
                original_l_free = C.L_FREE
                C.L_FREE = L_FREE_LIVE
                self.grid.integrate_scan(pose, pts)
                C.L_FREE = original_l_free

            # Map open space from Infinity rays (corridors)
            inf_mask = ~np.isfinite(ranges) | (ranges >= self.scan.r_max)
            if inf_mask.any():
                a = self.scan._angles[inf_mask]
                r = self.scan.r_max - 0.1  # project just inside max range
                x, y, th = pose
                c, s = math.cos(th), math.sin(th)
                lx = r * np.cos(a)
                ly = r * np.sin(a)
                wx = x + c * lx - s * ly
                wy = y + s * lx + c * ly
                free_world = np.stack([wx, wy], axis=1)
                
                original_l_free = C.L_FREE
                C.L_FREE = L_FREE_LIVE
                self.grid.integrate_free(pose, free_world)
                C.L_FREE = original_l_free

        # 2. Depth-camera aux-obstacle layer (catches floating walls)
        if self.clear.ok:
            d = self.clear._depth_image()
            if d is not None:
                x_r, y_r, z_r = self.clear._robot_frame(d)
                valid = (np.isfinite(d) & (d > self.clear.depth_min)
                         & (d < C.AUX_OBSTACLE_MAX_RANGE)
                         & (z_r > C.AUX_OBSTACLE_Z_MIN)
                         & (z_r < C.AUX_OBSTACLE_Z_MAX))
                if valid.any():
                    rx, ry, rth = pose
                    cs, sn = math.cos(rth), math.sin(rth)
                    xr = x_r[valid]
                    yr = y_r[valid]
                    wx = rx + cs * xr - sn * yr
                    wy = ry + sn * xr + cs * yr
                    self.grid.mark_aux_obstacle_points(
                        np.stack([wx, wy], axis=1))

        # 3. Carve free under the robot
        rx, ry, _ = pose
        self.grid.mark_free_disc(rx, ry, C.ROBOT_CHASSIS_HALF_LENGTH * 0.85)

        # 4. Periodic stale-aux cleanup
        if self.tick % 32 == 0 and hasattr(self.grid, "clear_aux_where_free"):
            self.grid.clear_aux_where_free()

    # ─── Perception ──────────────────────────────────────────────────
    def _smooth_pillar(self, name, pdet, pose):
        """Running-mean world position for detected pillar."""
        rng = pdet.get("range", float("nan"))
        bearing = pdet["bearing"]
        if not np.isfinite(rng) or rng <= 0.0 or rng > C.PILLAR_MAX_DETECT_RANGE:
            return
        # Height validation
        est_h = pdet.get("est_height", float("nan"))
        if np.isfinite(est_h):
            h_min = C.PILLAR_HEIGHT * C.PILLAR_HEIGHT_MIN_FRAC
            h_max = C.PILLAR_HEIGHT * C.PILLAR_HEIGHT_MAX_FRAC
            if est_h < h_min or est_h > h_max:
                return
        # Aspect ratio validation
        aspect = pdet.get("aspect", float("nan"))
        if np.isfinite(aspect) and aspect > C.PILLAR_ASPECT_MAX:
            return

        rx, ry, rth = pose
        gx = rx + rng * math.cos(rth + bearing)
        gy = ry + rng * math.sin(rth + bearing)

        # Outlier rejection
        buf = self._pillar_obs[name]
        if len(buf) >= 2:
            xs = np.array([p[0] for p in buf])
            ys = np.array([p[1] for p in buf])
            mx, my = float(xs.mean()), float(ys.mean())
            if math.hypot(gx - mx, gy - my) > C.PILLAR_OUTLIER_REJECT_M:
                return

        buf.append((gx, gy))
        if len(buf) > C.PILLAR_OBS_AVG_N:
            buf.pop(0)
        if len(buf) >= C.PILLAR_MIN_OBS_FOR_CONFIRM:
            xs = np.array([p[0] for p in buf])
            ys = np.array([p[1] for p in buf])
            self.pillar_world_pos[name] = (float(xs.mean()), float(ys.mean()))
            if not self._announced[name]:
                wx, wy = self.pillar_world_pos[name]
                print(f"[perc] {name.upper()} pillar confirmed @ "
                      f"({wx:+.2f}, {wy:+.2f}) after {len(buf)} obs",
                      flush=True)
                self._announced[name] = True

    def _run_perception(self, pose):
        """Run pillar + poison detection."""
        if self.perc is None:
            return
        dets = self.perc.detect()
        if dets.get("blue") is not None:
            self._smooth_pillar("blue", dets["blue"], pose)
        if dets.get("yellow") is not None:
            self._smooth_pillar("yellow", dets["yellow"], pose)
        added = self.perc.project_green_floor_to_grid(self.grid, pose)
        if added > 0 and not self._poison_announced:
            self._poison_announced = True
            print("[perc] GREEN poison patch — first projection accepted",
                  flush=True)

    # ─── 3D cloud accumulation ────────────────────────────────────────
    def _maybe_accumulate_cloud(self, pose):
        if not self.clear.ok:
            return
        if self.sim_time - self._last_cloud_t < CLOUD_PERIOD_S:
            return
        pts = self.clear.depth_to_world_cloud(pose)
        if pts.shape[0] > 0:
            self.cloud_chunks.append(pts)
        self._last_cloud_t = self.sim_time

    # ─── PNG snapshots ────────────────────────────────────────────────
    def _maybe_save_png(self, pose):
        if (self.sim_time - self._last_png_t) < PNG_PERIOD_S:
            return
        self._last_png_t = self.sim_time
        stem = f"runner_{int(self.sim_time):04d}.png"
        text = (f"MAP_RUNNER  state={self.state}  "
                f"t={self.sim_time:6.2f}s\n"
                f"pose=({pose[0]:+.2f}, {pose[1]:+.2f}, {pose[2]:+.2f})\n"
                f"icp_corrections={self.icp.corrections_applied}  "
                f"target={self.cur_target_color or '—'}")
        try:
            self.grid.save_png(
                os.path.join(self.maps_dir, stem),
                pose=pose,
                path_world=getattr(self.driver, 'path', None) or self.last_path_world,
                standoff_world=self.cur_standoff,
                state_text=text,
            )
        except Exception as e:
            print(f"[map] live PNG failed: {e}", flush=True)

    # ─── Planning ────────────────────────────────────────────────────
    def _plan_leg(self, color):
        target = self.pillar_world_pos.get(color)
        if target is None:
            return None, None
        rx, ry, _ = self._get_pose()
        path, idx, standoff, _ = plan_to_pillar(
            self.grid, self._get_pose(), target,
            standoff=C.PILLAR_STANDOFF,
            keep_arc_to=(rx, ry),
        )
        return path, standoff

    def _start_leg(self, color):
        self.cur_target_color = color
        path, standoff = self._plan_leg(color)
        if path is None:
            print(f"[mission] FAILED to plan to {color} pillar at "
                  f"{self.pillar_world_pos.get(color)}", flush=True)
            return False
        self.driver.set_path(path, final_tol=C.WAYPOINT_REACH_TOL)
        self.cur_standoff = standoff
        self.last_path_world = path
        self.t_last_plan = self.sim_time
        plen = sum(math.hypot(b[0] - a[0], b[1] - a[1])
                   for a, b in zip(path[:-1], path[1:]))
        if color == "blue":
            self.path_length_blue_m = plen
        else:
            self.path_length_yellow_m = plen
        print(f"[mission] {color.upper()} leg planned: standoff="
              f"({standoff[0]:+.2f}, {standoff[1]:+.2f}) "
              f"len={len(path)} pts ≈ {plen:.2f} m", flush=True)
        return True

    def _path_invalid(self):
        if not self.driver.path:
            return True
        try:
            cost, hard = self.grid.costmap()
        except Exception:
            return False
        cells = hard.shape[0]
        for wx, wy in self.driver.path:
            ix, iy = self.grid.w2i(wx, wy)
            if not (0 <= ix < cells and 0 <= iy < cells):
                return True
            if hard[ix, iy]:
                return True
        return False

    def _pillar_reached(self, color):
        target = self.pillar_world_pos.get(color)
        if target is None:
            return False
        rx, ry, _ = self._get_pose()
        return math.hypot(rx - target[0], ry - target[1]) <= MISSION_REACH_DIST_M

    def _aux_clear(self):
        return self.clear.min_forward_at_chassis_height()

    def _dist_to_target(self):
        """Euclidean distance to the active pillar (or inf)."""
        if self.cur_target_color is None:
            return float("inf")
        target = self.pillar_world_pos.get(self.cur_target_color)
        if target is None:
            return float("inf")
        rx, ry, _ = self._get_pose()
        return math.hypot(rx - target[0], ry - target[1])

    def _remaining_path_length(self):
        """Remaining path distance from current pose."""
        if self.driver.path and hasattr(self.driver, 'remaining_distance'):
            return self.driver.remaining_distance(self._get_pose())
        return 0.0

    # ─── FSM state handlers ──────────────────────────────────────────
    def _state_init_scan(self):
        """Rotate in place to build initial 360° map."""
        pose = self._get_pose()
        if self._init_scan_start_th is None:
            self._init_scan_start_th = pose[2]
            self._init_scan_total = 0.0

        # Spin in place
        self.io.set_cmd(0.0, INIT_SCAN_W)
        self.v_cmd, self.w_cmd = 0.0, INIT_SCAN_W

        # Track total rotation
        dth = abs(pose[2] - self._init_scan_start_th)
        if dth > math.pi:
            dth = 2 * math.pi - dth
        # Use time-based estimate (more robust than angle tracking with wrapping)
        total_angle = INIT_SCAN_W * (self.sim_time - self.t_state)

        if total_angle >= INIT_SCAN_REVS * 2 * math.pi:
            self.io.stop()
            print(f"[init] initial scan complete: "
                  f"{total_angle / (2 * math.pi):.1f} revolutions, "
                  f"icp_corrections={self.icp.corrections_applied}",
                  flush=True)
            self._transition(State.EXPLORE)
            self.t_explore_start = self.sim_time

    def _state_explore(self):
        """Frontier-based autonomous exploration."""
        pose = self._get_pose()

        # Check if both pillars found → transition to mission
        both_found = (self.pillar_world_pos["blue"] is not None
                      and self.pillar_world_pos["yellow"] is not None)
        if both_found:
            print("[explore] Both pillars found — starting mission!", flush=True)
            self.io.stop()
            self.t_mission_start = self.sim_time
            self.stuck.reset(pose, self.sim_time)
            if self._start_leg("blue"):
                self._transition(State.GO_BLUE)
            else:
                self._transition(State.FAILED)
            return

        # Timeout check
        if (self.sim_time - self.t_explore_start) > EXPLORE_TIMEOUT_S:
            print("[explore] TIMEOUT — exploration exceeded limit", flush=True)
            self._transition(State.FAILED)
            return

        # Frontier-based exploration: find a frontier and drive toward it
        if (self.sim_time - self._explore_last_plan_t) > EXPLORE_REPLAN_S:
            frontier = self.explorer.find_frontier(pose)
            if frontier is not None:
                self._explore_frontier = frontier
                # Plan a path to the frontier
                from planning import plan_to_world
                path_w, path_idx, _ = plan_to_world(
                    self.grid, pose, frontier)
                if path_w is not None:
                    self.driver.set_path(path_w, final_tol=0.30)
                    self._explore_path = path_w
                    self._explore_last_plan_t = self.sim_time
                    self._frontiers_exhausted_t = None
                else:
                    # Planning failed — mark time and try reactive
                    self._explore_frontier = None
            else:
                # No frontiers found
                if self._frontiers_exhausted_t is None:
                    self._frontiers_exhausted_t = self.sim_time
                    print("[explore] frontiers exhausted — "
                          f"grace period {PILLAR_GRACE_AFTER_EXPLORE_S}s",
                          flush=True)
                elif (self.sim_time - self._frontiers_exhausted_t
                      > PILLAR_GRACE_AFTER_EXPLORE_S):
                    missing = [c for c in ("blue", "yellow")
                               if self.pillar_world_pos.get(c) is None]
                    print(f"[explore] grace expired, still missing: {missing}",
                          flush=True)
                    self._transition(State.FAILED)
                    return
                self._explore_last_plan_t = self.sim_time

        # If we have a path from the frontier planner, use DWA to follow it
        if self.driver.has_path():
            ranges = np.asarray(
                self.scan.lidar.getRangeImage(), dtype=np.float32) if self.scan else np.zeros(0)
            angles = self.scan._angles if self.scan else np.zeros(0)
            v, w, status = self.driver.step(
                pose, ranges, angles,
                aux_clear=self._aux_clear(),
            )
            if status == "arrived":
                self.driver.set_path([])
                self._explore_last_plan_t = -math.inf  # replan immediately
                v, w = 0.0, 0.0
            elif status == "blocked":
                v, w = 0.0, 0.8  # turn to find a way around
            self.io.set_cmd(v, w)
            self.driver.set_current_velocity(v, w)
            self.v_cmd, self.w_cmd = v, w
        else:
            # No valid frontier path — use reactive exploration
            self._simple_explore(pose)

    def _simple_explore(self, pose):
        """Simple reactive exploration: drive forward, turn at obstacles."""
        if self.scan is None:
            self.io.set_cmd(0.15, 0.0)
            self.v_cmd, self.w_cmd = 0.15, 0.0
            return

        ranges = np.asarray(self.scan.lidar.getRangeImage(), dtype=np.float32)
        if ranges.size != self.scan.n:
            self.io.set_cmd(0.15, 0.0)
            self.v_cmd, self.w_cmd = 0.15, 0.0
            return

        angles = self.scan._angles
        valid = np.isfinite(ranges) & (ranges > self.scan.r_min) & (ranges < self.scan.r_max)

        # Check forward clearance (within ±30°)
        fwd_mask = valid & (np.abs(angles) < math.radians(30))
        fwd_min = float(np.min(ranges[fwd_mask])) if fwd_mask.any() else float("inf")

        # Check left and right clearance
        left_mask = valid & (angles > math.radians(30)) & (angles < math.radians(90))
        right_mask = valid & (angles < math.radians(-30)) & (angles > math.radians(-90))
        left_min = float(np.min(ranges[left_mask])) if left_mask.any() else float("inf")
        right_min = float(np.min(ranges[right_mask])) if right_mask.any() else float("inf")

        # Reactive driving
        if fwd_min < 0.35:
            # Too close to wall — turn toward the side with more space
            v = 0.0
            w = 1.2 if left_min > right_min else -1.2
        elif fwd_min < 0.60:
            # Approaching wall — slow down and steer
            v = 0.10
            w = 0.6 if left_min > right_min else -0.6
        else:
            # Open space — drive forward with slight wall-following tendency
            v = 0.25
            # Bias toward the side with more unexplored space
            w = 0.0
            if left_min < 0.40 and right_min > 0.40:
                w = -0.3  # wall on left, steer slightly right
            elif right_min < 0.40 and left_min > 0.40:
                w = 0.3   # wall on right, steer slightly left

        self.io.set_cmd(v, w)
        self.v_cmd, self.w_cmd = v, w

    def _state_go_pillar(self, color, ranges_data):
        """Drive toward pillar using DWA planner."""
        # Arrival check
        if self._pillar_reached(color):
            self.io.stop()
            now = self.sim_time
            if color == "blue":
                self.t_blue_reached = now
                leg = now - self.t_mission_start
                print(f"[mission] BLUE pillar REACHED at "
                      f"t={now:.2f}s (leg = {leg:.2f}s)", flush=True)
                self.t_at_pillar_start = now
                self.driver.set_path([])
                self._transition(State.AT_BLUE)
            else:
                self.t_yellow_reached = now
                blue_t = self.t_blue_reached or self.t_mission_start
                leg = now - blue_t
                total = now - self.t_mission_start
                print(f"[mission] YELLOW pillar REACHED at "
                      f"t={now:.2f}s (BLUE→YELLOW = {leg:.2f}s, "
                      f"total mission = {total:.2f}s)", flush=True)
                self.t_at_pillar_start = now
                self.driver.set_path([])
                self._transition(State.AT_YELLOW)
            return

        # Replan if stale or invalid
        if (not self.driver.has_path()
                or self._path_invalid()
                or (self.sim_time - self.t_last_plan)
                    > MISSION_REPLAN_PERIOD_S):
            self._start_leg(color)

        # Drive
        v, w, status = self.driver.step(
            self._get_pose(), ranges_data["ranges"], ranges_data["angles"],
            aux_clear=self._aux_clear(),
        )
        if status == "blocked":
            self.io.set_cmd(0.0, w)
            self.driver.set_current_velocity(0.0, w)
            self.v_cmd, self.w_cmd = 0.0, w
            self.stuck.update(self._get_pose(), w, self.sim_time)
            if (self.stuck.stuck(self.sim_time, timeout_s=C.STUCK_TIMEOUT_S)
                    or self.stuck.oscillating()):
                self._enter_recovery(f"stuck-going-to-{color}")
            return
        if status == "arrived":
            self.driver.set_path([])
            self.t_last_plan = -math.inf
            return
        self.io.set_cmd(v, w)
        self.driver.set_current_velocity(v, w)
        self.v_cmd, self.w_cmd = v, w
        self.stuck.update(self._get_pose(), w, self.sim_time)

        # Progress-based stuck check
        rem = self._remaining_path_length()
        if not hasattr(self, '_rem_dist_ref'):
            self._rem_dist_ref = rem
            self._rem_dist_t = self.sim_time
        if rem < self._rem_dist_ref - C.STUCK_PROGRESS_MIN_M:
            self._rem_dist_ref = rem
            self._rem_dist_t = self.sim_time
        no_progress = (self.sim_time - self._rem_dist_t) > C.STUCK_TIMEOUT_S

        if (self.stuck.stuck(self.sim_time, timeout_s=C.STUCK_TIMEOUT_S)
                or self.stuck.oscillating()
                or no_progress):
            if no_progress:
                print(f"[stuck] no goal-progress for "
                      f"{self.sim_time - self._rem_dist_t:.1f}s "
                      f"(rem={rem:.2f}m)", flush=True)
            self._rem_dist_ref = rem
            self._rem_dist_t = self.sim_time
            self._enter_recovery(f"stuck-going-to-{color}")

    def _state_at_pillar(self, next_color):
        """Dwell at pillar, then start next leg."""
        self.io.stop()
        self.v_cmd, self.w_cmd = 0.0, 0.0
        if self.sim_time - self.t_at_pillar_start < MISSION_DWELL_AT_PILLAR_S:
            return
        if next_color is None:
            self._transition(State.DONE)
            return
        self.stuck.reset(self._get_pose(), self.sim_time)
        if hasattr(self, '_rem_dist_ref'):
            del self._rem_dist_ref
        if self._start_leg(next_color):
            self._transition(State.GO_YELLOW if next_color == "yellow"
                              else State.GO_BLUE)
        else:
            self._transition(State.FAILED)

    def _state_recovery(self):
        """Run recovery maneuver (reverse + spin)."""
        v, w, done = self.recovery.step(self.sim_time)
        self.io.set_cmd(v, w)
        self.v_cmd, self.w_cmd = v, w
        if done:
            print("[recovery] END", flush=True)
            self.stuck.reset(self._get_pose(), self.sim_time)
            self.t_last_plan = -math.inf
            if hasattr(self, '_rem_dist_ref'):
                del self._rem_dist_ref
            if self._prev_state in (State.GO_BLUE, State.GO_YELLOW):
                self._transition(self._prev_state)
            elif self._prev_state == State.EXPLORE:
                self._transition(State.EXPLORE)
            else:
                self._transition(State.EXPLORE)

    def _enter_recovery(self, reason):
        self.recovery_chain += 1
        if self.recovery_chain >= C.RECOVERY_MAX_CHAIN:
            print(f"[mission] recovery chain exhausted ({reason}) — "
                  f"abandoning", flush=True)
            self._transition(State.FAILED)
            return
        print(f"[recovery] BEGIN ({reason}) "
              f"[total: {self.recovery_chain}]", flush=True)
        self.recovery.begin(self.sim_time)
        self.driver.set_path([])
        self.stuck.reset(self._get_pose(), self.sim_time)
        self._prev_state = self.state
        self._transition(State.RECOVERY)

    # ─── Tick logging ────────────────────────────────────────────────
    def _log_tick(self):
        """Write diagnostic data for this tick."""
        if self.tick % LOG_EVERY_TICKS != 0:
            return
        pose = self._get_pose()
        odom_raw = self.odom.pose()
        self.logger.log_tick(
            t=self.sim_time,
            tick=self.tick,
            state=self.state,
            pose=pose,
            odom_raw=odom_raw,
            icp_applied=False,  # per-tick flag (not aggregate)
            icp_error=0.0,
            v_cmd=self.v_cmd,
            w_cmd=self.w_cmd,
            target_pillar=self.cur_target_color,
            dist_to_target=self._dist_to_target(),
            path_length=self._remaining_path_length(),
            stuck=self.stuck.stuck(self.sim_time, timeout_s=C.STUCK_TIMEOUT_S)
                  if hasattr(self.stuck, '_ref') and self.stuck._ref is not None
                  else False,
            recovery_count=self.recovery_chain,
            extra={
                "icp_total": self.icp.corrections_applied,
                "blue": self.pillar_world_pos.get("blue") is not None,
                "yellow": self.pillar_world_pos.get("yellow") is not None,
            },
        )

    # ─── Finalize ────────────────────────────────────────────────────
    def _finalize(self):
        """Write all outputs."""
        print("[mission] finalising...", flush=True)
        self.io.stop()
        pose = self._get_pose()

        # 1. Final PNG
        try:
            export_final_png(
                os.path.join(self.maps_dir, "final_map.png"),
                self.grid,
                pose_history=self.pose_history,
                pillar_positions={
                    "blue": self.pillar_world_pos.get("blue"),
                    "yellow": self.pillar_world_pos.get("yellow"),
                },
                show_aux=True,
                show_poison=True,
                title="Map Runner — Final occupancy + trajectory",
            )
        except Exception as e:
            print(f"[final] final_map.png failed: {e}", flush=True)

        # 2. 3D point cloud
        try:
            if self.cloud_chunks:
                all_pts = np.concatenate(self.cloud_chunks, axis=0)
                export_ply(
                    os.path.join(self.maps_dir, "scene.ply"),
                    all_pts, voxel=0.04,
                )
            else:
                print("[final] no 3D points captured", flush=True)
        except Exception as e:
            print(f"[final] scene.ply failed: {e}", flush=True)

        # 3. Binary map
        try:
            occ = self.grid.occupied_mask().astype(bool)
            free = self.grid.free_mask().astype(bool)
            unk = self.grid.unknown_mask().astype(bool)
            poison = self.grid.poison.astype(bool)
            aux = self.grid.aux_obstacle.astype(bool)

            def _pos_or_nan(p):
                return (np.array([p[0], p[1]], dtype=np.float32)
                        if p is not None
                        else np.array([np.nan, np.nan], dtype=np.float32))

            start_pose = (np.array(self.pose_history[0], dtype=np.float32)
                          if self.pose_history
                          else np.array([np.nan, np.nan, np.nan],
                                        dtype=np.float32))

            np.savez_compressed(
                os.path.join(self.maps_dir, "map.npz"),
                occupied=occ, free=free, unknown=unk,
                aux_obstacle=aux, poison=poison,
                resolution=np.float32(self.grid.res),
                origin=np.array(self.grid.origin, dtype=np.float32),
                cells=np.int32(self.grid.cells),
                pose_history=np.array(self.pose_history, dtype=np.float32),
                start_pose=start_pose,
                pillar_blue=_pos_or_nan(self.pillar_world_pos.get("blue")),
                pillar_yellow=_pos_or_nan(self.pillar_world_pos.get("yellow")),
            )
            print(f"[final] wrote map.npz "
                  f"(occ={int(occ.sum())} free={int(free.sum())} "
                  f"unk={int(unk.sum())} aux={int(aux.sum())} "
                  f"poison={int(poison.sum())})", flush=True)
        except Exception as e:
            print(f"[final] map.npz failed: {e}", flush=True)

        # 4. ICP drift stats
        stats = self.icp.drift_stats()
        print(f"[icp] drift stats: {stats}", flush=True)

        # 5. Mission summary log
        self.logger.write_summary(
            final_state=self.state,
            t_end=self.sim_time,
            t_blue_reached=self.t_blue_reached,
            t_yellow_reached=self.t_yellow_reached,
            t_mission_start=self.t_mission_start,
            path_length_blue=self.path_length_blue_m,
            path_length_yellow=self.path_length_yellow_m,
            pillar_blue=self.pillar_world_pos.get("blue"),
            pillar_yellow=self.pillar_world_pos.get("yellow"),
            icp_corrections_total=self.icp.corrections_applied,
            poison_cells=int(self.grid.poison.sum()),
        )
        self.logger.close()

    # ─── Main loop ───────────────────────────────────────────────────
    def run(self):
        try:
            while self.io.step() != -1:
                self.sim_time += self.dt
                self.tick += 1

                # ── Sensor pipeline (runs in EVERY state) ──
                self._update_pose()

                # Always update the corrected pose from odometry first
                self._corrected_pose = self.odom.pose()

                # Run ICP correction
                self._run_icp()

                pose = self._get_pose()

                if self.tick % 5 == 0:
                    self.pose_history.append(pose)

                self._update_grid(pose)
                self._run_perception(pose)
                self._maybe_accumulate_cloud(pose)

                # ── LiDAR data for DWA ──
                ranges_arr = np.zeros(0)
                angles_arr = np.zeros(0)
                if self.scan is not None:
                    ranges_arr = np.asarray(
                        self.scan.lidar.getRangeImage(), dtype=np.float32)
                    angles_arr = self.scan._angles
                ranges_data = {"ranges": ranges_arr, "angles": angles_arr}

                # ── FSM dispatch ──
                if self.state == State.INIT_SCAN:
                    self._state_init_scan()
                elif self.state == State.EXPLORE:
                    self._state_explore()
                elif self.state == State.GO_BLUE:
                    self._state_go_pillar("blue", ranges_data)
                elif self.state == State.AT_BLUE:
                    self._state_at_pillar("yellow")
                elif self.state == State.GO_YELLOW:
                    self._state_go_pillar("yellow", ranges_data)
                elif self.state == State.AT_YELLOW:
                    self._state_at_pillar(None)
                elif self.state == State.RECOVERY:
                    self._state_recovery()
                elif self.state == State.DONE:
                    self.io.stop()
                    self.v_cmd, self.w_cmd = 0.0, 0.0
                elif self.state == State.FAILED:
                    self.io.stop()
                    self.v_cmd, self.w_cmd = 0.0, 0.0

                # ── Outputs ──
                self._maybe_save_png(pose)
                self._log_tick()

                # ── Periodic console status ──
                if self.tick % 64 == 0:
                    blue_tag = (
                        f"({self.pillar_world_pos['blue'][0]:+.2f},"
                        f"{self.pillar_world_pos['blue'][1]:+.2f})"
                        if self.pillar_world_pos.get("blue") else "?"
                    )
                    yellow_tag = (
                        f"({self.pillar_world_pos['yellow'][0]:+.2f},"
                        f"{self.pillar_world_pos['yellow'][1]:+.2f})"
                        if self.pillar_world_pos.get("yellow") else "?"
                    )
                    print(
                        f"[t={self.sim_time:6.2f}s] {self.state:12s} "
                        f"pose=({pose[0]:+.2f},{pose[1]:+.2f},{pose[2]:+.2f}) "
                        f"icp={self.icp.corrections_applied} "
                        f"B={blue_tag} Y={yellow_tag}",
                        flush=True,
                    )

                # Flush log periodically
                if self.tick % 128 == 0:
                    self.logger.flush()

        except KeyboardInterrupt:
            print("[runner] interrupted", flush=True)
        finally:
            self._finalize()


def main():
    runner = MapRunner()
    runner.run()


if __name__ == "__main__":
    main()
