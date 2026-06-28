r"""mak_02_controller — SLAM + frontier-exploration mission controller.

A from-scratch Python port of the gmapping-style frontier-exploration approach
(github.com/HanwenCao/Frontier_Exploration) adapted to the Webots ROSbot maze:

* **Phase A — SLAM.**  Wheel+IMU odometry *predicts* each step; a correlative
  scan matcher *corrects* it against the map's likelihood field before any scan
  is integrated.  This is what keeps walls thin (the old controller drifted).
* **Phase B — clean grid.**  Log-odds occupancy with a hit-count-gated, decaying
  depth-camera layer for floating walls, plus a per-pixel green-poison layer.
* **Phase C — exploration.**  Frontier detection + utility goal selection +
  A* + pure-pursuit, wrapped in a mission FSM that visits BLUE then YELLOW.

Mission FSM::

    INIT_SCAN -> EXPLORE_BLUE -> GO_BLUE -> EXPLORE_YELLOW -> GO_YELLOW -> DONE
                      \__________ RECOVERY __________/   (on stuck)

Run it by setting the Rosbot's ``controller`` field to ``mak_02_controller`` in
the Webots world.  Press ``Q`` to finalise and write outputs early.
"""
from __future__ import annotations

import math
import os
import sys

# Make sibling packages importable when Webots launches this file directly.
_HERE = os.path.dirname(os.path.abspath(__file__))
if _HERE not in sys.path:
    sys.path.insert(0, _HERE)

import numpy as np

import settings as S
from geometry import (relative_pose, compose_pose, wrap_angle, transform_points,
                      inverse_transform_points, pose_distance)
from hardware.robot_io import RobotIO
from slam.odometry import Odometry
from slam.occupancy_grid import OccupancyGrid
from slam.likelihood_field import LikelihoodField
from slam.scan_matcher import ScanMatcher
from mapping.lidar_model import LidarModel
from mapping.depth_model import DepthModel
from mapping.cloud_map import CloudMap
from perception.color_detector import ColorDetector
from perception.pillar_tracker import PillarTracker
from exploration.frontier import find_frontiers
from exploration.goal_selection import select_goal
from exploration.astar import Planner
from control.pure_pursuit import follow
from control.local_planner import DWAPlanner, widest_gap_bearing
import viz


# states
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

        self.odom = Odometry()
        self.grid = OccupancyGrid()
        self.field = LikelihoodField.from_grid(self.grid)
        self.matcher = ScanMatcher()
        self.planner = Planner(self.grid)
        self.dwa = DWAPlanner()
        self.pillars = PillarTracker()
        self.scan_body = np.empty((0, 2))   # live lidar in robot frame, per tick
        self._last_aux_world = np.empty((0, 2))   # live depth floating-wall pts
        self._last_aux_t = -1e9

        # 3D voxel cloud map: continuous depth accumulation -> 3D render + an
        # accurate 2D floating-wall footprint projected into grid.cloud_obs.
        self.cloud = CloudMap() if S.CLOUD_ENABLED else None
        self._last_cloud_t = -1e9
        self._last_cloud_proj_t = -1e9
        self._blue_3d_done = False

        specs = self.io.lidar_specs()
        self.lidar_model = LidarModel(*specs) if specs else None
        d = self.io.depth_specs()
        # depth_specs -> (w, h, fov, dmin, dmax)
        self.depth_model = (DepthModel(d[0], d[1], d[2], S.CAMERA_MOUNT_Z, d[3], d[4])
                            if d else None)
        c = self.io.camera_specs()
        self.color = ColorDetector(c[0], c[1], c[2], S.CAMERA_MOUNT_Z) if c else None

        print(f"[mak02] lidar specs (n,fov,rmin,rmax) = {specs}", flush=True)
        print(f"[mak02] camera specs (w,h,fov)        = {c}", flush=True)
        print(f"[mak02] depth specs (w,h,fov,min,max) = {d}", flush=True)

        # SLAM pose state
        self.corrected = (0.0, 0.0, 0.0)
        self.raw_prev = None
        self.predicted = (0.0, 0.0, 0.0)
        self.last_sm_pose = (0.0, 0.0, 0.0)
        self.sm_updates = 0
        self._last_sm_norm = 0.0       # diagnostic: scan-match score (0..~1)
        self._last_sm_ok = False       # diagnostic: was the last match accepted

        # mission state
        self.state = INIT_SCAN
        self.start_pose = None
        self.scan_rot = 0.0
        self.expl_goal = None
        self.plan = None
        self.plan_goal = None
        self.last_plan_t = -1e9
        self.blacklist = []

        # timing
        self.sim_time = 0.0
        self.tick = 0
        self.t_blue = None
        self.t_yellow = None
        self._done_at = None

        # recovery / stuck
        self.return_state = EXPLORE_BLUE
        self._go_fail_until = {"blue": 0.0, "yellow": 0.0}  # cooldown after a failed GO
        self._go_attempts = {"blue": 0, "yellow": 0}        # dead-end fail counter
        self._go_entry_dist = {"blue": None, "yellow": None}  # dist at GO episode start
        self.recovery_phase = None
        self.recovery_until = 0.0
        self.recovery_chain = 0
        self._recovery_align_target = 0.0   # absolute heading recovery rotates to
        self._anchor_pose = (0.0, 0.0, 0.0)
        self._anchor_time = 0.0
        self._bump_anchor = None        # (pose, time) for the virtual bumper

        # io / history
        self.pose_history = []
        self._last_snap_t = -1e9
        self.maps_dir = os.environ.get(
            "MAZE_MAPPING_OUTPUT_DIR", os.path.join(_HERE, S.OUTPUT_DIRNAME))
        os.makedirs(self.maps_dir, exist_ok=True)

        # live exploration window (OpenCV/matplotlib; degrades to off)
        self.live = None
        try:
            from live_view import LiveView
            self.live = LiveView(self.grid)
        except Exception as e:
            print("[live] init failed:", e)

        self._banner()

    # ----------------------------------------------------------- startup
    def _banner(self):
        print("=" * 64, flush=True)
        print("mak_02_controller — SLAM + frontier exploration", flush=True)
        print("FSM: INIT_SCAN -> EXPLORE_BLUE -> GO_BLUE -> "
              "EXPLORE_YELLOW -> GO_YELLOW -> DONE", flush=True)
        print(f"outputs -> {self.maps_dir}", flush=True)
        print("=" * 64, flush=True)

    # -------------------------------------------------------- odom + slam
    def _odom_and_predict(self):
        wl, wr = self.io.read_encoders()
        yaw = self.io.read_yaw()
        if wl is not None:
            self.odom.update(wl, wr, yaw, self.dt)
        raw_now = self.odom.pose()
        if self.raw_prev is None:
            self.raw_prev = raw_now
            rel = (0.0, 0.0, 0.0)
            # seed corrected heading from the IMU
            self.corrected = (0.0, 0.0, raw_now[2])
        else:
            rel = relative_pose(self.raw_prev, raw_now)
            self.raw_prev = raw_now
        self.predicted = compose_pose(self.corrected, rel)
        self.scan_rot += abs(rel[2])

    def _slam_update_due(self):
        d = pose_distance(self.predicted, self.last_sm_pose)
        a = abs(wrap_angle(self.predicted[2] - self.last_sm_pose[2]))
        return (self.sm_updates == 0) or (d > S.SM_UPDATE_LIN_M) or (a > S.SM_UPDATE_ANG_RAD)

    def _read_scan(self):
        """Read the lidar once per tick into the shared robot-frame buffer."""
        if self.lidar_model is None:
            self.scan_body = np.empty((0, 2))
            return
        ranges = self.io.read_lidar_ranges()
        if ranges is None:
            self.scan_body = np.empty((0, 2))
            return
        body, _ = self.lidar_model.ranges_to_body(ranges)
        self.scan_body = body

    def _slam_step(self):
        """Correct the predicted pose against the map, then integrate the scan."""
        if not self._slam_update_due():
            self.corrected = self.predicted        # dead-reckon between updates
            return
        body = self.scan_body
        if body.shape[0] < 10:
            self.corrected = self.predicted
            return

        if self.field.n_occupied >= 10:
            matched, info = self.matcher.match(self.predicted, body, self.field)
            self.corrected = matched
            self._last_sm_norm = info["norm"]
            self._last_sm_ok = info["accepted"]
        else:
            self.corrected = self.predicted        # bootstrap: nothing to match to

        world = transform_points(body, *self.corrected)
        self.grid.integrate_scan(self.corrected, world)
        self._integrate_depth()

        self.sm_updates += 1
        self.last_sm_pose = self.corrected
        if self.sm_updates % S.FIELD_REBUILD_EVERY == 0:
            self.field.rebuild(self.grid)

    def _integrate_depth(self):
        """Fold the depth camera into its OWN obstacle layer (mark + raytrace
        clear).  The lidar never writes there, so floating walls the lidar misses
        persist, while the depth's own clearing erases stale/false marks."""
        if self.depth_model is None:
            return
        depth = self.io.read_depth()
        if depth is None:
            return
        hits, frees = self.depth_model.depth_rays_world(
            depth, self.corrected, S.DEPTH_CLEAR_RANGE)
        self.grid.integrate_depth_rays(self.corrected, hits, frees)
        # Live floating-wall obstacle points for the DWA (immediate avoidance).
        self._last_aux_world = hits
        self._last_aux_t = self.sim_time
        self._accumulate_cloud(depth)

    def _accumulate_cloud(self, depth):
        """Fold the depth frame into the 3D voxel cloud (continuous during
        exploration) and periodically re-project its collision band into the
        grid's accurate floating-wall layer (grid.cloud_obs)."""
        if self.cloud is None:
            return
        if self.sim_time - self._last_cloud_t >= S.CLOUD_ACCUM_PERIOD_S:
            pts = self.depth_model.cloud_world(depth, self.corrected)
            if pts.shape[0]:
                self.cloud.add_points_world(pts)
            self._last_cloud_t = self.sim_time
        # Only publish the projection into the planner when explicitly enabled —
        # the accumulating cloud has no clearing and otherwise smears the map shut.
        if (S.CLOUD_OBS_TO_PLANNER
                and self.sim_time - self._last_cloud_proj_t >= S.CLOUD_PROJECT_PERIOD_S):
            self._project_cloud_to_grid()
            self._last_cloud_proj_t = self.sim_time

    def _project_cloud_to_grid(self):
        """Project collision-band voxels to 2D, drop the cells the lidar already
        maps as a normal wall (this layer is for floating walls the lidar
        misses), and publish the result as grid.cloud_obs for the planner."""
        if self.cloud is None:
            return
        mask = self.cloud.project_to_2d(self.grid)
        if mask.any():
            occ = self.grid.occupied_mask()
            n = max(1, S.CLOUD_OBS_BLIND_CELLS)
            try:
                from scipy.ndimage import binary_dilation
                yy, xx = np.ogrid[-n:n + 1, -n:n + 1]
                near_wall = binary_dilation(occ, structure=(xx * xx + yy * yy) <= n * n)
            except Exception:
                near_wall = occ
            mask &= ~near_wall
        self.grid.set_cloud_obs(mask)

    # --------------------------------------------------------- perception
    def _perception_step(self):
        if self.color is None or self.tick % S.PERCEPTION_EVERY_TICKS != 0:
            return
        bgr = self.io.read_rgb_bgr()
        if bgr is None:
            return
        depth = self.io.read_depth()
        dets = self.color.detect_pillars(bgr, depth)
        x, y, th = self.corrected
        for name in ("blue", "yellow"):
            det = dets.get(name)
            if self.color.is_valid_pillar(det):
                rng = det["range"]
                br = det["bearing"]
                gx = x + rng * math.cos(th + br)
                gy = y + rng * math.sin(th + br)
                self.pillars.update(name, gx, gy)
                # Close-range, consistent views CONFIRM (enable "reached").
                self.pillars.observe(name, gx, gy, rng)
        # green poison floor projection
        pts = self.color.green_floor_points_world(bgr, self.corrected)
        if pts.shape[0]:
            self.grid.mark_poison_points(pts[:, 0], pts[:, 1])

    # ------------------------------------------------------------- mission
    def _fsm(self):
        # keep the robot's own cell plannable
        self.grid.mark_free_disc(self.corrected[0], self.corrected[1],
                                 S.ROBOT_RADIUS * 0.85)
        st = self.state
        if st == INIT_SCAN:
            return self._do_init_scan()
        if st == RECOVERY:
            return self._do_recovery()
        if st == EXPLORE_BLUE:
            # Mission success is being WITHIN reach of the pillar — register it in
            # EXPLORE too, not only in GO.  The robot was physically reaching blue
            # (0.08 m) while chasing a frontier just beyond it, then livelocking
            # on top of the pillar because EXPLORE never checked arrival.
            if self._pillar_in_reach("blue"):
                self._on_pillar_reached("blue")
                return 0.0, 0.0
            r = self._maybe_go("blue", GO_BLUE)
            return r if r is not None else self._do_explore(EXPLORE_BLUE)
        if st == GO_BLUE:
            return self._do_go("blue", GO_BLUE)
        if st == EXPLORE_YELLOW:
            if self._pillar_in_reach("yellow"):
                self._on_pillar_reached("yellow")
                return 0.0, 0.0
            r = self._maybe_go("yellow", GO_YELLOW)
            return r if r is not None else self._do_explore(EXPLORE_YELLOW)
        if st == GO_YELLOW:
            return self._do_go("yellow", GO_YELLOW)
        return 0.0, 0.0   # DONE

    def _pillar_in_reach(self, name):
        """True if the pillar is CONFIRMED (seen close & consistent) and the robot
        is within PILLAR_REACH_DIST of its (close-snapped) position.  Requiring
        confirmation is what stops the false arrival at a far through-a-gap
        estimate: the robot must actually drive up to the pillar first."""
        if not self.pillars.is_confirmed(name):
            return False
        p = self.pillars.pos.get(name)
        return p is not None and pose_distance(self.corrected, p) < S.PILLAR_REACH_DIST

    def _maybe_go(self, name, go_state):
        """From an EXPLORE state, switch to ``go_state`` only if the pillar is
        known AND a path to it currently exists (and we're past the post-failure
        cooldown).  This prevents the EXPLORE<->GO flip-flop when the pillar is
        known but walled off behind an unmapped/too-narrow passage — instead we
        keep exploring to open the route.  Returns a ``(v, w)`` command if we
        switched, else ``None``.
        """
        if not self.pillars.known(name):
            return None
        if self.sim_time < self._go_fail_until[name]:
            return None
        path, _ = self.planner.plan(self.corrected, self.pillars.pos[name])
        if not path:
            self._go_fail_until[name] = self.sim_time + S.GO_FAIL_COOLDOWN_S
            return None
        self._enter(go_state)
        self.plan = path                       # reuse the path we just found
        self.plan_goal = self.pillars.pos[name]
        self.last_plan_t = self.sim_time
        return self._do_go(name, go_state)

    def _enter(self, state):
        if self.state != state:
            print(f"[fsm] {self.state} -> {state}  (t={self.sim_time:.2f}s)", flush=True)
        self.state = state
        self.plan = None
        self.plan_goal = None
        self.expl_goal = None
        self._go_entry_dist = {"blue": None, "yellow": None}  # new GO episode
        self._reset_anchor()

    # --- INIT_SCAN ---
    def _do_init_scan(self):
        if self.scan_rot >= S.INITIAL_SCAN_REVS * 2.0 * math.pi:
            self.start_pose = self.corrected
            self._enter(EXPLORE_BLUE)
            return 0.0, 0.0
        return 0.0, S.INIT_SPIN_W

    # --- EXPLORE ---
    def _do_explore(self, ret_state):
        # need a (new) frontier goal?
        if self.expl_goal is None or self._reached(self.expl_goal):
            self.expl_goal = self._pick_frontier()
            self.plan = None
        if self.expl_goal is None:
            # No reachable frontier — spin to look around, but STILL run the
            # stuck-check so a wedged robot recovers instead of freezing here
            # forever (this branch previously skipped it -> the permanent freeze).
            self._stuck_check(ret_state)
            return 0.0, 0.6
        v, w, failed = self._navigate(self.expl_goal)
        if failed:
            self.blacklist.append(self.expl_goal)
            self.expl_goal = None
            self._stuck_check(ret_state)
            return 0.0, 0.6
        self._stuck_check(ret_state)
        return v, w

    def _pick_frontier(self):
        frontiers = find_frontiers(self.grid)
        if not frontiers:
            return None

        # Score frontiers by REAL A* path length, not straight-line distance:
        # this filters out frontiers walled off from the robot (the cause of the
        # dead-end thrashing) and ranks the reachable ones by true travel cost.
        def dist_fn(centroid):
            path, _ = self.planner.plan(self.corrected, centroid)
            if not path:
                return None                     # unreachable -> never selected
            return self.planner.path_length(path)

        best, _ = select_goal(frontiers, self.corrected,
                              self.start_pose or self.corrected,
                              blacklist=self.blacklist, dist_fn=dist_fn)
        return best.centroid_world if best is not None else None

    # --- GO_PILLAR ---
    def _do_go(self, name, ret_state):
        explore = EXPLORE_BLUE if name == "blue" else EXPLORE_YELLOW
        target = self.pillars.pos.get(name)
        if target is None:                   # lost the estimate; go back to explore
            self._enter(explore)
            return 0.0, 0.0
        if self._go_entry_dist[name] is None:    # start of a GO episode
            self._go_entry_dist[name] = pose_distance(self.corrected, target)
        if self._pillar_in_reach(name):          # confirmed AND within reach
            self._go_entry_dist[name] = None
            self._on_pillar_reached(name)
            return 0.0, 0.0
        v, w, failed = self._navigate(target)
        if failed:
            # No route right now — record the (failed) episode and explore to open
            # a route (or learn a dead-end). Avoids the instant GO<->EXPLORE bounce.
            self._register_go_failure(name, target)
            self._enter(explore)
            return 0.0, 0.6
        # Stuck mid-GO returns to EXPLORE (NOT GO): otherwise recovery loops
        # GO->RECOVERY->GO forever and the dead-end GO-suppression — which is only
        # checked on the EXPLORE->GO transition — never takes effect.
        self._stuck_check(explore)
        if self.state == RECOVERY:           # stuck mid-GO == this episode failed
            self._register_go_failure(name, target)
        return v, w

    def _register_go_failure(self, name, target):
        """Count a failed GO episode.  After GO_DEADEND_FAILS with little net
        approach, back off this approach (cooldown + blacklist the spot) so the
        robot EXPLORES for a route in from another side instead of ramming the
        same wall.  We no longer stamp a permanent barrier here — the depth
        obstacle layer already maps real floating walls into A*, and the big
        barriers were walling the robot off."""
        entry = self._go_entry_dist[name]
        self._go_entry_dist[name] = None
        cur = pose_distance(self.corrected, target)
        progress = (entry - cur) if entry is not None else 0.0
        if progress >= S.GO_DEADEND_PROGRESS_M:
            self._go_attempts[name] = 0          # genuinely closing in — not stuck
            self._go_fail_until[name] = self.sim_time + S.GO_FAIL_COOLDOWN_S
            return
        self._go_attempts[name] += 1
        if self._go_attempts[name] < S.GO_DEADEND_FAILS:
            self._go_fail_until[name] = self.sim_time + S.GO_FAIL_COOLDOWN_S
            return
        # Dead-end: stop hammering this approach for a while and explore elsewhere.
        self._go_attempts[name] = 0
        self._go_fail_until[name] = self.sim_time + S.GO_DEADEND_COOLDOWN_S
        self.blacklist.append((self.corrected[0], self.corrected[1]))
        print(f"[deadend] {name} not reachable from here — backing off & exploring "
              f"for another route (GO suppressed {S.GO_DEADEND_COOLDOWN_S:.0f}s)",
              flush=True)

    def _on_pillar_reached(self, name):
        if name == "blue":
            if self.t_blue is None:
                self.t_blue = self.sim_time
                print(f"[mission] BLUE reached at t={self.t_blue:.2f}s", flush=True)
                self._render_blue_3d()
            self._enter(EXPLORE_YELLOW)
        else:
            if self.t_yellow is None:
                self.t_yellow = self.sim_time
                print(f"[mission] YELLOW reached at t={self.t_yellow:.2f}s", flush=True)
            self._enter(DONE)

    # --- shared navigation ---
    def _navigate(self, goal):
        """Plan (throttled) to ``goal`` and pure-pursuit follow. -> (v, w, failed)."""
        need = (self.plan is None
                or self.plan_goal is None
                or pose_distance(self.plan_goal, goal) > S.WAYPOINT_REACH_TOL
                or (self.sim_time - self.last_plan_t) > S.PLAN_REPLAN_PERIOD_S)
        if need:
            path, _ = self.planner.plan(self.corrected, goal)
            self.plan = path
            self.plan_goal = goal
            self.last_plan_t = self.sim_time
        if not self.plan:
            return 0.0, 0.0, True

        # Pure pursuit chooses WHERE to aim (a carrot on the global path); the
        # DWA local planner decides HOW to get there safely using the live lidar
        # (obstacle rejection + corridor centring) — the move_base equivalent.
        cmd = follow(self.corrected, self.plan)
        if cmd["reached"]:
            return 0.0, 0.0, False
        carrot = cmd["target"] or self.plan[-1]
        carrot_body = inverse_transform_points(
            np.array([carrot], dtype=float), *self.corrected)[0]
        obstacles = self._local_obstacles_body()
        v, w, _ = self.dwa.compute((float(carrot_body[0]), float(carrot_body[1])),
                                   obstacles)
        return v, w, False

    def _ir_points_body(self, front_only=True):
        """IR/ToF readings as (M, 2) obstacle points in the robot frame.

        These fill the 2D lidar's <0.2 m blind zone (its min range), so the DWA
        stops clipping close corners.  ``front_only`` keeps just the forward
        pair (the rear pair is used by the rear-collision check instead).
        """
        ranges = self.io.read_ranges()
        pts = []
        for d, (sx, sy, sth) in zip(ranges, S.RANGE_SENSOR_POSES):
            if d is None:
                continue
            if front_only and abs(sth) > 1.0:      # rear sensors face ~pi
                continue
            pts.append((sx + d * math.cos(sth), sy + d * math.sin(sth)))
        return np.asarray(pts, dtype=float).reshape(-1, 2)

    def _live_floating_obstacles_body(self):
        """Live depth points that the LIDAR cannot see (floating walls), in the
        robot frame.  A depth point is kept only if no lidar-occupied cell sits
        within ``LIVE_FLOATING_BLIND_CELLS`` of it — i.e. the lidar is genuinely
        blind there.  This is what lets the robot avoid a floating wall it can see
        without the normal-wall depth double-counting that chokes passages.
        """
        aw = self._last_aux_world
        if aw.shape[0] == 0 or (self.sim_time - self._last_aux_t) >= 0.5:
            return np.empty((0, 2))
        rx, ry, _ = self.corrected
        keep = (aw[:, 0] - rx) ** 2 + (aw[:, 1] - ry) ** 2 < S.LP_MAPPED_OBS_RADIUS ** 2
        aw = aw[keep]
        if aw.shape[0] == 0:
            return np.empty((0, 2))
        ix, iy, m = self.grid.world_to_grid_arr(aw[:, 0], aw[:, 1])
        ix, iy, aw = ix[m], iy[m], aw[m]
        if aw.shape[0] == 0:
            return np.empty((0, 2))
        occ = self.grid.occupied_mask()
        n = S.LIVE_FLOATING_BLIND_CELLS
        blind = np.ones(aw.shape[0], dtype=bool)
        c = self.grid.cells
        for dx in range(-n, n + 1):
            for dy in range(-n, n + 1):
                jx = np.clip(ix + dx, 0, c - 1)
                jy = np.clip(iy + dy, 0, c - 1)
                blind &= ~occ[jx, jy]
        aw = aw[blind]
        if aw.shape[0] == 0:
            return np.empty((0, 2))
        return inverse_transform_points(aw, *self.corrected)

    def _local_obstacles_body(self):
        """Live lidar + forward IR + mapped aux/poison nearby, in the robot frame.

        The lidar can't see floating walls (it passes over/under them) or poison,
        so the DWA local planner must also respect the cells the SLAM grid has
        already learned are obstacles — otherwise it drives straight into them.
        Forward IR adds close-range hits the lidar is blind to (<0.2 m).
        """
        scan = self.scan_body
        ir = self._ir_points_body(front_only=True)
        if ir.shape[0]:
            scan = ir if scan.shape[0] == 0 else np.vstack([scan, ir])
        # Live depth thin-scan, but ONLY the lidar-BLIND points (genuine floating
        # walls).  Depth also sees the chassis band of NORMAL walls; feeding those
        # to the DWA double-counts walls the lidar already handles and chokes
        # narrow passages (the regression).  So we drop any depth point that has a
        # lidar-occupied cell nearby and keep only the truly invisible obstacles.
        fb = self._live_floating_obstacles_body()
        if fb.shape[0]:
            scan = fb if scan.shape[0] == 0 else np.vstack([scan, fb])
        mask = self.grid.depth_obs_mask() | self.grid.poison | self.grid.barrier
        if not mask.any():
            return scan
        ix, iy = np.where(mask)
        wx = self.grid.origin[0] + (ix + 0.5) * self.grid.res
        wy = self.grid.origin[1] + (iy + 0.5) * self.grid.res
        rx, ry, _ = self.corrected
        keep = (wx - rx) ** 2 + (wy - ry) ** 2 < S.LP_MAPPED_OBS_RADIUS ** 2
        if not keep.any():
            return scan
        world = np.stack([wx[keep], wy[keep]], axis=1)
        mapped_body = inverse_transform_points(world, *self.corrected)
        if scan.shape[0] == 0:
            return mapped_body
        return np.vstack([scan, mapped_body])

    def _reached(self, goal):
        return pose_distance(self.corrected, goal) < S.GOAL_REACH_TOL

    # --- stuck + recovery ---
    def _reset_anchor(self):
        self._anchor_pose = self.corrected
        self._anchor_time = self.sim_time

    def _stuck_check(self, ret_state):
        if pose_distance(self.corrected, self._anchor_pose) > S.STUCK_PROGRESS_MIN_M:
            self._reset_anchor()
            self.recovery_chain = 0          # real progress -> reset escalation
            return
        if (self.sim_time - self._anchor_time) > S.STUCK_TIMEOUT_S:
            print(f"[recovery] stuck in {self.state}; recovering", flush=True)
            self.return_state = ret_state
            self.state = RECOVERY
            self.recovery_phase = "reverse"
            self.recovery_until = self.sim_time + S.RECOVERY_REVERSE_T
            self.recovery_chain += 1
            if self.expl_goal is not None:
                self.blacklist.append(self.expl_goal)

    def _virtual_bumper(self, cmd_v):
        """Learn an invisible floating wall: if we push forward but don't move,
        stamp a sticky obstacle just ahead so the map routes around it."""
        if (self.state in (INIT_SCAN, RECOVERY, DONE)
                or cmd_v < S.BUMPER_MIN_CMD_V):
            self._bump_anchor = (self.corrected, self.sim_time)
            return
        if self._bump_anchor is None:
            self._bump_anchor = (self.corrected, self.sim_time)
            return
        apose, atime = self._bump_anchor
        if pose_distance(self.corrected, apose) > S.BUMPER_MIN_PROGRESS_M:
            self._bump_anchor = (self.corrected, self.sim_time)
            return
        if (self.sim_time - atime) > S.BUMPER_STALL_TIME_S:
            x, y, th = self.corrected
            bx = x + S.BUMPER_MARK_AHEAD * math.cos(th)
            by = y + S.BUMPER_MARK_AHEAD * math.sin(th)
            # A physically-confirmed invisible wall is a real obstacle -> stamp a
            # PERMANENT barrier (the map is lidar-only now; aux is gone).
            self.grid.mark_barrier_disc(bx, by, S.BUMPER_MARK_RADIUS)
            print(f"[bumper] invisible obstacle stamped @ ({bx:+.2f},{by:+.2f}) "
                  f"— replanning around it", flush=True)
            self.plan = None                 # force a reroute
            self._bump_anchor = (self.corrected, self.sim_time)

    def _rear_hard_blocked(self):
        """True only if something is HARD against the back (< RECOVERY_REAR_HARD).
        Reversing a little is almost always the right move to unwedge, so we only
        veto it when the rear is genuinely up against a wall."""
        ranges = self.io.read_ranges()
        for d, (sx, sy, sth) in zip(ranges, S.RANGE_SENSOR_POSES):
            if d is not None and abs(sth) > 1.0 and d < S.RECOVERY_REAR_HARD:
                return True
        s = self.scan_body
        if s.shape[0] == 0:
            return False
        behind = s[(s[:, 0] < 0.0) & (np.abs(s[:, 1]) < S.RECOVERY_REAR_HALF_WIDTH)]
        if behind.shape[0] == 0:
            return False
        return float(np.min(np.hypot(behind[:, 0], behind[:, 1]))) < S.RECOVERY_REAR_HARD

    def _do_recovery(self):
        """Reverse straight out of the wedge, THEN rotate IN PLACE until the head
        is aligned into the widest open gap, then drive straight.

        The turn no longer spins a fixed time (which over-rotated and swept a
        corner into a wall) — it rotates only until the heading points at the
        most open direction the lidar sees, then stops.  This is the user's
        "rotate on one axis, align the head toward the space, then move", applied
        in recovery only; normal corridor/corner turns still use the DWA."""
        # Phase 1 — reverse (skip only if the rear is hard against something).
        if self.recovery_phase == "reverse":
            if self.sim_time < self.recovery_until and not self._rear_hard_blocked():
                return -S.RECOVERY_REVERSE_V, 0.0
            # finished reversing -> lock the target heading = widest open gap
            self.recovery_phase = "align"
            gap = widest_gap_bearing(self.scan_body)        # bearing, robot frame
            self._recovery_align_target = wrap_angle(self.corrected[2] + gap)
            self.recovery_until = self.sim_time + S.RECOVERY_ALIGN_MAX_T
            # fall through into the align phase this tick

        # Phase 2 — rotate in place until the heading faces the open gap.
        if self.recovery_phase == "align":
            err = wrap_angle(self._recovery_align_target - self.corrected[2])
            aligned = abs(err) < S.RECOVERY_ALIGN_TOL
            if not aligned and self.sim_time < self.recovery_until:
                w = math.copysign(S.RECOVERY_ALIGN_W, err)
                return 0.0, w
            # aligned (or timed out) -> hand back to the seeking state, now facing
            # open space so the next forward command drives INTO the gap.
            self.plan = None
            self.expl_goal = None
            self._reset_anchor()
            self.recovery_phase = None
            self.state = self.return_state
            if self.recovery_chain >= S.RECOVERY_MAX_CHAIN:
                self.blacklist.clear()           # avoid starving exploration
                self.recovery_chain = 0
            return 0.0, 0.0
        return 0.0, 0.0

    def _render_blue_3d(self):
        """At blue identification: freshly project the 3D cloud to 2D and dump the
        3D render + the projected-2D planning map (the user-requested 'render the
        3D map and convert it to 2D' step before planning blue->yellow)."""
        if self.cloud is None:
            return
        try:
            self._project_cloud_to_grid()
            viz.save_ply(os.path.join(self.maps_dir, "scene_blue.ply"), self.cloud)
            viz.save_cloud_3d_png(
                os.path.join(self.maps_dir, "cloud3d_blue.png"), self.cloud,
                pose_history=self.pose_history, pillars=self.pillars.pos,
                title=f"3D map at BLUE  t={self.sim_time:.1f}s")
            viz.save_snapshot(
                os.path.join(self.maps_dir, "proj2d_blue.png"), self.grid,
                pose=self.corrected, pillars=self.pillars.pos,
                state_text=f"3D->2D projection @ BLUE  t={self.sim_time:.1f}s\n"
                           f"cloud voxels={len(self.cloud)} "
                           f"cloud_obs cells={int(self.grid.cloud_obs.sum())}")
            print(f"[3d] rendered 3D cloud ({len(self.cloud)} voxels) + 2D "
                  f"projection at BLUE", flush=True)
        except Exception as e:
            print("[3d] blue render failed:", e, flush=True)

    # --------------------------------------------------------------- io
    def _maybe_snapshot(self):
        if (self.sim_time - self._last_snap_t) < S.SNAPSHOT_PERIOD_S:
            return
        self._last_snap_t = self.sim_time
        txt = (f"{self.state}  t={self.sim_time:6.2f}s\n"
               f"pose=({self.corrected[0]:+.2f},{self.corrected[1]:+.2f},"
               f"{self.corrected[2]:+.2f})\n"
               f"occ={int(self.grid.occupied_mask().sum())} "
               f"sm={self.sm_updates}")
        try:
            viz.save_snapshot(
                os.path.join(self.maps_dir, f"mak02_{int(self.sim_time):04d}.png"),
                self.grid, pose=self.corrected, path_world=self.plan,
                goal=self.expl_goal, pillars=self.pillars.pos, state_text=txt)
        except Exception as e:
            print("[viz] snapshot failed:", e, flush=True)

    def _maybe_live(self):
        """Refresh the live exploration window (frontiers + plan + trajectory)."""
        if self.live is None or not self.live.ok or self.tick % 8 != 0:
            return
        try:
            fr = (find_frontiers(self.grid)
                  if self.state in (EXPLORE_BLUE, EXPLORE_YELLOW) else None)
            txt = (f"{self.state} t={self.sim_time:.1f}s "
                   f"occ={int(self.grid.occupied_mask().sum())} sm={self.sm_updates}")
            self.live.update(self.grid, pose=self.corrected, plan=self.plan,
                             frontiers=fr, goal=self.expl_goal,
                             pillars=self.pillars.pos,
                             traj=self.pose_history[-400:], text=txt)
        except Exception as e:
            print("[live] update failed; disabling:", e, flush=True)
            self.live.ok = False

    def _finalize(self):
        if self.live is not None:
            self.live.close()
        print("[mak02] finalising — writing outputs...", flush=True)
        try:
            viz.save_final_map(os.path.join(self.maps_dir, "final_map.png"),
                               self.grid, self.pose_history, self.pillars.pos)
        except Exception as e:
            print("[viz] final map failed:", e, flush=True)
        try:
            viz.save_npz(os.path.join(self.maps_dir, "map.npz"), self.grid,
                         self.pose_history, self.pillars.pos, self.start_pose)
        except Exception as e:
            print("[viz] npz failed:", e, flush=True)
        if self.cloud is not None:
            try:
                viz.save_ply(os.path.join(self.maps_dir, "scene.ply"), self.cloud)
                viz.save_cloud_3d_png(
                    os.path.join(self.maps_dir, "cloud3d_final.png"), self.cloud,
                    pose_history=self.pose_history, pillars=self.pillars.pos,
                    title="mak_02 final 3D voxel cloud")
            except Exception as e:
                print("[viz] 3D export failed:", e, flush=True)
        b = "-" if self.t_blue is None else f"{self.t_blue:.2f}s"
        y = "-" if self.t_yellow is None else f"{self.t_yellow:.2f}s"
        seg = ("-" if (self.t_blue is None or self.t_yellow is None)
               else f"{self.t_yellow - self.t_blue:.2f}s")
        print("=" * 64, flush=True)
        print(f"[mission] start->BLUE : {b}", flush=True)
        print(f"[mission] BLUE->YELLOW: {seg}", flush=True)
        print(f"[mission] total       : {y}", flush=True)
        print("=" * 64, flush=True)

    # ------------------------------------------------------------- loop
    def run(self):
        try:
            while self.io.step() != -1:
                self.sim_time += self.dt
                self.tick += 1

                self._odom_and_predict()
                self._read_scan()
                self._slam_step()
                self._perception_step()

                if self.tick % 5 == 0:
                    self.pose_history.append(self.corrected)

                keys = self.io.poll_key()
                if ord('Q') in keys or ord('q') in keys:
                    print("[mak02] Q pressed — finalising.", flush=True)
                    self.io.stop()
                    break

                v, w = self._fsm()
                self.io.set_cmd(v, w)
                self._virtual_bumper(v)
                self._maybe_snapshot()
                self._maybe_live()

                if self.state == DONE:
                    if self._done_at is None:
                        self._done_at = self.sim_time
                    elif self.sim_time - self._done_at > S.DONE_LINGER_S:
                        self.io.stop()
                        break

                if self.tick % S.LOG_EVERY_TICKS == 0:
                    print(f"[t={self.sim_time:6.2f}s] {self.state} "
                          f"pose=({self.corrected[0]:+.2f},{self.corrected[1]:+.2f},"
                          f"{self.corrected[2]:+.2f}) v={v:+.2f} w={w:+.2f} "
                          f"occ={int(self.grid.occupied_mask().sum())} "
                          f"dobs={int(self.grid.depth_obs_mask().sum())} "
                          f"cobs={int(self.grid.cloud_obs.sum())} "
                          f"vox={len(self.cloud) if self.cloud else 0} "
                          f"bar={int(self.grid.barrier.sum())} "
                          f"sm={self._last_sm_norm:.2f}{'' if self._last_sm_ok else '!'} "
                          f"blue={self.pillars.known('blue')}"
                          f"{'*' if self.pillars.is_confirmed('blue') else ''} "
                          f"yellow={self.pillars.known('yellow')}"
                          f"{'*' if self.pillars.is_confirmed('yellow') else ''}",
                          flush=True)
        except KeyboardInterrupt:
            print("[mak02] interrupted", flush=True)
        finally:
            self._finalize()


def main():
    Mak02Controller().run()


if __name__ == "__main__":
    main()
