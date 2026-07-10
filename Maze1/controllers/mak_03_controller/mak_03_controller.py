"""
Maze1 Navigation Controller (mak_03).

This module provides a unified navigation controller integrating SLAM, 
frontier-based exploration, A* global routing, and DWA local planning. 
It operates a state machine to autonomously navigate the robot towards 
successive objectives while avoiding dynamically mapped obstacles. 
Optimized for Maze1's specific grid extent and exploration radius.
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
from return_path import ReturnPathFollower
from return_path_diag import ReturnPathTracker


class Mission:
    INIT_SCAN = "INIT_SCAN"
    EXPLORE_BLUE = "EXPLORE_BLUE"
    GO_BLUE = "GO_BLUE"
    RETURN_PATH = "RETURN_PATH"
    EXPLORE_YELLOW = "EXPLORE_YELLOW"
    GO_YELLOW = "GO_YELLOW"
    RECOVERY = "RECOVERY"
    DONE = "DONE"


class Mak03Controller:
    def __init__(self):
        self.io = RobotIO()
        self.dt = self.io.dt

        specs = self.io.lidar_specs()
        if specs is not None:
            n_beams, fov, r_min, r_max = specs
            self.lidar = LidarModel(n_beams, fov, r_min, r_max)
        else:
            self.lidar = None
            print("[mak_03] WARNING: no lidar found")

        self.grid = OccupancyGrid()
        self.odom = Odometry()

        cam = self.io.camera_specs()
        if cam is not None:
            self.percep = Perception(cam[0], cam[1], cam[2])
        else:
            self.percep = None
            print("[mak_03] WARNING: no camera found")

        dspecs = self.io.depth_specs()
        if dspecs is not None:
            dw, dh, dfov, dmin, dmax = dspecs
            self.depth_model = DepthModel(dw, dh, dfov, depth_min=dmin, depth_max=dmax)
        else:
            self.depth_model = None
            print("[mak_03] WARNING: no depth camera found — low floating panels "
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
        self.return_follower = ReturnPathFollower(turn_w_max=C.W_MAX)   # remembers start->blue route
        self.go_fail_until = 0.0
        self.no_frontier_until = 0.0
        self.cur_v = 0.0
        self.raw_lidar_ranges = None

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
        self.green_block = False     # True when imminent poison is confirmed for
                                     # GREEN_REFLEX_CONFIRM_FRAMES consecutive frames
        self._green_streak = 0       # consecutive frames the reflex has fired
        self.ir_block = False
        self.outdir = os.path.join(os.path.dirname(os.path.abspath(__file__)),
                                   C.OUTPUT_DIRNAME)
        os.makedirs(self.outdir, exist_ok=True)
        self.return_diag = (ReturnPathTracker(
            self.outdir, stall_v=C.RETURN_DIAG_STALL_V, stall_s=C.RETURN_DIAG_STALL_S,
            log_every_ticks=C.RETURN_DIAG_LOG_EVERY_TICKS,
            near_obs_r=C.RETURN_DIAG_NEAR_OBS_R, fwd_cone_deg=C.RETURN_DIAG_FWD_CONE_DEG)
            if C.RETURN_DIAG_ENABLED else None)

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
            self.raw_lidar_ranges = self.io.read_lidar_ranges()
            if self.raw_lidar_ranges is not None:
                self.scan_body, self.scan_ranges = self.lidar.ranges_to_body(self.raw_lidar_ranges)

    def _moved_enough(self):
        if self._pose_at_last_integrate is None:
            return True
        dx, dy, dth = relative_pose(self._pose_at_last_integrate, self.pose)
        return (math.hypot(dx, dy) > C.SM_MIN_TRAVEL_M or
                abs(dth) > C.SM_MIN_TURN_RAD)

    def _slam_step(self):
        """
        Execute a single SLAM iteration with gated integration and scan matching.

        Integrates lidar scans into the occupancy grid only after sufficient 
        displacement to minimize local erosion. Utilizes conservative scan matching 
        to correct odometry drift and align successive scans with the existing map.
        """
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
        # Project depth-validated green footprint onto the occupancy map layer.
        green_world = self.percep.green_floor_world(bgr, depth, self.pose)
        self.grid.add_poison_points(green_world)
        # Engage green-poison safety reflex based on multi-frame confirmation.
        if self.percep.green_reflex(bgr, depth):
            self._green_streak += 1
        else:
            self._green_streak = 0
        self.green_block = (C.GREEN_REFLEX_ENABLED
                            and self._green_streak >= C.GREEN_REFLEX_CONFIRM_FRAMES)

    def _depth_aux_step(self):
        """
        Integrate depth camera readings into the auxiliary obstacle layer.

        Maintains persistent obstacle representations for environmental features 
        undetectable by planar lidar. Hit probabilities are reinforced by positive 
        readings and decayed by clear sight-lines.
        """
        if self.depth_model is None:
            return
        depth = self.io.read_depth()
        if depth is None:
            return
        bearings, hit_ranges, hit_mask, clear_mask = self.depth_model.thin_scan(depth)
        self.grid.integrate_depth_obstacles(self.pose, bearings, hit_ranges,
                                            hit_mask, clear_mask, self.depth_model.depth_min)

    def _ir_step(self):
        """
        Process infrared bumper sensor data to enforce collision prevention.

        Functions as a low-level, map-independent safety mechanism for 
        obstacles outside the depth sensor's effective range.
        """
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
            # Ensure the robot's current position remains traversable in the costmap
            # to prevent path planning deadlocks in constrained environments.
            cix, ciy = self.grid.world_to_grid(self.pose[0], self.pose[1])
            r = max(3, int(round(0.12 / self.grid.res)))  # ~0.12 m, res-independent
            n = self.grid.n
            x0, x1 = max(0, cix - r), min(n, cix + r + 1)
            y0, y1 = max(0, ciy - r), min(n, ciy + r + 1)
            self.lethal[x0:x1, y0:y1] = False
            sub = self.cost[x0:x1, y0:y1]
            np.minimum(sub, C.COST_OBS_WEIGHT * 0.5, out=sub)  # capped, not free
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

    def _plan_to_live(self, goal_world, t):
        """A* to ``goal_world`` on the live costmap, forcing one costmap rebuild +
        retry if the first attempt fails (start/goal may sit in a stale lethal
        cell).  Passed to return_path.ReturnPathFollower.plan_step so the retrace
        policy can replan without importing the grid/costmap itself."""
        path = self._plan_to(goal_world)
        if path is None:
            self._ensure_costmap(t, force=True)
            path = self._plan_to(goal_world)
        return path

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
        """
        Detect prolonged immobility indicating a navigation failure.

        Monitors translational progress over a specified temporal window 
        to trigger recovery routines if the robot fails to advance.

        Args:
            t: Current simulation time.

        Returns:
            bool: True if the robot is deemed stuck, False otherwise.
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
        """
        Serve as a secondary deadlock detection mechanism.

        Monitors both positional and angular state to identify complete 
        stagnation, capturing edge cases not addressed by translational checks.

        Args:
            t: Current simulation time.

        Returns:
            bool: True if the robot is completely frozen, False otherwise.
        """
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
        if self.recovery_return == Mission.RETURN_PATH:
            # Transition to frontier exploration if the return path becomes permanently obstructed.
            if self.recovery_chain >= C.RETURN_PATH_MAX_RECOVERY_CHAIN:
                print(f"[return_path] recovery exhausted ({self.recovery_chain} chained "
                      f"attempts) near ({self.pose[0]:+.2f},{self.pose[1]:+.2f}) -- "
                      f"abandoning retrace, failing over to EXPLORE_YELLOW")
                if self.return_diag is not None:
                    self.return_diag.close(t, self.pose)
                self.recovery_chain = 0
                self.path = []
                self.goal_world = None
                self._reset_progress(t)
                self.state = self.recovery_return = Mission.EXPLORE_YELLOW
                return 0.0, 0.0
        elif self.recovery_chain >= C.RECOVERY_MAX_CHAIN and self.goal_world is not None:
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
        goal = pw
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
            # force a fresh costmap before frontier selection so we plan on the
            # most up-to-date data (also re-runs the anti-self-boxing disc)
            self._ensure_costmap(t, force=True)
            path, goal = self._select_frontier()
            if path is not None:
                self.path, self.goal_world = path, goal
                self._plan_stamp = t
            else:
                # Initiate localized rescanning when no valid frontiers are present.
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
        # arrival check.  For yellow, only trust the distance once the pillar
        # has been seen whole (top/middle/bottom bands all unoccluded) --
        # otherwise a partial/edge-on sighting can put a slightly-off
        # pillar_world estimate inside PILLAR_REACH_DIST while the robot is
        # actually still approaching around an obstruction.
        if target == "yellow":
            full_view = self.percep.pillar_full_view.get("yellow", False) if self.percep else False
            reached = full_view and pose_distance(self.pose, pw) <= C.PILLAR_REACH_DIST
        else:
            reached = pose_distance(self.pose, pw) <= C.PILLAR_REACH_DIST
        if reached:
            reached_cb(t)
            self.io.stop()
            self.path = []
            self.dwa.reset()
            self._reset_progress(t)   # don't inherit stale stuck timer into next state
            self.state = next_state
            return 0.0, 0.0
        # keep the goal fresh as the estimate refines
        goal = pw
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
        route = self.return_follower.trigger()
        print(f"[return_path] triggered -> retracing {len(route)} waypoints")
        if self.return_diag is not None:
            self.return_diag.dump_route(t, route)

    def _reached_yellow(self, t):
        self.t_yellow = t
        print(f"\n*** YELLOW pillar reached at t = {t:.2f} s "
              f"(blue->yellow = {t - self.t_blue:.2f} s) ***\n")

    def _follow_return_path(self, t):
        """
        Retrace the recorded approach route using dynamic A* sub-goals.

        Integrates the live-costmap A* planner with the established carrot and DWA 
        pipeline to navigate the return path. Bypasses raw breadcrumbs in favor 
        of obstacle-aware trajectories to prevent entanglement with mapped lethals.

        Args:
            t: Current simulation time.
        """
        # Specific regional override: transition directly to YELLOW planning 
        # from the problematic north pocket near (1.77, 1.67).
        RP_HANDOFF_XY = (1.77, 1.67)
        RP_HANDOFF_R = 0.45
        if math.hypot(self.pose[0] - RP_HANDOFF_XY[0],
                      self.pose[1] - RP_HANDOFF_XY[1]) < RP_HANDOFF_R:
            print(f"[return_path] reached wedge region {RP_HANDOFF_XY} -> "
                  f"planning to yellow (GO_YELLOW)")
            if self.return_diag is not None:
                self.return_diag.close(t, self.pose)
            self.path = []
            self.goal_world = None
            self._reset_progress(t)
            self.dwa.reset()
            self.state = Mission.GO_YELLOW
            return 0.0, 0.0

        path, done = self.return_follower.plan_step(
            self.pose, lambda g: self._plan_to_live(g, t), t)
        if done:
            self.io.stop()
            self.path = []
            self.dwa.reset()
            self.state = Mission.DONE
            print("[return_path] route fully retraced -> DONE")
            if self.return_diag is not None:
                self.return_diag.mark_done(t, self.pose)
                self.return_diag.close(t, self.pose)
            return 0.0, 0.0
        self.path = path
        self.goal_world = path[-1] if path else None
        tv, tw, aligning = self.return_follower.turn_command(self.pose)
        if aligning:
            v, w = tv, tw
            self.carrot = None            # not carrot-driving during the aligned turn
            self.dwa.reset()              # keep the DWA rate-limiter from fighting the handoff
        else:
            v, w, _ = self._drive_to(self.path, v_cap=C.V_CRUISE)
        if self.return_diag is not None:
            nearest_aux, nearest_poison = self.return_diag.nearby_obstacles(
                self.grid, self.pose[0], self.pose[1])
            min_ahead = self.return_diag.min_lidar_ahead(self.scan_body)
            self.return_diag.update(t, self.pose, v, w, self.cur_v, self.path,
                                    self.carrot, min_ahead, nearest_aux, nearest_poison)
        return v, w

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
            return self._go("blue", Mission.RETURN_PATH, t, self._reached_blue)
        if st == Mission.RETURN_PATH:
            return self._follow_return_path(t)
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
            print("[mak_03] finalize save error:", e)
        self._print_timing()
        if self.return_diag is not None:
            self.return_diag.close(self.io.time(), self.pose)
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
        print(f"[mak_03] start pose (odom) = ({self.pose[0]:.2f},{self.pose[1]:.2f},"
              f"{math.degrees(self.pose[2]):.1f} deg)")

        while self.io.step() != -1:
            self.tick += 1
            t = self.io.time()

            keys = self.io.poll_key()
            if ord('Q') in keys or ord('q') in keys:
                print("[mak_03] Q pressed -> finalize")
                break

            self._sense()
            self._slam_step()
            self.return_follower.record(self.pose[0], self.pose[1])
            if self.tick % C.PERCEPTION_EVERY_TICKS == 0:
                self._perception_step()
            if self.tick % C.DEPTH_AUX_EVERY_TICKS == 0:
                self._depth_aux_step()
            self._ir_step()
            self._ensure_costmap(t)
            # keep the robot's own cell plannable in the costmap snapshot
            self.grid.mark_free_disc(self.pose[0], self.pose[1], C.ROBOT_RADIUS * 0.8)

            v, w = self._step_fsm(t)

            # Apply position and heading watchdogs globally across active driving states.
            driving = self.state in (Mission.EXPLORE_BLUE, Mission.GO_BLUE,
                                     Mission.RETURN_PATH,
                                     Mission.EXPLORE_YELLOW, Mission.GO_YELLOW)
            if driving and (self._is_stuck(t) or self._is_frozen(t)):
                self._enter_recovery(t, self.state)
                v, w = 0.0, 0.0

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
    ctrl = Mak03Controller()
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
