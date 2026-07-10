"""Maze5 frontier-exploration navigation controller for the Webots ROSbot.

Maze5 is the baseline stack: the robot localises with wheel + IMU odometry
refined by a light lidar scan matcher, maps with a 2-D lidar log-odds grid,
explores with visual frontiers, plans with A*, and drives with a DWA local
planner on the live lidar (so wall clearance is drift-immune). It has no
depth-camera aux layer or chassis-IR bumper (there are no low/floating panels to
cover), so its pipeline is a strict subset of the other mazes'.

Mission (from Modularbeit.pdf): drive from the start to the BLUE pillar, then to
the YELLOW pillar, in the least simulation time, without touching walls or the
green poison floor.  No Webots Supervisor is used.

Mission state machine:
    INIT_SCAN -> EXPLORE_BLUE -> GO_BLUE -> EXPLORE_YELLOW -> GO_YELLOW -> DONE
    (RECOVERY is reachable from any driving state when progress stalls.)

Observability: every run emits leveled console logs plus a structured JSON-Lines
event log at ``maps/run_events.jsonl`` (state transitions, pillar timing,
recovery, hardware/sensor faults, exceptions) via the vendored ``observability``
module; each control-loop stage is wrapped in a fault barrier so a single bad
sensor frame cannot abort the mission.

Run: set the Rosbot node's ``controller`` field to ``mak_02_controller`` (already
set in Maze5.wbt).  Press 'Q' in the sim to finalise outputs early.
"""
from __future__ import annotations

import math
import os
import time
from typing import Optional, Callable

import numpy as np

import config as C
import astar
import frontier as FR
import local_planner as LP
from geometry import (compose_pose, relative_pose, wrap_angle, pose_distance,
                      transform_points, inverse_transform_points)
from mapping import LidarModel, OccupancyGrid
from observability import get_logger, RunEventLog, guarded_stage
from odometry import Odometry
from perception import Perception
from robot_io import RobotInterface
from viz import Visualizer

#: Logger name for this maze's controller (shared observability convention).
LOGGER_NAME = "navctl.maze5"


class MissionState:
    """Named states of the mission finite-state machine (see module docstring)."""
    INIT_SCAN = "INIT_SCAN"
    EXPLORE_BLUE = "EXPLORE_BLUE"
    GO_BLUE = "GO_BLUE"
    EXPLORE_YELLOW = "EXPLORE_YELLOW"
    GO_YELLOW = "GO_YELLOW"
    RECOVERY = "RECOVERY"
    DONE = "DONE"


class NavigationController:
    """Top-level sense-plan-act controller for the maze mission.

    Owns the hardware interface, the occupancy map, localisation, perception,
    the local planner and the mission state machine, and runs them all from
    :meth:`run`. All device access goes through :class:`RobotInterface`; every
    other collaborator consumes plain NumPy arrays and pose tuples, so the
    control logic stays simulator-agnostic and unit-testable.
    """

    def __init__(self):
        """Discover devices, build the pipeline, and open the run event log.

        Missing optional sensors (lidar/camera) degrade gracefully with a
        warning rather than aborting, so the controller still runs — and logs
        why — on a partially-equipped robot.
        """
        self.log = get_logger(LOGGER_NAME)
        self.outdir = os.path.join(os.path.dirname(os.path.abspath(__file__)),
                                   C.OUTPUT_DIRNAME)
        os.makedirs(self.outdir, exist_ok=True)
        self.events = RunEventLog(self.outdir, logger=self.log)

        self.hardware = RobotInterface(logger=self.log)
        self.dt = self.hardware.dt

        specs = self.hardware.lidar_specs()
        if specs is not None:
            n_beams, fov, r_min, r_max = specs
            self.lidar = LidarModel(n_beams, fov, r_min, r_max)
        else:
            self.lidar = None
            self.log.warning("no lidar found — mapping/DWA will be degraded")

        self.occupancy_grid = OccupancyGrid()
        self.odometry = Odometry()

        cam = self.hardware.camera_specs()
        if cam is not None:
            self.perception = Perception(cam[0], cam[1], cam[2])
        else:
            self.perception = None
            self.log.warning("no camera found — pillar/poison perception disabled")

        self.local_planner = LP.DWAPlanner(ctrl_dt=self.dt)
        self.visualizer = Visualizer(self.occupancy_grid)

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
        self.state = MissionState.INIT_SCAN
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
        self.recovery_return = MissionState.EXPLORE_BLUE
        self.recovery_chain = 0
        self._creep_target = None
        self._creep_t0 = 0.0

        # timing table
        self.t_start = None
        self.t_blue = None
        self.t_yellow = None

        self.tick = 0
        self._last_snapshot_t = -1e9
        self.green_block = False

    # ===================================================================== #
    #  Sensing & localisation
    # ===================================================================== #
    def update_sensing(self) -> None:
        """Reads wheel/IMU/lidar hardware and advances the pose belief by one increment."""
        wl, wr = self.hardware.read_encoders()
        yaw = self.hardware.read_yaw()
        self.odometry.update(wl, wr, yaw, self.dt)
        raw = self.odometry.pose()
        if self._last_raw is None:
            self.pose = raw
            self._last_raw = raw
        else:
            inc = relative_pose(self._last_raw, raw)
            self.pose = compose_pose(self.pose, inc)
            self._last_raw = raw

        if self.lidar is not None:
            ranges = self.hardware.read_lidar_ranges()
            if ranges is not None:
                self.scan_body, self.scan_ranges = self.lidar.ranges_to_body(ranges)

    def _has_moved_enough(self) -> bool:
        """Determines if the robot has moved or turned enough to warrant re-integrating a scan.
        
        Returns:
            bool: True if sufficient translation or rotation has occurred.
        """
        if self._pose_at_last_integrate is None:
            return True
        dx, dy, dth = relative_pose(self._pose_at_last_integrate, self.pose)
        return (math.hypot(dx, dy) > C.SM_MIN_TRAVEL_M or
                abs(dth) > C.SM_MIN_TURN_RAD)

    def run_slam_step(self) -> None:
        """Matches the latest lidar cloud against the map, then integrates it."""
        if self.scan_body.shape[0] == 0 or not self._has_moved_enough():
            return
        if C.SM_ENABLED:
            corrected, hit = self.occupancy_grid.scan_match(self.pose, self.scan_body)
            self.pose = corrected
        self.occupancy_grid.integrate_scan(self.pose, self.scan_body, self.scan_ranges)
        self.occupancy_grid.mark_free_disc(self.pose[0], self.pose[1], C.ROBOT_RADIUS * 0.8)
        self._pose_at_last_integrate = self.pose

    def update_perception(self) -> None:
        """Detects pillars and the green poison floor from the RGB(-D) camera."""
        if self.perception is None:
            return
        bgr = self.hardware.read_rgb_bgr()
        if bgr is None:
            return
        depth = self.hardware.read_depth()
        self.perception.update_pillars(bgr, depth, self.pose)
        green_world = self.perception.green_floor_world(bgr, self.pose)
        self.occupancy_grid.add_poison_points(green_world)
        self.green_block = self.perception.green_reflex(bgr)

    # ===================================================================== #
    #  Planning
    # ===================================================================== #
    def refresh_costmap(self, t: float, force: bool = False) -> None:
        """Rebuilds the A* costmap when it is stale or forced.
        
        Args:
            t (float): Current simulation time in seconds.
            force (bool, optional): If True, forces an immediate rebuild. Defaults to False.
        """
        if force or self.cost is None or (t - self._last_costmap_t) >= C.REPLAN_PERIOD_S:
            self.cost, self.lethal = self.occupancy_grid.build_costmap()
            self._last_costmap_t = t

    def plan_path_to(self, goal_world: tuple[float, float]) -> Optional[list[tuple[float, float]]]:
        """Plans a simplified A* world-space path to the target goal.
        
        Args:
            goal_world (tuple[float, float]): The (x, y) target coordinate in world space.
            
        Returns:
            Optional[list[tuple[float, float]]]: A list of world coordinates defining the path, or None if unreachable.
        """
        if self.cost is None:
            return None
        start = self.occupancy_grid.world_to_grid(self.pose[0], self.pose[1])
        goal = self.occupancy_grid.world_to_grid(goal_world[0], goal_world[1])
        cells = astar.plan(self.cost, self.lethal, start, goal)
        if cells is None:
            return None
        cells = astar.simplify(cells, self.lethal)
        return [self.occupancy_grid.grid_to_world(ix, iy) for (ix, iy) in cells]

    def _is_blacklisted(self, wxy: tuple[float, float]) -> bool:
        """Checks if a world coordinate is near a previously-abandoned (unreachable) goal.
        
        Args:
            wxy (tuple[float, float]): The (x, y) coordinate to verify.
            
        Returns:
            bool: True if the coordinate is blacklisted.
        """
        for bx, by in self.blacklist:
            if math.hypot(wxy[0] - bx, wxy[1] - by) < C.FRONTIER_BLACKLIST_R:
                return True
        return False

    def select_frontier_goal(self) -> tuple[Optional[list[tuple[float, float]]], Optional[tuple[float, float]]]:
        """Identifies and selects the optimal reachable frontier.
        
        Returns:
            tuple[Optional[list[tuple[float, float]]], Optional[tuple[float, float]]]: 
                A tuple containing the planned path and the goal world coordinate.
                Returns (None, None) if no frontiers are available or reachable.
        """
        self.frontier_mask = FR.detect_frontier_cells(self.occupancy_grid, self.lethal)
        cands = FR.find_frontiers(self.occupancy_grid, self.lethal, self.pose[:2], self.start_xy)
        cands = [c for c in cands if not self._is_blacklisted(c["world"])]
        if not cands:
            return None, None
        # cheapest first: evaluate the closest handful with real A* path length
        rx, ry = self.pose[0], self.pose[1]
        cands.sort(key=lambda c: math.hypot(c["world"][0] - rx, c["world"][1] - ry))
        best = None
        for c in cands[:7]:
            path = self.plan_path_to(c["world"])
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
    def _path_length(path: list[tuple[float, float]]) -> float:
        """Calculates the total Euclidean length of a world-space polyline path.
        
        Args:
            path (list[tuple[float, float]]): The path as a sequence of (x, y) coordinates.
            
        Returns:
            float: The total path length in meters.
        """
        return sum(math.hypot(path[i + 1][0] - path[i][0], path[i + 1][1] - path[i][1])
                   for i in range(len(path) - 1))

    def _pillar_standoff(self, pillar_world: tuple[float, float]) -> tuple[float, float]:
        """Calculates a reachable standoff point just short of the pillar center.
        
        Args:
            pillar_world (tuple[float, float]): The (x, y) world coordinate of the pillar.
            
        Returns:
            tuple[float, float]: The (x, y) standoff coordinate in world space.
        """
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
    def _poison_body(self) -> np.ndarray:
        """Retrieves mapped poison points near the robot, transformed into the body frame.
        
        Returns:
            np.ndarray: An array of (x, y) body frame coordinates representing nearby poison.
        """
        pw = self.occupancy_grid.poison_points_near(self.pose[0], self.pose[1], C.DWA_SLOWDOWN_DIST + 0.4)
        if pw.shape[0] == 0:
            return np.empty((0, 2))
        return inverse_transform_points(pw, self.pose[0], self.pose[1], self.pose[2])

    def drive_along_path(self, path: list[tuple[float, float]], v_cap: Optional[float] = None) -> tuple[float, float, bool]:
        """Executes Carrot + DWA navigation toward the end of the specified path.
        
        Args:
            path (list[tuple[float, float]]): The A* path as a sequence of (x, y) world coordinates.
            v_cap (Optional[float], optional): Hard upper limit on linear velocity. Defaults to None.
            
        Returns:
            tuple[float, float, bool]: A tuple containing the `(v, w)` command in m/s and rad/s,
                and a boolean flag indicating if the goal is near.
        """
        if not path:
            return 0.0, 0.0, True
        carrot, near_goal = LP.choose_carrot(path, self.pose, self.cur_v)
        self.carrot = carrot
        poison_b = self._poison_body()
        v, w = self.local_planner.compute(self.pose, carrot, self.scan_body, poison_b,
                                green_block=self.green_block, v_cap=v_cap)
        return v, w, near_goal

    # ===================================================================== #
    #  Stuck detection & recovery
    # ===================================================================== #
    def _reset_progress(self, t: float) -> None:
        """Resets the stuck-watchdog reference to the current pose and time.
        
        Args:
            t (float): Current simulation time in seconds.
        """
        self._progress_ref = (self.pose[0], self.pose[1])
        self._progress_ref_t = t

    def _is_stuck(self, t: float, commanding_forward: bool) -> bool:
        """Checks if the robot has made no forward progress within the stuck timeout.

        Only arms while `commanding_forward` is set, so an intentional in-place
        spin is not mistaken for being stuck.
        
        Args:
            t (float): Current simulation time in seconds.
            commanding_forward (bool): True if the robot is being commanded to move forward.
            
        Returns:
            bool: True if the robot is deemed stuck.
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
        if not commanding_forward:
            self._progress_ref_t = t  # don't accrue stuck time while intentionally stopped
            return False
        return (t - self._progress_ref_t) > C.STUCK_TIMEOUT_S

    def _is_frozen(self, t: float) -> bool:
        """Checks if neither position nor heading has changed for too long.
        
        Acts as a backstop in driving states to catch deadlocks where the command
        is completely zeroed out, bypassing the forward-command stuck check.
        
        Args:
            t (float): Current simulation time in seconds.
            
        Returns:
            bool: True if the robot has not moved nor turned for the timeout duration.
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

    def _enter_recovery(self, t: float, return_state: str) -> None:
        """Enters the RECOVERY state, choosing reverse-then-spin or spin-only.
        
        Args:
            t (float): Current simulation time in seconds.
            return_state (str): The state to transition back to after recovery.
        """
        self.recovery_return = return_state
        self.state = MissionState.RECOVERY
        self.recovery_phase = "reverse"
        self.recovery_t0 = t
        self.recovery_chain += 1
        self._frozen_ref = None   # reset the frozen watchdog on entering recovery
        self.local_planner.reset()
        rear = LP.rear_clearance(self.scan_body, 0.18)
        if rear < C.RECOVERY_REAR_MIN_CLEAR:
            self.recovery_phase = "spin"   # no room behind -> spin instead
        self.log.warning("recovery enter (%s); rear=%.2fm chain=%d",
                         self.recovery_phase, rear, self.recovery_chain)
        self.events.event("recovery_enter", level="WARNING", sim_time=t,
                          tick=self.tick, state=return_state,
                          phase=self.recovery_phase, rear_clear=round(rear, 3),
                          chain=self.recovery_chain)

    def _run_recovery(self, t: float) -> tuple[float, float]:
        """Executes the reverse-then-spin recovery manoeuvre.
        
        Args:
            t (float): Current simulation time in seconds.
            
        Returns:
            tuple[float, float]: The `(v, w)` recovery command.
        """
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
        return_state = self.recovery_return
        if self.recovery_chain >= C.RECOVERY_MAX_CHAIN:
            if self.goal_world is not None:
                self.blacklist.append(self.goal_world)   # give up on this goal
            self.recovery_chain = 0
            # GO_BLUE/GO_YELLOW's goal is a straight-line pillar standoff point,
            # not blacklist-aware -- bouncing straight back into the same GO_*
            # state re-derives the SAME (unreachable) standoff and loops forever
            # (this is the "GO/RECOVERY bounce" documented for the sibling maze4
            # controller). After repeated failed recoveries, fall back to
            # frontier EXPLORE instead so a different approach angle gets tried,
            # with a cooldown so _try_go_to_target doesn't immediately re-commit.
            if return_state in (MissionState.GO_BLUE, MissionState.GO_YELLOW):
                return_state = (MissionState.EXPLORE_BLUE
                                 if return_state == MissionState.GO_BLUE
                                 else MissionState.EXPLORE_YELLOW)
                self.go_fail_until = t + C.GO_FAIL_COOLDOWN_S
                self.log.warning("recovery exhausted in %s; falling back to %s",
                                 self.recovery_return, return_state)
        self.path = []
        self.goal_world = None
        self._reset_progress(t)
        self.state = return_state
        return 0.0, 0.0

    # ===================================================================== #
    #  MissionState state machine
    # ===================================================================== #
    def _try_go_to_target(self, target: str, go_state: str, t: float) -> bool:
        """Switches EXPLORE->GO if the pillar is known and an A* path exists.
        
        Args:
            target (str): The pillar target identifier ("blue" or "yellow").
            go_state (str): The corresponding GO state to switch to.
            t (float): Current simulation time in seconds.
            
        Returns:
            bool: True if the transition to the GO state was successfully triggered.
        """
        pw = self.perception.pillar_world.get(target) if self.perception else None
        if pw is None or t < self.go_fail_until:
            return False
        # throttle the reachability A* so we don't run it every 32 ms tick
        if (t - getattr(self, "_last_go_try", -1e9)) < 0.4:
            return False
        self._last_go_try = t
        goal = self._pillar_standoff(pw)
        path = self.plan_path_to(goal)
        if path is None:
            return False
        self.path = path
        self.goal_world = goal
        self.state = go_state
        self._reset_progress(t)
        self.local_planner.reset()
        self.log.info("%s pillar known at (%.2f,%.2f) -> %s",
                      target, pw[0], pw[1], go_state)
        self.events.event("state_transition", sim_time=t, tick=self.tick,
                          state=go_state, target=target,
                          pillar_world=[round(pw[0], 3), round(pw[1], 3)])
        return True

    def _explore(self, target: str, go_state: str, t: float) -> tuple[float, float]:
        """Explores toward the target: commits to it if seen, else picks a frontier.
        
        Args:
            target (str): The pillar target identifier to look out for.
            go_state (str): The corresponding GO state to transition to when seen.
            t (float): Current simulation time in seconds.
            
        Returns:
            tuple[float, float]: The `(v, w)` command for exploration.
        """
        if self._try_go_to_target(target, go_state, t):
            return self.drive_along_path(self.path)[:2]
        # (re)select a frontier goal when needed
        need_new = (not self.path or self.goal_world is None or
                    pose_distance(self.pose, self.goal_world) < C.GOAL_REACH_TOL or
                    (t - self._last_plan_t()) > C.REPLAN_PERIOD_S)
        if need_new:
            path, goal = self.select_frontier_goal()
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
        v, w, _ = self.drive_along_path(self.path)
        return v, w

    def _last_plan_t(self) -> float:
        """Retrieves the simulation time of the most recent successful plan.
        
        Returns:
            float: The time in seconds, or negative infinity if no plan exists.
        """
        return getattr(self, "_plan_stamp", -1e9)

    def _go(self, target: str, next_state: str, t: float, reached_cb: Callable[[float], None]) -> tuple[float, float]:
        """Drives toward a known pillar, triggering a callback and advancing state upon arrival.
        
        Args:
            target (str): The pillar target identifier ("blue" or "yellow").
            next_state (str): The next state to transition to after reaching the target.
            t (float): Current simulation time in seconds.
            reached_cb (Callable[[float], None]): Callback function invoked upon arrival.
            
        Returns:
            tuple[float, float]: The `(v, w)` motion command.
        """
        pw = self.perception.pillar_world.get(target) if self.perception else None
        if pw is None:
            self.state = self.recovery_return = (MissionState.EXPLORE_BLUE
                if target == "blue" else MissionState.EXPLORE_YELLOW)
            return 0.0, 0.0

        # ------------------------------------------------------------- #
        # Arrival check -- SIMPLE and ground-truth, on purpose.
        #
        # pillar_world[target] is a running average of camera/depth pillar
        # detections; up close the colour blob can clip the camera frame and
        # bias that average, so pose_distance(pose, pw) can stay stuck above
        # PILLAR_REACH_DIST forever even though the robot is touching the
        # pillar (this was the root cause of the GO<->RECOVERY loop). Rather
        # than trying to fully de-bias perception, just trust the live lidar
        # for the arrival decision: once committed to this target, the only
        # thing that close ahead of the robot IS the pillar. So "reached" is
        # simply: the tracked position estimate says we're close, OR the
        # live lidar dead ahead already reads within PILLAR_TOUCH_DIST.
        # ------------------------------------------------------------- #
        reach_dist = C.PILLAR_REACH_DIST_BLUE if target == "blue" else C.PILLAR_REACH_DIST_YELLOW
        front = LP.front_clearance(self.scan_body, half_width=0.15)
        dist_to_pw = pose_distance(self.pose, pw)
        if dist_to_pw <= reach_dist or front < C.PILLAR_TOUCH_DIST:
            self._creep_target = None
            reached_cb(t)
            self.hardware.stop()
            self.path = []
            self.local_planner.reset()
            self.state = next_state
            return 0.0, 0.0

        # ------------------------------------------------------------- #
        # Final-approach OPEN-LOOP CREEP (bypasses DWA on purpose).
        #
        # Confirmed by log evidence: the DWA scorer can settle on v=0 a few
        # cm short of a perfectly valid, obstacle-free goal (goal 0.14m away,
        # 0.16m of path left, v=0.00, front=0.54 -- nothing physically
        # blocking it). Rather than keep tuning that scorer, once within
        # GO_CREEP_DIST of the pillar estimate, stop trusting it: drive
        # straight at a fixed speed with simple proportional heading
        # correction, bounded by GO_CREEP_MAX_S so it can never hang.
        # ------------------------------------------------------------- #
        if dist_to_pw <= C.GO_CREEP_DIST:
            if self._creep_target != target:
                self._creep_target = target
                self._creep_t0 = t
            if (t - self._creep_t0) > C.GO_CREEP_MAX_S:
                self.log.warning("%s: creep cap reached, forcing arrival (d2t=%.2fm)",
                                 target, dist_to_pw)
                self._creep_target = None
                reached_cb(t)
                self.hardware.stop()
                self.path = []
                self.local_planner.reset()
                self.state = next_state
                return 0.0, 0.0
            bearing = wrap_angle(math.atan2(pw[1] - self.pose[1], pw[0] - self.pose[0])
                                  - self.pose[2])
            w = max(-C.W_MAX, min(C.W_MAX, C.GO_CREEP_KW * bearing))
            return C.GO_CREEP_V, w
        self._creep_target = None

        # keep the goal/standoff fresh as the estimate refines
        goal = self._pillar_standoff(pw)
        if (self.goal_world is None or pose_distance(goal, self.goal_world) > 0.15
                or not self.path or (t - self._last_plan_t()) > C.REPLAN_PERIOD_S):
            path = self.plan_path_to(goal)
            if path is None:
                # blocked: fall back to exploring to open a route
                self.go_fail_until = t + C.GO_FAIL_COOLDOWN_S
                self.state = (MissionState.EXPLORE_BLUE if target == "blue"
                              else MissionState.EXPLORE_YELLOW)
                self.log.warning("no path to %s; exploring to open a route", target)
                self.events.event("go_blocked", level="WARNING", sim_time=t,
                                  tick=self.tick, state=self.state, target=target)
                return 0.0, 0.0
            self.path, self.goal_world = path, goal
            self._plan_stamp = t
        v, w, _near = self.drive_along_path(self.path, v_cap=C.V_CRUISE)
        return v, w

    def _on_blue_reached(self, t: float) -> None:
        """Records the BLUE-pillar arrival time and emits a milestone event.
        
        Args:
            t (float): Current simulation time in seconds.
        """
        self.t_blue = t
        self.log.info("BLUE pillar reached at t=%.2fs (start->blue = %.2fs)",
                      t, t - self.t_start)
        self.events.event("pillar_reached", sim_time=t, tick=self.tick,
                          state=self.state, pillar="blue",
                          split_s=round(t - self.t_start, 2))

    def _on_yellow_reached(self, t: float) -> None:
        """Records the YELLOW-pillar arrival time and emits a milestone event.
        
        Args:
            t (float): Current simulation time in seconds.
        """
        self.t_yellow = t
        self.log.info("YELLOW pillar reached at t=%.2fs (blue->yellow = %.2fs)",
                      t, t - self.t_blue)
        self.events.event("pillar_reached", sim_time=t, tick=self.tick,
                          state=self.state, pillar="yellow",
                          split_s=round(t - self.t_blue, 2))

    def step_mission(self, t: float) -> tuple[float, float]:
        """Advances the mission FSM by one tick and returns the necessary motion command.
        
        Args:
            t (float): Current simulation time in seconds.
            
        Returns:
            tuple[float, float]: The `(v, w)` command for the current state.
        """
        st = self.state
        if st == MissionState.INIT_SCAN:
            if self._prev_theta_for_spin is None:
                self._prev_theta_for_spin = self.pose[2]
            self.spin_accum += abs(wrap_angle(self.pose[2] - self._prev_theta_for_spin))
            self._prev_theta_for_spin = self.pose[2]
            if self.spin_accum >= C.INITIAL_SCAN_REVS * 2.0 * math.pi:
                self.state = MissionState.EXPLORE_BLUE
                self._reset_progress(t)
                self.log.info("initial scan complete -> EXPLORE_BLUE")
                self.events.event("state_transition", sim_time=t, tick=self.tick,
                                  state=MissionState.EXPLORE_BLUE, reason="init_scan_done")
                return 0.0, 0.0
            return 0.0, C.INIT_SPIN_W

        if st == MissionState.EXPLORE_BLUE:
            return self._explore("blue", MissionState.GO_BLUE, t)
        if st == MissionState.GO_BLUE:
            return self._go("blue", MissionState.EXPLORE_YELLOW, t, self._on_blue_reached)
        if st == MissionState.EXPLORE_YELLOW:
            return self._explore("yellow", MissionState.GO_YELLOW, t)
        if st == MissionState.GO_YELLOW:
            return self._go("yellow", MissionState.DONE, t, self._on_yellow_reached)
        if st == MissionState.RECOVERY:
            return self._run_recovery(t)
        # DONE
        return 0.0, 0.0

    # ===================================================================== #
    #  Visualisation / output
    # ===================================================================== #
    def _visualize(self, t: float, force: bool = False) -> None:
        """Renders the live map, costmap, and path overlay, periodically saving snapshots.
        
        Args:
            t (float): Current simulation time in seconds.
            force (bool, optional): If True, forces a render and snapshot irrespective of the period. Defaults to False.
        """
        snap = force or (t - self._last_snapshot_t) >= C.SNAPSHOT_PERIOD_S
        # render live at ~10 Hz (every 3rd tick); always render when snapping
        if not (snap or force or self.tick % 3 == 0):
            return
        pillars = {k: v for k, v in (self.perception.pillar_world.items()
                                     if self.perception else [])}
        img = self.visualizer.render(self.pose, self.frontier_mask, self.path,
                              self.carrot, pillars, self.state, t)
        self.visualizer.show(img)
        if snap:
            self.visualizer.save(img, os.path.join(self.outdir, "live_map.png"))
            self._last_snapshot_t = t

    def _log_timing(self) -> None:
        """Prints the start->blue->yellow timing metrics and logs them as telemetry events."""
        start_to_blue = (self.t_blue - self.t_start) if self.t_blue is not None else None
        blue_to_yellow = (self.t_yellow - self.t_blue
                          if (self.t_yellow is not None and self.t_blue is not None)
                          else None)
        total = (self.t_yellow - self.t_start
                 if (self.t_yellow is not None and self.t_start is not None) else None)
        self.log.info("TIMING  start->blue=%s  blue->yellow=%s  total=%s",
                      f"{start_to_blue:.2f}s" if start_to_blue is not None else "(n/a)",
                      f"{blue_to_yellow:.2f}s" if blue_to_yellow is not None else "(n/a)",
                      f"{total:.2f}s" if total is not None else "(n/a)")
        self.events.event("timing_table",
                          start_to_blue_s=round(start_to_blue, 2) if start_to_blue is not None else None,
                          blue_to_yellow_s=round(blue_to_yellow, 2) if blue_to_yellow is not None else None,
                          total_s=round(total, 2) if total is not None else None)

    def finalize(self) -> None:
        """Stops the robot and writes the final map, timing table, and accumulated telemetry to disk."""
        self.hardware.stop()
        self.hardware.step()
        self.refresh_costmap(self.hardware.time(), force=True)
        self._visualize(self.hardware.time(), force=True)
        try:
            self.visualizer.save(self.visualizer.render(self.pose, self.frontier_mask, self.path,
                          self.carrot, {k: v for k, v in (self.perception.pillar_world.items()
                          if self.perception else [])}, self.state, self.hardware.time()),
                          os.path.join(self.outdir, "final_map.png"))
            np.savez_compressed(os.path.join(self.outdir, "map.npz"),
                                L=self.occupancy_grid.L, poison=self.occupancy_grid.poison)
        except Exception as exc:
            self.log.error("finalize save error: %s", exc, exc_info=True)
            self.events.event("fault", level="ERROR", stage="finalize_save",
                              state=self.state, error=str(exc))
        self._log_timing()
        self.events.event("run_end", state=self.state)
        self.events.close()
        self.visualizer.close()

    # ===================================================================== #
    #  Main loop
    # ===================================================================== #
    def run(self) -> None:
        """Executes the full sense-plan-act loop until the mission concludes or 'Q' is pressed.

        Each pipeline stage is isolated by `guarded_stage`: an unexpected
        fault in one stage is logged (console + JSONL, with tick/state context)
        and the loop continues on the next tick with the last safe command,
        rather than aborting the whole mission on a single bad frame.
        """
        # prime sensors
        if self.hardware.step() == -1:
            return
        self.update_sensing()
        self.start_xy = (self.pose[0], self.pose[1])
        self.t_start = self.hardware.time()
        self.log.info("start pose (odom) = (%.2f, %.2f, %.1f deg)",
                      self.pose[0], self.pose[1], math.degrees(self.pose[2]))
        # config fingerprint: prove which build is actually running (rules out
        # a stale cached module being picked up instead of the latest edits).
        self.log.info(
            "config fingerprint: V_MAX=%.2f V_CRUISE=%.2f DWA_TIGHT_GAP_VFRAC=%.2f "
            "PILLAR_TOUCH_DIST=%.2f PILLAR_REACH_DIST_BLUE=%.2f file=%s",
            C.V_MAX, C.V_CRUISE, C.DWA_TIGHT_GAP_VFRAC, C.PILLAR_TOUCH_DIST,
            C.PILLAR_REACH_DIST_BLUE, os.path.abspath(C.__file__))
        self.events.event("run_start", sim_time=self.t_start, tick=0,
                          state=self.state, dt=self.dt,
                          start_xy=[round(self.pose[0], 3), round(self.pose[1], 3)],
                          devices=self.hardware.device_inventory())

        while self.hardware.step() != -1:
            self.tick += 1
            t = self.hardware.time()

            keys = self.hardware.poll_key()
            if ord('Q') in keys or ord('q') in keys:
                self.log.info("Q pressed -> finalize")
                self.events.event("operator_quit", sim_time=t, tick=self.tick,
                                  state=self.state)
                break

            ctx = dict(sim_time=t, tick=self.tick, state=self.state)
            with guarded_stage("sensing", self.log, self.events, **ctx):
                self.update_sensing()
            with guarded_stage("slam", self.log, self.events, **ctx):
                self.run_slam_step()
            if self.tick % C.PERCEPTION_EVERY_TICKS == 0:
                with guarded_stage("perception", self.log, self.events, **ctx):
                    self.update_perception()
            with guarded_stage("costmap", self.log, self.events, **ctx):
                self.refresh_costmap(t)
                # keep the robot's own cell plannable in the costmap snapshot
                self.occupancy_grid.mark_free_disc(self.pose[0], self.pose[1],
                                                   C.ROBOT_RADIUS * 0.8)

            v, w = 0.0, 0.0
            with guarded_stage("mission", self.log, self.events, **ctx):
                v, w = self.step_mission(t)

            # stuck check only while we expect to be moving forward
            driving = self.state in (MissionState.EXPLORE_BLUE, MissionState.GO_BLUE,
                                     MissionState.EXPLORE_YELLOW, MissionState.GO_YELLOW)
            if driving and (self._is_stuck(t, commanding_forward=(v > 0.03))
                            or self._is_frozen(t)):
                self._enter_recovery(t, self.state)
                v, w = 0.0, 0.0

            if self.green_block and v > 0.0:
                v = 0.0  # hard poison reflex safety net

            self.hardware.set_cmd(v, w)
            self.cur_v = v

            with guarded_stage("visualize", self.log, self.events, **ctx):
                self._visualize(t)
            if self.tick % C.LOG_EVERY_TICKS == 0:
                self._log_status(t)

            if self.state == MissionState.DONE:
                # linger briefly so the final frame is visible, then stop
                if not hasattr(self, "_done_t"):
                    self._done_t = t
                if t - self._done_t > C.DONE_LINGER_S:
                    break

        self.finalize()

    def _log_status(self, t: float) -> None:
        """Emits a periodic one-line status summary to the console and as a JSONL heartbeat.
        
        Args:
            t (float): Current simulation time in seconds.
        """
        occ = int(self.occupancy_grid.occupied_mask().sum())
        pois = int(self.occupancy_grid.poison.sum())
        pb = self.perception.pillar_world.get("blue") if self.perception else None
        py = self.perception.pillar_world.get("yellow") if self.perception else None
        # DIAGNOSTIC: when actively chasing a pillar, print the exact numbers the
        # arrival check (_go) is deciding on -- the live pillar_world estimate,
        # the distance derived from it, and the live front-lidar range -- so a
        # stall/recovery is provable from the log instead of guessed at.
        target = ("blue" if self.state == MissionState.GO_BLUE else
                  "yellow" if self.state == MissionState.GO_YELLOW else None)
        diag = ""
        if target is not None:
            pw = self.perception.pillar_world.get(target) if self.perception else None
            reach = C.PILLAR_REACH_DIST_BLUE if target == "blue" else C.PILLAR_REACH_DIST_YELLOW
            front = LP.front_clearance(self.scan_body, half_width=0.15)
            if pw is not None:
                d2t = pose_distance(self.pose, pw)
                diag = (f" tgt={target}@({pw[0]:+.2f},{pw[1]:+.2f}) d2t={d2t:.2f}"
                        f"/{reach:.2f} front={front:.2f}/{C.PILLAR_TOUCH_DIST:.2f}")
                # extra: what is the standoff GOAL/path actually targeting, and
                # is the pure-pursuit carrot considered "at the goal" already?
                # This distinguishes "DWA won't close the last stretch to a
                # correct nearby goal" from "the goal itself was never placed
                # close enough" -- the two remaining candidate root causes.
                if self.goal_world is not None:
                    d2g = pose_distance(self.pose, self.goal_world)
                    plen = self._path_length(self.path) if self.path else 0.0
                    diag += (f" goal=({self.goal_world[0]:+.2f},{self.goal_world[1]:+.2f})"
                             f" d2goal={d2g:.2f} pathlen={plen:.2f} npts={len(self.path)}")
                else:
                    diag += " goal=None"
            else:
                diag = f" tgt={target}@unknown front={front:.2f}/{C.PILLAR_TOUCH_DIST:.2f}"
        self.log.info(
            "t=%6.1f %-13s pose=(%+.2f,%+.2f,%+6.1f) v=%.2f occ=%d pois=%d "
            "blue=%s yellow=%s green_block=%s%s",
            t, self.state, self.pose[0], self.pose[1], math.degrees(self.pose[2]),
            self.cur_v, occ, pois, "Y" if pb else "-", "Y" if py else "-",
            "Y" if self.green_block else "-", diag)
        self.events.event("status", sim_time=t, tick=self.tick, state=self.state,
                          pose=[round(self.pose[0], 3), round(self.pose[1], 3),
                                round(self.pose[2], 3)], v=round(self.cur_v, 3),
                          occ=occ, poison=pois,
                          blue=pb is not None, yellow=py is not None,
                          green_block=self.green_block)


def main() -> None:
    """Constructs and runs the navigation controller, ensuring the robot is stopped on exit.

    Any exception that escapes the per-stage fault barriers is logged with a
    traceback and recorded to the event log; the robot is then stopped so a
    crashed controller never leaves the wheels driving.
    """
    log = get_logger(LOGGER_NAME)
    controller = None
    try:
        controller = NavigationController()
        controller.run()
    except Exception as exc:  # noqa: BLE001 - top-level safety net
        log.critical("unhandled controller error: %s", exc, exc_info=True)
        if controller is not None:
            try:
                controller.events.event("fault", level="CRITICAL", stage="top_level",
                                        error=f"{type(exc).__name__}: {exc}")
                controller.events.close()
            except Exception:  # noqa: BLE001 - telemetry must never re-raise here
                pass
            try:
                controller.hardware.stop()
            except Exception:  # noqa: BLE001 - best-effort stop on a failed run
                pass


if __name__ == "__main__":
    main()
