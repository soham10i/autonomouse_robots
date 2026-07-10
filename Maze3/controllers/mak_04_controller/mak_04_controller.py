"""mak_04_controller — Maze3 navigation: teleop_mapping mapping + mak_03 strategy.

A fusion controller.  It keeps the mak_03 NAVIGATION strategy unchanged --
visual-frontier exploration, A* global planning, a DWA local planner on the live
lidar, the persistent depth "aux" layer + confidence-gated poison, recovery, and
the BLUE->YELLOW mission FSM.  Mapping is kept teleop-crisp (raw-odom base,
L_OCC_THRESH=0.85) but MAP-CONSISTENT for long runs:

  * Lidar is integrated only after the robot has MOVED (moved_enough gate), so a
    stationary/pivoting robot does not re-stamp free-rays through nearby walls and
    erase them.
  * A CONSERVATIVE scan-matcher (C.SM_ENABLED) aligns each scan to the existing
    map before integration and accepts only confident, SMALL corrections, so walls
    stay crisp and slow odom drift is removed without the large-jump jitter that
    smeared earlier maps.  (Disabling it + integrating every tick let the map erase
    its own thin walls on long pivot-heavy runs -- the Maze1 looping bug.)

Mission (from Modularbeit.pdf): drive from the start to the BLUE pillar, then to
the YELLOW pillar, in the least simulation time, without touching walls or the
green poison floor.  No Webots Supervisor is used.

FSM:  INIT_SCAN -> EXPLORE_BLUE -> GO_BLUE -> EXPLORE_YELLOW -> GO_YELLOW -> DONE
      (RECOVERY is reachable from any driving state when progress stalls.  The
       binary green-poison reflex was REMOVED: poison avoidance is entirely
       map-based now -- depth-validated green projection feeds an accurate poison
       layer that A* + the DWA costmap route around.)

Run: set the Rosbot node's ``controller`` field to ``mak_04_controller`` (already
set in Maze3.wbt).  Press 'Q' in the sim to finalise outputs early.
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
from observability import get_logger, RunEventLog, guarded_stage
from odometry import Odometry
from perception import Perception
from robot_io import RobotInterface
from viz import Visualizer

#: Logger name for this maze's controller (shared observability convention).
LOGGER_NAME = "navctl.maze3"


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

        Missing optional sensors (lidar/camera/depth) degrade gracefully with a
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

        dspecs = self.hardware.depth_specs()
        if dspecs is not None:
            dw, dh, dfov, dmin, dmax = dspecs
            self.depth_model = DepthModel(dw, dh, dfov, depth_min=dmin, depth_max=dmax)
        else:
            self.depth_model = None
            self.log.warning("no depth camera found — low floating panels rely "
                             "on the IR bumper only")

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
        self._no_frontier_strikes = 0  # consecutive starved spin windows (escape escalation)
        self._escape_until = 0.0       # sim time until which the reactive escape drive runs
        self.cur_v = 0.0
        self.raw_lidar_ranges = None

        # stuck / recovery
        self._progress_ref = None
        self._progress_ref_t = 0.0
        self._frozen_ref = None
        self._frozen_t = 0.0
        self.recovery_phase = None
        self.recovery_t0 = 0.0
        self.recovery_return = MissionState.EXPLORE_BLUE
        self.recovery_chain = 0

        # timing table
        self.t_start = None
        self.t_blue = None
        self.t_yellow = None

        self.tick = 0
        self._last_snapshot_t = -1e9
        self.green_block = False     # retained False: the binary reflex was removed;
                                     # poison avoidance is entirely map-based now
        self.ir_block = False

    # ===================================================================== #
    #  Sensing & localisation
    # ===================================================================== #
    def update_sensing(self):
        """Read wheel/IMU/lidar and advance the pose belief by one increment."""
        wl, wr = self.hardware.read_encoders()
        yaw = self.hardware.read_yaw()
        self.odometry.update(wl, wr, yaw, self.dt)
        raw = self.odometry.pose()
        if self._last_raw is None:
            self.pose = raw
            self._last_raw = raw
        else:
            inc = relative_pose(self._last_raw, raw)
            if self._is_wheel_slip(inc):
                inc = (0.0, 0.0, inc[2])  # heading is IMU-sourced (trustworthy);
                                          # only the encoder-derived translation
                                          # is dropped -- see _is_wheel_slip.
            self.pose = compose_pose(self.pose, inc)
            self._last_raw = raw

        if self.lidar is not None:
            self.raw_lidar_ranges = self.hardware.read_lidar_ranges()
            if self.raw_lidar_ranges is not None:
                self.scan_body, self.scan_ranges = self.lidar.ranges_to_body(self.raw_lidar_ranges)

    def _is_wheel_slip(self, inc):
        """True if this tick's encoder-reported translation is almost certainly
        slip (wheels turning while pinned against an obstacle) rather than real
        motion, per live lidar clearance in the commanded direction of travel.

        Wheel encoders cannot distinguish "driving forward" from "wheels
        spinning against a wall" -- they report the same delta either way. The
        IMU-sourced heading is immune to this (Odometry.update takes yaw
        straight from the inertial unit), but x/y translation is pure
        differential-drive dead-reckoning and will happily integrate a phantom
        few cm/tick forever while wedged. That phantom translation then fools
        _has_moved_enough() into re-running scan-match + integrate_scan against
        a predicted pose that never actually advanced, which is what smears/
        drifts the map during a stuck episode (the log's "map got drifted").
        This is a single-tick, sensor-grounded check -- much faster than the
        multi-second _is_stuck()/_is_frozen() watchdogs, which exist to trigger
        RECOVERY, not to protect the pose belief itself.
        """
        dx, dy, _ = inc
        if math.hypot(dx, dy) < 1e-4 or self.scan_body is None or self.scan_body.shape[0] == 0:
            return False
        if self.cur_v > C.SLIP_MIN_CMD_V and LP.front_clearance(self.scan_body, 0.16) < C.SLIP_CLEAR_DIST:
            return True
        if self.cur_v < -C.SLIP_MIN_CMD_V and LP.rear_clearance(self.scan_body, 0.18) < C.SLIP_CLEAR_DIST:
            return True
        return False

    def _has_moved_enough(self):
        """True once the robot has moved/turned enough to re-integrate a scan."""
        if self._pose_at_last_integrate is None:
            return True
        dx, dy, dth = relative_pose(self._pose_at_last_integrate, self.pose)
        return (math.hypot(dx, dy) > C.SM_MIN_TRAVEL_M or
                abs(dth) > C.SM_MIN_TURN_RAD)

    def run_slam_step(self):
        """Map-consistent SLAM step: gated integration + conservative scan-match.

        Reverses the two "teleop-parity" choices that let the map erase its own
        walls on long, pivot-heavy runs (Maze1 looping bug; latent here too):
          * MOVED_ENOUGH GATE restored -- the scan is integrated only after the
            robot has actually moved (C.SM_MIN_TRAVEL_M / C.SM_MIN_TURN_RAD), so a
            stationary/pivoting robot no longer re-stamps free-rays through nearby
            walls every tick and clears them (that pivot-erosion flooded the aux
            layer and made the costmap oscillate).
          * SCAN-MATCH re-enabled (C.SM_ENABLED) and CONSERVATIVE -- each scan is
            aligned to the existing map before integration, so beams land back on
            the SAME wall cells (reinforcing them) instead of grazing past and
            free-clearing them, and slow odometry drift is corrected.  The matcher
            accepts ONLY confident, SMALL corrections (see mapping.scan_match), so
            it de-drifts without the large-jump jitter that smeared earlier maps.
        """
        if self.scan_body.shape[0] == 0 or not self._has_moved_enough():
            return
        if C.SM_ENABLED:
            corrected, hit = self.occupancy_grid.scan_match(self.pose, self.scan_body)
            self.pose = corrected
        self.occupancy_grid.integrate_scan(self.pose, self.scan_body, self.scan_ranges)
        self.occupancy_grid.mark_free_disc(self.pose[0], self.pose[1], C.ROBOT_RADIUS * 0.8)
        self._pose_at_last_integrate = self.pose

    def update_perception(self):
        """Detect pillars and the green poison floor from the RGB(-D) camera."""
        if self.perception is None:
            return
        bgr = self.hardware.read_rgb_bgr()
        if bgr is None:
            return
        depth = self.hardware.read_depth()
        self.perception.update_pillars(bgr, depth, self.pose)
        # Depth-validated green projection -> an ACCURATE poison footprint.  The
        # binary green_block reflex is gone (see config); poison avoidance now
        # lives entirely in the mapped poison layer + A*/DWA costmap.
        green_world = self.perception.green_floor_world(bgr, depth, self.pose)
        self.occupancy_grid.add_poison_points(green_world)

    def _update_depth_aux(self):
        """Update the PERSISTENT depth-obstacle (aux) layer from the depth image.

        In-band hits reinforce a per-cell confidence where the 2-D lidar is
        blind; clear sight-lines only DECAY it, and the decay never reaches
        inside the < 0.6 m depth blind shell -- so a floating wall stays mapped
        through the dead zone where neither sensor can see it (the floating-wall
        fix).  A gentle global decay bounds the layer so it cannot smear shut.
        See mapping.OccupancyGrid.integrate_depth_obstacles.
        """
        if self.depth_model is None:
            return
        depth = self.hardware.read_depth()
        if depth is None:
            return
        bearings, hit_ranges, hit_mask, clear_mask = self.depth_model.thin_scan(depth)
        self.occupancy_grid.integrate_depth_obstacles(self.pose, bearings, hit_ranges,
                                            hit_mask, clear_mask, self.depth_model.depth_min)

    def _update_ir_bumper(self):
        """Close-range chassis-IR hard-stop, independent of mapping (last resort
        for the lowest panel, WallShort(15), which sits below where the depth
        camera reliably resolves at very close range)."""
        if not C.IR_BUMPER_ENABLED:
            self.ir_block = False
            return
        ir = self.hardware.read_ir_m()
        if not ir:
            self.ir_block = False
            return
        vals = [v for v in ir.values() if np.isfinite(v)]
        self.ir_block = bool(vals) and min(vals) < C.IR_BUMPER_STOP_DIST

    # ===================================================================== #
    #  Planning
    # ===================================================================== #
    def refresh_costmap(self, t, force=False):
        """Rebuild the A* costmap when it is stale (or when ``force`` is set)."""
        if force or self.cost is None or (t - self._last_costmap_t) >= C.REPLAN_PERIOD_S:
            self.cost, self.lethal = self.occupancy_grid.build_costmap()
            # ---- anti-self-boxing: the robot IS here, so clear its disc ----
            # The lethal inflation can make the robot's own cell and all
            # neighbours impassable (e.g. corridor walls within HARD_OBS_DIST
            # on both sides).  When that happens A* cannot even START and
            # select_frontier_goal() returns nothing → permanent spin deadlock.
            # Fix: force a small disc around the robot to be non-lethal with
            # finite (high) cost.  The DWA local planner still uses live lidar
            # for real wall clearance, so safety is preserved.
            cix, ciy = self.occupancy_grid.world_to_grid(self.pose[0], self.pose[1])
            # Base ~0.12 m disc, widened when the robot has been frontier-starved
            # for a while -- a false-positive poison/aux ring just past the base
            # disc can box A* in completely with no escape (see config note on
            # NO_FRONTIER_MAX_STRIKES).
            box_m = min(0.12 + self._no_frontier_strikes * C.BOX_CLEAR_STEP_M,
                       C.BOX_CLEAR_MAX_M)
            r = max(3, int(round(box_m / self.occupancy_grid.res)))  # res-independent
            n = self.occupancy_grid.n
            x0, x1 = max(0, cix - r), min(n, cix + r + 1)
            y0, y1 = max(0, ciy - r), min(n, ciy + r + 1)
            self.lethal[x0:x1, y0:y1] = False
            sub = self.cost[x0:x1, y0:y1]
            np.minimum(sub, C.COST_OBS_WEIGHT * 0.5, out=sub)  # capped, not free
            self._last_costmap_t = t

    def plan_path_to(self, goal_world):
        """Plan a simplified A* world-space path to the goal; ``None`` if unreachable."""
        if self.cost is None:
            return None
        start = self.occupancy_grid.world_to_grid(self.pose[0], self.pose[1])
        goal = self.occupancy_grid.world_to_grid(goal_world[0], goal_world[1])
        cells = astar.plan(self.cost, self.lethal, start, goal)
        if cells is None:
            return None
        cells = astar.simplify(cells, self.lethal, self.cost)
        return [self.occupancy_grid.grid_to_world(ix, iy) for (ix, iy) in cells]

    def _is_blacklisted(self, wxy):
        """True if the world point is near a previously-abandoned (unreachable) goal."""
        for bx, by in self.blacklist:
            if math.hypot(wxy[0] - bx, wxy[1] - by) < C.FRONTIER_BLACKLIST_R:
                return True
        return False

    def select_frontier_goal(self):
        """Pick the best reachable frontier; return (path, goal_world) or (None,None)."""
        self.frontier_mask = FR.detect_frontier_cells(self.occupancy_grid, self.lethal)
        extra_r = min(self._no_frontier_strikes * C.EXPL_RADIUS_RELAX_STEP_M,
                     C.EXPL_RADIUS_RELAX_MAX_M)
        cands = FR.find_frontiers(self.occupancy_grid, self.lethal, self.pose[:2],
                                  self.start_xy, extra_radius=extra_r)
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
    def _path_length(path):
        """Total Euclidean length of a world-space polyline path."""
        return sum(math.hypot(path[i + 1][0] - path[i][0], path[i + 1][1] - path[i][1])
                   for i in range(len(path) - 1))

    # ===================================================================== #
    #  Driving
    # ===================================================================== #
    def _mapped_obstacles_body(self):
        """Mapped poison + aux (low-panel) points near the robot, body frame.

        Folded into one array for the DWA local planner — both are static,
        mapped obstacle sources the live lidar can't (always) see directly.
        """
        near_r = C.DWA_SLOWDOWN_DIST + 0.4
        pw = self.occupancy_grid.poison_points_near(self.pose[0], self.pose[1], near_r)
        aw = self.occupancy_grid.aux_points_near(self.pose[0], self.pose[1], near_r)
        parts = [a for a in (pw, aw) if a.shape[0] > 0]
        if not parts:
            return np.empty((0, 2))
        world = np.concatenate(parts, axis=0)
        return inverse_transform_points(world, self.pose[0], self.pose[1], self.pose[2])

    def drive_along_path(self, path, v_cap=None):
        """Carrot + DWA toward the end of ``path``; returns (v, w, near_goal)."""
        if not path:
            return 0.0, 0.0, True
        carrot, near_goal = LP.choose_carrot(path, self.pose, self.cur_v)
        self.carrot = carrot
        extra_b = self._mapped_obstacles_body()
        v, w = self.local_planner.compute(self.pose, carrot, self.scan_body, extra_b,
                                green_block=False, v_cap=v_cap)
        return v, w, near_goal

    # ===================================================================== #
    #  Stuck detection & recovery
    # ===================================================================== #
    def _reset_progress(self, t):
        """Reset the stuck-watchdog reference to the current pose and time."""
        self._progress_ref = (self.pose[0], self.pose[1])
        self._progress_ref_t = t

    def _is_stuck(self, t):
        """Pure POSITION watchdog: no translation for STUCK_TIMEOUT_S in a
        driving state means stuck, regardless of what is currently commanded.

        Maze1's mak_03_controller documents this exact bug under the same
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
            self._no_frontier_strikes = 0
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

    def _run_recovery(self, t):
        """Execute the reverse-then-spin recovery manoeuvre; return ``(v, w)``."""
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
        if self.recovery_chain >= C.RECOVERY_MAX_CHAIN:
            if self.goal_world is not None:
                self.blacklist.append(self.goal_world)   # give up on this goal
            else:
                # No goal to blacklist means recovery kept firing during a
                # frontier-starved spin (goal_world is None there) -- the old
                # code silently did nothing in this case and the spin/recover
                # loop repeated forever. Jump straight to max escalation so
                # _explore()'s escape drive fires on the very next tick
                # instead of waiting out more rescan windows.
                self._no_frontier_strikes = C.NO_FRONTIER_MAX_STRIKES
                self._escape_until = t + C.ESCAPE_DRIVE_T
            self.recovery_chain = 0
        self.path = []
        self.goal_world = None
        self._reset_progress(t)
        self.state = self.recovery_return
        return 0.0, 0.0

    # ===================================================================== #
    #  MissionState state machine
    # ===================================================================== #
    def _try_go_to_target(self, target, go_state, t):
        """Switch EXPLORE->GO iff the pillar is known AND an A* path exists now."""
        pw = self.perception.pillar_world.get(target) if self.perception else None
        if pw is None or t < self.go_fail_until:
            return False
        # throttle the reachability A* so we don't run it every 32 ms tick
        if (t - getattr(self, "_last_go_try", -1e9)) < 0.4:
            return False
        self._last_go_try = t
        goal = pw
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

    def _explore(self, target, go_state, t):
        """Explore toward the target: commit to it if seen, else pick a frontier."""
        if self._try_go_to_target(target, go_state, t):
            return self.drive_along_path(self.path)[:2]
        # (re)select a frontier goal when needed
        need_new = (not self.path or self.goal_world is None or
                    pose_distance(self.pose, self.goal_world) < C.GOAL_REACH_TOL or
                    (t - self._last_plan_t()) > C.REPLAN_PERIOD_S)
        if need_new:
            # force a fresh costmap before frontier selection so we plan on the
            # most up-to-date data (also re-runs the anti-self-boxing disc)
            self.refresh_costmap(t, force=True)
            path, goal = self.select_frontier_goal()
            if path is not None:
                self.path, self.goal_world = path, goal
                self._plan_stamp = t
                self._no_frontier_strikes = 0
            else:
                # nothing to explore: rescan in place for a while. Escalate
                # ONCE per rescan window (not every tick -- this branch runs
                # every tick while starved) so the search radius / anti-boxing
                # disc widen at a bounded rate instead of jumping to max
                # instantly (see config.NO_FRONTIER_MAX_STRIKES).
                if t >= self.no_frontier_until:
                    self.no_frontier_until = t + C.NO_FRONTIER_SPIN_S
                    if self._no_frontier_strikes < C.NO_FRONTIER_MAX_STRIKES:
                        self._no_frontier_strikes += 1
                    if self._no_frontier_strikes >= C.NO_FRONTIER_MAX_STRIKES:
                        self._escape_until = t + C.ESCAPE_DRIVE_T
                self.path, self.goal_world = [], None
                if t < self._escape_until:
                    return self._escape_drive()
                return 0.0, C.INIT_SPIN_W
        v, w, _ = self.drive_along_path(self.path)
        return v, w

    def _escape_drive(self):
        """Reactive, map-independent escape burst: last resort once frontier
        search has been starved for C.NO_FRONTIER_MAX_STRIKES rescan windows.

        Drives on LIVE lidar only (front/side clearance), bypassing the mapped
        costmap/frontier logic that produced the starvation in the first
        place -- guarantees the robot physically relocates instead of
        spinning in the same spot forever.
        """
        front = LP.front_clearance(self.scan_body, 0.16)
        side = LP.freer_side(self.scan_body)
        if front > C.ESCAPE_MIN_FRONT_CLEAR:
            return C.ESCAPE_DRIVE_V, side * C.ESCAPE_STEER_W * 0.3
        return 0.0, side * C.ESCAPE_STEER_W

    def _last_plan_t(self):
        """Simulation time of the most recent successful plan (or ``-inf``)."""
        return getattr(self, "_plan_stamp", -1e9)

    def _go(self, target, next_state, t, reached_cb):
        """Drive to a known pillar; on arrival fire the callback and advance the FSM."""
        pw = self.perception.pillar_world.get(target) if self.perception else None
        if pw is None:
            self.state = self.recovery_return = (MissionState.EXPLORE_BLUE
                if target == "blue" else MissionState.EXPLORE_YELLOW)
            return 0.0, 0.0
        # arrival check
        if pose_distance(self.pose, pw) <= C.PILLAR_REACH_DIST:
            reached_cb(t)
            self.hardware.stop()
            self.path = []
            self.local_planner.reset()
            self._reset_progress(t)   # don't inherit stale stuck timer into next state
            self.state = next_state
            return 0.0, 0.0
        # keep the goal fresh as the estimate refines
        goal = pw
        if (self.goal_world is None or pose_distance(goal, self.goal_world) > 0.15
                or not self.path or (t - self._last_plan_t()) > C.REPLAN_PERIOD_S):
            path = self.plan_path_to(goal)
            if path is None:
                # Force costmap rebuild and retry once before giving up
                self.refresh_costmap(t, force=True)
                path = self.plan_path_to(goal)
            if path is None:
                # truly blocked: fall back to exploring to open a route
                self.go_fail_until = t + C.GO_FAIL_COOLDOWN_S
                self.state = (MissionState.EXPLORE_BLUE if target == "blue"
                              else MissionState.EXPLORE_YELLOW)
                self.log.warning("no path to %s; exploring to open a route", target)
                self.events.event("go_blocked", level="WARNING", sim_time=t,
                                  tick=self.tick, state=self.state, target=target)
                return 0.0, 0.0
            self.path, self.goal_world = path, goal
            self._plan_stamp = t
        v, w, near = self.drive_along_path(self.path, v_cap=C.V_CRUISE)
        return v, w

    def _on_blue_reached(self, t):
        """Record the BLUE-pillar arrival time and emit a milestone event."""
        self.t_blue = t
        self.log.info("BLUE pillar reached at t=%.2fs (start->blue = %.2fs)",
                      t, t - self.t_start)
        self.events.event("pillar_reached", sim_time=t, tick=self.tick,
                          state=self.state, pillar="blue",
                          split_s=round(t - self.t_start, 2))

    def _on_yellow_reached(self, t):
        """Record the YELLOW-pillar arrival time and emit a milestone event."""
        self.t_yellow = t
        self.log.info("YELLOW pillar reached at t=%.2fs (blue->yellow = %.2fs)",
                      t, t - self.t_blue)
        self.events.event("pillar_reached", sim_time=t, tick=self.tick,
                          state=self.state, pillar="yellow",
                          split_s=round(t - self.t_blue, 2))

    def step_mission(self, t):
        """Advance the mission FSM by one tick and return the ``(v, w)`` command."""
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
    def _visualize(self, t, force=False):
        """Render the live map/costmap/path overlay and periodically snapshot it."""
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

    def _log_timing(self):
        """Print the start->blue->yellow timing table and log it as an event."""
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

    def finalize(self):
        """Stop the robot and write the final map, timing table, and telemetry."""
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
                                L=self.occupancy_grid.L, poison=self.occupancy_grid.poison, aux=self.occupancy_grid.aux)
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
    def run(self):
        """Run the full sense-plan-act loop until the mission ends or 'Q' is hit.

        Each pipeline stage is isolated by :func:`guarded_stage`: an unexpected
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
            if self.tick % C.DEPTH_AUX_EVERY_TICKS == 0:
                prev_aux_hits = int(self.occupancy_grid.aux.sum())
                with guarded_stage("depth_aux", self.log, self.events, **ctx):
                    self._update_depth_aux()
                new_aux_hits = int(self.occupancy_grid.aux.sum())
                if new_aux_hits > prev_aux_hits and self.path:
                    # A floating wall just confirmed (crossed AUX_MIN_HITS)
                    # while we're committed to a path -- force an immediate A*
                    # replan through the now-updated, wider-margin costmap
                    # instead of waiting out REPLAN_PERIOD_S. Without this the
                    # planner only reacts to a newly-detected floating wall
                    # once DWA/recovery notices it live, by which point the
                    # robot may already be grazing it.
                    self._last_costmap_t = -1e9
                    self._plan_stamp = -1e9
                    self.events.event("aux_confirmed_replan", sim_time=t,
                                      tick=self.tick, state=self.state,
                                      aux_cells=new_aux_hits)
            with guarded_stage("ir_bumper", self.log, self.events, **ctx):
                self._update_ir_bumper()
            with guarded_stage("costmap", self.log, self.events, **ctx):
                self.refresh_costmap(t)
                # keep the robot's own cell plannable in the costmap snapshot
                self.occupancy_grid.mark_free_disc(self.pose[0], self.pose[1],
                                                   C.ROBOT_RADIUS * 0.8)

            v, w = 0.0, 0.0
            with guarded_stage("mission", self.log, self.events, **ctx):
                v, w = self.step_mission(t)

            # position watchdog applies in every driving state, REGARDLESS of
            # what is currently commanded (see _is_stuck docstring) -- this is
            # what catches the no-reachable-frontier rescan spin (v=0, w!=0).
            driving = self.state in (MissionState.EXPLORE_BLUE, MissionState.GO_BLUE,
                                     MissionState.EXPLORE_YELLOW, MissionState.GO_YELLOW)
            if driving and (self._is_stuck(t) or self._is_frozen(t)):
                self._enter_recovery(t, self.state)
                v, w = 0.0, 0.0

            if self.ir_block and v > 0.0:
                v = 0.0  # hard chassis-IR bumper safety net (low floating panels)

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

    def _log_status(self, t):
        """Emit a periodic one-line status summary (console + a JSONL heartbeat)."""
        occ = int(self.occupancy_grid.occupied_mask().sum())
        pois = int(self.occupancy_grid.poison.sum())
        aux = int(self.occupancy_grid.aux.sum())
        pb = self.perception.pillar_world.get("blue") if self.perception else None
        py = self.perception.pillar_world.get("yellow") if self.perception else None
        self.log.info(
            "t=%6.1f %-13s pose=(%+.2f,%+.2f,%+6.1f) v=%.2f occ=%d pois=%d aux=%d "
            "blue=%s yellow=%s ir_block=%s",
            t, self.state, self.pose[0], self.pose[1], math.degrees(self.pose[2]),
            self.cur_v, occ, pois, aux, "Y" if pb else "-", "Y" if py else "-",
            "Y" if self.ir_block else "-")
        self.events.event("status", sim_time=t, tick=self.tick, state=self.state,
                          pose=[round(self.pose[0], 3), round(self.pose[1], 3),
                                round(self.pose[2], 3)], v=round(self.cur_v, 3),
                          occ=occ, poison=pois, aux=aux,
                          blue=pb is not None, yellow=py is not None,
                          ir_block=self.ir_block)


def main():
    """Construct and run the controller, guaranteeing the robot is stopped.

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
