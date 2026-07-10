"""return_path.py — remembers the route driven to the BLUE pillar and, once
triggered, retraces it by threading the recorded breadcrumbs as ORDERED A*
SUB-GOALS on the caller's live costmap.

Why not just steer the raw recorded polyline?  Because the breadcrumbs are
dead-reckoned pose samples: they drift out of alignment with the map, and the
map itself keeps updating (scan-match corrections + persistent aux/floating-wall
accumulation) after they were laid.  Replaying the raw line therefore routes the
robot through cells that are now costmap-LETHAL (measured at ~0.007 m from walls
at the blue-exit pinch) -- cells the DWA is designed to refuse -- so the robot
wedges at the first wall and never leaves blue.  Every state that DOES traverse
tight Maze1/Maze2/Maze4 corridors (GO_BLUE / GO_YELLOW) navigates via A* on the
live costmap, which never enters a lethal cell and rides the medial-axis centre
of a gap.  So the retrace does the same: it keeps the breadcrumb ORDER (the
corridor to follow back) but lets A* thread it safely.

This module owns the retrace POLICY (breadcrumb recording, the retrace cursor,
sub-goal selection, replan cadence, done detection).  It stays free of any
grid / costmap / astar import: the caller injects a ``plan_fn(goal_xy) -> path``
that runs A* from the robot's current pose to a goal on the current costmap.
Driving the returned path is still the caller's carrot + DWA, so live obstacle
clearance is unchanged.
"""
from __future__ import annotations

import math


def _wrap(a):
    return math.atan2(math.sin(a), math.cos(a))


class ReturnPathFollower:
    def __init__(self, crumb_spacing=0.15, trim_dist=0.30, goal_tol=0.16,
                 subgoal_lookahead=0.8, replan_period=1.0,
                 align_lookahead=1.0, align_enter_deg=55.0, align_exit_deg=20.0,
                 turn_kp=2.5, turn_w_max=2.2):
        self.crumb_spacing = crumb_spacing        # m between recorded breadcrumbs
        self.trim_dist = trim_dist                # m: advance the cursor past a reached waypoint
        self.goal_tol = goal_tol                  # m: "reached the final breadcrumb" tolerance
        self.subgoal_lookahead = subgoal_lookahead  # m arc-length ahead to aim A* each cycle
        self.replan_period = replan_period        # s between A* replans (GO_*-style cadence)
        # heading-alignment (U-turn) params: rotate toward a FAR, stable point on
        # the path when the required turn is big, instead of chasing a close
        # pure-pursuit carrot that swings across the nose and oscillates.
        self.align_lookahead = align_lookahead    # m along the path to aim the turn at
        self.align_enter_deg = align_enter_deg    # start aligning above this heading error
        self.align_exit_deg = align_exit_deg      # stop aligning below this (hysteresis)
        self.turn_kp = turn_kp                    # proportional gain on heading error
        self.turn_w_max = turn_w_max              # rad/s cap on the in-place turn rate
        self._breadcrumbs = []
        self._return_path = []
        self.active = False
        # retrace state (reset at trigger)
        self._idx = 0
        self._cur_path = []
        self._cur_goal = None
        self._last_plan_t = -1e9
        self._aligning = False

    # ------------------------------------------------------------------ record
    def record(self, x, y):
        """Call every tick BEFORE the trigger; drops a waypoint every
        ``crumb_spacing`` metres of actual travel.  No-op once triggered, so it
        cannot affect anything up to (and including) reaching the blue pillar."""
        if self.active:
            return
        if (not self._breadcrumbs or
                math.hypot(x - self._breadcrumbs[-1][0],
                           y - self._breadcrumbs[-1][1]) >= self.crumb_spacing):
            self._breadcrumbs.append((x, y))

    def trigger(self):
        """Call once, when the blue pillar is confirmed reached.  Freezes the
        recorded route, reverses it (blue -> start), and arms the retrace."""
        self._return_path = list(reversed(self._breadcrumbs))
        self.active = True
        self._idx = 0
        self._cur_path = []
        self._cur_goal = None
        self._last_plan_t = -1e9
        self._aligning = False
        return list(self._return_path)

    def path(self):
        return list(self._return_path)

    # ------------------------------------------------------------------ retrace
    def _subgoal(self, idx):
        """The breadcrumb ~``subgoal_lookahead`` metres of arc-length ahead of the
        cursor -- a meaningful A* target rather than the next 0.15 m crumb."""
        route = self._return_path
        acc = 0.0
        j = idx
        while j < len(route) - 1:
            acc += math.hypot(route[j + 1][0] - route[j][0],
                              route[j + 1][1] - route[j][1])
            j += 1
            if acc >= self.subgoal_lookahead:
                break
        return route[j], j

    def plan_step(self, pose, plan_fn, t):
        """Advance the retrace by one control cycle.

        ``plan_fn(goal_xy) -> path_world | None`` must run A* from the robot's
        current pose to ``goal_xy`` on the live costmap and return a world-point
        path (or None if unreachable).

        Returns ``(path_world, done)``: the centred A* path to drive this cycle
        (possibly the last good one, or [] if nothing could be planned yet), and
        ``done`` once the whole route has been retraced.
        """
        px, py = pose[0], pose[1]
        route = self._return_path
        if not route:
            return [], True

        # advance the retrace cursor past breadcrumbs already reached
        while (self._idx < len(route) - 1 and
               math.hypot(px - route[self._idx][0],
                          py - route[self._idx][1]) < self.trim_dist):
            self._idx += 1

        # whole route retraced?
        if (self._idx >= len(route) - 1 and
                math.hypot(px - route[-1][0], py - route[-1][1]) < self.goal_tol):
            return [], True

        # sub-goal a fixed arc-length ahead, A*-planned on the GO_*-style cadence
        subgoal, sub_idx = self._subgoal(self._idx)
        need = (not self._cur_path or self._cur_goal is None
                or math.hypot(subgoal[0] - self._cur_goal[0],
                              subgoal[1] - self._cur_goal[1]) > 0.15
                or (t - self._last_plan_t) > self.replan_period)
        if need:
            path = plan_fn(subgoal)
            if path is None:
                # the near sub-goal may sit in a now-blocked cell; try farther
                # breadcrumbs before giving up (caller's watchdog handles a true
                # dead end)
                for far in range(sub_idx + 1, len(route)):
                    path = plan_fn(route[far])
                    if path is not None:
                        break
            if path is not None:
                self._cur_path = path
                self._cur_goal = subgoal
                self._last_plan_t = t

        return list(self._cur_path), False

    # ------------------------------------------------------------- U-turn assist
    def _point_along(self, path, dist):
        """The point ``dist`` metres of arc-length along ``path`` from its start
        (clamped to the end).  A FAR, stable heading target -- unlike a close
        pure-pursuit carrot it barely moves as the robot rotates."""
        if not path:
            return None
        if len(path) == 1:
            return path[0]
        acc = 0.0
        for i in range(len(path) - 1):
            ax, ay = path[i]
            bx, by = path[i + 1]
            seg = math.hypot(bx - ax, by - ay)
            if acc + seg >= dist:
                f = (dist - acc) / seg if seg > 1e-9 else 0.0
                return (ax + (bx - ax) * f, ay + (by - ay) * f)
            acc += seg
        return path[-1]

    def turn_command(self, pose):
        """Stable in-place rotation for a BIG heading change (the ~180 deg U-turn
        the robot must make at the blue pillar before retracing).

        Returns ``(v, w, active)``.  ``active`` True means the caller should apply
        this (v=0, w) rotation toward a far point on the path INSTEAD of the
        carrot+DWA drive; the far target keeps its bearing steady through the turn
        so the robot rotates once and settles, rather than oscillating against a
        close carrot that swings across its nose.  Hysteresis (enter/exit angles)
        prevents chatter, and it only engages on sharp turns -- gentle bends drive
        through normally.
        """
        path = self._cur_path
        target = self._point_along(path, self.align_lookahead)
        if target is None:
            self._aligning = False
            return 0.0, 0.0, False
        px, py, pth = pose
        err = _wrap(math.atan2(target[1] - py, target[0] - px) - pth)
        err_deg = abs(math.degrees(err))
        if self._aligning:
            if err_deg < self.align_exit_deg:
                self._aligning = False
                return 0.0, 0.0, False
        elif err_deg < self.align_enter_deg:
            return 0.0, 0.0, False
        else:
            self._aligning = True
        w = max(-self.turn_w_max, min(self.turn_w_max, self.turn_kp * err))
        return 0.0, w, True
