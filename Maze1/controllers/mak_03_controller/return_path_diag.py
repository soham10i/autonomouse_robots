"""return_path_diag.py — observation-only tracker for the RETURN_PATH state.

Records what the robot is doing while it retraces the breadcrumb route back
from the BLUE pillar (return_path.py) and writes a structured per-tick log
plus a stall report.  This module reads state the caller already computed
and never returns a v/w or touches the path -- it cannot change driving
behaviour, only describe it, so it is safe to run every time without risking
a repeat of the earlier "static-overlay" fix that changed real behaviour and
had to be reverted.

Output (per run, overwritten): <outdir>/diag/return_path.log (CSV) and
<outdir>/diag/return_path_route.csv (the planned waypoints, dumped once at
trigger time so the retrace target itself can be inspected/plotted).

How to read a stall from the log: filter to rows where cur_v stays under
RETURN_DIAG_STALL_V for several consecutive rows: the min_lidar_ahead /
nearest_aux / nearest_poison columns at that point say WHAT the robot
believes is blocking it, and carrot_dist says whether the carrot itself is
sitting somewhere unreachable (e.g. beyond a wall) rather than the DWA
genuinely finding no safe velocity.
"""
from __future__ import annotations

import math
import os


class ReturnPathTracker:
    def __init__(self, outdir, stall_v=0.05, stall_s=5.0, log_every_ticks=8,
                 near_obs_r=0.6, fwd_cone_deg=45.0):
        self.stall_v = stall_v
        self.stall_s = stall_s
        self.log_every_ticks = log_every_ticks
        self.near_obs_r = near_obs_r
        self.fwd_cone_rad = math.radians(fwd_cone_deg)

        self._tick = 0
        self._stall_since = None
        self._stall_flagged = False   # already printed for the CURRENT stall run
        self.stall_events = []        # [{start_t, end_t, duration_s, pose}]
        self._last_waypoints_left = None

        diag_dir = os.path.join(outdir, "diag")
        os.makedirs(diag_dir, exist_ok=True)
        self._route_path = os.path.join(diag_dir, "return_path_route.csv")
        self._log_path = os.path.join(diag_dir, "return_path.log")
        self._fh = open(self._log_path, "w")
        self._closed = False
        self._fh.write("t,pose_x,pose_y,pose_deg,v,w,cur_v,waypoints_left,"
                       "path_len_left_m,carrot_x,carrot_y,carrot_dist,"
                       "min_lidar_ahead,nearest_aux,nearest_poison,stall\n")

    # -------------------------------------------------------------- #
    def dump_route(self, t, route_world):
        """Call once, right after return_path.ReturnPathFollower.trigger()."""
        with open(self._route_path, "w") as f:
            f.write("idx,x,y\n")
            for i, (x, y) in enumerate(route_world):
                f.write(f"{i},{x:.3f},{y:.3f}\n")
        print(f"[return_path][DIAG] route dumped ({len(route_world)} waypoints) "
              f"-> {self._route_path}")

    @staticmethod
    def _path_length(path):
        if not path or len(path) < 2:
            return 0.0
        return sum(math.hypot(path[i + 1][0] - path[i][0], path[i + 1][1] - path[i][1])
                   for i in range(len(path) - 1))

    def nearby_obstacles(self, grid, x, y):
        """Nearest mapped aux / poison point distance within near_obs_r (m);
        returns inf if none found.  Read-only grid queries, no side effects."""
        pw = grid.poison_points_near(x, y, self.near_obs_r)
        aw = grid.aux_points_near(x, y, self.near_obs_r)
        nearest_poison = (min(math.hypot(px - x, py - y) for px, py in pw)
                          if len(pw) else float("inf"))
        nearest_aux = (min(math.hypot(ax - x, ay - y) for ax, ay in aw)
                      if len(aw) else float("inf"))
        return nearest_aux, nearest_poison

    def min_lidar_ahead(self, scan_body):
        """Min lidar range within the forward cone, body frame (m); inf if none."""
        if scan_body is None or len(scan_body) == 0:
            return float("inf")
        best = float("inf")
        for x, y in scan_body:
            if x <= 0:
                continue
            if abs(math.atan2(y, x)) > self.fwd_cone_rad:
                continue
            r = math.hypot(x, y)
            if r < best:
                best = r
        return best

    # -------------------------------------------------------------- #
    def update(self, t, pose, v, w, cur_v, path, carrot,
              min_lidar_ahead, nearest_aux, nearest_poison):
        self._tick += 1
        waypoints_left = len(path) if path else 0

        if self._last_waypoints_left is not None and waypoints_left < self._last_waypoints_left:
            print(f"[return_path][DIAG] waypoint consumed: {self._last_waypoints_left} "
                  f"-> {waypoints_left} left, t={t:.2f} pose=({pose[0]:+.2f},{pose[1]:+.2f})")
        self._last_waypoints_left = waypoints_left

        stalling = cur_v < self.stall_v
        if stalling:
            if self._stall_since is None:
                self._stall_since = t
                self._stall_flagged = False
            elif not self._stall_flagged and (t - self._stall_since) >= self.stall_s:
                self._stall_flagged = True
                print(f"[return_path][DIAG] STALL: v={cur_v:.3f} m/s for "
                     f"{t - self._stall_since:.1f}s at pose=({pose[0]:+.2f},{pose[1]:+.2f}) "
                     f"min_lidar_ahead={min_lidar_ahead:.2f} nearest_aux={nearest_aux:.2f} "
                     f"nearest_poison={nearest_poison:.2f} waypoints_left={waypoints_left}")
        else:
            if self._stall_since is not None:
                dur = t - self._stall_since
                if dur >= self.stall_s:
                    self.stall_events.append({"start_t": self._stall_since, "end_t": t,
                                              "duration_s": dur, "pose": pose})
            self._stall_since = None
            self._stall_flagged = False

        if self._tick % self.log_every_ticks == 0 or self._stall_flagged:
            cx, cy = carrot if carrot is not None else (float("nan"), float("nan"))
            carrot_dist = (math.hypot(cx - pose[0], cy - pose[1])
                          if carrot is not None else -1.0)
            path_len_left = self._path_length(path)
            def fin(x):
                return "inf" if math.isinf(x) else f"{x:.3f}"
            self._fh.write(
                f"{t:.2f},{pose[0]:.3f},{pose[1]:.3f},{math.degrees(pose[2]):.1f},"
                f"{v:.3f},{w:.3f},{cur_v:.3f},{waypoints_left},{path_len_left:.2f},"
                f"{cx:.3f},{cy:.3f},{carrot_dist:.3f},"
                f"{fin(min_lidar_ahead)},{fin(nearest_aux)},{fin(nearest_poison)},"
                f"{int(stalling)}\n")
            self._fh.flush()

    def mark_done(self, t, pose):
        self._fh.write(f"# route retraced/exited at t={t:.2f} pose=({pose[0]:.3f},{pose[1]:.3f})\n")
        self._fh.flush()

    def close(self, t=None, pose=None):
        if self._closed:
            return
        self._closed = True
        # an unresolved stall still running when the sim ends is the exact
        # failure mode this tracker exists to catch -- count it too
        if self._stall_since is not None and t is not None:
            dur = t - self._stall_since
            if dur >= self.stall_s:
                self.stall_events.append({"start_t": self._stall_since, "end_t": t,
                                          "duration_s": dur,
                                          "pose": pose if pose is not None else (float("nan"),) * 3})
        if self.stall_events:
            total = sum(e["duration_s"] for e in self.stall_events)
            worst = max(self.stall_events, key=lambda e: e["duration_s"])
            print(f"[return_path][DIAG] summary: {len(self.stall_events)} stall event(s), "
                 f"total {total:.1f}s, worst {worst['duration_s']:.1f}s at "
                 f"pose=({worst['pose'][0]:+.2f},{worst['pose'][1]:+.2f}) "
                 f"[t={worst['start_t']:.1f}-{worst['end_t']:.1f}]")
        else:
            print("[return_path][DIAG] summary: no stall events recorded")
        self._fh.close()
