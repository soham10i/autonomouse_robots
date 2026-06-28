"""DWA local planner — the move_base local-planner equivalent (pure math).

Pure pursuit only *tracks* a path; it cannot react to a wall it is about to
clip.  This planner closes that gap the way ROS ``move_base`` does in the
reference repo: it samples forward (v, w) commands, rolls each out into a short
trajectory, **rejects any that pass closer than ``LP_SAFE_RADIUS`` to a live
lidar point**, and scores the survivors by

    score = W_HEADING * heading_alignment
          - W_DIST    * distance_to_carrot_at_end
          + W_CLEAR   * clearance            (-> centres the robot in corridors)
          + W_VEL     * v

The clearance term is what makes the robot hug the *centre* of a passage instead
of grazing a wall — exactly the behaviour shown in the frontier-exploration
video.  Everything is in the robot frame (origin at the robot, +x forward), so
the planner needs no map and works straight off the current scan.
"""
from __future__ import annotations

import math

import numpy as np

import settings as S


def widest_gap_sign(obstacles_body):
    """Return +1.0 (turn left/CCW) or -1.0 (right/CW) toward the MORE OPEN side,
    given live obstacle points in the robot frame.  Used by recovery to rotate
    toward free space instead of a fixed direction.  A side with no obstacles
    (or farther ones) is the open side."""
    o = np.asarray(obstacles_body, dtype=np.float64).reshape(-1, 2)
    if o.shape[0] == 0:
        return 1.0
    rng = np.hypot(o[:, 0], o[:, 1])
    left = rng[o[:, 1] >= 0.0]
    right = rng[o[:, 1] < 0.0]
    lmax = float(left.max()) if left.size else 1e9     # empty side == wide open
    rmax = float(right.max()) if right.size else 1e9
    return 1.0 if lmax >= rmax else -1.0


class DWAPlanner:
    def __init__(self, v_max=None, w_max=None):
        self.v_max = S.V_MAX if v_max is None else v_max
        self.w_max = S.W_MAX if w_max is None else w_max

    def _rollout(self, v, w):
        """Return trajectory points (K, 2) in the robot frame and final heading."""
        n = max(1, int(round(S.LP_HORIZON_S / S.LP_STEP_S)))
        x = y = th = 0.0
        pts = np.empty((n, 2), dtype=np.float64)
        for i in range(n):
            th += w * S.LP_STEP_S
            x += v * math.cos(th) * S.LP_STEP_S
            y += v * math.sin(th) * S.LP_STEP_S
            pts[i, 0] = x
            pts[i, 1] = y
        return pts, th

    def _clearance(self, traj, obstacles):
        """Min distance from any trajectory point to any obstacle point."""
        if obstacles.shape[0] == 0:
            return float("inf")
        best = float("inf")
        for px, py in traj:
            d = np.hypot(obstacles[:, 0] - px, obstacles[:, 1] - py)
            m = float(d.min())
            if m < best:
                best = m
        return best

    def compute(self, carrot_body, obstacles_body, v_max=None):
        """Pick a safe ``(v, w)`` toward ``carrot_body`` given live ``obstacles_body``.

        ``carrot_body``    : (gx, gy) lookahead/goal point in the robot frame.
        ``obstacles_body`` : (N, 2) lidar hits in the robot frame.
        Returns ``(v, w, info)``; ``info['boxed']`` is True if every forward
        trajectory was rejected (the caller should rotate / recover).
        """
        v_max = self.v_max if v_max is None else v_max
        obstacles_body = np.asarray(obstacles_body, dtype=np.float64).reshape(-1, 2)
        if obstacles_body.shape[0] > S.LP_SUBSAMPLE:
            idx = np.linspace(0, obstacles_body.shape[0] - 1, S.LP_SUBSAMPLE).astype(int)
            obstacles_body = obstacles_body[idx]

        gx, gy = carrot_body

        # NOTE: no global "proximity slowdown" speed cap.  That omnidirectional
        # cap made the robot crawl down every corridor (side walls are always
        # ~0.12 m away) which tripped the stuck-watchdog and pinned it.  Speed is
        # instead governed by the per-trajectory clearance rejection below — the
        # standard DWA mechanism: fast trajectories that approach an obstacle are
        # rejected, so the robot only slows for things in its ACTUAL path, and
        # drives at full speed when the way ahead is clear.
        vs = np.linspace(0.0, v_max, S.LP_V_SAMPLES)
        ws = np.linspace(-self.w_max, self.w_max, S.LP_W_SAMPLES)

        best = None
        best_score = -math.inf
        for v in vs:
            for w in ws:
                if v == 0.0 and w == 0.0:
                    continue
                traj, th_end = self._rollout(v, w)
                clear = self._clearance(traj, obstacles_body)
                if clear < S.LP_SAFE_RADIUS:
                    continue                       # would collide -> reject
                ex, ey = traj[-1]
                end_dist = math.hypot(gx - ex, gy - ey)
                bearing = math.atan2(gy - ey, gx - ex)
                align = math.cos(th_end - bearing)
                score = (S.LP_W_HEADING * align
                         - S.LP_W_DIST * end_dist
                         + S.LP_W_CLEAR * min(clear, S.LP_CLEAR_CAP)
                         + S.LP_W_VEL * v
                         - S.LP_W_STRAIGHT * abs(w))   # damp spin oscillation

                if score > best_score:
                    best_score = score
                    best = (float(v), float(w))

        if best is None:
            # Fully boxed in — spin in place to find an opening, biased toward
            # whichever side has MORE room so the tail doesn't swing into the
            # near wall; fall back to the carrot side when clearances tie.
            spin = S.LP_SEARCH_W if gy >= 0 else -S.LP_SEARCH_W
            if obstacles_body.shape[0]:
                left = obstacles_body[obstacles_body[:, 1] > 0.0]
                right = obstacles_body[obstacles_body[:, 1] < 0.0]
                dl = float(np.min(np.hypot(left[:, 0], left[:, 1]))) if left.shape[0] else 1e9
                dr = float(np.min(np.hypot(right[:, 0], right[:, 1]))) if right.shape[0] else 1e9
                if abs(dl - dr) > 1e-3:
                    spin = S.LP_SEARCH_W if dl >= dr else -S.LP_SEARCH_W
            return 0.0, spin, {"boxed": True, "score": None}
        return best[0], best[1], {"boxed": False, "score": best_score}
