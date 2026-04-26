"""Pure-pursuit-style waypoint follower with a hard collision veto.

The follower owns a list of world-frame path points (the simplified A* output).
Each ``step`` returns ``(v, w, status)`` where ``status`` is one of:
    "follow"   normal driving
    "blocked"  forward path blocked despite all sources — caller should replan
    "arrived"  reached the final waypoint

Brake sources combined: lidar forward cone min range AND the depth-camera
chassis-height clearance from ``ClearanceChecker``.

The follower picks a lookahead point ``L`` ahead along the path, computes the
heading error ``α`` to L, and commands::

    w = clamp(K_h * α, ±W_MAX)
    v = V_MAX * cos(α) * brake * narrow_scale * approach_scale

with hard cutoffs (``v=0``) when ``|α|`` is large or fwd clearance is small.
"""
import math

import numpy as np

import config as C
from utils import wrap_angle, clamp


def _forward_clear(ranges, angles, half_angle=math.radians(28.0)):
    if ranges is None or angles is None or ranges.size == 0:
        return float("inf")
    valid = np.isfinite(ranges) & (ranges > 0.05) & (np.abs(angles) < half_angle)
    if not valid.any():
        return float("inf")
    return float(np.min(ranges[valid]))


def _side_min(ranges, angles, side):
    if ranges is None or angles is None or ranges.size == 0:
        return float("inf")
    if side == "left":
        lo, hi = math.radians(15.0), math.radians(85.0)
    else:
        lo, hi = math.radians(-85.0), math.radians(-15.0)
    m = np.isfinite(ranges) & (ranges > 0.05) & (angles >= lo) & (angles <= hi)
    if not m.any():
        return float("inf")
    return float(np.min(ranges[m]))


class PathFollower:
    def __init__(self):
        self.path = []                # list of (wx, wy)
        self.idx = 0                  # current "anchor" waypoint index
        self.final_tol = C.WAYPOINT_REACH_TOL

    # ----------------------------- public API ---------------------------

    def set_path(self, path_world, final_tol=None):
        self.path = list(path_world) if path_world else []
        self.idx = 0
        if final_tol is not None:
            self.final_tol = final_tol
        else:
            self.final_tol = C.WAYPOINT_REACH_TOL

    def has_path(self):
        return len(self.path) >= 1

    def remaining_distance(self, pose):
        if not self.path:
            return 0.0
        x, y, _ = pose
        d = math.hypot(self.path[-1][0] - x, self.path[-1][1] - y)
        return d

    def step(self, pose, ranges=None, angles=None, aux_clear=None,
             cur_speed=None):
        if not self.has_path():
            return 0.0, 0.0, "arrived"

        x, y, th = pose
        # advance the anchor index past any waypoints we have already passed
        while self.idx < len(self.path) - 1:
            wx, wy = self.path[self.idx]
            if math.hypot(wx - x, wy - y) < C.WAYPOINT_REACH_TOL:
                self.idx += 1
            else:
                break

        # final waypoint reached?
        last_x, last_y = self.path[-1]
        if math.hypot(last_x - x, last_y - y) < self.final_tol:
            return 0.0, 0.0, "arrived"

        # --- pick lookahead point --------------------------------------
        v_ref = cur_speed if cur_speed is not None else C.V_MAX
        L = clamp(C.LOOKAHEAD_BASE + C.LOOKAHEAD_K_V * abs(v_ref),
                  C.LOOKAHEAD_MIN, C.LOOKAHEAD_MAX)
        target = self._lookahead_point(x, y, L)

        # --- compute heading error and base cmd ------------------------
        dx = target[0] - x
        dy = target[1] - y
        desired_th = math.atan2(dy, dx)
        alpha = wrap_angle(desired_th - th)

        if abs(alpha) < C.PF_HEADING_DEADBAND:
            w = 0.0
        else:
            w = clamp(C.PF_K_HEADING * alpha, -C.W_MAX, C.W_MAX)

        if abs(alpha) > C.PF_BIG_HEADING_STOP:
            # Need to spin in place before committing to a forward command.
            return 0.0, w, "follow"

        v_base = C.V_MAX * math.cos(alpha)
        v_base = max(C.V_MIN_FORWARD, v_base)

        # --- collision veto + speed scheduling -------------------------
        fwd = _forward_clear(ranges, angles)
        if aux_clear is not None and np.isfinite(aux_clear):
            fwd = min(fwd, float(aux_clear))

        if fwd < C.PF_FWD_BRAKE_DIST:
            return 0.0, self._evasive_w(w, ranges, angles), "blocked"

        if fwd < C.PF_FWD_SLOW_DIST:
            slow = (fwd - C.PF_FWD_BRAKE_DIST) / max(
                C.PF_FWD_SLOW_DIST - C.PF_FWD_BRAKE_DIST, 1e-3
            )
            v_base *= clamp(slow, 0.2, 1.0)

        # narrow corridor: scale by side clearance
        sl = _side_min(ranges, angles, "left")
        sr = _side_min(ranges, angles, "right")
        side = min(sl, sr)
        if side < C.PF_NARROW_SCALE_DIST:
            v_base *= clamp(side / C.PF_NARROW_SCALE_DIST, 0.35, 1.0)

        # near-final approach: scale by remaining distance
        rem = math.hypot(last_x - x, last_y - y)
        if rem < 0.6:
            v_base *= clamp(rem / 0.6, 0.3, 1.0)

        v = clamp(v_base, 0.0, C.V_MAX)
        return v, w, "follow"

    # ----------------------------- helpers ------------------------------

    def _lookahead_point(self, rx, ry, L):
        """Walk forward along the path from the current anchor; return the
        first point that is at least ``L`` metres from the robot. If none
        exists, return the final waypoint.
        """
        for i in range(self.idx, len(self.path)):
            wx, wy = self.path[i]
            if math.hypot(wx - rx, wy - ry) >= L:
                return wx, wy
        return self.path[-1]

    def _evasive_w(self, desired_w, ranges, angles):
        """When braked, prefer to keep the desired turn sign; otherwise
        rotate toward the freer side."""
        if abs(desired_w) > 0.25:
            return math.copysign(C.W_MAX * 0.7, desired_w)
        sl = _side_min(ranges, angles, "left")
        sr = _side_min(ranges, angles, "right")
        sign = 1.0 if sl > sr else -1.0
        return sign * C.W_MAX * 0.6
