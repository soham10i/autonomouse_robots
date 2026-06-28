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
        # ── sharp-corner stop-and-pivot (Layer B) ──
        self._sharp = {}              # vertex index -> outgoing-segment heading
        self._pivot_active = False    # currently pivoting in place
        self._pivot_vertex = None     # which vertex index we are pivoting at
        self._pivot_target_th = None  # heading to align with before resuming

    # ----------------------------- public API ---------------------------

    def set_path(self, path_world, final_tol=None):
        self.path = list(path_world) if path_world else []
        self.idx = 0
        if final_tol is not None:
            self.final_tol = final_tol
        else:
            self.final_tol = C.WAYPOINT_REACH_TOL
        self._compute_sharp_vertices()
        self._pivot_active = False
        self._pivot_vertex = None
        self._pivot_target_th = None

    def _compute_sharp_vertices(self):
        """Flag interior path vertices whose turn is sharp enough that a
        pure-pursuit arc would cut the corner and clip the inside wall.

        For each such vertex we store the heading of its OUTGOING segment
        — the heading the robot must pivot to before resuming forward
        motion. Only interior vertices (0 < i < N) qualify; the first
        vertex is handled by the existing big-heading-stop logic and the
        last is the goal.
        """
        self._sharp = {}
        p = self.path
        turn_thr = math.radians(C.PF_PIVOT_TURN_DEG)
        for i in range(1, len(p) - 1):
            hx0, hy0 = p[i][0] - p[i - 1][0], p[i][1] - p[i - 1][1]
            hx1, hy1 = p[i + 1][0] - p[i][0], p[i + 1][1] - p[i][1]
            if hx0 * hx0 + hy0 * hy0 < 1e-9:
                continue
            if hx1 * hx1 + hy1 * hy1 < 1e-9:
                continue
            h0 = math.atan2(hy0, hx0)
            h1 = math.atan2(hy1, hx1)
            if abs(wrap_angle(h1 - h0)) >= turn_thr:
                self._sharp[i] = h1   # heading to align to after the pivot

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

        # --- sharp-corner stop-and-pivot (Layer B) ---------------------
        # If we're approaching (or in) a flagged sharp vertex, pivot in
        # place instead of letting pure pursuit round the corner.
        pv = self._pivot_step(x, y, th)
        if pv is not None:
            return pv

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

        # --- corridor centering ----------------------------------------
        # When both side clearances are tight AND we're already roughly
        # aligned with the path, blend in a wall-equidistant correction.
        # This stops the bot from zig-zagging on the discrete A* waypoints
        # in narrow passages — it tracks the actual corridor centerline.
        # We deliberately disable the correction at large asymmetry, which
        # usually means a junction (one side opening up).
        sl_pre = _side_min(ranges, angles, "left")
        sr_pre = _side_min(ranges, angles, "right")
        if (np.isfinite(sl_pre) and np.isfinite(sr_pre)
                and sl_pre < C.PF_CENTER_TRIGGER_DIST
                and sr_pre < C.PF_CENTER_TRIGGER_DIST
                and abs(alpha) < C.PF_CENTER_HEADING_LIMIT):
            asymmetry = sr_pre - sl_pre   # +ve means right is farther → turn right
            if abs(asymmetry) < C.PF_CENTER_MAX_ASYM:
                w_center = clamp(
                    -C.PF_CENTER_GAIN * asymmetry,
                    -C.W_MAX * 0.5, C.W_MAX * 0.5,
                )
                # asymmetry > 0 (right wall farther) means the bot is closer
                # to the LEFT wall; we want to turn RIGHT (negative w in our
                # convention: +w = CCW = left). Hence the leading minus.
                w = clamp(w + w_center, -C.W_MAX, C.W_MAX)

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

    # ----------------------------- pivot machine ------------------------

    def _next_sharp_vertex(self):
        """Lowest flagged sharp-vertex index at or ahead of the anchor."""
        for i in sorted(self._sharp):
            if i >= self.idx:
                return i
        return None

    def _pivot_cmd(self, err):
        """Angular command for a given heading error, with a floor so the
        last few degrees aren't crawled."""
        w = clamp(C.PF_K_HEADING * err, -C.PF_PIVOT_W, C.PF_PIVOT_W)
        if abs(w) < C.PF_PIVOT_W_MIN:
            w = math.copysign(C.PF_PIVOT_W_MIN, err)
        return w

    def _pivot_step(self, x, y, th):
        """Stop-and-pivot state machine for sharp corners.

        Returns ``(v, w, status)`` while a pivot is in progress or has
        just been triggered; returns ``None`` when no pivot applies (so
        ``step`` falls through to normal pure-pursuit driving).

        Pivoting commands ``v = 0`` — the IR reflex bypasses zero-forward
        commands, so there is no conflict between the two layers.
        """
        done_tol = math.radians(C.PF_PIVOT_DONE_DEG)

        # Already pivoting: spin until aligned with the outgoing segment.
        if self._pivot_active:
            err = wrap_angle(self._pivot_target_th - th)
            if abs(err) < done_tol:
                # Pivot finished — advance the anchor past the vertex so
                # the lookahead now tracks the post-corner segment.
                self._pivot_active = False
                self.idx = max(self.idx, self._pivot_vertex + 1)
                return None   # fall through to normal follow this tick
            return 0.0, self._pivot_cmd(err), "follow"

        # Not pivoting: should we start? Trigger when within
        # PF_PIVOT_TRIGGER_DIST of the next sharp vertex.
        nxt = self._next_sharp_vertex()
        if nxt is not None:
            vx, vy = self.path[nxt]
            if math.hypot(vx - x, vy - y) < C.PF_PIVOT_TRIGGER_DIST:
                self._pivot_active = True
                self._pivot_vertex = nxt
                self._pivot_target_th = self._sharp[nxt]
                err = wrap_angle(self._pivot_target_th - th)
                return 0.0, self._pivot_cmd(err), "follow"
        return None

    # ----------------------------- helpers ------------------------------

    def _lookahead_point(self, rx, ry, L):
        """Walk forward along the path from the current anchor; return the
        first point at least ``L`` metres away.

        Crucially, the lookahead is **clamped at the next sharp vertex** —
        it never looks *past* a sharp corner. Without this, a long
        lookahead aims diagonally across the corner while the robot is
        still far from it, so pure pursuit rounds the turn before the
        pivot machine ever triggers. Clamping makes the robot drive
        straight INTO the corner; ``_pivot_step`` then takes over.
        """
        nxt_sharp = self._next_sharp_vertex()
        for i in range(self.idx, len(self.path)):
            wx, wy = self.path[i]
            if nxt_sharp is not None and i >= nxt_sharp:
                # Reached the sharp vertex — clamp here, don't look past it.
                return self.path[nxt_sharp]
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
