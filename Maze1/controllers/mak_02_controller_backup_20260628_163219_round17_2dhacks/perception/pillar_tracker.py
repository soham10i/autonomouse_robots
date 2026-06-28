"""Pillar position tracker with depth-verified close confirmation.

The blue/yellow pillars are first *located* from camera detections (a running
mean of world positions).  That estimate is good enough to navigate toward, but
it is BIASED when the pillar is only seen through/under a floating wall from afar
(the blue pillar sits under WallMedium(3)): the visible sliver's centroid and the
oblique view pull the estimate ~0.4 m off the true pillar.  Declaring "reached"
on odometric distance to that biased estimate let the robot stop ~0.8 m short of
the real pillar (it never actually arrived).

Fix — a pillar is only **confirmed** (allowed to count as reached) once the robot
has driven close enough to take a CLEAN depth measurement of it:

* the detection is within the depth camera's reliable close band
  ``[PILLAR_CONFIRM_MIN_RANGE, PILLAR_DEPTH_CONFIRM_RANGE]`` (the camera is blind
  < 0.6 m, so this is the closest reliable proof the robot is genuinely AT the
  pillar, not glimpsing it through a floating wall from a distance), and
* it agrees with the tracked estimate (``PILLAR_CONFIRM_SNAP_TOL``) so a stray
  blob can't confirm.

After ``PILLAR_CONFIRM_HITS`` such close readings the pillar is confirmed and its
estimate is **snapped** to the accurate close measurement.  The mission FSM then
requires ``is_confirmed(name)`` AND odometric proximity before arriving, so the
robot drives up to the real pillar (routing around the floating wall if a close
confirmation came from the blocked side).

This module imports only numpy/settings (no Webots), so it is unit-tested.
"""
from __future__ import annotations

import math

import numpy as np

import settings as S


def select_viewpoint(grid, pillar, robot_xy):
    """Nearest position from which the robot can actually SEE the pillar to
    confirm it: a non-lethal point in the depth confirm band around the pillar
    with a CLEAR line of sight to it.

    The pillar estimate can sit in an occluded pocket (blue is walled in, only
    visible from the south/west) and the robot would park at the estimate unable
    to confirm.  Navigating to a viewpoint instead takes it to a spot where the
    depth camera has a clean close view.  Returns ``(x, y)`` or ``None`` if no
    clear viewpoint is found (caller falls back to the estimate)."""
    px, py = pillar
    rx, ry = robot_xy
    best = None
    best_d = float("inf")
    for r in S.PILLAR_VIEW_RADII:
        for k in range(S.PILLAR_VIEW_CANDIDATES):
            a = 2.0 * math.pi * k / S.PILLAR_VIEW_CANDIDATES
            vx, vy = px + r * math.cos(a), py + r * math.sin(a)
            if grid.is_lethal_world(vx, vy):
                continue
            if not grid.segment_clear(vx, vy, px, py,
                                      stop_short=S.PILLAR_LOS_STOP_SHORT):
                continue
            dd = math.hypot(vx - rx, vy - ry)
            if dd < best_d:
                best_d, best = dd, (vx, vy)
    return best


class PillarTracker:
    """Running-mean world position of each pillar plus a depth-verified
    confirmation flag (see module docstring)."""

    def __init__(self):
        self.obs = {"blue": [], "yellow": []}
        self.pos = {"blue": None, "yellow": None}
        self.confirmed = {"blue": False, "yellow": False}
        self._confirm_hits = {"blue": 0, "yellow": 0}

    def observe(self, name, gx, gy, rng, los_clear=True):
        """Fold one VALID detection (already gated by ColorDetector.is_valid_pillar).

        ``gx, gy``    : detected pillar world position (robot pose + range·bearing).
        ``rng``       : depth-measured range to the pillar (m); may be inf/NaN if
                        the depth was invalid — such a detection updates the
                        position but can never *confirm* (no close depth proof).
        ``los_clear`` : True iff there is a clear GROUND path from the robot to the
                        detection (no wall / floating wall between them).  A
                        through/under-floating-wall sighting has ``los_clear=False``
                        and must NOT confirm — otherwise the robot declares the
                        pillar reached while a wall it cannot drive through is in
                        the way (the WallMedium(3) false-arrival).
        """
        cur = self.pos[name]
        if cur is not None and math.hypot(gx - cur[0], gy - cur[1]) > S.PILLAR_OUTLIER_REJECT_M:
            return                                   # gross outlier — ignore

        buf = self.obs[name]
        buf.append((gx, gy))
        if len(buf) > S.PILLAR_OBS_AVG_N:
            buf.pop(0)
        if len(buf) >= 3:
            xs = np.fromiter((p[0] for p in buf), dtype=float)
            ys = np.fromiter((p[1] for p in buf), dtype=float)
            self.pos[name] = (float(xs.mean()), float(ys.mean()))

        # --- depth-verified close confirmation (needs a CLEAR path, not a glimpse
        # through a floating wall) ---
        close = (np.isfinite(rng)
                 and S.PILLAR_CONFIRM_MIN_RANGE <= rng <= S.PILLAR_DEPTH_CONFIRM_RANGE)
        est = self.pos[name]
        consistent = (est is None
                      or math.hypot(gx - est[0], gy - est[1]) <= S.PILLAR_CONFIRM_SNAP_TOL)
        if close and consistent and los_clear:
            self._confirm_hits[name] += 1
            if self._confirm_hits[name] >= S.PILLAR_CONFIRM_HITS:
                self.confirmed[name] = True
                # snap to the accurate close reading; re-anchor the running buffer
                self.pos[name] = (gx, gy)
                self.obs[name] = [(gx, gy)]
        # far / occluded / through-wall detections simply don't add a confirm hit
        # (no reset, so a brief occlusion mid-approach doesn't lose progress).

    # Backward-compatible alias: a position-only update can never confirm.
    def update(self, name, gx, gy):
        self.observe(name, gx, gy, float("inf"))

    def known(self, name):
        return self.pos[name] is not None

    def is_confirmed(self, name):
        return self.confirmed[name]
