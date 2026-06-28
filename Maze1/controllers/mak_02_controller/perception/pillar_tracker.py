"""Pillar position tracking with a close-range confirmation gate.

Webots-independent (math / numpy / settings only) so it is unit-testable.

``update`` folds every range-gated detection into the running-mean position that
STEERS exploration/GO toward the pillar — far sightings included, so the robot
heads the right way early.  But the mission only counts a pillar as REACHABLE
once ``observe`` has seen it from CLOSE range a few consistent times.  A far
sliver-through-a-gap view biases the estimate toward open floor; gating "reached"
on a close confirm forces the robot to actually drive up to the pillar before it
counts — which kills the false arrival at empty floor.
"""
from __future__ import annotations

import math

import numpy as np

import settings as S


class PillarTracker:
    def __init__(self):
        self.obs = {"blue": [], "yellow": []}
        self.pos = {"blue": None, "yellow": None}
        self.confirmed = {"blue": False, "yellow": False}
        self._close = {"blue": [], "yellow": []}     # recent close readings

    def update(self, name, gx, gy):
        """Fold a detection into the running-mean ESTIMATE (steers navigation)."""
        cur = self.pos[name]
        if cur is not None and math.hypot(gx - cur[0], gy - cur[1]) > S.PILLAR_OUTLIER_REJECT_M:
            return
        buf = self.obs[name]
        buf.append((gx, gy))
        if len(buf) > S.PILLAR_OBS_AVG_N:
            buf.pop(0)
        if len(buf) >= 3:
            xs = np.fromiter((p[0] for p in buf), dtype=float)
            ys = np.fromiter((p[1] for p in buf), dtype=float)
            self.pos[name] = (float(xs.mean()), float(ys.mean()))

    def observe(self, name, gx, gy, rng):
        """Feed a detection toward CLOSE-RANGE confirmation.

        A reading only counts when its range is in the close band
        ``[PILLAR_CONFIRM_RANGE_MIN, MAX]`` AND it agrees with the other recent
        close readings (``PILLAR_CONFIRM_SNAP_TOL``).  After
        ``PILLAR_CONFIRM_HITS`` such readings the pillar is confirmed and its
        position is SNAPPED to the close-reading mean (de-biasing any far
        through-gap estimate).  Out-of-band readings reset the close streak.
        """
        if self.confirmed[name]:
            return
        if not (S.PILLAR_CONFIRM_RANGE_MIN <= rng <= S.PILLAR_CONFIRM_RANGE_MAX):
            self._close[name] = []                # broke the close streak
            return
        cl = self._close[name]
        if cl:
            mx = sum(p[0] for p in cl) / len(cl)
            my = sum(p[1] for p in cl) / len(cl)
            if math.hypot(gx - mx, gy - my) > S.PILLAR_CONFIRM_SNAP_TOL:
                self._close[name] = [(gx, gy)]    # inconsistent -> restart streak
                return
        cl.append((gx, gy))
        if len(cl) >= S.PILLAR_CONFIRM_HITS:
            xs = np.fromiter((p[0] for p in cl), dtype=float)
            ys = np.fromiter((p[1] for p in cl), dtype=float)
            self.pos[name] = (float(xs.mean()), float(ys.mean()))   # snap/de-bias
            self.confirmed[name] = True

    def is_confirmed(self, name):
        return bool(self.confirmed.get(name))

    def known(self, name):
        return self.pos[name] is not None
