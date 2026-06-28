"""Phase A — correlative scan matcher (coarse-to-fine).

This is the piece the old controller lacked.  Wheel odometry *predicts* where
the robot moved; the scan matcher then *corrects* that prediction by sliding and
rotating the current lidar scan over the map's likelihood field until the
endpoints best line up with already-mapped walls.  Integrating scans at the
corrected pose is what keeps walls one cell thick instead of smearing into
fat, doubled bands.

The search is brute-force but cheap:

1. **Coarse** pass over a wide (x, y, theta) window at grid resolution.
2. **Fine** pass over a small window centred on the coarse winner.

For each candidate rotation the scan is rotated once and then only translated,
so scoring a whole translation grid costs one rotation plus N cheap lookups.
"""
from __future__ import annotations

import numpy as np

import settings as S
from geometry import transform_points


def _offsets(half, step):
    """Symmetric inclusive offset array, e.g. half=0.16, step=0.04 -> 9 values."""
    n = int(round(2.0 * half / step)) + 1
    return np.linspace(-half, half, n)


class ScanMatcher:
    def __init__(self):
        self.max_beams = S.SM_MAX_BEAMS

    def _subsample(self, body_pts):
        n = body_pts.shape[0]
        if n <= self.max_beams:
            return body_pts
        idx = np.linspace(0, n - 1, self.max_beams).astype(np.int64)
        return body_pts[idx]

    def _search(self, center, body_pts, field, lin_half, lin_step, ang_half, ang_step):
        cx, cy, cth = center
        lin = _offsets(lin_half, lin_step)
        ang = _offsets(ang_half, ang_step)
        best_score = -1.0
        best_pose = center
        for dth in ang:
            th = cth + float(dth)
            rot = transform_points(body_pts, 0.0, 0.0, th)   # rotate once
            for dx in lin:
                ox = cx + float(dx)
                for dy in lin:
                    oy = cy + float(dy)
                    world = rot + (ox, oy)
                    sc = field.score_world(world)
                    if sc > best_score:
                        best_score = sc
                        best_pose = (ox, oy, th)
        return best_pose, best_score

    def match(self, predicted_pose, body_pts, field):
        """Refine ``predicted_pose`` against ``field`` using ``body_pts``.

        Returns ``(pose, info)`` where ``info`` carries the normalised score and
        whether the match was accepted.  On a low-confidence match (too few
        endpoints landing near walls) the predicted pose is returned unchanged.
        """
        info = {"accepted": False, "score": 0.0, "norm": 0.0, "reason": ""}
        body_pts = np.asarray(body_pts, dtype=np.float64)
        if not S.SM_ENABLED:
            info["reason"] = "disabled"
            return predicted_pose, info
        if field.n_occupied < 10 or body_pts.shape[0] < 10:
            info["reason"] = "insufficient-map-or-scan"
            return predicted_pose, info

        pts = self._subsample(body_pts)
        n = pts.shape[0]

        coarse_pose, _ = self._search(
            predicted_pose, pts, field,
            S.SM_COARSE_LIN_HALF, S.SM_COARSE_LIN_STEP,
            S.SM_COARSE_ANG_HALF, S.SM_COARSE_ANG_STEP,
        )
        best_pose, best_score = self._search(
            coarse_pose, pts, field,
            S.SM_FINE_LIN_HALF, S.SM_FINE_LIN_STEP,
            S.SM_FINE_ANG_HALF, S.SM_FINE_ANG_STEP,
        )

        norm = best_score / max(n, 1)
        info["score"] = best_score
        info["norm"] = norm
        if norm >= S.SM_MIN_SCORE_FRAC:
            info["accepted"] = True
            return best_pose, info
        info["reason"] = "low-score"
        return predicted_pose, info
