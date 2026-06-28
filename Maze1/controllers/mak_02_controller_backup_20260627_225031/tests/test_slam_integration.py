"""Closed-loop SLAM sanity test (Phase A + B together, no Webots).

A robot translates inside a rectangular room.  Its odometry *drifts* (a steady
error is added every step).  We feed the drifted prediction plus the (noise-free)
scan into the scan matcher, integrate at the corrected pose, and check that the
corrected trajectory tracks ground truth far better than raw odometry would —
i.e. the matcher is actually removing drift, which is the whole point of Phase A.
"""
import math
import unittest

import tests._bootstrap  # noqa: F401
import numpy as np

import settings as S
from slam.occupancy_grid import OccupancyGrid
from slam.likelihood_field import LikelihoodField
from slam.scan_matcher import ScanMatcher
from geometry import inverse_transform_points, transform_points, pose_distance


def _room_points():
    pts = []
    xs = np.arange(-1.2, 1.2 + 1e-9, 0.025)
    ys = np.arange(-1.0, 1.0 + 1e-9, 0.025)
    for x in xs:
        pts.append((x, -1.0)); pts.append((x, 1.0))
    for y in ys:
        pts.append((-1.2, y)); pts.append((1.2, y))
    return np.array(pts)


class TestSlamIntegration(unittest.TestCase):
    def test_matcher_removes_drift(self):
        room = _room_points()
        grid = OccupancyGrid()
        field = LikelihoodField.from_grid(grid)
        matcher = ScanMatcher()

        # ground-truth poses: translate gently along +x
        true_poses = [(0.04 * k, 0.0, 0.0) for k in range(8)]

        drift = np.array([0.025, 0.015, 0.0])   # per-step odometry error
        accumulated = np.zeros(3)
        corrected = (0.0, 0.0, 0.0)
        raw_odom_err = 0.0

        for k, true in enumerate(true_poses):
            # scan the robot would actually see (room is convex => all walls visible)
            body = inverse_transform_points(room, true[0], true[1], true[2])

            if k == 0:
                corrected = true                 # bootstrap: first frame defines map
            else:
                accumulated += drift
                predicted = (true[0] + accumulated[0],
                             true[1] + accumulated[1],
                             true[2] + accumulated[2])
                raw_odom_err = pose_distance(predicted, true)
                corrected, info = matcher.match(predicted, body, field)

            world = transform_points(body, *corrected)
            grid.integrate_scan(corrected, world)
            field.rebuild(grid)

        final_err = pose_distance(corrected, true_poses[-1])
        # corrected pose should be close to truth and much better than raw odom
        self.assertLess(final_err, 0.05, f"corrected error too high: {final_err:.3f}")
        self.assertLess(final_err, raw_odom_err,
                        "scan matching did not improve on raw odometry")

    def test_walls_stay_thin(self):
        """Integrating the same wall from several poses must not fatten it."""
        room = _room_points()
        grid = OccupancyGrid()
        field = LikelihoodField.from_grid(grid)
        matcher = ScanMatcher()

        for k in range(6):
            true = (0.03 * k, 0.0, 0.0)
            body = inverse_transform_points(room, *true)
            if k == 0:
                corrected = true
            else:
                predicted = (true[0] + 0.02, true[1], true[2])  # drift
                corrected, _ = matcher.match(predicted, body, field)
            world = transform_points(body, *corrected)
            grid.integrate_scan(corrected, world)
            field.rebuild(grid)

        # The right wall is at x = 1.2.  Count occupied cells per column near it;
        # a clean map keeps the wall ~1 cell thick, not a smeared band.
        occ = grid.occupied_mask()
        ix_lo, _ = grid.world_to_grid(1.2 - 0.12, 0.0)
        ix_hi, _ = grid.world_to_grid(1.2 + 0.12, 0.0)
        band = occ[ix_lo:ix_hi + 1, :]
        # for each row that has any wall, how many columns are occupied?
        rows_with_wall = band.any(axis=0)
        thickness = band.sum(axis=0)[rows_with_wall]
        if thickness.size:
            self.assertLessEqual(float(thickness.mean()), 2.5,
                                 "wall is smeared (too thick)")


if __name__ == "__main__":
    unittest.main()
