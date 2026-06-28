import math
import unittest

import tests._bootstrap  # noqa: F401
import numpy as np

import settings as S
from slam.occupancy_grid import OccupancyGrid
from slam.likelihood_field import LikelihoodField
from slam.scan_matcher import ScanMatcher
from geometry import transform_points, wrap_angle


def _box_points():
    """Dense points on a rectangular box of walls around the origin."""
    pts = []
    xs = np.arange(-1.0, 1.0 + 1e-9, 0.03)
    ys = np.arange(-0.8, 0.8 + 1e-9, 0.03)
    for x in xs:
        pts.append((x, -0.8))
        pts.append((x, 0.8))
    for y in ys:
        pts.append((-1.0, y))
        pts.append((1.0, y))
    return np.array(pts)


class TestScanMatcher(unittest.TestCase):
    def setUp(self):
        self.box = _box_points()
        self.grid = OccupancyGrid()
        ix, iy, m = self.grid.world_to_grid_arr(self.box[:, 0], self.box[:, 1])
        self.grid.L[ix[m], iy[m]] = S.L_MAX
        self.field = LikelihoodField.from_grid(self.grid)
        self.matcher = ScanMatcher()

    def test_recovers_true_pose(self):
        true_pose = (0.0, 0.0, 0.0)
        # Robot at the true pose sees the box; body points == world points here.
        body = self.box.copy()
        # Odometry handed us a wrong guess inside the coarse window.
        predicted = (0.06, -0.04, 0.03)

        matched, info = self.matcher.match(predicted, body, self.field)
        self.assertTrue(info["accepted"], f"match rejected: {info}")
        self.assertLess(abs(matched[0] - true_pose[0]), 0.03)
        self.assertLess(abs(matched[1] - true_pose[1]), 0.03)
        self.assertLess(abs(wrap_angle(matched[2] - true_pose[2])), math.radians(2.0))

    def test_match_beats_prediction(self):
        predicted = (0.08, 0.06, 0.05)
        body = self.box.copy()
        matched, info = self.matcher.match(predicted, body, self.field)
        err_pred = math.hypot(predicted[0], predicted[1])
        err_matched = math.hypot(matched[0], matched[1])
        self.assertLess(err_matched, err_pred)

    def test_empty_map_returns_prediction(self):
        empty_field = LikelihoodField.from_grid(OccupancyGrid())
        predicted = (0.1, 0.1, 0.1)
        matched, info = self.matcher.match(predicted, self.box, empty_field)
        self.assertFalse(info["accepted"])
        self.assertEqual(matched, predicted)


if __name__ == "__main__":
    unittest.main()
