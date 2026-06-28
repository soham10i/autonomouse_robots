import math
import unittest

import tests._bootstrap  # noqa: F401
import numpy as np

from mapping.lidar_model import LidarModel


class TestLidarModel(unittest.TestCase):
    def test_forward_beam(self):
        # 5 beams over 180 deg => middle beam points straight ahead (+x).
        lm = LidarModel(5, math.pi, r_min=0.05, r_max=8.0)
        pts, valid = lm.ranges_to_body([1.0, 1.0, 1.0, 1.0, 1.0])
        self.assertEqual(pts.shape[0], 5)
        self.assertTrue(valid.all())
        # middle (index 2) is the straight-ahead beam
        self.assertTrue(np.allclose(pts[2], [1.0, 0.0], atol=1e-9))

    def test_drops_out_of_range(self):
        lm = LidarModel(5, math.pi)
        pts, valid = lm.ranges_to_body([float("inf"), 1.0, 1.0, 0.0, 1.0])
        # inf and 0.0 dropped -> 3 valid
        self.assertEqual(pts.shape[0], 3)
        self.assertEqual(int(valid.sum()), 3)


if __name__ == "__main__":
    unittest.main()
