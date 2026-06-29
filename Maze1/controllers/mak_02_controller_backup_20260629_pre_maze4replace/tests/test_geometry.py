import math
import unittest

import tests._bootstrap  # noqa: F401
import numpy as np

import geometry as G


class TestGeometry(unittest.TestCase):
    def test_wrap_angle(self):
        # Convention is [-pi, pi): +pi maps to -pi.
        self.assertAlmostEqual(G.wrap_angle(3 * math.pi), -math.pi, places=6)
        self.assertAlmostEqual(G.wrap_angle(-3 * math.pi), -math.pi, places=6)
        self.assertAlmostEqual(G.wrap_angle(0.0), 0.0, places=6)
        self.assertAlmostEqual(G.wrap_angle(math.pi / 2), math.pi / 2, places=6)

    def test_transform_roundtrip(self):
        pts = np.array([[1.0, 0.0], [0.0, 2.0], [-1.0, -1.0]])
        x, y, th = 0.5, -0.3, 0.7
        world = G.transform_points(pts, x, y, th)
        back = G.inverse_transform_points(world, x, y, th)
        self.assertTrue(np.allclose(back, pts, atol=1e-9))

    def test_transform_known(self):
        # 90 deg rotation: body +x -> world +y
        pts = np.array([[1.0, 0.0]])
        world = G.transform_points(pts, 0.0, 0.0, math.pi / 2)
        self.assertTrue(np.allclose(world, [[0.0, 1.0]], atol=1e-9))

    def test_relative_pose(self):
        dx, dy, dth = G.relative_pose((0, 0, 0), (1, 0, math.pi / 2))
        self.assertAlmostEqual(dx, 1.0, places=6)
        self.assertAlmostEqual(dy, 0.0, places=6)
        self.assertAlmostEqual(dth, math.pi / 2, places=6)


if __name__ == "__main__":
    unittest.main()
