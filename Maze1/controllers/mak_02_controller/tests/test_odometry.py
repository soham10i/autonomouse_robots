import math
import unittest

import tests._bootstrap  # noqa: F401

import settings as S
from slam.odometry import Odometry
from geometry import relative_pose, compose_pose, wrap_angle


class TestOdometry(unittest.TestCase):
    def test_drive_straight(self):
        od = Odometry()
        od.update(0.0, 0.0, 0.0, 0.032)        # prime
        # advance both wheels by 1 rad -> arc = WHEEL_RADIUS each
        od.update(1.0, 1.0, 0.0, 0.032)
        x, y, th = od.pose()
        self.assertAlmostEqual(x, S.WHEEL_RADIUS, places=6)
        self.assertAlmostEqual(y, 0.0, places=6)
        self.assertAlmostEqual(th, 0.0, places=6)

    def test_imu_sets_heading(self):
        od = Odometry()
        od.update(0.0, 0.0, 1.2, 0.032)
        self.assertAlmostEqual(od.pose()[2], 1.2, places=6)

    def test_compose_is_inverse_of_relative(self):
        a = (0.5, -0.3, 0.7)
        b = (1.2, 0.4, -0.5)
        rel = relative_pose(a, b)
        bx, by, bth = compose_pose(a, rel)
        self.assertAlmostEqual(bx, b[0], places=6)
        self.assertAlmostEqual(by, b[1], places=6)
        self.assertAlmostEqual(wrap_angle(bth - b[2]), 0.0, places=6)


if __name__ == "__main__":
    unittest.main()
