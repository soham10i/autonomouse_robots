import math
import unittest

import tests._bootstrap  # noqa: F401

import settings as S
from control.pure_pursuit import follow


class TestPurePursuit(unittest.TestCase):
    def test_straight_ahead(self):
        pose = (0.0, 0.0, 0.0)
        path = [(0.0, 0.0), (0.5, 0.0), (1.0, 0.0)]
        cmd = follow(pose, path)
        self.assertGreater(cmd["v"], 0.1)
        self.assertLess(abs(cmd["w"]), 0.05)
        self.assertFalse(cmd["reached"])

    def test_turn_left_pivots(self):
        pose = (0.0, 0.0, 0.0)
        path = [(0.0, 0.0), (0.0, 0.5), (0.0, 1.0)]
        cmd = follow(pose, path)
        self.assertGreater(cmd["w"], 0.0)            # turn left
        self.assertEqual(cmd["v"], 0.0)              # pivot in place

    def test_goal_reached(self):
        pose = (1.0, 0.0, 0.0)
        path = [(0.0, 0.0), (1.0, 0.0)]
        cmd = follow(pose, path)
        self.assertTrue(cmd["reached"])

    def test_heading_error_sign(self):
        # target to the right => negative w
        pose = (0.0, 0.0, 0.0)
        path = [(0.0, 0.0), (0.0, -0.5), (0.0, -1.0)]
        cmd = follow(pose, path)
        self.assertLess(cmd["w"], 0.0)


if __name__ == "__main__":
    unittest.main()
