import unittest

import tests._bootstrap  # noqa: F401

from exploration.frontier import Frontier
from exploration.goal_selection import select_goal


class TestGoalSelection(unittest.TestCase):
    def test_prefers_high_utility(self):
        big_far = Frontier((1.0, 0.0), (0, 0), 20)
        small_near = Frontier((0.5, 0.0), (0, 0), 5)
        best, score = select_goal([big_far, small_near], (0, 0, 0), (0, 0, 0))
        self.assertIs(best, big_far)

    def test_filters_too_close(self):
        near = Frontier((0.1, 0.0), (0, 0), 50)   # < FRONTIER_MIN_DIST_M
        best, _ = select_goal([near], (0, 0, 0), (0, 0, 0))
        self.assertIsNone(best)

    def test_filters_beyond_radius(self):
        far = Frontier((9.0, 0.0), (0, 0), 50)    # > EXPL_MAX_RADIUS_M from start
        best, _ = select_goal([far], (0, 0, 0), (0, 0, 0))
        self.assertIsNone(best)

    def test_blacklist_suppresses(self):
        f = Frontier((1.0, 0.0), (0, 0), 20)
        best, _ = select_goal([f], (0, 0, 0), (0, 0, 0), blacklist=[(1.0, 0.05)])
        self.assertIsNone(best)

    def test_unreachable_dist_fn_skips(self):
        f = Frontier((1.0, 0.0), (0, 0), 20)
        best, _ = select_goal([f], (0, 0, 0), (0, 0, 0), dist_fn=lambda c: None)
        self.assertIsNone(best)


if __name__ == "__main__":
    unittest.main()
