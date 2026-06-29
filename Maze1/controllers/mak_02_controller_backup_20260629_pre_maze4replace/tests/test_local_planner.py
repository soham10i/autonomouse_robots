import math
import unittest

import tests._bootstrap  # noqa: F401
import numpy as np

import settings as S
from control.local_planner import DWAPlanner, widest_gap_sign, widest_gap_bearing


def _wall(x, y0, y1, n=40):
    ys = np.linspace(y0, y1, n)
    return np.stack([np.full_like(ys, x), ys], axis=1)


class TestWidestGapBearing(unittest.TestCase):
    def test_no_obstacles_faces_forward(self):
        self.assertAlmostEqual(widest_gap_bearing(np.empty((0, 2))), 0.0)

    def test_wall_on_right_turns_left(self):
        # a near wall along the robot's right (-y) side -> open gap is to the left
        wall = _wall(0.3, -0.6, -0.1, n=30)
        b = widest_gap_bearing(wall)
        self.assertGreater(b, 0.0)

    def test_wall_ahead_turns_around(self):
        # a near wall straight ahead spanning +-y -> widest gap is behind (~pi)
        ys = np.linspace(-0.5, 0.5, 40)
        wall = np.stack([np.full_like(ys, 0.3), ys], axis=1)
        b = widest_gap_bearing(wall)
        self.assertGreater(abs(b), math.pi / 2)

    def test_far_obstacles_ignored(self):
        # obstacles beyond max_range don't constrain the turn
        far = _wall(5.0, -1.0, 1.0, n=20)
        self.assertAlmostEqual(widest_gap_bearing(far, max_range=2.0), 0.0)


class TestLocalPlanner(unittest.TestCase):
    def setUp(self):
        self.dwa = DWAPlanner()

    def test_clear_ahead_drives_straight(self):
        # carrot straight ahead, no obstacles -> forward, ~no turn
        v, w, info = self.dwa.compute((2.0, 0.0), np.empty((0, 2)))
        self.assertGreater(v, 0.1)
        self.assertLess(abs(w), 0.2)
        self.assertFalse(info["boxed"])

    def test_corridor_centres(self):
        # symmetric walls at y = +/-0.3, carrot ahead -> go straight, centred
        obs = np.vstack([_wall(0.8, -0.3, -0.3 + 0.001),  # degenerate guard
                         _wall(1.0, 0.30, 0.30),
                         _wall(1.0, -0.30, -0.30)])
        obs = np.vstack([
            np.stack([np.linspace(0.2, 1.5, 30), np.full(30, 0.30)], axis=1),
            np.stack([np.linspace(0.2, 1.5, 30), np.full(30, -0.30)], axis=1),
        ])
        v, w, info = self.dwa.compute((2.0, 0.0), obs)
        self.assertGreater(v, 0.0)
        self.assertLess(abs(w), 0.25, "should stay centred, not veer to a wall")

    def test_wall_ahead_avoided(self):
        # Close wall across the path: any fast straight command would come within
        # LP_SAFE_RADIUS of it, so the planner must slow and/or steer away.
        obs = _wall(0.28, -1.0, 1.0, n=60)
        v, w, info = self.dwa.compute((2.0, 0.0), obs)
        drives_straight_fast = (v > 0.2 and abs(w) < 0.05)
        self.assertFalse(drives_straight_fast, "drove into the wall")
        if not info["boxed"]:
            traj, _ = self.dwa._rollout(v, w)
            self.assertGreaterEqual(self.dwa._clearance(traj, obs),
                                    S.LP_SAFE_RADIUS - 1e-9)

    def test_chosen_trajectory_is_safe(self):
        # whatever it picks must keep >= LP_SAFE_RADIUS from obstacles
        obs = _wall(0.5, -0.2, 1.0, n=50)   # wall on the left-ahead
        v, w, info = self.dwa.compute((2.0, 0.0), obs)
        if not info["boxed"]:
            traj, _ = self.dwa._rollout(v, w)
            clr = self.dwa._clearance(traj, obs)
            self.assertGreaterEqual(clr, S.LP_SAFE_RADIUS - 1e-9)

    def test_tight_corridor_does_not_crawl(self):
        # A ~0.26 m corridor (side walls at +/-0.13), nothing ahead.  The robot
        # MUST drive at a meaningful speed — close side walls must not force a
        # crawl (the omnidirectional-slowdown bug that pinned it in passages).
        obs = np.vstack([
            np.stack([np.linspace(0.1, 1.6, 50), np.full(50, 0.13)], axis=1),
            np.stack([np.linspace(0.1, 1.6, 50), np.full(50, -0.13)], axis=1),
        ])
        v, w, info = self.dwa.compute((2.0, 0.0), obs)
        self.assertFalse(info["boxed"])
        self.assertGreater(v, 0.15, "crawled in a clear corridor")
        self.assertLess(abs(w), 0.25, "should go straight down the corridor")

    def test_boxed_in_spins(self):
        # obstacles in a tight ring close all around -> boxed, v == 0
        ang = np.linspace(-math.pi, math.pi, 60)
        r = S.LP_SAFE_RADIUS * 0.5
        obs = np.stack([r * np.cos(ang), r * np.sin(ang)], axis=1)
        v, w, info = self.dwa.compute((2.0, 0.0), obs)
        self.assertEqual(v, 0.0)
        self.assertTrue(info["boxed"])


class TestWidestGapSign(unittest.TestCase):
    def test_open_left(self):
        # obstacles only on the right -> rotate left (+1) toward open space
        obs = np.stack([np.linspace(0.2, 1.0, 20), np.full(20, -0.2)], axis=1)
        self.assertEqual(widest_gap_sign(obs), 1.0)

    def test_open_right(self):
        obs = np.stack([np.linspace(0.2, 1.0, 20), np.full(20, 0.2)], axis=1)
        self.assertEqual(widest_gap_sign(obs), -1.0)

    def test_empty_defaults_left(self):
        self.assertEqual(widest_gap_sign(np.empty((0, 2))), 1.0)


if __name__ == "__main__":
    unittest.main()
