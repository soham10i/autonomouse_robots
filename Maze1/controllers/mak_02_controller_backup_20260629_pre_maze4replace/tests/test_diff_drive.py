import unittest

import tests._bootstrap  # noqa: F401
import control.diff_drive as dd
import settings as S


class TestDiffDrive(unittest.TestCase):
    def test_straight_equal_wheels(self):
        wl, wr = dd.cmd_to_wheels(0.2, 0.0)
        self.assertAlmostEqual(wl, wr, places=9)
        self.assertAlmostEqual(wl, 0.2 / S.WHEEL_RADIUS, places=6)

    def test_spin_opposite_wheels(self):
        wl, wr = dd.cmd_to_wheels(0.0, 1.0)
        self.assertAlmostEqual(wl, -wr, places=9)
        self.assertGreater(wr, 0.0)

    def test_roundtrip(self):
        for v, w in [(0.2, 0.5), (-0.1, 1.2), (0.3, -0.7)]:
            wl, wr = dd.cmd_to_wheels(v, w)
            v2, w2 = dd.wheels_to_cmd(wl, wr)
            self.assertAlmostEqual(v, v2, places=6)
            self.assertAlmostEqual(w, w2, places=6)

    def test_clamp(self):
        v, w = dd.clamp_twist(99.0, -99.0)
        self.assertEqual(v, S.V_MAX)
        self.assertEqual(w, -S.W_MAX)


if __name__ == "__main__":
    unittest.main()
