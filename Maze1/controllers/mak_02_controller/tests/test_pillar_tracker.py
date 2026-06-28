import unittest

import tests._bootstrap  # noqa: F401

import settings as S
from perception.pillar_tracker import PillarTracker


class TestPillarTracker(unittest.TestCase):
    def test_far_sightings_estimate_but_never_confirm(self):
        pt = PillarTracker()
        far = S.PILLAR_CONFIRM_RANGE_MAX + 1.5     # well beyond the close band
        for _ in range(10):
            pt.update("blue", 2.0, 1.0)
            pt.observe("blue", 2.0, 1.0, far)
        self.assertTrue(pt.known("blue"))          # estimate exists (steers nav)
        self.assertFalse(pt.is_confirmed("blue"))  # but NOT confirmed

    def test_close_consistent_views_confirm_and_snap(self):
        pt = PillarTracker()
        # bias the estimate with far views first
        far = S.PILLAR_CONFIRM_RANGE_MAX + 2.0
        for _ in range(5):
            pt.update("blue", 2.0, 1.0)
            pt.observe("blue", 2.0, 1.0, far)
        self.assertFalse(pt.is_confirmed("blue"))
        # now close, consistent views at the TRUE spot (1.4, 0.6)
        rng = 0.5 * (S.PILLAR_CONFIRM_RANGE_MIN + S.PILLAR_CONFIRM_RANGE_MAX)
        for _ in range(S.PILLAR_CONFIRM_HITS):
            pt.observe("blue", 1.4, 0.6, rng)
        self.assertTrue(pt.is_confirmed("blue"))
        # position snapped to the close readings, de-biased away from (2,1)
        self.assertAlmostEqual(pt.pos["blue"][0], 1.4, places=5)
        self.assertAlmostEqual(pt.pos["blue"][1], 0.6, places=5)

    def test_inconsistent_close_views_do_not_confirm(self):
        pt = PillarTracker()
        rng = 0.5 * (S.PILLAR_CONFIRM_RANGE_MIN + S.PILLAR_CONFIRM_RANGE_MAX)
        jump = S.PILLAR_CONFIRM_SNAP_TOL + 0.5
        # alternate between two far-apart spots -> streak keeps resetting
        for i in range(2 * S.PILLAR_CONFIRM_HITS):
            gx = 0.0 if i % 2 == 0 else jump
            pt.observe("blue", gx, 0.0, rng)
        self.assertFalse(pt.is_confirmed("blue"))

    def test_out_of_band_reading_resets_streak(self):
        pt = PillarTracker()
        rng = 0.5 * (S.PILLAR_CONFIRM_RANGE_MIN + S.PILLAR_CONFIRM_RANGE_MAX)
        far = S.PILLAR_CONFIRM_RANGE_MAX + 1.0
        # build up to one short of confirmation, then a far reading wipes it
        for _ in range(S.PILLAR_CONFIRM_HITS - 1):
            pt.observe("blue", 1.0, 1.0, rng)
        pt.observe("blue", 1.0, 1.0, far)          # resets the close streak
        pt.observe("blue", 1.0, 1.0, rng)          # only 1 in the streak now
        self.assertFalse(pt.is_confirmed("blue"))


if __name__ == "__main__":
    unittest.main()
