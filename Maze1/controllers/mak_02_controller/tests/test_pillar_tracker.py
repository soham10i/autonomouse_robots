import unittest

import tests._bootstrap  # noqa: F401

import settings as S
from perception.pillar_tracker import PillarTracker


class TestPillarTracker(unittest.TestCase):
    def test_far_detections_locate_but_never_confirm(self):
        # Seeing the pillar from afar (range > PILLAR_DEPTH_CONFIRM_RANGE) gives a
        # position estimate but must NOT confirm — the robot hasn't driven up to it.
        t = PillarTracker()
        for _ in range(6):
            t.observe("blue", 2.0, 0.0, 2.0)
        self.assertTrue(t.known("blue"))
        self.assertAlmostEqual(t.pos["blue"][0], 2.0, places=3)
        self.assertFalse(t.is_confirmed("blue"))

    def test_close_detections_confirm_and_snap(self):
        # Readings inside the depth confirm band, agreeing with the estimate,
        # confirm after PILLAR_CONFIRM_HITS and snap the estimate to the reading.
        t = PillarTracker()
        for _ in range(S.PILLAR_CONFIRM_HITS):
            t.observe("blue", 2.0, 0.0, 0.8)
        self.assertTrue(t.is_confirmed("blue"))
        self.assertAlmostEqual(t.pos["blue"][0], 2.0, places=3)
        self.assertAlmostEqual(t.pos["blue"][1], 0.0, places=3)

    def test_close_reading_corrects_a_through_wall_biased_estimate(self):
        # THE BUG: a far/through-wall view biases the estimate (~0.4 m off the true
        # pillar).  Driving up and taking close readings of the TRUE pillar must
        # both confirm AND snap the estimate onto the true position.
        t = PillarTracker()
        biased = (1.74, -0.06)        # what the through-wall view produced
        true = (1.96, -0.41)          # the actual blue pillar (this run's frame)
        for _ in range(3):            # establish the biased estimate (far)
            t.observe("blue", biased[0], biased[1], 2.0)
        self.assertFalse(t.is_confirmed("blue"))
        for _ in range(S.PILLAR_CONFIRM_HITS):   # close readings of the real pillar
            t.observe("blue", true[0], true[1], 0.8)
        self.assertTrue(t.is_confirmed("blue"))
        self.assertAlmostEqual(t.pos["blue"][0], true[0], places=3)
        self.assertAlmostEqual(t.pos["blue"][1], true[1], places=3)

    def test_close_but_inconsistent_reading_does_not_confirm(self):
        # A close reading far from the tracked estimate (beyond SNAP_TOL) is a
        # different blob, not the tracked pillar — it must not confirm.
        t = PillarTracker()
        for _ in range(S.PILLAR_OBS_AVG_N):       # firmly set the estimate at (2,0)
            t.observe("blue", 2.0, 0.0, 2.0)
        for _ in range(S.PILLAR_CONFIRM_HITS):    # close, but ~1 m off the estimate
            t.observe("blue", 2.0, 1.0, 0.8)
        self.assertFalse(t.is_confirmed("blue"))

    def test_position_only_update_never_confirms(self):
        t = PillarTracker()
        for _ in range(6):
            t.update("blue", 2.0, 0.0)            # rng = inf -> cannot confirm
        self.assertTrue(t.known("blue"))
        self.assertFalse(t.is_confirmed("blue"))


if __name__ == "__main__":
    unittest.main()
