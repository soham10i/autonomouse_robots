import unittest

import tests._bootstrap  # noqa: F401

import numpy as np

import settings as S
from perception.pillar_tracker import PillarTracker, select_viewpoint
from slam.occupancy_grid import OccupancyGrid


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

    def test_through_wall_sighting_does_not_confirm(self):
        # A close, consistent reading but with NO clear ground path (a floating
        # wall between robot and pillar) must NOT confirm — the robot is seeing
        # the pillar under the wall, not standing at it (WallMedium(3) case).
        t = PillarTracker()
        for _ in range(S.PILLAR_CONFIRM_HITS):
            t.observe("blue", 2.0, 0.0, 0.8, los_clear=False)
        self.assertFalse(t.is_confirmed("blue"))
        # once the path is clear (robot routed around), it confirms
        for _ in range(S.PILLAR_CONFIRM_HITS):
            t.observe("blue", 2.0, 0.0, 0.8, los_clear=True)
        self.assertTrue(t.is_confirmed("blue"))

    def test_position_only_update_never_confirms(self):
        t = PillarTracker()
        for _ in range(6):
            t.update("blue", 2.0, 0.0)            # rng = inf -> cannot confirm
        self.assertTrue(t.known("blue"))
        self.assertFalse(t.is_confirmed("blue"))


class TestSelectViewpoint(unittest.TestCase):
    def test_picks_clear_side_of_enclosed_pillar(self):
        # Pillar at origin enclosed on N/E/W (open only to the SOUTH), robot to
        # the north.  The viewpoint must be on the clear south side with a clear
        # line of sight — not parked behind a wall (the enclosed-blue case).
        g = OccupancyGrid()
        for t in np.arange(-0.7, 0.7 + 1e-9, g.res):
            for (wx, wy) in [(t, 0.35), (0.35, t), (-0.35, t)]:   # N, E, W walls
                ix, iy = g.world_to_grid(wx, wy)
                g.L[ix, iy] = S.L_MAX
        vp = select_viewpoint(g, (0.0, 0.0), (0.0, 1.5))   # robot to the north
        self.assertIsNotNone(vp)
        self.assertLess(vp[1], 0.0, "viewpoint should be on the open (south) side")
        self.assertTrue(g.segment_clear(vp[0], vp[1], 0.0, 0.0,
                                        stop_short=S.PILLAR_LOS_STOP_SHORT))

    def test_open_pillar_returns_a_viewpoint(self):
        g = OccupancyGrid()
        vp = select_viewpoint(g, (0.0, 0.0), (1.0, 0.0))
        self.assertIsNotNone(vp)
        r = np.hypot(vp[0], vp[1])
        self.assertGreaterEqual(r, min(S.PILLAR_VIEW_RADII) - 1e-9)


if __name__ == "__main__":
    unittest.main()
