import unittest

import tests._bootstrap  # noqa: F401
import numpy as np

import settings as S
from slam.occupancy_grid import OccupancyGrid


class TestOccupancyGrid(unittest.TestCase):
    def test_coord_roundtrip(self):
        g = OccupancyGrid()
        for wx, wy in [(0.0, 0.0), (1.23, -2.1), (-3.9, 3.9)]:
            ix, iy = g.world_to_grid(wx, wy)
            bx, by = g.grid_to_world(ix, iy)
            self.assertLess(abs(bx - wx), g.res)
            self.assertLess(abs(by - wy), g.res)

    def test_integrate_scan_wall_free_unknown(self):
        g = OccupancyGrid()
        pose = (0.0, 0.0, 0.0)
        ys = np.linspace(-0.2, 0.2, 9)
        hits = np.stack([np.full_like(ys, 1.0), ys], axis=1)
        g.integrate_scan(pose, hits)

        wix, wiy = g.world_to_grid(1.0, 0.0)
        self.assertTrue(g.occupied_mask()[wix, wiy], "wall cell should be occupied")

        mix, miy = g.world_to_grid(0.5, 0.0)
        self.assertTrue(g.free_mask()[mix, miy], "midpoint should be free")

        fix, fiy = g.world_to_grid(3.0, 0.0)
        self.assertTrue(g.unknown_mask()[fix, fiy], "far cell should be unknown")

    def test_aux_hit_count_gating(self):
        g = OccupancyGrid()
        pt = np.array([[2.0, 0.0]])
        for _ in range(S.AUX_MIN_HITS - 1):
            g.mark_aux_points(pt)
        ix, iy = g.world_to_grid(2.0, 0.0)
        self.assertFalse(g.aux_mask()[ix, iy], "below threshold => not an obstacle")
        g.mark_aux_points(pt)  # reach threshold
        self.assertTrue(g.aux_mask()[ix, iy], "at threshold => obstacle")

    def test_aux_decay_where_free(self):
        g = OccupancyGrid()
        ix, iy = g.world_to_grid(0.0, 0.0)
        g.L[ix, iy] = S.L_MIN                      # lidar says free
        wx, wy = g.grid_to_world(ix, iy)
        for _ in range(S.AUX_MIN_HITS):
            g.mark_aux_points(np.array([[wx, wy]]))
        self.assertTrue(g.aux_mask()[ix, iy])
        cleared = g.decay_aux_where_free()
        self.assertGreaterEqual(cleared, 1)
        self.assertFalse(g.aux_mask()[ix, iy])

    def test_sticky_floating_wall_survives_decay(self):
        # A floating wall's cell reads FREE on the lidar, but a sticky aux mark
        # (depth-confirmed / bumper-stamped) must NOT be erased by decay.
        g = OccupancyGrid()
        ix, iy = g.world_to_grid(1.0, 0.5)
        g.L[ix, iy] = S.L_MIN                      # lidar says free
        wx, wy = g.grid_to_world(ix, iy)
        g.mark_aux_disc(wx, wy, 0.04)              # sticky stamp
        self.assertTrue(g.aux_mask()[ix, iy])
        g.decay_aux_where_free()
        self.assertTrue(g.aux_mask()[ix, iy], "sticky floating wall was erased")

    def test_low_count_aux_still_decays(self):
        g = OccupancyGrid()
        ix, iy = g.world_to_grid(-1.0, 0.0)
        g.L[ix, iy] = S.L_MIN
        wx, wy = g.grid_to_world(ix, iy)
        for _ in range(S.AUX_MIN_HITS):            # below sticky threshold
            g.mark_aux_points(np.array([[wx, wy]]))
        self.assertTrue(g.aux_mask()[ix, iy])
        g.decay_aux_where_free()
        self.assertFalse(g.aux_mask()[ix, iy], "noise-level aux should still clear")

    def test_reconcile_clears_sticky_aux_shadowing_a_wall(self):
        # Sticky aux sprayed just in FRONT of a real lidar wall is redundant
        # (the lidar already maps the wall) and must be cleared even though it is
        # sticky — this is the corner-bulge fix.
        g = OccupancyGrid()
        wix, wiy = g.world_to_grid(1.0, 0.0)
        g.L[wix, wiy] = S.L_MAX                     # real lidar wall
        ax, ay = g.grid_to_world(wix - 1, wiy)      # aux one cell in front
        g.mark_aux_disc(ax, ay, 0.02)              # sticky
        self.assertTrue(g.aux_mask()[wix - 1, wiy])
        cleared = g.reconcile_aux_with_walls()
        self.assertGreaterEqual(cleared, 1)
        self.assertFalse(g.aux_mask()[wix - 1, wiy], "wall-shadow aux not cleared")

    def test_reconcile_keeps_isolated_floating_wall(self):
        # A genuine floating wall has NO lidar-occupied neighbour, so it must
        # survive reconciliation.
        g = OccupancyGrid()
        ix, iy = g.world_to_grid(-1.5, 1.5)
        wx, wy = g.grid_to_world(ix, iy)
        g.mark_aux_disc(wx, wy, 0.04)              # sticky floating wall, no wall near
        self.assertTrue(g.aux_mask()[ix, iy])
        g.reconcile_aux_with_walls()
        self.assertTrue(g.aux_mask()[ix, iy], "isolated floating wall was erased")

    def test_free_disc_clears_sticky_aux_in_footprint(self):
        # The robot's own footprint is provably free, so mark_free_disc must
        # clear even sticky aux there (else a false-aux box traps it forever).
        g = OccupancyGrid()
        g.mark_aux_disc(0.0, 0.0, 0.04)           # sticky aux at the robot
        ix, iy = g.world_to_grid(0.0, 0.0)
        self.assertTrue(g.aux_mask()[ix, iy])
        g.mark_free_disc(0.0, 0.0, S.ROBOT_RADIUS * 0.85)
        self.assertFalse(g.aux_mask()[ix, iy], "sticky aux not cleared in footprint")

    def test_clear_aux_disc_removes_sticky(self):
        g = OccupancyGrid()
        g.mark_aux_disc(1.0, 1.0, 0.06)           # sticky blob
        ix, iy = g.world_to_grid(1.0, 1.0)
        self.assertTrue(g.aux_mask()[ix, iy])
        cleared = g.clear_aux_disc(1.0, 1.0, 0.25)
        self.assertGreaterEqual(cleared, 1)
        self.assertFalse(g.aux_mask()[ix, iy])

    def test_barrier_is_lethal_and_survives_all_clears(self):
        # A learned floating-wall barrier must be lethal for planning AND immune
        # to every aux-clearing path (footprint clear, escape clear, decay,
        # reconcile) — otherwise the robot re-routes under the wall it learned.
        g = OccupancyGrid()
        g.mark_barrier_disc(1.0, 1.0, 0.10)
        ix, iy = g.world_to_grid(1.0, 1.0)
        self.assertTrue(g.barrier[ix, iy])
        self.assertTrue(g.lethal_mask()[ix, iy], "barrier not lethal")
        _, hard = g.costmap()
        self.assertTrue(hard[ix, iy], "barrier not hard-lethal in costmap")
        # none of the clearing operations may remove it
        g.mark_free_disc(1.0, 1.0, 0.30)
        g.clear_aux_disc(1.0, 1.0, 0.30)
        g.L[ix, iy] = S.L_MIN
        g.decay_aux_where_free()
        g.reconcile_aux_with_walls()
        self.assertTrue(g.barrier[ix, iy], "barrier was wrongly cleared")

    def test_nearest_lethal_dist_and_is_lethal_world(self):
        # Recovery uses these to decide whether there is room to rotate (and not
        # to back into poison/walls).  A single wall cell 0.20 m away must read as
        # ~0.20 m; a point on that cell must read lethal; open space must be inf.
        g = OccupancyGrid()
        wx, wy = 1.0, 1.0
        ix, iy = g.world_to_grid(wx + 0.20, wy)     # wall 0.20 m to the +x side
        g.L[ix, iy] = S.L_MAX
        d = g.nearest_lethal_dist(wx, wy, max_r=0.5)
        self.assertAlmostEqual(d, 0.20, delta=g.res)      # within one cell
        self.assertTrue(g.is_lethal_world(wx + 0.20, wy))
        self.assertFalse(g.is_lethal_world(wx, wy))
        # nothing within the window -> infinite clearance (free to rotate)
        self.assertEqual(g.nearest_lethal_dist(-3.0, -3.0, max_r=0.3), float("inf"))
        # poison and off-map both count as lethal for the reverse guard
        g.poison[g.world_to_grid(0.0, 0.0)] = True
        self.assertTrue(g.is_lethal_world(0.0, 0.0))
        self.assertTrue(g.is_lethal_world(99.0, 99.0))    # off-map

    def test_depth_layer_marks_and_is_lethal(self):
        g = OccupancyGrid()
        g.integrate_depth_rays((0.0, 0.0, 0.0), np.array([[1.0, 0.0]]), np.empty((0, 2)))
        ix, iy = g.world_to_grid(1.0, 0.0)
        self.assertTrue(g.depth_obs_mask()[ix, iy], "depth hit not marked")
        self.assertTrue(g.lethal_mask()[ix, iy])
        _, hard = g.costmap()
        self.assertTrue(hard[ix, iy], "depth obstacle not hard-lethal for A*")

    def test_depth_layer_raytrace_clears_stale_mark(self):
        # A floating wall is marked, then disappears; depth seeing clear THROUGH
        # that bearing must erase it (self-correcting raytrace clearing).
        g = OccupancyGrid()
        for _ in range(3):
            g.integrate_depth_rays((0.0, 0.0, 0.0), np.array([[1.0, 0.0]]), np.empty((0, 2)))
        ix, iy = g.world_to_grid(1.0, 0.0)
        self.assertTrue(g.depth_obs_mask()[ix, iy])
        for _ in range(8):
            g.integrate_depth_rays((0.0, 0.0, 0.0), np.empty((0, 2)), np.array([[1.6, 0.0]]))
        self.assertFalse(g.depth_obs_mask()[ix, iy], "stale depth mark not cleared")

    def test_lidar_does_not_touch_depth_layer(self):
        # The lidar passes UNDER a floating wall and would mark its cell free;
        # it must never write the depth layer, or the floating wall is lost.
        g = OccupancyGrid()
        g.integrate_depth_rays((0.0, 0.0, 0.0), np.array([[1.0, 0.0]]), np.empty((0, 2)))
        ix, iy = g.world_to_grid(1.0, 0.0)
        before = float(g.depth_L[ix, iy])
        g.integrate_scan((0.0, 0.0, 0.0), np.array([[2.0, 0.0]]))   # lidar sweeps through
        self.assertEqual(float(g.depth_L[ix, iy]), before, "lidar altered depth layer")
        self.assertTrue(g.depth_obs_mask()[ix, iy], "floating wall lost to lidar")

    def test_depth_mark_skipped_where_lidar_wall(self):
        # depth_obs is ONLY for obstacles the lidar misses; a depth hit on a cell
        # the lidar already sees as a wall must be skipped (no double-mapping).
        g = OccupancyGrid()
        ix, iy = g.world_to_grid(1.0, 0.0)
        g.L[ix, iy] = S.L_MAX                       # lidar wall here
        for _ in range(3):
            g.integrate_depth_rays((0.0, 0.0, 0.0), np.array([[1.0, 0.0]]), np.empty((0, 2)))
        self.assertFalse(g.depth_obs_mask()[ix, iy], "depth double-marked a lidar wall")

    def test_depth_blind_zone_mark_not_cleared(self):
        # A low slab marked while far must NOT be erased once it enters the depth
        # blind zone and the camera sees clear past it (the on-approach erasure
        # that let the robot drive under WallMedium(9)).
        g = OccupancyGrid()
        for _ in range(3):
            g.integrate_depth_rays((0.0, 0.0, 0.0), np.array([[0.4, 0.0]]), np.empty((0, 2)))
        ix, iy = g.world_to_grid(0.4, 0.0)
        self.assertTrue(g.depth_obs_mask()[ix, iy])
        for _ in range(8):                          # now sees clear out to 1.6 m
            g.integrate_depth_rays((0.0, 0.0, 0.0), np.empty((0, 2)), np.array([[1.6, 0.0]]))
        self.assertTrue(g.depth_obs_mask()[ix, iy], "blind-zone slab wrongly cleared")

    def test_poison_preserved_by_free_disc(self):
        g = OccupancyGrid()
        g.mark_poison_points(np.array([0.0]), np.array([0.0]))
        self.assertTrue(g.is_poison_world(0.0, 0.0))
        g.mark_free_disc(0.0, 0.0, 0.3)
        self.assertTrue(g.is_poison_world(0.0, 0.0), "poison must never be erased")


if __name__ == "__main__":
    unittest.main()
