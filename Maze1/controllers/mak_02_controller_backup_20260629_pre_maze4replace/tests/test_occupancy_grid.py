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

    # --- Maze4-ported boolean aux floating-wall layer (integrate_aux) ----------
    @staticmethod
    def _hit_col(rng):
        return (np.array([0.0]), np.array([float(rng)]),
                np.array([True]), np.array([False]))   # bearings,ranges,hit,clear

    @staticmethod
    def _clear_col():
        return (np.array([0.0]), np.array([0.0]),
                np.array([False]), np.array([True]))

    def test_aux_layer_marks_and_is_lethal(self):
        g = OccupancyGrid()
        b, r, h, c = self._hit_col(1.0)
        g.integrate_aux((0.0, 0.0, 0.0), b, r, h, c, depth_min=0.05)
        ix, iy = g.world_to_grid(1.0, 0.0)
        self.assertTrue(g.depth_obs_mask()[ix, iy], "depth hit not marked")
        self.assertTrue(g.lethal_mask()[ix, iy])
        _, hard = g.costmap()
        self.assertTrue(hard[ix, iy], "floating wall not hard-lethal for A*")

    def test_aux_raytrace_clears_stale_mark(self):
        # A floating wall is marked, then disappears; depth seeing CLEAR through
        # that bearing must erase it (boolean self-correcting raytrace clear).
        g = OccupancyGrid()
        b, r, h, c = self._hit_col(1.0)
        g.integrate_aux((0.0, 0.0, 0.0), b, r, h, c, depth_min=0.05)
        ix, iy = g.world_to_grid(1.0, 0.0)
        self.assertTrue(g.depth_obs_mask()[ix, iy])
        b, r, h, c = self._clear_col()
        g.integrate_aux((0.0, 0.0, 0.0), b, r, h, c, depth_min=0.05)
        self.assertFalse(g.depth_obs_mask()[ix, iy], "stale aux mark not cleared")

    def test_lidar_does_not_touch_aux_layer(self):
        # The lidar passes UNDER a floating wall and marks its cell free; it must
        # never write the aux layer, or the floating wall is lost.
        g = OccupancyGrid()
        b, r, h, c = self._hit_col(1.0)
        g.integrate_aux((0.0, 0.0, 0.0), b, r, h, c, depth_min=0.05)
        ix, iy = g.world_to_grid(1.0, 0.0)
        g.integrate_scan((0.0, 0.0, 0.0), np.array([[2.0, 0.0]]))   # lidar through it
        self.assertTrue(g.aux[ix, iy], "lidar erased the floating wall from aux")
        self.assertTrue(g.depth_obs_mask()[ix, iy], "floating wall lost to lidar")

    def test_aux_mark_skipped_where_lidar_wall(self):
        # aux is ONLY for obstacles the lidar misses; a depth hit within
        # AUX_LIDAR_BLIND_CELLS of a lidar wall must be skipped (no double-map).
        g = OccupancyGrid()
        ix, iy = g.world_to_grid(1.0, 0.0)
        g.L[ix, iy] = S.L_MAX                       # lidar wall here
        b, r, h, c = self._hit_col(1.0)
        g.integrate_aux((0.0, 0.0, 0.0), b, r, h, c, depth_min=0.05)
        self.assertFalse(g.depth_obs_mask()[ix, iy], "aux double-marked a lidar wall")

    def test_aux_blind_zone_mark_not_cleared(self):
        # A slab marked while far must NOT be erased once it enters the depth
        # blind zone: clearing starts at depth_min, so a clear column from a pose
        # where the cell is closer than depth_min cannot reach (and erase) it.
        g = OccupancyGrid()
        b, r, h, c = self._hit_col(1.0)
        g.integrate_aux((0.0, 0.0, 0.0), b, r, h, c, depth_min=0.6)
        ix, iy = g.world_to_grid(1.0, 0.0)
        self.assertTrue(g.depth_obs_mask()[ix, iy])
        # robot has advanced to x=0.6, so the cell at x=1.0 is now 0.4 m ahead,
        # inside the 0.6 m blind zone; a clear column must not erase it.
        b, r, h, c = self._clear_col()
        g.integrate_aux((0.6, 0.0, 0.0), b, r, h, c, depth_min=0.6)
        self.assertTrue(g.depth_obs_mask()[ix, iy], "blind-zone slab wrongly cleared")

    def test_poison_preserved_by_free_disc(self):
        g = OccupancyGrid()
        g.mark_poison_points(np.array([0.0]), np.array([0.0]))
        self.assertTrue(g.is_poison_world(0.0, 0.0))
        g.mark_free_disc(0.0, 0.0, 0.3)
        self.assertTrue(g.is_poison_world(0.0, 0.0), "poison must never be erased")


if __name__ == "__main__":
    unittest.main()
