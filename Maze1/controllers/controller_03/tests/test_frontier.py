import unittest

import tests._bootstrap  # noqa: F401

import settings as S
from slam.occupancy_grid import OccupancyGrid
from exploration.frontier import find_frontiers


class TestFrontier(unittest.TestCase):
    def test_free_block_has_boundary_frontier(self):
        g = OccupancyGrid()
        # Carve a free block; everything around stays unknown (L == 0).
        cx, cy = g.world_to_grid(0.0, 0.0)
        for dx in range(-5, 6):
            for dy in range(-5, 6):
                g.L[cx + dx, cy + dy] = S.L_MIN  # free

        frontiers = find_frontiers(g)
        self.assertGreaterEqual(len(frontiers), 1)
        biggest = max(frontiers, key=lambda f: f.size)
        self.assertGreaterEqual(biggest.size, S.FRONTIER_MIN_CLUSTER)
        # centroid should sit near the block centre
        self.assertLess(abs(biggest.centroid_world[0]), 0.4)
        self.assertLess(abs(biggest.centroid_world[1]), 0.4)

    def test_no_frontier_when_all_known(self):
        g = OccupancyGrid()
        g.L[:] = S.L_MIN  # entirely free, nothing unknown
        self.assertEqual(find_frontiers(g), [])


if __name__ == "__main__":
    unittest.main()
