import unittest

import tests._bootstrap  # noqa: F401
import numpy as np

from exploration.astar import astar_cells, nearest_free_cell


class TestAStar(unittest.TestCase):
    def test_path_around_wall(self):
        cost = np.zeros((10, 10), dtype=float)
        hard = np.zeros((10, 10), dtype=bool)
        hard[5, 0:8] = True            # wall blocks ix=5 for iy 0..7; gap at 8,9
        path = astar_cells(cost, hard, (0, 0), (9, 0), res=0.04)
        self.assertIsNotNone(path)
        self.assertEqual(path[0], (0, 0))
        self.assertEqual(path[-1], (9, 0))
        for ix, iy in path:
            self.assertFalse(hard[ix, iy], "path must avoid lethal cells")

    def test_no_path_when_blocked(self):
        cost = np.zeros((10, 10), dtype=float)
        hard = np.zeros((10, 10), dtype=bool)
        hard[5, :] = True              # full wall, no gap
        path = astar_cells(cost, hard, (0, 0), (9, 0), res=0.04)
        self.assertIsNone(path)

    def test_prefers_lower_cost(self):
        # Two routes equal length; one has soft cost -> A* takes the cheap one.
        cost = np.zeros((3, 5), dtype=float)
        hard = np.zeros((3, 5), dtype=bool)
        cost[1, :] = 5.0               # middle row expensive
        path = astar_cells(cost, hard, (0, 2), (2, 2), res=1.0)
        self.assertIsNotNone(path)
        # should not dwell on the expensive middle row more than necessary
        middle = sum(1 for ix, iy in path if ix == 1)
        self.assertLessEqual(middle, 1)

    def test_nearest_free_cell(self):
        hard = np.zeros((5, 5), dtype=bool)
        hard[2, 2] = True
        self.assertEqual(nearest_free_cell(hard, (2, 2), 3) != (2, 2), True)


if __name__ == "__main__":
    unittest.main()
