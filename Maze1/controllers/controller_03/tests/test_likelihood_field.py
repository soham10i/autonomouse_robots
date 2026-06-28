import unittest

import tests._bootstrap  # noqa: F401
import numpy as np

import settings as S
from slam.occupancy_grid import OccupancyGrid
from slam.likelihood_field import LikelihoodField


class TestLikelihoodField(unittest.TestCase):
    def _wall_grid(self):
        g = OccupancyGrid()
        # vertical wall at x = 1.0 across a y band
        ys = np.linspace(-0.5, 0.5, 26)
        xs = np.full_like(ys, 1.0)
        ix, iy, m = g.world_to_grid_arr(xs, ys)
        g.L[ix[m], iy[m]] = S.L_MAX
        return g

    def test_alignment_scores_higher(self):
        g = self._wall_grid()
        lf = LikelihoodField.from_grid(g)
        self.assertGreater(lf.n_occupied, 10)

        ys = np.linspace(-0.5, 0.5, 26)
        on_wall = np.stack([np.full_like(ys, 1.0), ys], axis=1)
        off_wall = on_wall + np.array([0.3, 0.0])  # shifted 30 cm off

        self.assertGreater(lf.score_world(on_wall), lf.score_world(off_wall))

    def test_empty_map_zero(self):
        g = OccupancyGrid()
        lf = LikelihoodField.from_grid(g)
        self.assertEqual(lf.n_occupied, 0)
        self.assertEqual(lf.score_world(np.array([[0.0, 0.0]])), 0.0)


if __name__ == "__main__":
    unittest.main()
