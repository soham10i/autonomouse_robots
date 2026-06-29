"""Likelihood field for correlative scan matching.

Given the occupied cells of the map, the likelihood field assigns every cell a
score that decays with distance to the nearest wall::

    field(cell) = exp( -d(cell)^2 / (2 * sigma^2) )

where ``d`` is the Euclidean distance (in metres) to the closest occupied cell.
A scan placed at a candidate pose is scored by summing ``field`` over the cells
its endpoints fall in: the better the endpoints line up with mapped walls, the
higher the score.  This is the standard front-end used by gmapping-style SLAM
and is robust in featureless corridors because it needs no point
correspondences.
"""
from __future__ import annotations

import numpy as np

import settings as S


class LikelihoodField:
    def __init__(self, origin, res, cells):
        self.origin = np.asarray(origin, dtype=np.float64)
        self.res = res
        self.cells = cells
        self.field = np.zeros((cells, cells), dtype=np.float32)
        self.n_occupied = 0

    @classmethod
    def from_grid(cls, grid, sigma_m=None):
        lf = cls(grid.origin, grid.res, grid.cells)
        lf.rebuild(grid, sigma_m)
        return lf

    def rebuild(self, grid, sigma_m=None):
        """Recompute the field from the grid's current occupied mask."""
        sigma_m = S.SM_SIGMA_M if sigma_m is None else sigma_m
        occ = grid.occupied_mask()
        self.n_occupied = int(occ.sum())
        if self.n_occupied == 0:
            self.field[:] = 0.0
            return
        try:
            from scipy.ndimage import distance_transform_edt
            dist = distance_transform_edt(~occ) * self.res
        except Exception:
            # Crude fallback: only occupied cells score.
            self.field[:] = 0.0
            self.field[occ] = 1.0
            return
        self.field = np.exp(-(dist * dist) / (2.0 * sigma_m * sigma_m)).astype(np.float32)

    def score_world(self, pts_world):
        """Sum the field over the cells that ``pts_world`` (N, 2) fall in.

        Points outside the grid contribute zero.
        """
        if self.n_occupied == 0 or pts_world.shape[0] == 0:
            return 0.0
        ix = np.floor((pts_world[:, 0] - self.origin[0]) / self.res).astype(np.int32)
        iy = np.floor((pts_world[:, 1] - self.origin[1]) / self.res).astype(np.int32)
        m = (ix >= 0) & (ix < self.cells) & (iy >= 0) & (iy < self.cells)
        if not m.any():
            return 0.0
        return float(self.field[ix[m], iy[m]].sum())
