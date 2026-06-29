"""Phase C — frontier detection and clustering.

A *frontier* is the boundary between mapped-free and not-yet-seen space: a free
cell that touches at least one unknown cell.  Driving to frontiers is what makes
the robot explore — each one is a doorway into unmapped territory.  We cluster
the frontier cells into connected blobs and return a representative,
*stand-able* cell per cluster (one the robot footprint actually fits on), so the
planner is never handed a goal jammed against a wall.
"""
from __future__ import annotations

import numpy as np

import settings as S


class Frontier:
    __slots__ = ("centroid_world", "cell", "size")

    def __init__(self, centroid_world, cell, size):
        self.centroid_world = centroid_world   # (wx, wy)
        self.cell = cell                       # (ix, iy) representative free cell
        self.size = size                       # number of frontier cells

    def __repr__(self):
        return (f"Frontier(c=({self.centroid_world[0]:.2f},"
                f"{self.centroid_world[1]:.2f}), size={self.size})")


def _neighbour_unknown(unknown):
    """True where a cell has a 4-connected unknown neighbour."""
    out = np.zeros_like(unknown)
    out[:-1, :] |= unknown[1:, :]
    out[1:, :] |= unknown[:-1, :]
    out[:, :-1] |= unknown[:, 1:]
    out[:, 1:] |= unknown[:, :-1]
    return out


def find_frontiers(grid, min_cluster=None):
    """Return a list of :class:`Frontier` for the grid's current state."""
    min_cluster = S.FRONTIER_MIN_CLUSTER if min_cluster is None else min_cluster

    free = grid.free_mask()
    unknown = grid.unknown_mask()
    # Only keep frontier cells the robot footprint can actually occupy.
    standable = ~grid.inflated_lethal(S.ROBOT_RADIUS)
    frontier_mask = free & _neighbour_unknown(unknown) & standable
    if not frontier_mask.any():
        return []

    try:
        from scipy.ndimage import label
        structure = np.ones((3, 3), dtype=int)   # 8-connectivity
        labels, n = label(frontier_mask, structure=structure)
    except Exception:
        # Fallback: treat the whole mask as one cluster.
        labels = frontier_mask.astype(int)
        n = 1

    frontiers = []
    for lab in range(1, n + 1):
        cells = np.argwhere(labels == lab)
        if cells.shape[0] < min_cluster:
            continue
        mean_ix = float(cells[:, 0].mean())
        mean_iy = float(cells[:, 1].mean())
        # representative cell = cluster member nearest the centroid
        d2 = (cells[:, 0] - mean_ix) ** 2 + (cells[:, 1] - mean_iy) ** 2
        rep = cells[int(np.argmin(d2))]
        wx, wy = grid.grid_to_world(int(rep[0]), int(rep[1]))
        frontiers.append(Frontier((wx, wy), (int(rep[0]), int(rep[1])), cells.shape[0]))
    return frontiers
