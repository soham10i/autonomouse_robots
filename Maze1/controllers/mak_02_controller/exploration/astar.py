"""A* path planning on the occupancy costmap.

``astar_cells`` is the pure grid-space search (operates on plain NumPy arrays so
it is trivially testable).  ``Planner`` wraps an :class:`OccupancyGrid`,
building the costmap, snapping start/goal out of lethal cells, and converting
the cell path back to world coordinates.

Cost of stepping into a cell = geometric move length (``res`` or ``res*sqrt2``)
plus the cell's soft inflation cost, so A* both finds short paths and prefers
the middle of free space.  Lethal cells are never expanded.
"""
from __future__ import annotations

import heapq
import math

import numpy as np

import settings as S

_SQRT2 = math.sqrt(2.0)
_NEIGHBOURS_8 = [(-1, 0, 1.0), (1, 0, 1.0), (0, -1, 1.0), (0, 1, 1.0),
                 (-1, -1, _SQRT2), (-1, 1, _SQRT2), (1, -1, _SQRT2), (1, 1, _SQRT2)]
_NEIGHBOURS_4 = [(-1, 0, 1.0), (1, 0, 1.0), (0, -1, 1.0), (0, 1, 1.0)]


def astar_cells(cost, hard_lethal, start, goal, res=1.0, diagonal=True,
                max_nodes=80000):
    """Grid-space A*.  Returns a list of ``(ix, iy)`` cells or ``None``.

    ``cost`` is a 2D float array of soft per-cell costs (``inf`` allowed).
    ``hard_lethal`` is a bool array of impassable cells.
    """
    nx, ny = cost.shape
    sx, sy = start
    gx, gy = goal
    if not (0 <= sx < nx and 0 <= sy < ny and 0 <= gx < nx and 0 <= gy < ny):
        return None
    if hard_lethal[sx, sy] or hard_lethal[gx, gy]:
        return None

    nbrs = _NEIGHBOURS_8 if diagonal else _NEIGHBOURS_4

    g_score = {(sx, sy): 0.0}
    came_from = {}
    open_heap = [(math.hypot(gx - sx, gy - sy) * res, 0.0, (sx, sy))]
    closed = set()
    expanded = 0

    while open_heap:
        _, gc, cur = heapq.heappop(open_heap)
        if cur in closed:
            continue
        closed.add(cur)
        expanded += 1
        if cur == (gx, gy):
            return _reconstruct(came_from, cur)
        if expanded > max_nodes:
            return None

        cx, cy = cur
        for dxx, dyy, base in nbrs:
            nxx, nyy = cx + dxx, cy + dyy
            if not (0 <= nxx < nx and 0 <= nyy < ny):
                continue
            if hard_lethal[nxx, nyy]:
                continue
            soft = cost[nxx, nyy]
            if not math.isfinite(soft):
                continue
            tentative = gc + base * res + soft * res
            key = (nxx, nyy)
            if tentative < g_score.get(key, math.inf):
                g_score[key] = tentative
                came_from[key] = cur
                f = tentative + math.hypot(gx - nxx, gy - nyy) * res
                heapq.heappush(open_heap, (f, tentative, key))
    return None


def _reconstruct(came_from, cur):
    path = [cur]
    while cur in came_from:
        cur = came_from[cur]
        path.append(cur)
    path.reverse()
    return path


def nearest_free_cell(hard_lethal, cell, max_radius_cells):
    """Spiral outward from ``cell`` to the nearest non-lethal in-bounds cell."""
    nx, ny = hard_lethal.shape
    cx, cy = cell
    cx = min(max(cx, 0), nx - 1)
    cy = min(max(cy, 0), ny - 1)
    if not hard_lethal[cx, cy]:
        return (cx, cy)
    for r in range(1, max_radius_cells + 1):
        for dx in range(-r, r + 1):
            for dy in range(-r, r + 1):
                if max(abs(dx), abs(dy)) != r:
                    continue
                x, y = cx + dx, cy + dy
                if 0 <= x < nx and 0 <= y < ny and not hard_lethal[x, y]:
                    return (x, y)
    return None


class Planner:
    def __init__(self, grid):
        self.grid = grid

    def plan(self, start_world, goal_world, diagonal=True):
        """Plan from ``start_world`` to ``goal_world``.

        Returns ``(world_path, cells)`` where ``world_path`` is a list of
        ``(x, y)`` cell centres, or ``(None, None)`` if no path exists.
        """
        cost, hard = self.grid.costmap()
        snap_r = max(1, int(round(0.5 / self.grid.res)))

        s = self.grid.world_to_grid(start_world[0], start_world[1])
        g = self.grid.world_to_grid(goal_world[0], goal_world[1])
        s = nearest_free_cell(hard, s, snap_r)
        g = nearest_free_cell(hard, g, snap_r)
        if s is None or g is None:
            return None, None

        cells = astar_cells(cost, hard, s, g, res=self.grid.res, diagonal=diagonal,
                            max_nodes=80000)
        if cells is None:
            return None, None
        world = [self.grid.grid_to_world(ix, iy) for ix, iy in cells]
        return world, cells

    def path_length(self, world_path):
        if not world_path or len(world_path) < 2:
            return 0.0
        total = 0.0
        for a, b in zip(world_path[:-1], world_path[1:]):
            total += math.hypot(b[0] - a[0], b[1] - a[1])
        return total
