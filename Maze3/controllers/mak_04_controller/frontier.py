"""Frontier detection + clustering for exploration.

A *frontier* cell is FREE and 4-adjacent to at least one UNKNOWN cell — the
boundary between explored and unexplored space.  Clusters of frontier cells are
candidate exploration goals; the main controller ranks them by
``info_gain - dist`` using A* path length and picks the best reachable one.
"""
from __future__ import annotations

import math

import numpy as np

import config as C


def detect_frontier_cells(grid, lethal):
    """Return a boolean grid of frontier cells (free, touching unknown, safe)."""
    free = grid.free_mask()
    unknown = grid.unknown_mask()
    # a free cell adjacent (4-conn) to any unknown cell
    adj_unknown = np.zeros_like(unknown)
    adj_unknown[1:, :] |= unknown[:-1, :]
    adj_unknown[:-1, :] |= unknown[1:, :]
    adj_unknown[:, 1:] |= unknown[:, :-1]
    adj_unknown[:, :-1] |= unknown[:, 1:]
    frontier = free & adj_unknown & (~lethal)
    return frontier


def _cluster(frontier):
    """Label 8-connected frontier clusters -> list of (cells list)."""
    n = frontier.shape[0]
    visited = np.zeros_like(frontier)
    clusters = []
    xs, ys = np.nonzero(frontier)
    cells = set(zip(xs.tolist(), ys.tolist()))
    for seed in list(cells):
        if visited[seed]:
            continue
        stack = [seed]
        comp = []
        visited[seed] = True
        while stack:
            cx, cy = stack.pop()
            comp.append((cx, cy))
            for dx in (-1, 0, 1):
                for dy in (-1, 0, 1):
                    if dx == 0 and dy == 0:
                        continue
                    nx, ny = cx + dx, cy + dy
                    if 0 <= nx < n and 0 <= ny < n and frontier[nx, ny] and not visited[nx, ny]:
                        visited[nx, ny] = True
                        stack.append((nx, ny))
        clusters.append(comp)
    return clusters


def find_frontiers(grid, lethal, robot_xy, start_xy, extra_radius=0.0):
    """Return a list of candidate dicts: ``{cell, world, size}`` (filtered).

    Filtered by minimum cluster size, a minimum distance from the robot, and a
    maximum radius from the start pose (so a gap in the boundary cannot lure the
    robot off into the unbounded floor outside the maze). ``extra_radius`` widens
    that cap temporarily (see ``config.EXPL_RADIUS_RELAX_*``) when the caller has
    been unable to find any in-range frontier for a while.
    """
    frontier = detect_frontier_cells(grid, lethal)
    if not frontier.any():
        return []
    clusters = _cluster(frontier)
    rx, ry = robot_xy
    sx, sy = start_xy
    out = []
    for comp in clusters:
        if len(comp) < C.FRONTIER_MIN_CLUSTER:
            continue
        arr = np.array(comp)
        # centroid cell -> snap to an actual frontier cell for a valid goal
        cix = int(round(arr[:, 0].mean()))
        ciy = int(round(arr[:, 1].mean()))
        # nearest cluster cell to the centroid
        d2 = (arr[:, 0] - cix) ** 2 + (arr[:, 1] - ciy) ** 2
        cix, ciy = arr[int(np.argmin(d2))]
        wx, wy = grid.grid_to_world(cix, ciy)
        if math.hypot(wx - rx, wy - ry) < C.FRONTIER_MIN_DIST_M:
            continue
        if math.hypot(wx - sx, wy - sy) > C.EXPL_MAX_RADIUS_M + extra_radius:
            continue
        out.append({"cell": (int(cix), int(ciy)), "world": (wx, wy), "size": len(comp)})
    return out
