"""Frontier detection + clustering for exploration.

A *frontier* cell is FREE and 4-adjacent to at least one UNKNOWN cell — the
boundary between explored and unexplored space.  Clusters of frontier cells are
candidate exploration goals; the main controller ranks them by
``info_gain - dist`` using A* path length and picks the best reachable one.
"""
from __future__ import annotations

import math
from typing import Any, Dict, List, Set, Tuple

import numpy as np
import numpy.typing as npt

import config as C


def detect_frontier_cells(grid: Any,
                          lethal: npt.NDArray[np.bool_]) -> npt.NDArray[np.bool_]:
    """Return a boolean grid of valid frontier cells.

    A valid frontier cell must be:
      1. Explored and free (according to ``grid.free_mask()``)
      2. 4-adjacent to at least one unexplored cell
      3. Safe (not marked as ``lethal``)

    Args:
        grid: The main :class:`mapping.OccupancyGrid` instance.
        lethal: ``(N, N)`` boolean mask of impassable obstacles (to exclude
            frontiers that lie too close to walls).

    Returns:
        ``(N, N)`` boolean mask where ``True`` marks a frontier cell.
    """
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


def _cluster(frontier: npt.NDArray[np.bool_]) -> List[List[Tuple[int, int]]]:
    """Extract 8-connected clusters of frontier cells.

    Args:
        frontier: ``(N, N)`` boolean mask of frontier cells.

    Returns:
        A list of clusters, where each cluster is a list of ``(x, y)`` cell
        coordinates belonging to that connected component.
    """
    n = frontier.shape[0]
    visited = np.zeros_like(frontier)
    clusters: List[List[Tuple[int, int]]] = []
    xs, ys = np.nonzero(frontier)
    cells: Set[Tuple[int, int]] = set(zip(xs.tolist(), ys.tolist()))
    for seed in list(cells):
        if visited[seed]:
            continue
        stack: List[Tuple[int, int]] = [seed]
        comp: List[Tuple[int, int]] = []
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


def find_frontiers(grid: Any,
                   lethal: npt.NDArray[np.bool_],
                   robot_xy: Tuple[float, float],
                   start_xy: Tuple[float, float],
                   extra_radius: float = 0.0) -> List[Dict[str, Any]]:
    """Return a filtered list of candidate frontier goals.

    Filters raw frontier clusters by:
      1. Minimum cluster size (rejects single-pixel noise).
      2. Minimum distance from the robot (rejects trivial goals immediately
         underneath the chassis).
      3. Maximum radius from the start pose (prevents the robot from driving
         out the maze entrance into the unbounded void).

    The centroid of each surviving cluster is snapped to the nearest actual
    cell belonging to that cluster, to ensure the exact goal coordinate is
    guaranteed reachable (not e.g. floating in the middle of a U-shaped wall).

    Args:
        grid: The main :class:`mapping.OccupancyGrid` instance.
        lethal: ``(N, N)`` boolean mask of impassable obstacles.
        robot_xy: Current robot world position ``(x, y)`` in metres.
        start_xy: Initial robot world position ``(x, y)`` in metres.
        extra_radius: Additional meters to widen the start-radius cap
            (used when the controller temporarily relaxes the cap).

    Returns:
        A list of dictionary descriptors for each valid frontier:
        ``{"cell": (int, int), "world": (float, float), "size": int}``.
    """
    frontier = detect_frontier_cells(grid, lethal)
    if not frontier.any():
        return []
    clusters = _cluster(frontier)
    rx, ry = robot_xy
    sx, sy = start_xy
    out: List[Dict[str, Any]] = []
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
