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

import config as C


def detect_frontier_cells(grid: Any, lethal: np.ndarray) -> np.ndarray:
    """Return a boolean grid of frontier cells (free, touching unknown, safe).
    
    Args:
        grid (Any): An OccupancyGrid instance.
        lethal (np.ndarray): The boolean lethal obstacle mask from the costmap.

    Returns:
        np.ndarray: A boolean mask where True indicates a valid frontier cell.
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


def _cluster(frontier: np.ndarray) -> List[List[Tuple[int, int]]]:
    """Label 8-connected frontier clusters -> list of (cells list).
    
    Args:
        frontier (np.ndarray): The boolean frontier mask.

    Returns:
        List[List[Tuple[int, int]]]: A list of clusters, where each cluster is a list of (x, y) cell indices.
    """
    n = frontier.shape[0]
    visited = np.zeros_like(frontier)
    clusters: List[List[Tuple[int, int]]] = []
    xs, ys = np.nonzero(frontier)
    cells: Set[Tuple[int, int]] = set(zip(xs.tolist(), ys.tolist()))
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


def find_frontiers(grid: Any, lethal: np.ndarray, robot_xy: Tuple[float, float], start_xy: Tuple[float, float]) -> List[Dict[str, Any]]:
    """Return a list of candidate dicts: ``{cell, world, size}`` (filtered).

    Filtered by minimum cluster size, a minimum distance from the robot, and a
    maximum radius from the start pose (so a gap in the boundary cannot lure the
    robot off into the unbounded floor outside the maze).
    
    Args:
        grid (Any): An OccupancyGrid instance.
        lethal (np.ndarray): The boolean lethal obstacle mask from the costmap.
        robot_xy (Tuple[float, float]): Current robot position in world coordinates (x, y).
        start_xy (Tuple[float, float]): Starting robot position in world coordinates (x, y).

    Returns:
        List[Dict[str, Any]]: A list of dictionaries, each describing a frontier candidate 
        with keys 'cell' (Tuple[int, int]), 'world' (Tuple[float, float]), and 'size' (int).
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
        if math.hypot(wx - sx, wy - sy) > C.EXPL_MAX_RADIUS_M:
            continue
        out.append({"cell": (int(cix), int(ciy)), "world": (wx, wy), "size": len(comp)})
    return out
