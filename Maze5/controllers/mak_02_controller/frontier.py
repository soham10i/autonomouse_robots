"""Frontier detection and clustering logic for autonomous exploration.

A *frontier* cell is defined as a FREE cell that is 4-adjacent to at least one
UNKNOWN cell, representing the boundary between explored and unexplored space.
Clusters of frontier cells act as candidate exploration goals. The main controller
evaluates these candidates by a heuristic prioritizing information gain (cluster size)
and penalizing path distance, selecting the optimal reachable frontier.
"""
from __future__ import annotations

import math
from typing import Any, TYPE_CHECKING

import numpy as np

import config as C

if TYPE_CHECKING:
    from mapping import OccupancyGrid


def detect_frontier_cells(grid: OccupancyGrid, lethal: np.ndarray) -> np.ndarray:
    """Identifies all valid frontier cells within the current occupancy grid.

    A valid frontier cell must be marked as free space, must be 4-connected to
    at least one unknown cell, and must not overlap with lethal obstacle regions.

    Args:
        grid (OccupancyGrid): The current log-odds occupancy map instance.
        lethal (np.ndarray): A 2D boolean array where True indicates a lethal cell
            (impassable obstacles and their hard inflation radius).

    Returns:
        np.ndarray: A 2D boolean array of the same shape as the grid, where True
            indicates a valid frontier cell.
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


def _cluster(frontier: np.ndarray) -> list[list[tuple[int, int]]]:
    """Groups contiguous frontier cells into 8-connected components (clusters).

    Args:
        frontier (np.ndarray): A 2D boolean array indicating frontier cells.

    Returns:
        list[list[tuple[int, int]]]: A list of clusters, where each cluster is a
            list of cell coordinates ``(x, y)`` belonging to that connected component.
    """
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


def find_frontiers(
    grid: OccupancyGrid,
    lethal: np.ndarray,
    robot_xy: tuple[float, float],
    start_xy: tuple[float, float]
) -> list[dict[str, Any]]:
    """Extracts, filters, and formats frontier clusters as candidate navigation goals.

    Clusters are filtered out if they fall below a minimum size threshold, lie too close
    to the current robot position, or exceed a maximum exploration radius measured from
    the initial starting pose. The maximum radius bounds the exploration to prevent
    the robot from chasing gaps onto unbounded exterior regions.

    Args:
        grid (OccupancyGrid): The current occupancy map instance.
        lethal (np.ndarray): A 2D boolean array indicating lethal (impassable) cells.
        robot_xy (tuple[float, float]): The current ``(x, y)`` world coordinates of the robot.
        start_xy (tuple[float, float]): The initial ``(x, y)`` world coordinates of the robot
            at mission start.

    Returns:
        list[dict[str, Any]]: A list of dictionaries representing viable frontier goals.
            Each dictionary contains:
            - ``"cell"`` (tuple[int, int]): Grid coordinates of the representative frontier cell.
            - ``"world"`` (tuple[float, float]): World coordinates of the representative cell.
            - ``"size"`` (int): The number of cells comprising the frontier cluster.
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
        cix, ciy = arr[int(np.argmin(d2))]  # type: ignore
        wx, wy = grid.grid_to_world(cix, ciy)
        if math.hypot(wx - rx, wy - ry) < C.FRONTIER_MIN_DIST_M:
            continue
        if math.hypot(wx - sx, wy - sy) > C.EXPL_MAX_RADIUS_M:
            continue
        out.append({"cell": (int(cix), int(ciy)), "world": (wx, wy), "size": len(comp)})
    return out
