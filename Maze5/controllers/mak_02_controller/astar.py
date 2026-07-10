"""A* algorithm implementation on an occupancy costmap with line-of-sight path simplification.

This module provides a pure NumPy and standard-library implementation of the A* search
algorithm, designed to run on a discretized grid representation of the environment.
It includes post-processing capabilities to simplify the resulting cell paths into fewer,
longer linear segments by ensuring clear line-of-sight via Bresenham's line algorithm.
"""
from __future__ import annotations

import heapq
import math
from typing import Optional

import numpy as np

import config as C

_SQRT2 = math.sqrt(2.0)
_NEIGHBORS = [
    (-1, 0, 1.0), (1, 0, 1.0), (0, -1, 1.0), (0, 1, 1.0),
    (-1, -1, _SQRT2), (-1, 1, _SQRT2), (1, -1, _SQRT2), (1, 1, _SQRT2)
]


def _nearest_free(lethal: np.ndarray, ix: int, iy: int, max_r: int = 8) -> Optional[tuple[int, int]]:
    """Snaps a potentially lethal target cell to the closest non-lethal cell.

    Conducts a spiraling radial search up to a maximum radius around the target
    cell coordinates to locate a traversable grid cell.

    Args:
        lethal (np.ndarray): A 2D boolean array where True indicates impassable terrain.
        ix (int): The x-coordinate of the target cell in grid space.
        iy (int): The y-coordinate of the target cell in grid space.
        max_r (int, optional): The maximum integer radius to search. Defaults to 8.

    Returns:
        Optional[tuple[int, int]]: A tuple ``(x, y)`` of the closest non-lethal cell,
            or None if no such cell is found within the specified maximum radius.
    """
    n = lethal.shape[0]
    if 0 <= ix < n and 0 <= iy < n and not lethal[ix, iy]:
        return ix, iy
    for r in range(1, max_r + 1):
        for dx in range(-r, r + 1):
            for dy in range(-r, r + 1):
                if max(abs(dx), abs(dy)) != r:
                    continue
                jx, jy = ix + dx, iy + dy
                if 0 <= jx < n and 0 <= jy < n and not lethal[jx, jy]:
                    return jx, jy
    return None


def plan(
    cost: np.ndarray,
    lethal: np.ndarray,
    start_cell: tuple[int, int],
    goal_cell: tuple[int, int],
    allow_start_lethal: bool = True
) -> Optional[list[tuple[int, int]]]:
    """Computes an optimal path from the start cell to the goal cell using A*.

    The heuristic function utilizes octile distance for accurate diagonal movement estimation.
    If the specified goal cell is located on a lethal obstacle, the algorithm automatically
    snaps the goal to the nearest free cell before searching.

    Args:
        cost (np.ndarray): A 2D float array representing the traversal cost penalty
            of each cell, added on top of the base Euclidean distance cost.
        lethal (np.ndarray): A 2D boolean array indicating impassable obstacle regions.
        start_cell (tuple[int, int]): Grid coordinates ``(x, y)`` of the starting point.
        goal_cell (tuple[int, int]): Grid coordinates ``(x, y)`` of the desired destination.
        allow_start_lethal (bool, optional): If True, allows the search to begin even if
            the starting cell is currently marked lethal (useful if the robot is slightly
            inside an inflation boundary). Defaults to True.

    Returns:
        Optional[list[tuple[int, int]]]: A continuous sequence of contiguous grid cell
            coordinates ``(x, y)`` from start to goal, or None if no path could be found.
    """
    n = cost.shape[0]
    sx, sy = start_cell
    gx, gy = goal_cell
    snapped = _nearest_free(lethal, gx, gy)
    if snapped is None:
        return None
    gx, gy = snapped
    if not (0 <= sx < n and 0 <= sy < n):
        return None

    res = C.GRID_RESOLUTION

    def h(x: int, y: int) -> float:
        dx, dy = abs(x - gx), abs(y - gy)
        return (max(dx, dy) + (_SQRT2 - 1.0) * min(dx, dy)) * res

    start = (sx, sy)
    goal = (gx, gy)
    open_heap = [(h(sx, sy), 0.0, start)]
    g_score = {start: 0.0}
    came = {}
    closed = set()

    while open_heap:
        _, g, cur = heapq.heappop(open_heap)
        if cur in closed:
            continue
        if cur == goal:
            path = [cur]
            while cur in came:
                cur = came[cur]
                path.append(cur)
            path.reverse()
            return path
        closed.add(cur)
        cx, cy = cur
        for dx, dy, step in _NEIGHBORS:
            nx, ny = cx + dx, cy + dy
            if not (0 <= nx < n and 0 <= ny < n):
                continue
            if lethal[nx, ny] and not (allow_start_lethal and (nx, ny) == start):
                continue
            nb = (nx, ny)
            if nb in closed:
                continue
            extra = cost[nx, ny]
            if not math.isfinite(extra):
                continue
            tentative = g + step * res + extra * res
            if tentative < g_score.get(nb, math.inf):
                g_score[nb] = tentative
                came[nb] = cur
                heapq.heappush(open_heap, (tentative + h(nx, ny), tentative, nb))
    return None


def _line_clear(lethal: np.ndarray, a: tuple[int, int], b: tuple[int, int]) -> bool:
    """Evaluates whether the discrete line segment between two cells is clear of obstacles.

    Implements Bresenham's line algorithm to trace the segment and test against the
    lethal obstacle grid map.

    Args:
        lethal (np.ndarray): A 2D boolean array of impassable cells.
        a (tuple[int, int]): Grid coordinates of the segment start point.
        b (tuple[int, int]): Grid coordinates of the segment end point.

    Returns:
        bool: True if the entire line segment traverses only non-lethal cells, False otherwise.
    """
    x0, y0 = a
    x1, y1 = b
    dx, dy = abs(x1 - x0), abs(y1 - y0)
    sx = 1 if x0 < x1 else -1
    sy = 1 if y0 < y1 else -1
    err = dx - dy
    x, y = x0, y0
    n = lethal.shape[0]
    while True:
        if not (0 <= x < n and 0 <= y < n) or lethal[x, y]:
            return False
        if x == x1 and y == y1:
            return True
        e2 = 2 * err
        if e2 > -dy:
            err -= dy
            x += sx
        if e2 < dx:
            err += dx
            y += sy


def simplify(cells: Optional[list[tuple[int, int]]], lethal: np.ndarray) -> Optional[list[tuple[int, int]]]:
    """Simplifies a cellular path using a greedy line-of-sight shortcut technique.

    Iterates through the original dense path and attempts to connect distant waypoints
    directly via straight-line segments. If the line segment is clear of lethal obstacles,
    intermediate points are discarded, yielding a more aggressive, geometrically direct route.

    Args:
        cells (Optional[list[tuple[int, int]]]): The input dense path of contiguous cells.
        lethal (np.ndarray): The 2D boolean obstacle map used for line-of-sight checks.

    Returns:
        Optional[list[tuple[int, int]]]: The simplified list of critical waypoints, or
            the original list if simplification is disabled or the path is too short.
    """
    if not C.PATH_SIMPLIFY or cells is None or len(cells) <= 2:
        return cells
    out = [cells[0]]
    i = 0
    n = len(cells)
    while i < n - 1:
        j = n - 1
        while j > i + 1 and not _line_clear(lethal, cells[i], cells[j]):
            j -= 1
        out.append(cells[j])
        i = j
    return out
