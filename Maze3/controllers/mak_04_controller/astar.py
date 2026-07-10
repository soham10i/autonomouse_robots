"""A* on the costmap + line-of-sight path simplification.  Pure NumPy/stdlib.

Performs grid-based A* search over the inflated costmap to find an optimal
path from the robot to a target cell, respecting lethal obstacles. Includes a
greedy line-of-sight simplifier to collapse the dense cell-by-cell path into
a sparse polyline of safe straight segments.
"""
from __future__ import annotations

import heapq
import math
from typing import Dict, List, Optional, Set, Tuple

import numpy as np
import numpy.typing as npt

import config as C

_SQRT2: float = math.sqrt(2.0)
_NEIGHBORS: List[Tuple[int, int, float]] = [
    (-1, 0, 1.0), (1, 0, 1.0), (0, -1, 1.0), (0, 1, 1.0),
    (-1, -1, _SQRT2), (-1, 1, _SQRT2), (1, -1, _SQRT2), (1, 1, _SQRT2)
]


def _nearest_free(lethal: npt.NDArray[np.bool_],
                  ix: int, iy: int, max_r: int = 12) -> Optional[Tuple[int, int]]:
    """Snap a lethal target cell to the closest non-lethal cell.

    Performs a spiral search outwards from ``(ix, iy)`` up to ``max_r`` cells.

    Args:
        lethal: ``(N, N)`` boolean mask of lethal obstacles.
        ix: Target cell x-coordinate.
        iy: Target cell y-coordinate.
        max_r: Maximum search radius in cells.

    Returns:
        The nearest free cell ``(jx, jy)``, or ``None`` if none found within radius.
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


def plan(cost: npt.NDArray[np.floating],
         lethal: npt.NDArray[np.bool_],
         start_cell: Tuple[int, int],
         goal_cell: Tuple[int, int],
         allow_start_lethal: bool = True) -> Optional[List[Tuple[int, int]]]:
    """A* search from ``start_cell`` to ``goal_cell``.

    Args:
        cost: ``(N, N)`` array of non-lethal traversal costs (e.g. inflation gradients).
        lethal: ``(N, N)`` boolean mask of impassable obstacles.
        start_cell: ``(x, y)`` origin cell.
        goal_cell: ``(x, y)`` destination cell.
        allow_start_lethal: If ``True``, permits the search to escape a starting
            cell that is currently marked lethal (prevents paralysis if the
            robot clips an obstacle).

    Returns:
        A list of contiguous ``(x, y)`` cells from start to goal inclusive, or
        ``None`` if no path exists.
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
    # heap elements: (f_score, g_score, (x, y))
    open_heap: List[Tuple[float, float, Tuple[int, int]]] = [(h(sx, sy), 0.0, start)]
    g_score: Dict[Tuple[int, int], float] = {start: 0.0}
    came: Dict[Tuple[int, int], Tuple[int, int]] = {}
    closed: Set[Tuple[int, int]] = set()

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
            extra = float(cost[nx, ny])
            if not math.isfinite(extra):
                continue
            tentative = g + step * res + extra * res
            if tentative < g_score.get(nb, math.inf):
                g_score[nb] = tentative
                came[nb] = cur
                heapq.heappush(open_heap, (tentative + h(nx, ny), tentative, nb))
    return None


def _line_clear(lethal: npt.NDArray[np.bool_],
                a: Tuple[int, int], b: Tuple[int, int],
                cost: Optional[npt.NDArray[np.floating]] = None,
                max_cost: Optional[float] = None) -> bool:
    """True if the integer segment ``a -> b`` crosses no lethal cell (Bresenham).

    When ``cost`` / ``max_cost`` are given, the segment must ALSO stay in cells
    of ``cost <= max_cost`` (i.e. genuinely open / central cells).  This stops
    the shortcut from straightening a carefully centred path through a tight gap
    back against a wall: a shortcut is only taken where there is room to spare.

    Args:
        lethal: ``(N, N)`` boolean mask of impassable obstacles.
        a: ``(x, y)`` segment start cell.
        b: ``(x, y)`` segment end cell.
        cost: Optional ``(N, N)`` array of traversal costs.
        max_cost: Optional cost threshold; cells exceeding this are treated as
            lethal for the purpose of shortcutting.

    Returns:
        ``True`` if the line segment is clear, ``False`` otherwise.
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
        if cost is not None and max_cost is not None and cost[x, y] > max_cost:
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


def simplify(cells: Optional[List[Tuple[int, int]]],
             lethal: npt.NDArray[np.bool_],
             cost: Optional[npt.NDArray[np.floating]] = None) -> Optional[List[Tuple[int, int]]]:
    """Greedy line-of-sight shortcut of a cell path (fewer, longer segments).

    Pass ``cost`` (the costmap) to keep shortcuts out of tight passages so the
    dense, centred A* path is preserved where clearance is scarce; omit it for
    plain lethal-only behaviour.

    Args:
        cells: Contiguous list of ``(x, y)`` cells from :func:`plan`.
        lethal: ``(N, N)`` boolean mask of impassable obstacles.
        cost: Optional ``(N, N)`` array of traversal costs.

    Returns:
        A sparse list of ``(x, y)`` vertices defining the simplified polyline,
        or ``None`` if the input was empty.
    """
    if not C.PATH_SIMPLIFY or cells is None or len(cells) <= 2:
        return cells
    max_cost = C.SIMPLIFY_MAX_COST if cost is not None else None
    out = [cells[0]]
    i = 0
    n = len(cells)
    while i < n - 1:
        j = n - 1
        while j > i + 1 and not _line_clear(lethal, cells[i], cells[j], cost, max_cost):
            j -= 1
        out.append(cells[j])
        i = j
    return out
