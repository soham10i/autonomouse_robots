"""A* on the costmap + line-of-sight path simplification.  Pure NumPy/stdlib."""
from __future__ import annotations

import heapq
import math

import numpy as np

import config as C

_SQRT2 = math.sqrt(2.0)
_NEIGHBORS = [(-1, 0, 1.0), (1, 0, 1.0), (0, -1, 1.0), (0, 1, 1.0),
              (-1, -1, _SQRT2), (-1, 1, _SQRT2), (1, -1, _SQRT2), (1, 1, _SQRT2)]


def _nearest_free(lethal, ix, iy, max_r=8):
    """Snap a lethal target cell to the closest non-lethal cell (spiral search)."""
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


def plan(cost, lethal, start_cell, goal_cell, allow_start_lethal=True):
    """A* from ``start_cell`` to ``goal_cell``.  Returns a list of (ix,iy) or None."""
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

    def h(x, y):
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


def _line_clear(lethal, a, b):
    """True if the integer segment a->b crosses no lethal cell (Bresenham)."""
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


def simplify(cells, lethal):
    """Greedy line-of-sight shortcut of a cell path (fewer, longer segments)."""
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
