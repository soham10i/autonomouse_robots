"""A* global planner + standoff-goal selection.

Operates on the cells of ``Grid2D``. Lethal cells (occupied, aux obstacle,
poison + their inflation) are rejected; the soft costmap is added to the
distance term so the path prefers the middle of free space.

For pillars we never plan to the cylinder cell itself — instead we generate
candidate **standoff** points around the pillar at ``PILLAR_STANDOFF`` metres
and pick the cheapest reachable one.
"""
import heapq
import math

import numpy as np

import config as C
from utils import bresenham, line_clear


# 8-connected neighbour offsets (dx, dy, base_step_cost).
_NEIGH4 = [(1, 0, 1.0), (-1, 0, 1.0), (0, 1, 1.0), (0, -1, 1.0)]
_NEIGH8 = _NEIGH4 + [
    (1, 1, math.sqrt(2)), (1, -1, math.sqrt(2)),
    (-1, 1, math.sqrt(2)), (-1, -1, math.sqrt(2)),
]


def _heuristic(ix, iy, gx, gy):
    dx = abs(ix - gx)
    dy = abs(iy - gy)
    if C.ASTAR_DIAGONAL:
        return (dx + dy) + (math.sqrt(2) - 2) * min(dx, dy)
    return dx + dy


def astar(cost, lethal, start, goal, max_nodes=C.ASTAR_MAX_NODES):
    """Run A* on ``cost``, returning a list of (ix, iy) cells from start→goal.

    ``cost`` : (W, H) float32 — per-cell soft cost (already > 0 for inflation,
                 ``inf`` for lethal). The base step cost is the geometric
                 distance; we add ``cost[next]``.
    ``lethal``: (W, H) bool — fast lethal lookup (mirrors cost==inf).
    Returns ``None`` if no path or capacity exceeded.
    """
    sx, sy = start
    gx, gy = goal
    cells_x, cells_y = cost.shape
    if not (0 <= sx < cells_x and 0 <= sy < cells_y
            and 0 <= gx < cells_x and 0 <= gy < cells_y):
        return None
    if lethal[gx, gy]:
        return None

    neigh = _NEIGH8 if C.ASTAR_DIAGONAL else _NEIGH4

    g_cost = {start: 0.0}
    parent = {start: None}
    open_heap = []
    heapq.heappush(open_heap, (_heuristic(sx, sy, gx, gy), 0.0, start))
    closed = np.zeros_like(lethal)

    seen = 0
    while open_heap:
        seen += 1
        if seen > max_nodes:
            return None
        f, g, cur = heapq.heappop(open_heap)
        cx, cy = cur
        if closed[cx, cy]:
            continue
        closed[cx, cy] = True
        if cur == goal:
            return _reconstruct(parent, cur)
        for dx, dy, step in neigh:
            nx, ny = cx + dx, cy + dy
            if nx < 0 or ny < 0 or nx >= cells_x or ny >= cells_y:
                continue
            if lethal[nx, ny] or closed[nx, ny]:
                continue
            extra = cost[nx, ny]
            if not np.isfinite(extra):
                continue
            ng = g + step + extra
            old = g_cost.get((nx, ny))
            if old is not None and ng >= old:
                continue
            g_cost[(nx, ny)] = ng
            parent[(nx, ny)] = cur
            f2 = ng + _heuristic(nx, ny, gx, gy)
            heapq.heappush(open_heap, (f2, ng, (nx, ny)))
    return None


def _reconstruct(parent, end):
    out = []
    cur = end
    while cur is not None:
        out.append(cur)
        cur = parent[cur]
    out.reverse()
    return out


def simplify_path(path_idx, lethal):
    """Line-of-sight smoothing: drop intermediate waypoints whose endpoints
    are mutually visible (no lethal cell between them)."""
    if not path_idx or len(path_idx) <= 2:
        return path_idx[:] if path_idx else []
    out = [path_idx[0]]
    i = 0
    n = len(path_idx)
    while i < n - 1:
        j = n - 1
        while j > i + 1:
            ax, ay = path_idx[i]
            bx, by = path_idx[j]
            if line_clear(lethal, ax, ay, bx, by):
                break
            j -= 1
        out.append(path_idx[j])
        i = j
    return out


def path_idx_to_world(path_idx, grid):
    return [grid.i2w(ix, iy) for ix, iy in path_idx]


def _nearest_free_cell(lethal, ix, iy, max_search=12):
    """If (ix, iy) is lethal, find the nearest free cell within ``max_search``
    cells (BFS-style ring expansion)."""
    cx, cy = lethal.shape
    if 0 <= ix < cx and 0 <= iy < cy and not lethal[ix, iy]:
        return ix, iy
    for r in range(1, max_search + 1):
        best = None
        best_d2 = None
        for dx in range(-r, r + 1):
            for dy in range(-r, r + 1):
                if max(abs(dx), abs(dy)) != r:
                    continue
                xx, yy = ix + dx, iy + dy
                if 0 <= xx < cx and 0 <= yy < cy and not lethal[xx, yy]:
                    d2 = dx * dx + dy * dy
                    if best is None or d2 < best_d2:
                        best = (xx, yy)
                        best_d2 = d2
        if best is not None:
            return best
    return None


def plan_to_world(grid, pose, goal_xy):
    """Plan from the robot's current cell to the world point ``goal_xy``.

    Returns ``(path_world, path_idx, costmap)`` or ``(None, None, costmap)``
    if the goal cannot be reached.
    """
    cost, lethal = grid.costmap()
    sx, sy = grid.w2i(pose[0], pose[1])
    gx, gy = grid.w2i(goal_xy[0], goal_xy[1])

    # If the start is inside the inflation halo (e.g. close to a wall or a
    # pillar), nudge it to the nearest free cell so A* doesn't refuse.
    start_fix = _nearest_free_cell(lethal, sx, sy, max_search=12)
    if start_fix is None:
        return None, None, cost
    sx, sy = start_fix

    goal_fix = _nearest_free_cell(lethal, gx, gy, max_search=10)
    if goal_fix is None:
        return None, None, cost
    gx, gy = goal_fix

    raw = astar(cost, lethal, (sx, sy), (gx, gy))
    if raw is None:
        return None, None, cost
    smooth = simplify_path(raw, lethal)
    return path_idx_to_world(smooth, grid), smooth, cost


def _candidate_standoffs(pillar_xy, standoff, n=12):
    cx, cy = pillar_xy
    for k in range(n):
        a = 2 * math.pi * k / n
        yield (cx + standoff * math.cos(a), cy + standoff * math.sin(a)), a


def plan_to_pillar(grid, pose, pillar_xy, standoff=None,
                   keep_arc_to=None, log=print):
    """Try multiple standoff approach points around ``pillar_xy``; return
    the cheapest reachable plan as ``(path_world, path_idx, standoff_xy, cost)``.

    If ``keep_arc_to`` is a (cx, cy) point (e.g. the robot), candidate angles
    are pre-sorted to favour those on the same side as the robot — produces
    much shorter paths in cluttered mazes.
    """
    if standoff is None:
        standoff = C.PILLAR_STANDOFF
    cost, lethal = grid.costmap()

    sx, sy = grid.w2i(pose[0], pose[1])
    start_fix = _nearest_free_cell(lethal, sx, sy, max_search=12)
    if start_fix is None:
        return None, None, None, cost
    sx, sy = start_fix

    # Sort candidate angles so we try the side facing the robot first.
    cands = list(_candidate_standoffs(pillar_xy, standoff, n=12))
    if keep_arc_to is not None:
        rx, ry = keep_arc_to
        ref = math.atan2(ry - pillar_xy[1], rx - pillar_xy[0])

        def _score(item):
            (_, _), ang = item
            d = ang - ref
            while d > math.pi:
                d -= 2 * math.pi
            while d <= -math.pi:
                d += 2 * math.pi
            return abs(d)

        cands.sort(key=_score)

    best = None
    best_cost = math.inf
    best_path = None
    for (gxw, gyw), _ang in cands:
        gx, gy = grid.w2i(gxw, gyw)
        gx, gy = max(0, min(grid.cells - 1, gx)), max(0, min(grid.cells - 1, gy))
        gf = _nearest_free_cell(lethal, gx, gy, max_search=8)
        if gf is None:
            continue
        gx, gy = gf
        raw = astar(cost, lethal, (sx, sy), (gx, gy))
        if raw is None:
            continue
        # path "cost" = sum of step + per-cell cost
        total = 0.0
        prev = raw[0]
        for nxt in raw[1:]:
            d = math.hypot(nxt[0] - prev[0], nxt[1] - prev[1])
            total += d + float(cost[nxt[0], nxt[1]])
            prev = nxt
        if total < best_cost:
            best_cost = total
            best = grid.i2w(gx, gy)
            best_path = raw

    if best_path is None:
        return None, None, None, cost
    smooth = simplify_path(best_path, lethal)
    return path_idx_to_world(smooth, grid), smooth, best, cost
