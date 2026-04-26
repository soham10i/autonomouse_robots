"""Math + grid helpers shared across modules."""
import math
import numpy as np


def wrap_angle(a):
    while a > math.pi:
        a -= 2.0 * math.pi
    while a <= -math.pi:
        a += 2.0 * math.pi
    return a


def clamp(v, lo, hi):
    return max(lo, min(hi, v))


def world_to_grid(wx, wy, origin, res):
    return (int(math.floor((wx - origin[0]) / res)),
            int(math.floor((wy - origin[1]) / res)))


def grid_to_world(ix, iy, origin, res):
    return origin[0] + (ix + 0.5) * res, origin[1] + (iy + 0.5) * res


def bresenham(x0, y0, x1, y1):
    """Integer Bresenham line, endpoints inclusive."""
    dx = abs(x1 - x0)
    dy = abs(y1 - y0)
    sx = 1 if x0 < x1 else -1
    sy = 1 if y0 < y1 else -1
    err = dx - dy
    x, y = x0, y0
    out = []
    while True:
        out.append((x, y))
        if x == x1 and y == y1:
            break
        e2 = 2 * err
        if e2 > -dy:
            err -= dy
            x += sx
        if e2 < dx:
            err += dx
            y += sy
    return out


def line_clear(grid_lethal, ix0, iy0, ix1, iy1):
    """True if the integer Bresenham line in grid_lethal[ix, iy] hits no lethal cell."""
    cells = grid_lethal.shape[0]
    for x, y in bresenham(ix0, iy0, ix1, iy1):
        if x < 0 or y < 0 or x >= cells or y >= grid_lethal.shape[1]:
            return False
        if grid_lethal[x, y]:
            return False
    return True


def angle_diff(a, b):
    return wrap_angle(a - b)
