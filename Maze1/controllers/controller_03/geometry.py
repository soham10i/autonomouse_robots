"""SE(2) pose helpers — pure NumPy, no Webots dependency.

A *pose* is the 3-tuple ``(x, y, theta)`` of the robot in the world frame.
Points are ``(N, 2)`` float arrays.  Everything here is deterministic and unit
testable.
"""
from __future__ import annotations

import math

import numpy as np


def wrap_angle(a: float) -> float:
    """Wrap an angle to ``[-pi, pi)`` (single-valued; ``+pi`` maps to ``-pi``)."""
    return (a + math.pi) % (2.0 * math.pi) - math.pi


def wrap_angle_arr(a: np.ndarray) -> np.ndarray:
    """Vectorised :func:`wrap_angle`."""
    return (a + math.pi) % (2.0 * math.pi) - math.pi


def transform_points(points: np.ndarray, x: float, y: float, theta: float) -> np.ndarray:
    """Rotate+translate body-frame ``points`` (N, 2) into the world frame.

    ``world = R(theta) @ body + [x, y]``.
    """
    if points.size == 0:
        return points.reshape(0, 2)
    c, s = math.cos(theta), math.sin(theta)
    px = points[:, 0]
    py = points[:, 1]
    wx = x + c * px - s * py
    wy = y + s * px + c * py
    return np.stack([wx, wy], axis=1)


def inverse_transform_points(points: np.ndarray, x: float, y: float, theta: float) -> np.ndarray:
    """World-frame ``points`` (N, 2) expressed in the body frame at ``(x,y,theta)``."""
    if points.size == 0:
        return points.reshape(0, 2)
    c, s = math.cos(theta), math.sin(theta)
    dx = points[:, 0] - x
    dy = points[:, 1] - y
    bx = c * dx + s * dy
    by = -s * dx + c * dy
    return np.stack([bx, by], axis=1)


def relative_pose(p_from, p_to):
    """Return the (dx, dy, dtheta) of ``p_to`` expressed in ``p_from``'s frame."""
    x0, y0, t0 = p_from
    x1, y1, t1 = p_to
    c, s = math.cos(t0), math.sin(t0)
    dx_w = x1 - x0
    dy_w = y1 - y0
    dx = c * dx_w + s * dy_w
    dy = -s * dx_w + c * dy_w
    return dx, dy, wrap_angle(t1 - t0)


def compose_pose(base, rel):
    """Apply body-frame increment ``rel`` to ``base`` (inverse of relative_pose).

    If ``rel = relative_pose(a, b)`` then ``compose_pose(a, rel) == b``.  Used to
    carry a wheel/IMU motion increment over onto the scan-matched pose.
    """
    bx, by, bth = base
    dx, dy, dth = rel
    c, s = math.cos(bth), math.sin(bth)
    x = bx + c * dx - s * dy
    y = by + s * dx + c * dy
    return x, y, wrap_angle(bth + dth)


def pose_distance(p_a, p_b):
    """Euclidean translation distance between two poses."""
    return math.hypot(p_a[0] - p_b[0], p_a[1] - p_b[1])
