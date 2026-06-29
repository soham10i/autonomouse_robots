"""SE(2) pose helpers and differential-drive kinematics — pure NumPy.

A *pose* is the 3-tuple ``(x, y, theta)`` of the robot in the world frame
(metres, radians).  Points are ``(N, 2)`` float arrays.  No Webots import here
so every consumer stays unit-testable outside the simulator.
"""
from __future__ import annotations

import math

import numpy as np

import config as C


def wrap_angle(a: float) -> float:
    """Wrap an angle to ``[-pi, pi)``."""
    return (a + math.pi) % (2.0 * math.pi) - math.pi


def wrap_angle_arr(a: np.ndarray) -> np.ndarray:
    return (a + math.pi) % (2.0 * math.pi) - math.pi


def transform_points(points: np.ndarray, x: float, y: float, theta: float) -> np.ndarray:
    """Body-frame ``points`` (N, 2) into the world frame: ``R(theta) p + [x,y]``."""
    if points.size == 0:
        return points.reshape(0, 2)
    c, s = math.cos(theta), math.sin(theta)
    px, py = points[:, 0], points[:, 1]
    return np.stack([x + c * px - s * py, y + s * px + c * py], axis=1)


def inverse_transform_points(points: np.ndarray, x: float, y: float, theta: float) -> np.ndarray:
    """World-frame ``points`` (N, 2) expressed in the body frame at ``(x,y,theta)``."""
    if points.size == 0:
        return points.reshape(0, 2)
    c, s = math.cos(theta), math.sin(theta)
    dx, dy = points[:, 0] - x, points[:, 1] - y
    return np.stack([c * dx + s * dy, -s * dx + c * dy], axis=1)


def pose_distance(a, b) -> float:
    return math.hypot(a[0] - b[0], a[1] - b[1])


def relative_pose(p_from, p_to):
    """(dx, dy, dtheta) of ``p_to`` expressed in ``p_from``'s body frame."""
    x0, y0, t0 = p_from
    x1, y1, t1 = p_to
    c, s = math.cos(t0), math.sin(t0)
    dx_w, dy_w = x1 - x0, y1 - y0
    return (c * dx_w + s * dy_w, -s * dx_w + c * dy_w, wrap_angle(t1 - t0))


def compose_pose(base, rel):
    """Apply a body-frame increment ``rel`` to ``base`` (inverse of relative_pose)."""
    bx, by, bth = base
    dx, dy, dth = rel
    c, s = math.cos(bth), math.sin(bth)
    return (bx + c * dx - s * dy, by + s * dx + c * dy, wrap_angle(bth + dth))


def cmd_to_wheels(v: float, w: float):
    """(v m/s, w rad/s) -> (left, right) wheel angular velocities (rad/s).

    ``w > 0`` is a CCW (left) turn, so the right wheel spins faster.
    """
    wl = (v - w * C.WHEEL_SEPARATION * 0.5) / C.WHEEL_RADIUS
    wr = (v + w * C.WHEEL_SEPARATION * 0.5) / C.WHEEL_RADIUS
    return wl, wr
