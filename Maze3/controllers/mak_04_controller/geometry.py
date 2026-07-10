"""SE(2) pose helpers and differential-drive kinematics — pure NumPy.

Provides the core geometric primitives consumed by every navigation module:
pose composition, body↔world point transforms, and the unicycle-to-differential
kinematic conversion.  A *pose* is the 3-tuple ``(x, y, theta)`` representing
the robot's position and heading in the world (odometry) frame.  Points are
``(N, 2)`` float64 arrays.

No Webots import so every consumer stays unit-testable outside the simulator.
"""
from __future__ import annotations

import math
from typing import Tuple

import numpy as np
import numpy.typing as npt

import config as C


def wrap_angle(a: float) -> float:
    """Wrap an angle to the half-open interval ``[-π, π)``.

    Args:
        a: Angle in radians (unbounded).

    Returns:
        The equivalent angle in ``[-π, π)``.
    """
    return (a + math.pi) % (2.0 * math.pi) - math.pi


def wrap_angle_arr(a: npt.NDArray[np.floating]) -> npt.NDArray[np.floating]:
    """Element-wise :func:`wrap_angle` for NumPy arrays.

    Args:
        a: Array of angles in radians.

    Returns:
        Array of wrapped angles in ``[-π, π)``.
    """
    return (a + math.pi) % (2.0 * math.pi) - math.pi


def transform_points(points: npt.NDArray[np.floating],
                     x: float, y: float, theta: float) -> npt.NDArray[np.floating]:
    """Transform body-frame points into the world frame.

    Applies the rigid-body transform ``R(theta) · p + [x, y]`` to each row of
    *points*.

    Args:
        points: ``(N, 2)`` array of ``[px, py]`` in the body frame.
        x: World-frame x-coordinate of the body origin.
        y: World-frame y-coordinate of the body origin.
        theta: Body heading in the world frame (radians).

    Returns:
        ``(N, 2)`` array of the same points expressed in the world frame.
    """
    if points.size == 0:
        return points.reshape(0, 2)
    c, s = math.cos(theta), math.sin(theta)
    px, py = points[:, 0], points[:, 1]
    return np.stack([x + c * px - s * py, y + s * px + c * py], axis=1)


def inverse_transform_points(points: npt.NDArray[np.floating],
                              x: float, y: float, theta: float) -> npt.NDArray[np.floating]:
    """Transform world-frame points into the body frame at ``(x, y, theta)``.

    The inverse of :func:`transform_points`.

    Args:
        points: ``(N, 2)`` array of ``[px, py]`` in the world frame.
        x: World-frame x-coordinate of the body origin.
        y: World-frame y-coordinate of the body origin.
        theta: Body heading in the world frame (radians).

    Returns:
        ``(N, 2)`` array of the same points expressed in the body frame.
    """
    if points.size == 0:
        return points.reshape(0, 2)
    c, s = math.cos(theta), math.sin(theta)
    dx, dy = points[:, 0] - x, points[:, 1] - y
    return np.stack([c * dx + s * dy, -s * dx + c * dy], axis=1)


def pose_distance(a: Tuple[float, float, float],
                  b: Tuple[float, float, float]) -> float:
    """Euclidean distance between the (x, y) components of two poses.

    Args:
        a: First pose ``(x, y, theta)``.
        b: Second pose ``(x, y, theta)``.

    Returns:
        The 2-D Euclidean distance ``‖a[:2] − b[:2]‖``.
    """
    return math.hypot(a[0] - b[0], a[1] - b[1])


def relative_pose(p_from: Tuple[float, float, float],
                  p_to: Tuple[float, float, float]) -> Tuple[float, float, float]:
    """Express ``p_to`` in the body frame of ``p_from``.

    Args:
        p_from: Reference pose ``(x, y, theta)``.
        p_to: Target pose ``(x, y, theta)``.

    Returns:
        ``(dx, dy, dtheta)`` of *p_to* expressed in *p_from*'s body frame,
        with *dtheta* wrapped to ``[-π, π)``.
    """
    x0, y0, t0 = p_from
    x1, y1, t1 = p_to
    c, s = math.cos(t0), math.sin(t0)
    dx_w, dy_w = x1 - x0, y1 - y0
    return (c * dx_w + s * dy_w, -s * dx_w + c * dy_w, wrap_angle(t1 - t0))


def compose_pose(base: Tuple[float, float, float],
                 rel: Tuple[float, float, float]) -> Tuple[float, float, float]:
    """Apply a body-frame increment to a base pose (inverse of :func:`relative_pose`).

    Args:
        base: Base pose ``(x, y, theta)`` in the world frame.
        rel: Body-frame increment ``(dx, dy, dtheta)``.

    Returns:
        The composed pose ``(x', y', theta')`` with *theta'* wrapped to ``[-π, π)``.
    """
    bx, by, bth = base
    dx, dy, dth = rel
    c, s = math.cos(bth), math.sin(bth)
    return (bx + c * dx - s * dy, by + s * dx + c * dy, wrap_angle(bth + dth))


def cmd_to_wheels(v: float, w: float) -> Tuple[float, float]:
    """Convert unicycle ``(v, w)`` to differential-drive wheel angular velocities.

    Uses the standard differential-drive kinematic model:
    ``ω_L = (v − w · L/2) / r``, ``ω_R = (v + w · L/2) / r``,
    where *L* is the wheel separation and *r* is the wheel radius.
    ``w > 0`` is a CCW (left) turn, so the right wheel spins faster.

    Args:
        v: Desired linear velocity (m/s).
        w: Desired angular velocity (rad/s, positive = CCW / left).

    Returns:
        ``(wl, wr)`` — left and right wheel angular velocities (rad/s).
    """
    wl = (v - w * C.WHEEL_SEPARATION * 0.5) / C.WHEEL_RADIUS
    wr = (v + w * C.WHEEL_SEPARATION * 0.5) / C.WHEEL_RADIUS
    return wl, wr

