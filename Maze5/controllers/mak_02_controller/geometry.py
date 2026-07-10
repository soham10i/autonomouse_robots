"""SE(2) pose helpers and differential-drive kinematics — pure NumPy.

A *pose* is the 3-tuple ``(x, y, theta)`` of the robot in the world frame
(meters, radians). Points are represented as ``(N, 2)`` float arrays. No Webots
import is present here so every consumer stays unit-testable outside the simulator.
"""
from __future__ import annotations

import math

import numpy as np

import config as C


def wrap_angle(a: float) -> float:
    """Wraps an angle to the interval [-pi, pi).

    Args:
        a (float): The input angle in radians.

    Returns:
        float: The wrapped angle constrained within [-pi, pi).
    """
    return (a + math.pi) % (2.0 * math.pi) - math.pi


def wrap_angle_arr(a: np.ndarray) -> np.ndarray:
    """Wraps an array of angles to the interval [-pi, pi).

    Args:
        a (np.ndarray): An array of input angles in radians.

    Returns:
        np.ndarray: An array of wrapped angles constrained within [-pi, pi).
    """
    return (a + math.pi) % (2.0 * math.pi) - math.pi


def transform_points(points: np.ndarray, x: float, y: float, theta: float) -> np.ndarray:
    """Transforms an array of points from the body frame to the world frame.

    Applies the SE(2) transformation: ``R(theta) * p + [x, y]``.

    Args:
        points (np.ndarray): A NumPy array of shape (N, 2) representing points in the body frame.
        x (float): The x-coordinate of the body frame origin in the world frame.
        y (float): The y-coordinate of the body frame origin in the world frame.
        theta (float): The orientation of the body frame relative to the world frame (radians).

    Returns:
        np.ndarray: A NumPy array of shape (N, 2) representing the transformed points in the world frame.
    """
    if points.size == 0:
        return points.reshape(0, 2)
    c, s = math.cos(theta), math.sin(theta)
    px, py = points[:, 0], points[:, 1]
    return np.stack([x + c * px - s * py, y + s * px + c * py], axis=1)


def inverse_transform_points(points: np.ndarray, x: float, y: float, theta: float) -> np.ndarray:
    """Transforms an array of points from the world frame to the body frame.

    Expresses world-frame points in the local coordinate system defined by ``(x, y, theta)``.

    Args:
        points (np.ndarray): A NumPy array of shape (N, 2) representing points in the world frame.
        x (float): The x-coordinate of the body frame origin in the world frame.
        y (float): The y-coordinate of the body frame origin in the world frame.
        theta (float): The orientation of the body frame relative to the world frame (radians).

    Returns:
        np.ndarray: A NumPy array of shape (N, 2) representing the points in the body frame.
    """
    if points.size == 0:
        return points.reshape(0, 2)
    c, s = math.cos(theta), math.sin(theta)
    dx, dy = points[:, 0] - x, points[:, 1] - y
    return np.stack([c * dx + s * dy, -s * dx + c * dy], axis=1)


def pose_distance(a: tuple[float, float, ...], b: tuple[float, float, ...]) -> float:
    """Calculates the 2D Euclidean distance between two poses or points.

    Args:
        a (tuple[float, float, ...]): The first coordinate, e.g., (x, y) or (x, y, theta).
        b (tuple[float, float, ...]): The second coordinate, e.g., (x, y) or (x, y, theta).

    Returns:
        float: The Euclidean distance in meters.
    """
    return math.hypot(a[0] - b[0], a[1] - b[1])


def relative_pose(p_from: tuple[float, float, float], p_to: tuple[float, float, float]) -> tuple[float, float, float]:
    """Computes the relative pose of a target frame expressed in a reference body frame.

    Args:
        p_from (tuple[float, float, float]): The reference pose ``(x0, y0, theta0)`` in the world frame.
        p_to (tuple[float, float, float]): The target pose ``(x1, y1, theta1)`` in the world frame.

    Returns:
        tuple[float, float, float]: A tuple ``(dx, dy, dtheta)`` representing the pose of ``p_to``
            expressed entirely in the coordinate frame of ``p_from``.
    """
    x0, y0, t0 = p_from
    x1, y1, t1 = p_to
    c, s = math.cos(t0), math.sin(t0)
    dx_w, dy_w = x1 - x0, y1 - y0
    return (c * dx_w + s * dy_w, -s * dx_w + c * dy_w, wrap_angle(t1 - t0))


def compose_pose(base: tuple[float, float, float], rel: tuple[float, float, float]) -> tuple[float, float, float]:
    """Applies a body-frame increment to a base pose to compute the resulting world pose.

    This function is the inverse mathematical operation of `relative_pose`.

    Args:
        base (tuple[float, float, float]): The base pose ``(bx, by, btheta)`` in the world frame.
        rel (tuple[float, float, float]): The relative increment ``(dx, dy, dtheta)`` in the body frame of ``base``.

    Returns:
        tuple[float, float, float]: A tuple ``(x, y, theta)`` representing the new composed pose in the world frame.
    """
    bx, by, bth = base
    dx, dy, dth = rel
    c, s = math.cos(bth), math.sin(bth)
    return (bx + c * dx - s * dy, by + s * dx + c * dy, wrap_angle(bth + dth))


def cmd_to_wheels(v: float, w: float) -> tuple[float, float]:
    """Converts linear and angular velocities to wheel angular velocities.

    Applies differential-drive kinematics to compute the required angular
    velocities for the left and right wheels to achieve the target body velocities.
    A positive angular velocity (`w`) represents a counter-clockwise turn, meaning
    the right wheel must spin faster than the left wheel.

    Args:
        v (float): Target forward linear velocity in meters per second (m/s).
        w (float): Target angular velocity in radians per second (rad/s).

    Returns:
        tuple[float, float]: A tuple containing:
            - wl (float): Target angular velocity for the left wheel (rad/s).
            - wr (float): Target angular velocity for the right wheel (rad/s).
    """
    wl = (v - w * C.WHEEL_SEPARATION * 0.5) / C.WHEEL_RADIUS
    wr = (v + w * C.WHEEL_SEPARATION * 0.5) / C.WHEEL_RADIUS
    return wl, wr
