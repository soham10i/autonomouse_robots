"""SE(2) pose helpers and differential-drive kinematics — pure NumPy.

A *pose* is the 3-tuple ``(x, y, theta)`` of the robot in the world frame
(metres, radians).  Points are ``(N, 2)`` float arrays.  No Webots import here
so every consumer stays unit-testable outside the simulator.
"""
from __future__ import annotations

import math
from typing import Tuple

import numpy as np

import config as C


def wrap_angle(a: float) -> float:
    """Wraps an angle to the [-pi, pi) range.

    Args:
        a (float): The angle in radians.

    Returns:
        float: The wrapped angle in radians.
    """
    return (a + math.pi) % (2.0 * math.pi) - math.pi


def wrap_angle_arr(a: np.ndarray) -> np.ndarray:
    """Wraps an array of angles to the [-pi, pi) range.

    Args:
        a (np.ndarray): An array of angles in radians.

    Returns:
        np.ndarray: An array of wrapped angles in radians.
    """
    return (a + math.pi) % (2.0 * math.pi) - math.pi


def transform_points(points: np.ndarray, x: float, y: float, theta: float) -> np.ndarray:
    """Transforms body-frame points into the world frame.

    Applies the transformation: R(theta) * p + [x, y].

    Args:
        points (np.ndarray): Array of shape (N, 2) representing points in the body frame.
        x (float): The x-coordinate of the robot in the world frame.
        y (float): The y-coordinate of the robot in the world frame.
        theta (float): The heading of the robot in the world frame.

    Returns:
        np.ndarray: Array of shape (N, 2) representing the transformed points in the world frame.
    """
    if points.size == 0:
        return points.reshape(0, 2)
    c, s = math.cos(theta), math.sin(theta)
    px, py = points[:, 0], points[:, 1]
    return np.stack([x + c * px - s * py, y + s * px + c * py], axis=1)


def inverse_transform_points(points: np.ndarray, x: float, y: float, theta: float) -> np.ndarray:
    """Transforms world-frame points into the body frame.

    Args:
        points (np.ndarray): Array of shape (N, 2) representing points in the world frame.
        x (float): The x-coordinate of the robot in the world frame.
        y (float): The y-coordinate of the robot in the world frame.
        theta (float): The heading of the robot in the world frame.

    Returns:
        np.ndarray: Array of shape (N, 2) representing the points expressed in the body frame.
    """
    if points.size == 0:
        return points.reshape(0, 2)
    c, s = math.cos(theta), math.sin(theta)
    dx, dy = points[:, 0] - x, points[:, 1] - y
    return np.stack([c * dx + s * dy, -s * dx + c * dy], axis=1)


def pose_distance(a: Tuple[float, float, float], b: Tuple[float, float, float]) -> float:
    """Calculates the Euclidean distance between two poses.

    Args:
        a (Tuple[float, float, float]): The first pose (x, y, theta).
        b (Tuple[float, float, float]): The second pose (x, y, theta).

    Returns:
        float: The distance between the two poses in meters.
    """
    return math.hypot(a[0] - b[0], a[1] - b[1])


def relative_pose(p_from: Tuple[float, float, float], p_to: Tuple[float, float, float]) -> Tuple[float, float, float]:
    """Calculates the relative pose of a target from a given base pose.

    Args:
        p_from (Tuple[float, float, float]): The base pose (x, y, theta).
        p_to (Tuple[float, float, float]): The target pose (x, y, theta).

    Returns:
        Tuple[float, float, float]: The relative pose (dx, dy, dtheta) expressed in the base pose's frame.
    """
    x0, y0, t0 = p_from
    x1, y1, t1 = p_to
    c, s = math.cos(t0), math.sin(t0)
    dx_w, dy_w = x1 - x0, y1 - y0
    return (c * dx_w + s * dy_w, -s * dx_w + c * dy_w, wrap_angle(t1 - t0))


def compose_pose(base: Tuple[float, float, float], rel: Tuple[float, float, float]) -> Tuple[float, float, float]:
    """Applies a body-frame relative pose increment to a base pose.

    This is the inverse of `relative_pose`.

    Args:
        base (Tuple[float, float, float]): The base pose (x, y, theta).
        rel (Tuple[float, float, float]): The relative pose increment (dx, dy, dtheta).

    Returns:
        Tuple[float, float, float]: The resulting composed pose in the world frame.
    """
    bx, by, bth = base
    dx, dy, dth = rel
    c, s = math.cos(bth), math.sin(bth)
    return (bx + c * dx - s * dy, by + s * dx + c * dy, wrap_angle(bth + dth))


def cmd_to_wheels(v: float, w: float) -> Tuple[float, float]:
    """Converts linear and angular velocities to wheel angular velocities.

    `w > 0` indicates a counter-clockwise (left) turn, causing the right wheel to spin faster.

    Args:
        v (float): The desired linear velocity in m/s.
        w (float): The desired angular velocity in rad/s.

    Returns:
        Tuple[float, float]: A tuple containing the left and right wheel angular velocities in rad/s.
    """
    wl = (v - w * C.WHEEL_SEPARATION * 0.5) / C.WHEEL_RADIUS
    wr = (v + w * C.WHEEL_SEPARATION * 0.5) / C.WHEEL_RADIUS
    return wl, wr
