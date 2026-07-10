"""Wheel and IMU odometry data fusion for robust pose estimation.

This module computes dead-reckoning pose increments using a combination of
differential-drive encoder deltas for translation and an Inertial Measurement
Unit (IMU) for drift-free heading (yaw). The main controller treats the per-step
change of this raw pose as a motion increment and composes it onto a global,
scan-matched pose belief.
"""
from __future__ import annotations

import math
from typing import Optional

import config as C
from geometry import wrap_angle


class Odometry:
    """Fuses wheel encoder and IMU measurements to maintain a local pose estimate.

    Attributes:
        x (float): The current x-coordinate in the local odometry frame (meters).
        y (float): The current y-coordinate in the local odometry frame (meters).
        theta (float): The current orientation (yaw) in the local odometry frame (radians).
    """

    def __init__(self) -> None:
        """Initializes the odometry estimator at the origin (0, 0, 0)."""
        self.x: float = 0.0
        self.y: float = 0.0
        self.theta: float = 0.0
        self._prev_l: Optional[float] = None
        self._prev_r: Optional[float] = None

    def update(self, wl: Optional[float], wr: Optional[float], yaw_imu: Optional[float], dt: float) -> None:
        """Updates the local pose estimate using the latest sensor readings.

        If wheel encoder readings are available, translation is updated via
        differential-drive kinematics. The IMU yaw directly supersedes encoder-derived
        heading if provided, as the simulator's InertialUnit is highly accurate and
        immune to wheel slip.

        Args:
        wl (Optional[float]): The accumulated angular position or velocity of the left wheel.
        wr (Optional[float]): The accumulated angular position or velocity of the right wheel.
        yaw_imu (Optional[float]): The absolute yaw measured by the IMU in radians.
        dt (float): The time delta since the last update (seconds). Currently unused
            directly in the kinematic update as encoders provide absolute positions.
        """
        if wl is None or wr is None:
            if yaw_imu is not None:
                self.theta = wrap_angle(yaw_imu)
            return
        if self._prev_l is None:
            self._prev_l, self._prev_r = wl, wr
            if yaw_imu is not None:
                self.theta = wrap_angle(yaw_imu)
            return
        
        d_left = (wl - self._prev_l) * C.WHEEL_RADIUS
        d_right = (wr - self._prev_r) * C.WHEEL_RADIUS
        self._prev_l, self._prev_r = wl, wr
        ds = 0.5 * (d_left + d_right)
        dtheta = (d_right - d_left) / C.WHEEL_SEPARATION
        theta_mid = self.theta + 0.5 * dtheta
        
        self.x += ds * math.cos(theta_mid)
        self.y += ds * math.sin(theta_mid)
        
        if yaw_imu is not None:
            self.theta = wrap_angle(yaw_imu)
        else:
            self.theta = wrap_angle(self.theta + dtheta)

    def pose(self) -> tuple[float, float, float]:
        """Retrieves the current dead-reckoned pose in the local odometry frame.

        Returns:
            tuple[float, float, float]: A tuple ``(x, y, theta)`` representing
                the current position and orientation.
        """
        return (self.x, self.y, self.theta)
