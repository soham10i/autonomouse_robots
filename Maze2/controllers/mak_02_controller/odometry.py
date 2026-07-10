"""Wheel + IMU odometry -> a raw pose increment source (pure math).

Translation comes from differential-drive encoder deltas; heading is taken from
the IMU yaw (Webots' InertialUnit yaw is effectively drift-free).  The main
controller treats the per-step change of this raw pose as a motion increment and
composes it onto its scan-matched belief pose.
"""
from __future__ import annotations

import math
from typing import Optional, Tuple

import config as C
from geometry import wrap_angle


class Odometry:
    """Wheel + IMU dead-reckoning producing an (x, y, theta) pose estimate in the odometry frame.

    Attributes:
        x (float): The current x-coordinate in meters.
        y (float): The current y-coordinate in meters.
        theta (float): The current heading in radians.
    """

    def __init__(self) -> None:
        """Initializes the odometry tracking with zeros."""
        self.x: float = 0.0
        self.y: float = 0.0
        self.theta: float = 0.0
        self._prev_l: Optional[float] = None
        self._prev_r: Optional[float] = None

    def update(self, wl: Optional[float], wr: Optional[float], yaw_imu: Optional[float], dt: float) -> None:
        """Updates the odometry estimate based on wheel encoder values and IMU heading.

        Args:
            wl (Optional[float]): The accumulated left wheel position in radians.
            wr (Optional[float]): The accumulated right wheel position in radians.
            yaw_imu (Optional[float]): The absolute heading from the IMU in radians.
            dt (float): The time delta since the last update in seconds.
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
        
        # Calculate distance traveled by each wheel
        d_left = (wl - self._prev_l) * C.WHEEL_RADIUS
        d_right = (wr - self._prev_r) * C.WHEEL_RADIUS
        self._prev_l, self._prev_r = wl, wr
        
        # Center displacement and heading change
        ds = 0.5 * (d_left + d_right)
        dtheta = (d_right - d_left) / C.WHEEL_SEPARATION
        theta_mid = self.theta + 0.5 * dtheta
        
        # Update coordinates
        self.x += ds * math.cos(theta_mid)
        self.y += ds * math.sin(theta_mid)
        
        if yaw_imu is not None:
            self.theta = wrap_angle(yaw_imu)
        else:
            self.theta = wrap_angle(self.theta + dtheta)

    def pose(self) -> Tuple[float, float, float]:
        """Returns the current raw pose estimate.

        Returns:
            Tuple[float, float, float]: The current pose (x, y, theta) in the odometry frame.
        """
        return (self.x, self.y, self.theta)
