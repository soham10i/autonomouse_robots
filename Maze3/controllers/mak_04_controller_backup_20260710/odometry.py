"""Wheel + IMU odometry -> a raw pose increment source (pure math).

Translation comes from differential-drive encoder deltas; heading is taken from
the IMU yaw (Webots' InertialUnit yaw is effectively drift-free).  The main
controller treats the per-step change of this raw pose as a motion increment and
composes it onto its scan-matched belief pose.
"""
from __future__ import annotations

import math

import config as C
from geometry import wrap_angle


class Odometry:
    """Wheel + IMU dead-reckoning producing an (x, y, theta) pose estimate in the odometry frame."""
    def __init__(self):
        self.x = 0.0
        self.y = 0.0
        self.theta = 0.0
        self._prev_l = None
        self._prev_r = None

    def update(self, wl, wr, yaw_imu, dt):
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

    def pose(self):
        return (self.x, self.y, self.theta)
