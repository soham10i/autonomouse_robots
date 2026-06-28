"""Wheel + IMU odometry -> a *raw* world pose (pure math).

The translation comes from differential-drive wheel encoder deltas; the heading
is taken from the IMU yaw when available (Webots' InertialUnit yaw is effectively
drift-free, so trusting it removes the worst of the heading error and lets the
scan matcher concentrate on correcting translation).

This raw pose is *not* the robot's believed pose — the SLAM layer treats the
per-step change in this raw pose as a motion increment and composes it onto the
scan-matched pose.  See :mod:`slam.scan_matcher` and the main controller.
"""
from __future__ import annotations

import math

import settings as S
from geometry import wrap_angle


class Odometry:
    def __init__(self):
        self.x = 0.0
        self.y = 0.0
        self.theta = 0.0
        self._prev_l = None
        self._prev_r = None

    def update(self, wl, wr, yaw_imu, dt):
        """Integrate one encoder step.  ``wl``/``wr`` are wheel angles (rad)."""
        if self._prev_l is None:
            self._prev_l = wl
            self._prev_r = wr
            if yaw_imu is not None:
                self.theta = wrap_angle(yaw_imu)
            return

        d_left = (wl - self._prev_l) * S.WHEEL_RADIUS
        d_right = (wr - self._prev_r) * S.WHEEL_RADIUS
        self._prev_l = wl
        self._prev_r = wr

        ds = 0.5 * (d_left + d_right)
        dtheta = (d_right - d_left) / S.WHEEL_SEPARATION
        theta_mid = self.theta + 0.5 * dtheta
        self.x += ds * math.cos(theta_mid)
        self.y += ds * math.sin(theta_mid)
        if yaw_imu is not None:
            self.theta = wrap_angle(yaw_imu)
        else:
            self.theta = wrap_angle(self.theta + dtheta)

    def pose(self):
        return (self.x, self.y, self.theta)
