"""Differential-drive wheel odometry with optional IMU yaw override.

`update()` takes the latest wheel-encoder angles (radians) and dt, and the
most recent absolute yaw from the IMU. We integrate linear speed from the
encoder deltas and trust the IMU heading when it is available — this is the
cheap version of the EKF that Phase 2b will replace.
"""
import math

import config as C
from utils import wrap_angle


class Odometry:
    def __init__(self):
        self.x = 0.0
        self.y = 0.0
        self.theta = 0.0
        self._prev_wl = None
        self._prev_wr = None

    def update(self, wl, wr, yaw_imu, dt):
        if self._prev_wl is None:
            self._prev_wl = wl
            self._prev_wr = wr
            if yaw_imu is not None:
                self.theta = wrap_angle(yaw_imu)
            return

        d_left = (wl - self._prev_wl) * C.ROBOT_WHEEL_RADIUS
        d_right = (wr - self._prev_wr) * C.ROBOT_WHEEL_RADIUS
        self._prev_wl = wl
        self._prev_wr = wr

        ds = 0.5 * (d_left + d_right)
        dtheta = (d_right - d_left) / C.ROBOT_WHEEL_SEPARATION

        theta_mid = self.theta + 0.5 * dtheta
        self.x += ds * math.cos(theta_mid)
        self.y += ds * math.sin(theta_mid)
        if yaw_imu is not None:
            self.theta = wrap_angle(yaw_imu)
        else:
            self.theta = wrap_angle(self.theta + dtheta)

    def pose(self):
        return self.x, self.y, self.theta
