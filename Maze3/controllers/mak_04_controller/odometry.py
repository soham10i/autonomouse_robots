"""Wheel + IMU odometry → a raw pose increment source (pure math).

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
    """Wheel + IMU dead-reckoning producing an ``(x, y, theta)`` pose estimate.

    Integrates encoder ticks and IMU yaw each timestep to maintain a running
    odometry-frame pose.  The main controller consumes the *difference* between
    consecutive calls to :meth:`pose` as a body-frame increment.

    Attributes:
        x: Current x-position in the odometry frame (metres).
        y: Current y-position in the odometry frame (metres).
        theta: Current heading in the odometry frame (radians).
    """

    def __init__(self) -> None:
        """Initialise the odometry state at the origin ``(0, 0, 0)``."""
        self.x: float = 0.0
        self.y: float = 0.0
        self.theta: float = 0.0
        self._prev_l: Optional[float] = None
        self._prev_r: Optional[float] = None

    def update(self, wl: Optional[float], wr: Optional[float],
               yaw_imu: Optional[float], dt: float) -> None:
        """Advance the pose estimate by one timestep.

        On the first call the encoder positions are latched as a zero reference;
        subsequent calls compute the forward kinematics increment.  If encoders
        are unavailable (``None``), only the IMU heading is updated.

        Args:
            wl: Left-wheel cumulative encoder angle (radians), or ``None``.
            wr: Right-wheel cumulative encoder angle (radians), or ``None``.
            yaw_imu: IMU yaw reading (radians), or ``None`` if unavailable.
            dt: Control timestep (seconds) — currently unused but kept for
                interface parity with velocity-based odometry variants.
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

    def pose(self) -> Tuple[float, float, float]:
        """Return the current odometry pose ``(x, y, theta)``."""
        return (self.x, self.y, self.theta)
