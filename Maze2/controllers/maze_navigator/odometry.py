"""Pose estimation — compass-corrected wheel odometry.

Heading (θ) comes from the compass (drift-free).
Translation (x, y) comes from wheel encoder deltas.

Webots coordinate convention:
  - θ = 0 points along +Y axis
  - x += dist * sin(θ)
  - y += dist * cos(θ)

Reference: Siegwart et al. (2011), Introduction to Autonomous Mobile Robots, §5.2
"""

import math
from constants import WHEEL_RADIUS


class Odometry:
    """Track robot pose (x, y, θ) using compass + encoders."""

    def __init__(self):
        self.x = 0.0       # meters, world frame
        self.y = 0.0       # meters, world frame
        self.theta = 0.0   # radians, from compass

        self._prev_left = None   # previous encoder reading (radians)
        self._prev_right = None

    def update(self, left_enc, right_enc, compass_heading):
        """Update pose estimate from new sensor readings.

        Args:
            left_enc:  current left encoder value (radians, cumulative)
            right_enc: current right encoder value (radians, cumulative)
            compass_heading: current compass reading (radians, [0, 2π))
        """
        # Use compass for heading (no drift)
        self.theta = compass_heading

        # First call — just store encoder values, no delta yet
        if self._prev_left is None:
            self._prev_left = left_enc
            self._prev_right = right_enc
            return

        # Compute encoder deltas
        dl = left_enc - self._prev_left
        dr = right_enc - self._prev_right
        self._prev_left = left_enc
        self._prev_right = right_enc

        # Average distance traveled
        dist = WHEEL_RADIUS * (dl + dr) / 2.0

        # Update position (Webots convention: θ=0 → +Y)
        self.x += dist * math.sin(self.theta)
        self.y += dist * math.cos(self.theta)

    def get_pose(self):
        """Return current pose as (x, y, θ)."""
        return self.x, self.y, self.theta
