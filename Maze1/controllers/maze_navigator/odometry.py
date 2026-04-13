"""Dead-reckoning pose estimation from wheel encoders + compass."""

import math
from constants import WHEEL_RADIUS, WHEEL_BASE


class Odometry:
    """Tracks robot pose (x, y, theta) using wheel encoders and compass."""

    def __init__(self):
        self.x = 0.0
        self.y = 0.0
        self.theta = 0.0  # radians, 0 = north (+Y)
        self._prev_left = None
        self._prev_right = None

    def update(self, left_enc: float, right_enc: float, compass_heading: float):
        """Update pose from encoder values and compass heading.

        Args:
            left_enc: left wheel encoder (radians, cumulative)
            right_enc: right wheel encoder (radians, cumulative)
            compass_heading: global heading from compass (radians)
        """
        if self._prev_left is None:
            self._prev_left = left_enc
            self._prev_right = right_enc
            self.theta = compass_heading
            return

        dl = (left_enc - self._prev_left) * WHEEL_RADIUS
        dr = (right_enc - self._prev_right) * WHEEL_RADIUS
        self._prev_left = left_enc
        self._prev_right = right_enc

        # Use compass for heading (no drift)
        self.theta = compass_heading

        # Average distance traveled
        dist = (dl + dr) / 2.0

        # Update position using compass heading
        self.x += dist * math.sin(self.theta)
        self.y += dist * math.cos(self.theta)

    def get_pose(self):
        """Returns (x, y, theta) in world coordinates."""
        return self.x, self.y, self.theta

    def distance_to(self, wx, wy):
        """Euclidean distance from current position to a world point."""
        return math.sqrt((wx - self.x) ** 2 + (wy - self.y) ** 2)

    def bearing_to(self, wx, wy):
        """Bearing angle from current position to a world point (radians)."""
        return math.atan2(wx - self.x, wy - self.y)

    def heading_error(self, target_bearing):
        """Signed heading error to a target bearing, normalized to [-pi, pi]."""
        err = target_bearing - self.theta
        while err > math.pi:
            err -= 2 * math.pi
        while err < -math.pi:
            err += 2 * math.pi
        return err
