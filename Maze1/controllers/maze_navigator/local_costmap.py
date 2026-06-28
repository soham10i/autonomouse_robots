"""Rolling local costmap centred on the robot.

Provides a small, fast-updating grid (default 3×3 m) that fuses raw
LiDAR hits at high frequency.  The global Grid2D is authoritative for
planning; this map is used *only* by the DWA planner for trajectory
collision checks.

Cells are 0 (free) or 1 (obstacle).  Inflation is applied via a
pre-computed disc kernel so costmap() returns in < 1 ms.
"""
import math
import numpy as np

import config as C


# ---------------------------------------------------------------------------
# Local costmap parameters
# ---------------------------------------------------------------------------
LOCAL_SIZE_M = 3.0                 # side length of the rolling window (m)
LOCAL_RES = 0.03                   # finer than the global grid for DWA precision
LOCAL_CELLS = int(LOCAL_SIZE_M / LOCAL_RES)
LOCAL_DECAY_TICKS = 8              # obstacle cells are cleared after N ticks
                                   # without re-observation (handles dynamics)
LOCAL_INFLATE_M = C.ROBOT_RADIUS + 0.03  # inflation radius


class LocalCostmap:
    """Fast rolling costmap for reactive local planning."""

    def __init__(self):
        self.size_m = LOCAL_SIZE_M
        self.res = LOCAL_RES
        self.cells = LOCAL_CELLS
        self.inflate_r = LOCAL_INFLATE_M

        # Obstacle hit counter (decays toward 0 each tick; obstacle when > 0)
        self._hits = np.zeros((self.cells, self.cells), dtype=np.int8)

        # Pre-compute the inflation structuring element
        n = max(1, int(math.ceil(self.inflate_r / self.res)))
        size = 2 * n + 1
        y, x = np.ogrid[-n:n + 1, -n:n + 1]
        self._inflate_kernel = (x * x + y * y) <= n * n

        # Centre of the grid in world frame (updated every tick)
        self._cx = 0.0
        self._cy = 0.0

    # ----------------------------- public API -------------------------------

    def update(self, pose, ranges, angles):
        """Re-centre the grid on the robot and integrate the latest scan."""
        rx, ry, rth = pose
        self._cx = rx
        self._cy = ry

        # Decay existing hits
        self._hits = np.clip(self._hits - 1, 0, LOCAL_DECAY_TICKS)

        # Convert lidar to world-frame obstacle points
        if ranges is None or angles is None or len(ranges) == 0:
            return
        ra = np.asarray(ranges, dtype=np.float32)
        an = np.asarray(angles, dtype=np.float64)
        valid = np.isfinite(ra) & (ra > 0.05) & (ra < self.size_m * 0.5)
        if not valid.any():
            return
        r = ra[valid]
        a = an[valid]
        wx = rx + r * np.cos(rth + a)
        wy = ry + r * np.sin(rth + a)

        # Project into local grid
        ix = np.floor((wx - (self._cx - self.size_m / 2)) / self.res).astype(int)
        iy = np.floor((wy - (self._cy - self.size_m / 2)) / self.res).astype(int)
        m = (ix >= 0) & (ix < self.cells) & (iy >= 0) & (iy < self.cells)
        ix = ix[m]
        iy = iy[m]
        self._hits[ix, iy] = LOCAL_DECAY_TICKS

    def obstacle_mask(self):
        """Raw obstacle mask (before inflation)."""
        return self._hits > 0

    def inflated_mask(self):
        """Inflated obstacle mask for trajectory collision checking."""
        raw = self.obstacle_mask()
        if not raw.any():
            return raw
        try:
            from scipy.ndimage import binary_dilation
            return binary_dilation(raw, structure=self._inflate_kernel)
        except ImportError:
            return raw  # fall back to uninflated

    def is_colliding(self, wx, wy):
        """Check if a world point hits the inflated local costmap."""
        ix = int(math.floor((wx - (self._cx - self.size_m / 2)) / self.res))
        iy = int(math.floor((wy - (self._cy - self.size_m / 2)) / self.res))
        if not (0 <= ix < self.cells and 0 <= iy < self.cells):
            return True  # outside local map → assume blocked
        inflated = self.inflated_mask()
        return bool(inflated[ix, iy])

    def w2local(self, wx, wy):
        """World → local grid index."""
        ix = int(math.floor((wx - (self._cx - self.size_m / 2)) / self.res))
        iy = int(math.floor((wy - (self._cy - self.size_m / 2)) / self.res))
        return ix, iy
