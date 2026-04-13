"""Occupancy grid map with log-odds updates and obstacle inflation."""

import math
from constants import (
    GRID_RESOLUTION, GRID_WIDTH, GRID_HEIGHT,
    CELL_UNKNOWN, CELL_FREE, CELL_OCCUPIED, CELL_POISON, CELL_INFLATED,
    LOG_ODDS_PRIOR, LOG_ODDS_OCC, LOG_ODDS_FREE,
    LOG_ODDS_MAX, LOG_ODDS_MIN, LOG_ODDS_OCC_THRESH, LOG_ODDS_FREE_THRESH,
    INFLATION_RADIUS, LIDAR_MAX_RANGE, LIDAR_MIN_RANGE,
)


class OccupancyGrid:
    """2D occupancy grid map using log-odds representation."""

    def __init__(self):
        self.width = GRID_WIDTH
        self.height = GRID_HEIGHT
        self.resolution = GRID_RESOLUTION
        # Origin in world coordinates (center of grid)
        self.origin_x = -(self.width * self.resolution) / 2.0
        self.origin_y = -(self.height * self.resolution) / 2.0
        # Log-odds grid
        self.log_odds = [[LOG_ODDS_PRIOR] * self.width for _ in range(self.height)]
        # Poison cells (marked separately, never cleared)
        self.poison_cells = set()
        # Inflated grid for planning (rebuilt on demand)
        self._planning_grid = None
        self._planning_dirty = True

    def world_to_grid(self, wx, wy):
        """Convert world coordinates to grid cell (col, row)."""
        col = int((wx - self.origin_x) / self.resolution)
        row = int((wy - self.origin_y) / self.resolution)
        return col, row

    def grid_to_world(self, col, row):
        """Convert grid cell center to world coordinates."""
        wx = self.origin_x + (col + 0.5) * self.resolution
        wy = self.origin_y + (row + 0.5) * self.resolution
        return wx, wy

    def in_bounds(self, col, row):
        return 0 <= col < self.width and 0 <= row < self.height

    def get_cell(self, col, row):
        """Get cell state from log-odds."""
        if not self.in_bounds(col, row):
            return CELL_UNKNOWN
        if (col, row) in self.poison_cells:
            return CELL_POISON
        lo = self.log_odds[row][col]
        if lo > LOG_ODDS_OCC_THRESH:
            return CELL_OCCUPIED
        elif lo < LOG_ODDS_FREE_THRESH:
            return CELL_FREE
        return CELL_UNKNOWN

    def update_from_lidar(self, robot_x, robot_y, robot_theta, ranges, num_points, lidar_fov):
        """Update grid from a LiDAR scan using Bresenham ray-casting.

        Args:
            robot_x, robot_y: robot position in world coords
            robot_theta: robot heading (radians, 0 = north/+Y)
            ranges: list of range values
            num_points: number of LiDAR points
            lidar_fov: field of view in radians
        """
        if not ranges or num_points == 0:
            return

        rx, ry = self.world_to_grid(robot_x, robot_y)
        if not self.in_bounds(rx, ry):
            return

        angle_increment = lidar_fov / num_points

        for i in range(num_points):
            r = ranges[i]
            if r < LIDAR_MIN_RANGE or r > LIDAR_MAX_RANGE or math.isinf(r) or math.isnan(r):
                # For max-range readings, mark ray as free but no obstacle at endpoint
                if math.isinf(r) or r > LIDAR_MAX_RANGE:
                    r_free = LIDAR_MAX_RANGE
                    angle = robot_theta + (i - num_points / 2.0) * angle_increment
                    end_wx = robot_x + r_free * math.sin(angle)
                    end_wy = robot_y + r_free * math.cos(angle)
                    ex, ey = self.world_to_grid(end_wx, end_wy)
                    self._trace_ray_free(rx, ry, ex, ey)
                continue

            angle = robot_theta + (i - num_points / 2.0) * angle_increment
            hit_wx = robot_x + r * math.sin(angle)
            hit_wy = robot_y + r * math.cos(angle)
            hx, hy = self.world_to_grid(hit_wx, hit_wy)

            # Mark cells along ray as free
            self._trace_ray_free(rx, ry, hx, hy)
            # Mark endpoint as occupied
            if self.in_bounds(hx, hy):
                self.log_odds[hy][hx] = min(
                    LOG_ODDS_MAX, self.log_odds[hy][hx] + LOG_ODDS_OCC
                )

        self._planning_dirty = True

    def _trace_ray_free(self, x0, y0, x1, y1):
        """Bresenham line from (x0,y0) to (x1,y1), marking cells as free (excluding endpoint)."""
        cells = self._bresenham(x0, y0, x1, y1)
        # Mark all cells except the last one as free
        for col, row in cells[:-1]:
            if self.in_bounds(col, row):
                self.log_odds[row][col] = max(
                    LOG_ODDS_MIN, self.log_odds[row][col] + LOG_ODDS_FREE
                )

    @staticmethod
    def _bresenham(x0, y0, x1, y1):
        """Bresenham's line algorithm. Returns list of (col, row) cells."""
        cells = []
        dx = abs(x1 - x0)
        dy = abs(y1 - y0)
        sx = 1 if x0 < x1 else -1
        sy = 1 if y0 < y1 else -1
        err = dx - dy

        while True:
            cells.append((x0, y0))
            if x0 == x1 and y0 == y1:
                break
            e2 = 2 * err
            if e2 > -dy:
                err -= dy
                x0 += sx
            if e2 < dx:
                err += dx
                y0 += sy
        return cells

    def mark_poison(self, wx, wy, radius=0.25):
        """Mark a circular region as poison in the grid."""
        cx, cy = self.world_to_grid(wx, wy)
        r_cells = int(math.ceil(radius / self.resolution))
        for dr in range(-r_cells, r_cells + 1):
            for dc in range(-r_cells, r_cells + 1):
                if dr * dr + dc * dc <= r_cells * r_cells:
                    nc, nr = cx + dc, cy + dr
                    if self.in_bounds(nc, nr):
                        self.poison_cells.add((nc, nr))
        self._planning_dirty = True

    def mark_occupied(self, wx, wy):
        """Mark a single world point as occupied."""
        col, row = self.world_to_grid(wx, wy)
        if self.in_bounds(col, row):
            self.log_odds[row][col] = LOG_ODDS_MAX
            self._planning_dirty = True

    def get_planning_grid(self):
        """Returns the inflated planning grid. Rebuilds if dirty.

        Returns a 2D list where True = passable, False = blocked.
        """
        if not self._planning_dirty and self._planning_grid is not None:
            return self._planning_grid

        grid = [[True] * self.width for _ in range(self.height)]

        # Mark occupied and poison cells + inflate
        for row in range(self.height):
            for col in range(self.width):
                cell = self.get_cell(col, row)
                if cell == CELL_OCCUPIED or cell == CELL_POISON:
                    # Inflate around this cell
                    for dr in range(-INFLATION_RADIUS, INFLATION_RADIUS + 1):
                        for dc in range(-INFLATION_RADIUS, INFLATION_RADIUS + 1):
                            if dr * dr + dc * dc <= INFLATION_RADIUS * INFLATION_RADIUS:
                                nr, nc = row + dr, col + dc
                                if 0 <= nr < self.height and 0 <= nc < self.width:
                                    grid[nr][nc] = False

        # Also mark unknown cells as blocked for planning
        # (only plan through explored areas)
        for row in range(self.height):
            for col in range(self.width):
                if self.get_cell(col, row) == CELL_UNKNOWN:
                    grid[row][col] = False

        self._planning_grid = grid
        self._planning_dirty = False
        return grid

    def is_explored(self, col, row):
        """Check if a cell has been explored (not unknown)."""
        return self.get_cell(col, row) != CELL_UNKNOWN

    def get_frontiers(self):
        """Find frontier cells (free cells adjacent to unknown cells).

        Returns list of (col, row) frontier cells.
        """
        frontiers = []
        for row in range(1, self.height - 1):
            for col in range(1, self.width - 1):
                if self.get_cell(col, row) != CELL_FREE:
                    continue
                # Check 4-connected neighbors for unknown
                for dc, dr in [(0, 1), (0, -1), (1, 0), (-1, 0)]:
                    nc, nr = col + dc, row + dr
                    if self.get_cell(nc, nr) == CELL_UNKNOWN:
                        frontiers.append((col, row))
                        break
        return frontiers
