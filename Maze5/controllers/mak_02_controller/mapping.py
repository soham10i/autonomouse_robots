"""Lidar occupancy grid, costmap generation, poison layer, and scan matcher.

Provides a pure NumPy implementation for environment mapping. It maintains a 2D
log-odds occupancy grid populated by ray-casting Lidar measurements, overlays
dynamically projected poison zones from the camera, and computes robust
correlative scan-matching to correct odometry drift. Indexing convention
strictly follows ``arr[ix, iy]`` where axis 0 represents the world x-coordinate
(column) and axis 1 represents the world y-coordinate (row).
"""
from __future__ import annotations

import math
from typing import Optional

import numpy as np

import config as C
from geometry import transform_points, wrap_angle


# --------------------------------------------------------------------------- #
#  Lidar range image -> body-frame points
# --------------------------------------------------------------------------- #
class LidarModel:
    """Projects raw planar Lidar range measurements into body-frame coordinates.

    The model assumes the Lidar sensor is aligned with the robot's z-axis, meaning
    the Lidar's local Cartesian frame matches the robot body frame. Beams are
    indexed such that beam ``i`` has heading ``theta_i = fov/2 - i*fov/(N-1)``.

    Attributes:
        n (int): Number of beams in the Lidar sweep.
        fov (float): Field of view of the Lidar in radians.
        r_min (float): Minimum valid range (closer readings are discarded as noise).
        r_max (float): Maximum valid range (farther readings are discarded).
        angles (np.ndarray): The heading angle (in radians) associated with each beam.
    """

    def __init__(
        self,
        n_beams: int,
        fov: float,
        r_min: Optional[float] = None,
        r_max: Optional[float] = None
    ) -> None:
        """Initializes the Lidar model and precomputes trigonometric tables.

        Args:
            n_beams (int): Total number of rays per scan.
            fov (float): Sensor field of view in radians.
            r_min (Optional[float], optional): Minimum valid range in meters. Defaults to config `LIDAR_RANGE_MIN`.
            r_max (Optional[float], optional): Maximum valid range in meters. Defaults to config `LIDAR_RANGE_MAX`.
        """
        self.n = int(n_beams)
        self.fov = float(fov)
        self.r_min = C.LIDAR_RANGE_MIN if r_min is None else max(r_min, C.LIDAR_RANGE_MIN)
        self.r_max = C.LIDAR_RANGE_MAX if r_max is None else min(r_max, 30.0)
        start = self.fov * 0.5
        step = self.fov / max(self.n - 1, 1)
        self.angles = start - step * np.arange(self.n)
        self._cos = np.cos(self.angles)
        self._sin = np.sin(self.angles)

    def ranges_to_body(self, ranges: np.ndarray | list[float]) -> tuple[np.ndarray, np.ndarray]:
        """Converts an array of polar ranges into Cartesian body-frame points.

        Args:
            ranges (np.ndarray | list[float]): Raw Lidar distance measurements in meters.

        Returns:
            tuple[np.ndarray, np.ndarray]: A tuple containing:
                - pts_body (np.ndarray): An ``(M, 2)`` array of ``(x, y)`` endpoints for the ``M`` valid beams.
                - r (np.ndarray): An ``(M,)`` array of the corresponding valid ranges.
        """
        ranges = np.asarray(ranges, dtype=np.float64)
        if ranges.size != self.n:
            return np.empty((0, 2)), np.empty((0,))
        valid = np.isfinite(ranges) & (ranges > self.r_min) & (ranges < self.r_max)
        r = ranges[valid]
        x = r * self._cos[valid]
        y = r * self._sin[valid]
        return np.stack([x, y], axis=1), r


# --------------------------------------------------------------------------- #
#  8-connected dilation helpers (used for inflation + scan-match field)
# --------------------------------------------------------------------------- #
def _dilate8(mask: np.ndarray) -> np.ndarray:
    """Applies a morphological 8-connected dilation to a boolean grid.

    Args:
        mask (np.ndarray): A 2D boolean array representing the input mask.

    Returns:
        np.ndarray: The dilated 2D boolean array.
    """
    out = mask.copy()
    out[1:, :] |= mask[:-1, :]
    out[:-1, :] |= mask[1:, :]
    out[:, 1:] |= mask[:, :-1]
    out[:, :-1] |= mask[:, 1:]
    out[1:, 1:] |= mask[:-1, :-1]
    out[1:, :-1] |= mask[:-1, 1:]
    out[:-1, 1:] |= mask[1:, :-1]
    out[:-1, :-1] |= mask[1:, 1:]
    return out


def _distance_bands(mask: np.ndarray, max_band: int) -> np.ndarray:
    """Computes the approximate Chebyshev distance in cells to the nearest True pixel.

    Args:
        mask (np.ndarray): A 2D boolean mask designating the source pixels.
        max_band (int): The maximum distance band to compute.

    Returns:
        np.ndarray: A 2D int16 array of distances. Distances beyond ``max_band``
            are clamped to ``max_band + 1``.
    """
    dist = np.full(mask.shape, max_band + 1, dtype=np.int16)
    cur = mask.copy()
    if not cur.any():
        return dist
    dist[cur] = 0
    for k in range(1, max_band + 1):
        nxt = _dilate8(cur)
        newly = nxt & ~cur
        if newly.any():
            dist[newly] = k
        cur = nxt
        if cur.all():
            break
    return dist


# --------------------------------------------------------------------------- #
#  Occupancy grid
# --------------------------------------------------------------------------- #
class OccupancyGrid:
    """2D log-odds occupancy map with poison layering and scan-matching capabilities.

    Integrates Lidar sweeps to build an environmental map, computes navigation costmaps,
    and maintains an auxiliary layer to track projected "poison" floor patches.

    Attributes:
        res (float): Resolution of the grid in meters per cell.
        n (int): Number of cells along one side of the square grid map.
        ox (float): World x-coordinate of the bottom-left grid corner (meters).
        oy (float): World y-coordinate of the bottom-left grid corner (meters).
        L (np.ndarray): The 2D float32 log-odds array storing occupancy probabilities.
        poison (np.ndarray): The 2D boolean array storing mapped poison regions.
    """

    def __init__(self) -> None:
        """Initializes the occupancy grid arrays using configuration settings."""
        self.res = C.GRID_RESOLUTION
        self.n = C.GRID_CELLS
        self.ox, self.oy = C.GRID_ORIGIN
        self.L = np.zeros((self.n, self.n), dtype=np.float32)
        self.poison = np.zeros((self.n, self.n), dtype=bool)

    # ----------------------------------------------------- coordinate maths
    def world_to_grid(self, wx: float, wy: float) -> tuple[int, int]:
        """Converts world coordinates to grid indices.

        Args:
            wx (float): World x-coordinate in meters.
            wy (float): World y-coordinate in meters.

        Returns:
            tuple[int, int]: The corresponding `(ix, iy)` grid indices.
        """
        ix = int((wx - self.ox) / self.res)
        iy = int((wy - self.oy) / self.res)
        return ix, iy

    def world_to_grid_arr(self, wx: np.ndarray, wy: np.ndarray) -> tuple[np.ndarray, np.ndarray]:
        """Vectorized conversion from world coordinates to grid indices.

        Args:
            wx (np.ndarray): Array of world x-coordinates.
            wy (np.ndarray): Array of world y-coordinates.

        Returns:
            tuple[np.ndarray, np.ndarray]: Arrays of integer `(ix, iy)` indices.
        """
        ix = ((np.asarray(wx) - self.ox) / self.res).astype(np.int32)
        iy = ((np.asarray(wy) - self.oy) / self.res).astype(np.int32)
        return ix, iy

    def grid_to_world(self, ix: int, iy: int) -> tuple[float, float]:
        """Converts grid indices to the world coordinates of the cell center.

        Args:
            ix (int): Grid column index.
            iy (int): Grid row index.

        Returns:
            tuple[float, float]: The `(wx, wy)` world coordinates of the cell center.
        """
        return (self.ox + (ix + 0.5) * self.res, self.oy + (iy + 0.5) * self.res)

    def in_bounds(self, ix: int, iy: int) -> bool:
        """Checks if a scalar index pair lies within the grid limits.

        Args:
            ix (int): Grid column index.
            iy (int): Grid row index.

        Returns:
            bool: True if the indices are strictly inside the grid array, False otherwise.
        """
        return 0 <= ix < self.n and 0 <= iy < self.n

    def _inb_arr(self, ix: np.ndarray, iy: np.ndarray) -> np.ndarray:
        """Vectorized bounds checking for grid indices.

        Args:
            ix (np.ndarray): Array of column indices.
            iy (np.ndarray): Array of row indices.

        Returns:
            np.ndarray: A boolean array where True indicates the point is within bounds.
        """
        return (ix >= 0) & (ix < self.n) & (iy >= 0) & (iy < self.n)

    # ----------------------------------------------------------- integration
    def integrate_scan(self, pose: tuple[float, float, float], pts_body: np.ndarray, ranges: np.ndarray) -> None:
        """Integrates a Lidar scan into the log-odds map.

        Applies Bresenham-like ray tracing to mark intermediate cells as free,
        and marks the final endpoint cell as occupied. The updates are mapped
        as log-odds additions to `self.L`.

        Args:
            pose (tuple[float, float, float]): The `(x, y, theta)` sensor origin in world coordinates.
            pts_body (np.ndarray): An `(M, 2)` array of valid beam endpoints in the local body frame.
            ranges (np.ndarray): An `(M,)` array of ranges corresponding to `pts_body`.
        """
        if pts_body.shape[0] == 0:
            return
        x, y, th = pose
        pts_world = transform_points(pts_body, x, y, th)
        rix, riy = self.world_to_grid(x, y)

        free_lin = []
        occ_lin = []
        n_cells = self.n
        for k in range(pts_world.shape[0]):
            ex, ey = pts_world[k, 0], pts_world[k, 1]
            rng = ranges[k]
            steps = int(rng / self.res)
            if steps > 1:
                ts = np.arange(steps) / float(steps)  # 0..(steps-1)/steps, excl. endpoint
                fx = x + (ex - x) * ts
                fy = y + (ey - y) * ts
                ixs = ((fx - self.ox) / self.res).astype(np.int32)
                iys = ((fy - self.oy) / self.res).astype(np.int32)
                ok = (ixs >= 0) & (ixs < n_cells) & (iys >= 0) & (iys < n_cells)
                free_lin.append(ixs[ok] * n_cells + iys[ok])
            if rng < C.LIDAR_MAX_INTEGRATE:
                eix, eiy = self.world_to_grid(ex, ey)
                if self.in_bounds(eix, eiy):
                    occ_lin.append(eix * n_cells + eiy)

        Lflat = self.L.reshape(-1)
        if free_lin:
            f = np.unique(np.concatenate(free_lin))
            np.add.at(Lflat, f, C.L_FREE)
        if occ_lin:
            o = np.unique(np.asarray(occ_lin, dtype=np.int64))
            np.add.at(Lflat, o, C.L_OCC)
        np.clip(self.L, C.L_MIN, C.L_MAX, out=self.L)

    def mark_free_disc(self, wx: float, wy: float, radius: float) -> None:
        """Forces a circular neighborhood around a coordinate to a 'free' state.

        Primarily used to forcibly clear the area immediately surrounding the robot,
        ensuring stray or self-intersecting Lidar hits do not trap the planner.
        This operation respects the poison layer (does not clear poison).

        Args:
            wx (float): Center world x-coordinate.
            wy (float): Center world y-coordinate.
            radius (float): Radius of the free disc in meters.
        """
        cix, ciy = self.world_to_grid(wx, wy)
        r = int(math.ceil(radius / self.res))
        x0, x1 = max(0, cix - r), min(self.n, cix + r + 1)
        y0, y1 = max(0, ciy - r), min(self.n, ciy + r + 1)
        if x0 >= x1 or y0 >= y1:
            return
        sub = self.L[x0:x1, y0:y1]
        np.minimum(sub, C.L_FREE_THRESH - 0.1, out=sub)

    def add_poison_points(self, pts_world: np.ndarray) -> None:
        """Stamps camera-projected green floor segments into the sticky poison layer.

        Args:
            pts_world (np.ndarray): An `(M, 2)` array of world coordinates classified
                as green poison by the perception module.
        """
        if pts_world.shape[0] == 0:
            return
        ix, iy = self.world_to_grid_arr(pts_world[:, 0], pts_world[:, 1])
        ok = self._inb_arr(ix, iy)
        self.poison[ix[ok], iy[ok]] = True

    # --------------------------------------------------------------- masks
    def occupied_mask(self) -> np.ndarray:
        """Retrieves a boolean mask representing definitively occupied cells.

        Returns:
            np.ndarray: A 2D boolean array where True denotes occupancy.
        """
        return self.L > C.L_OCC_THRESH

    def free_mask(self) -> np.ndarray:
        """Retrieves a boolean mask representing definitively free cells.

        Returns:
            np.ndarray: A 2D boolean array where True denotes free space.
        """
        return self.L < C.L_FREE_THRESH

    def unknown_mask(self) -> np.ndarray:
        """Retrieves a boolean mask representing undiscovered or uncertain cells.

        Returns:
            np.ndarray: A 2D boolean array where True denotes an unknown cell.
        """
        return (~self.occupied_mask()) & (~self.free_mask())

    # ------------------------------------------------- mapped obstacles query
    def poison_points_near(self, wx: float, wy: float, radius: float) -> np.ndarray:
        """Extracts mapped poison cell centers within a specified neighborhood.

        Args:
            wx (float): World x-coordinate of the query center.
            wy (float): World y-coordinate of the query center.
            radius (float): Maximum query distance in meters.

        Returns:
            np.ndarray: An `(M, 2)` array containing the `(wx, wy)` locations of
                poison cell centers found in the radius.
        """
        if not self.poison.any():
            return np.empty((0, 2))
        cix, ciy = self.world_to_grid(wx, wy)
        r = int(math.ceil(radius / self.res))
        x0, x1 = max(0, cix - r), min(self.n, cix + r + 1)
        y0, y1 = max(0, ciy - r), min(self.n, ciy + r + 1)
        sub = self.poison[x0:x1, y0:y1]
        lx, ly = np.nonzero(sub)
        if lx.size == 0:
            return np.empty((0, 2))
        wx_ = self.ox + (x0 + lx + 0.5) * self.res
        wy_ = self.oy + (y0 + ly + 0.5) * self.res
        return np.stack([wx_, wy_], axis=1)

    def is_poison_world(self, wx: float, wy: float) -> bool:
        """Determines if a specific world coordinate falls within a mapped poison cell.

        Args:
            wx (float): The queried world x-coordinate.
            wy (float): The queried world y-coordinate.

        Returns:
            bool: True if the cell is poisoned and within map bounds, False otherwise.
        """
        ix, iy = self.world_to_grid(wx, wy)
        return self.in_bounds(ix, iy) and bool(self.poison[ix, iy])

    # ------------------------------------------------------------ scan match
    def _score_field(self) -> np.ndarray:
        """Builds a scalar evaluation field used during scan matching.

        Cells are assigned float values based on proximity to occupied space:
        Occupied=1.0, immediate 8-connected neighbors=0.6, next outer ring=0.3.

        Returns:
            np.ndarray: A 2D float32 field representing correlation weights.
        """
        occ = self.occupied_mask()
        field = np.zeros((self.n, self.n), dtype=np.float32)
        field[occ] = 1.0
        d1 = _dilate8(occ) & ~occ
        field[d1] = 0.6
        d2 = _dilate8(_dilate8(occ)) & ~_dilate8(occ)
        field[d2] = 0.3
        return field

    def scan_match(
        self,
        pred_pose: tuple[float, float, float],
        pts_body: np.ndarray
    ) -> tuple[tuple[float, float, float], float]:
        """Performs correlative scan-matching to correct the predicted odometry pose.

        Executes an exhaustive search over a small local parameter space ``(x, y, yaw)``
        to align the current Lidar scan with the constructed map. Yields the highest
        scoring pose if it exceeds the minimum correlation threshold.

        Args:
            pred_pose (tuple[float, float, float]): The initial odometry estimate ``(x, y, yaw)``.
            pts_body (np.ndarray): An ``(M, 2)`` array of current Lidar endpoints in body coordinates.

        Returns:
            tuple[tuple[float, float, float], float]: A tuple containing the corrected
                pose ``(x, y, yaw)`` and the correlation ratio (fraction of hits). If the
                correlation is below `C.SM_MIN_HIT_FRAC`, the uncorrected `pred_pose` is returned.
        """
        m = pts_body.shape[0]
        if m == 0 or not self.occupied_mask().any():
            return pred_pose, 0.0
        if m > C.SM_MAX_BEAMS:
            idx = np.linspace(0, m - 1, C.SM_MAX_BEAMS).astype(int)
            pts = pts_body[idx]
        else:
            pts = pts_body
        field = self._score_field()
        px, py, pth = pred_pose

        lin = np.arange(-C.SM_LIN_HALF, C.SM_LIN_HALF + 1e-9, C.SM_LIN_STEP)
        ang = np.arange(-C.SM_ANG_HALF, C.SM_ANG_HALF + 1e-9, C.SM_ANG_STEP)
        best = (-1.0, 0.0, 0.0, 0.0)
        bx, by = pts[:, 0], pts[:, 1]
        for dth in ang:
            c, s = math.cos(pth + dth), math.sin(pth + dth)
            wx0 = px + c * bx - s * by
            wy0 = py + s * bx + c * by
            for dx in lin:
                wx = wx0 + dx
                for dy in lin:
                    wy = wy0 + dy
                    ix = ((wx - self.ox) / self.res).astype(np.int32)
                    iy = ((wy - self.oy) / self.res).astype(np.int32)
                    ok = (ix >= 0) & (ix < self.n) & (iy >= 0) & (iy < self.n)
                    if not ok.any():
                        continue
                    sc = field[ix[ok], iy[ok]].sum()
                    if sc > best[0]:
                        best = (sc, dx, dy, dth)
        score, dx, dy, dth = best
        hit_frac = score / max(m, 1)
        if hit_frac < C.SM_MIN_HIT_FRAC:
            return pred_pose, hit_frac
        return (px + dx, py + dy, wrap_angle(pth + dth)), hit_frac

    # -------------------------------------------------------------- costmap
    def build_costmap(self) -> tuple[np.ndarray, np.ndarray]:
        """Constructs a planning costmap overlay combining static obstacles and poison.

        Applies configuration-defined lethal inflation radii and soft cost decaying
        halos around both solid walls and mapped poison regions. Unknown cells
        remain unpenalized (optimistic) to allow navigation toward frontiers.

        Returns:
            tuple[np.ndarray, np.ndarray]: A tuple consisting of:
                - cost (np.ndarray): A 2D float32 array assigning an additive traversal
                  penalty per cell (with infinity denoting lethal zones).
                - lethal (np.ndarray): A 2D boolean array where True explicitly denotes
                  a strictly impassable cell.
        """
        occ = self.occupied_mask()
        res = self.res

        hard_b = max(1, int(round(C.HARD_OBS_DIST / res)))
        soft_b = max(hard_b, int(round((C.HARD_OBS_DIST + C.SOFT_OBS_HALO) / res)))
        d_obs = _distance_bands(occ, soft_b + 1)

        lethal = d_obs <= hard_b
        cost = np.zeros((self.n, self.n), dtype=np.float32)
        soft_zone = (d_obs > hard_b) & (d_obs <= soft_b)
        # exponential-ish decay across the soft halo
        frac = (soft_b - d_obs.astype(np.float32)) / max(1, (soft_b - hard_b))
        cost[soft_zone] += C.COST_OBS_WEIGHT * frac[soft_zone]

        if self.poison.any():
            ph = max(1, int(round(C.POISON_HARD_DIST / res)))
            ps = max(ph, int(round((C.POISON_HARD_DIST + C.POISON_SOFT_HALO) / res)))
            d_pois = _distance_bands(self.poison, ps + 1)
            lethal |= d_pois <= ph
            pz = (d_pois > ph) & (d_pois <= ps)
            pfrac = (ps - d_pois.astype(np.float32)) / max(1, (ps - ph))
            cost[pz] += C.COST_POISON_WEIGHT * pfrac[pz]

        cost[lethal] = np.inf
        return cost, lethal
