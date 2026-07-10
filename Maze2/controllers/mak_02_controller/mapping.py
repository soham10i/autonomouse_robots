"""Lidar occupancy grid, costmap, poison layer, depth-aux layer, scan matcher.

Pure NumPy (no Webots, no SciPy) so it is fully testable and has no fragile
dependencies.  Index convention throughout: ``arr[ix, iy]`` with axis 0 = world
x (column), axis 1 = world y (row).

Maze4, unlike Maze5, has 4 low wall panels the 2-D lidar partially or fully
misses (see config.py).  This module therefore adds a depth-camera-derived
``aux`` boolean layer ON TOP of the Maze5 lidar-only design.  Critically, the
aux layer is kept deliberately simple: a hit SETS a cell, a clear depth
sight-line CLEARS cells along it, every tick, with NO sticky-hit counters, NO
decay heuristics, and NO "reconcile with lidar walls" radius hack.  That
counter/decay/reconcile design is exactly what caused the documented Maze1 bug
history (HANDOFF.md: "purple thickening", a permanently boxed-in robot) --
raytrace-clearing every tick is the principled fix Maze1's own postmortem
deferred, and is correct from the start here because there is no legacy
behaviour to preserve.
"""
from __future__ import annotations

import math
from typing import Any, List, Optional, Tuple, Union

import numpy as np

import config as C
from geometry import transform_points, wrap_angle, wrap_angle_arr


# --------------------------------------------------------------------------- #
#  Lidar range image -> body-frame points
# --------------------------------------------------------------------------- #
class LidarModel:
    """Beam ``i`` points at ``theta_i = fov/2 - i*fov/(N-1)`` (Webots convention).

    The lidar sits on the robot's z-axis, so its x/y frame == the body frame.
    
    Attributes:
        n (int): Number of lidar beams.
        fov (float): Field of view in radians.
        r_min (float): Minimum valid range in meters.
        r_max (float): Maximum valid range in meters.
        angles (np.ndarray): Bearing angle for each beam.
    """

    def __init__(self, n_beams: int, fov: float, r_min: Optional[float] = None, r_max: Optional[float] = None) -> None:
        """Initializes the LidarModel with sensor parameters.

        Args:
            n_beams (int): Number of lidar beams.
            fov (float): Field of view in radians.
            r_min (Optional[float], optional): Minimum valid range. Defaults to `config.LIDAR_RANGE_MIN`.
            r_max (Optional[float], optional): Maximum valid range. Defaults to `config.LIDAR_RANGE_MAX` (capped at 30.0).
        """
        self.n: int = int(n_beams)
        self.fov: float = float(fov)
        self.r_min: float = C.LIDAR_RANGE_MIN if r_min is None else max(r_min, C.LIDAR_RANGE_MIN)
        self.r_max: float = C.LIDAR_RANGE_MAX if r_max is None else min(r_max, 30.0)
        start = self.fov * 0.5
        step = self.fov / max(self.n - 1, 1)
        self.angles: np.ndarray = start - step * np.arange(self.n)
        self._cos: np.ndarray = np.cos(self.angles)
        self._sin: np.ndarray = np.sin(self.angles)

    def ranges_to_body(self, ranges: Union[np.ndarray, List[float]]) -> Tuple[np.ndarray, np.ndarray]:
        """Convert raw ranges to body-frame Cartesian coordinates.

        Args:
            ranges (Union[np.ndarray, List[float]]): Array of range measurements.

        Returns:
            Tuple[np.ndarray, np.ndarray]: A tuple containing:
                - `pts_body` (np.ndarray): Shape (M, 2) array of (x, y) coordinates for valid beams.
                - `ranges_valid` (np.ndarray): Shape (M,) array of valid ranges.
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
    """Performs 8-connected dilation on a boolean mask.

    Args:
        mask (np.ndarray): A 2D boolean numpy array.

    Returns:
        np.ndarray: The dilated mask.
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
    """Computes approximate distance (in cells, Chebyshev) from each cell to the nearest True pixel.

    Args:
        mask (np.ndarray): Boolean mask representing occupied space.
        max_band (int): Maximum distance to compute before capping.

    Returns:
        np.ndarray: A 2D integer array of distances.
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
    """2-D log-odds occupancy map with poison/auxiliary obstacle layers, costmap construction and lidar scan-matching.
    
    Attributes:
        res (float): Map resolution (meters/cell).
        n (int): Map dimension in cells (n x n grid).
        ox (float): X origin in world coordinates.
        oy (float): Y origin in world coordinates.
        L (np.ndarray): Log-odds grid (Float32).
        poison (np.ndarray): Boolean lethal poison mask.
        poison_hits (np.ndarray): Float32 confidence grid for poison.
        aux (np.ndarray): Boolean depth-camera auxiliary obstacle layer.
        aux_hits (np.ndarray): Float32 confidence grid for auxiliary obstacles.
    """
    def __init__(self) -> None:
        """Initializes the empty occupancy grid layers."""
        self.res: float = C.GRID_RESOLUTION
        self.n: int = C.GRID_CELLS
        self.ox: float
        self.oy: float
        self.ox, self.oy = C.GRID_ORIGIN
        self.L: np.ndarray = np.zeros((self.n, self.n), dtype=np.float32)
        self.poison: np.ndarray = np.zeros((self.n, self.n), dtype=bool)
        # per-cell poison confidence (see add_poison_points): a cell is lethal
        # poison only once its confidence reaches C.POISON_MIN_HITS, so transient
        # projection-drift splashes decay away instead of sticking forever.
        self.poison_hits: np.ndarray = np.zeros((self.n, self.n), dtype=np.float32)
        # depth-camera auxiliary obstacle layer (NEW for Maze4) -- see module
        # docstring: boolean only, no counters, cleared by raytrace every tick.
        self.aux: np.ndarray = np.zeros((self.n, self.n), dtype=bool)
        # per-cell depth-obstacle CONFIDENCE (see integrate_depth_obstacles): the
        # boolean `aux` above is just `aux_hits >= C.AUX_MIN_HITS`.  Confidence
        # persists across the depth blind zone so floating walls are not forgotten
        # the instant the robot gets too close to see them.
        self.aux_hits: np.ndarray = np.zeros((self.n, self.n), dtype=np.float32)

    # ----------------------------------------------------- coordinate maths
    def world_to_grid(self, wx: float, wy: float) -> Tuple[int, int]:
        """Convert a world coordinate to grid indices.

        Args:
            wx (float): World X coordinate.
            wy (float): World Y coordinate.

        Returns:
            Tuple[int, int]: (ix, iy) cell indices.
        """
        ix = int((wx - self.ox) / self.res)
        iy = int((wy - self.oy) / self.res)
        return ix, iy

    def world_to_grid_arr(self, wx: Union[float, np.ndarray], wy: Union[float, np.ndarray]) -> Tuple[np.ndarray, np.ndarray]:
        """Convert array of world coordinates to grid indices.

        Args:
            wx (Union[float, np.ndarray]): World X coordinates.
            wy (Union[float, np.ndarray]): World Y coordinates.

        Returns:
            Tuple[np.ndarray, np.ndarray]: (ix, iy) cell indices.
        """
        ix = ((np.asarray(wx) - self.ox) / self.res).astype(np.int32)
        iy = ((np.asarray(wy) - self.oy) / self.res).astype(np.int32)
        return ix, iy

    def grid_to_world(self, ix: int, iy: int) -> Tuple[float, float]:
        """Convert grid indices to world coordinates (cell centre).

        Args:
            ix (int): Grid column index.
            iy (int): Grid row index.

        Returns:
            Tuple[float, float]: (wx, wy) world coordinates in meters.
        """
        return (self.ox + (ix + 0.5) * self.res, self.oy + (iy + 0.5) * self.res)

    def in_bounds(self, ix: int, iy: int) -> bool:
        """Check if grid indices are within the map bounds.

        Args:
            ix (int): Grid column index.
            iy (int): Grid row index.

        Returns:
            bool: True if inside bounds, False otherwise.
        """
        return 0 <= ix < self.n and 0 <= iy < self.n

    def _inb_arr(self, ix: np.ndarray, iy: np.ndarray) -> np.ndarray:
        """Vectorized bounds check."""
        return (ix >= 0) & (ix < self.n) & (iy >= 0) & (iy < self.n)

    def _ray_cells(self, pose: Tuple[float, float, float], bearing_body: float, r0: float, r1: float) -> np.ndarray:
        """Flat cell indices along a body-frame bearing from r0 to r1 (world)."""
        if r1 <= r0:
            return np.empty((0,), dtype=np.int64)
        x, y, th = pose
        ang = th + bearing_body
        steps = max(1, int((r1 - r0) / self.res))
        rs = r0 + (r1 - r0) * (np.arange(steps) / float(steps))
        fx = x + rs * math.cos(ang)
        fy = y + rs * math.sin(ang)
        ixs = ((fx - self.ox) / self.res).astype(np.int32)
        iys = ((fy - self.oy) / self.res).astype(np.int32)
        ok = (ixs >= 0) & (ixs < self.n) & (iys >= 0) & (iys < self.n)
        return (ixs[ok].astype(np.int64) * self.n + iys[ok].astype(np.int64))

    # ----------------------------------------------------------- integration
    def _grazing_mask(self, pts_body: np.ndarray, ranges: np.ndarray) -> np.ndarray:
        """True for beams that strike a surface too obliquely to trust their free ray.

        On a straight surface ``r(theta) = d / cos(theta - theta0)``, so
        ``|dr/dtheta| = r * tan(incidence)`` with incidence measured from the
        surface normal.  Comparing the measured slope against
        ``r * tan(LIDAR_GRAZE_MAX_DEG)`` therefore classifies each beam without
        ever needing the surface itself.

        Each beam is scored by the SMALLER of its two neighbouring slopes: at a
        depth discontinuity (a doorway edge, a pillar rim) one side is huge while
        the other still lies on the continuous surface, and it is that side which
        describes the geometry the beam actually grazed.  Beams with no continuous
        neighbour at all (isolated returns, speckle) score ``inf`` and are gated
        out, which is the conservative choice.

        Endpoint stamping is unaffected -- only the free ray is suppressed.

        Args:
            pts_body (np.ndarray): Shape (M, 2) beam endpoints in the body frame.
            ranges (np.ndarray): Shape (M,) range per beam, index-aligned to pts_body.

        Returns:
            np.ndarray: Shape (M,) boolean mask, True where the beam is grazing.
        """
        m = ranges.shape[0]
        if m < 3:
            return np.zeros(m, dtype=bool)
        bear = np.arctan2(pts_body[:, 1], pts_body[:, 0])
        dth = np.abs(wrap_angle_arr(np.diff(bear)))
        dr = np.abs(np.diff(ranges))
        slope = dr / np.maximum(dth, 1e-9)          # |dr/dtheta| across each gap
        inf = np.array([np.inf])
        s = np.minimum(np.concatenate([inf, slope]), np.concatenate([slope, inf]))
        return s > ranges * math.tan(math.radians(C.LIDAR_GRAZE_MAX_DEG))

    def integrate_scan(self, pose: Tuple[float, float, float], pts_body: np.ndarray, ranges: np.ndarray) -> None:
        """Ray-trace free space and stamp occupied endpoints from one scan.

        Args:
            pose (Tuple[float, float, float]): Current robot pose (x, y, yaw).
            pts_body (np.ndarray): Obstacle hit points in body frame.
            ranges (np.ndarray): Range values for each hit point.
        """
        if pts_body.shape[0] == 0:
            return
        x, y, th = pose
        pts_world = transform_points(pts_body, x, y, th)
        rix, riy = self.world_to_grid(x, y)
        grazing = self._grazing_mask(pts_body, ranges)

        free_lin = []
        occ_lin = []
        n_cells = self.n
        for k in range(pts_world.shape[0]):
            ex, ey = pts_world[k, 0], pts_world[k, 1]
            rng = ranges[k]
            steps = 0 if grazing[k] else int(rng / self.res)
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
        o = np.unique(np.asarray(occ_lin, dtype=np.int64)) if occ_lin else np.empty((0,), dtype=np.int64)
        if free_lin:
            f = np.unique(np.concatenate(free_lin))
            # A beam's last free sample sits ~1 cell short of its endpoint, so a
            # near-diagonal beam routinely free-marks a cell that another beam of
            # the SAME scan stamps occupied.  Occupied evidence wins: without this
            # subtraction a wall cell nets only L_OCC+L_FREE per scan and hovers a
            # single grazing ray above L_OCC_THRESH.
            if o.size:
                f = f[~np.isin(f, o)]
            if f.size:
                np.add.at(Lflat, f, C.L_FREE)
        if o.size:
            np.add.at(Lflat, o, C.L_OCC)
        np.clip(self.L, C.L_MIN, C.L_MAX, out=self.L)

    def integrate_depth_obstacles(self, pose: Tuple[float, float, float], bearings: np.ndarray, hit_ranges: np.ndarray, hit_mask: np.ndarray, clear_mask: np.ndarray, depth_min: float) -> None:
        """Persistent, lidar-gated depth-obstacle layer (the floating-wall fix).

        Replaces the old per-tick raytrace-clear ``integrate_aux``.  Marking is
        SPARSE -- one cell per bearing column at the nearest in-band hit (the
        thin-scan footprint), NOT the dense full-surface projection: dense marking
        plus persistence smeared the map shut in the cluttered tilted-wall cluster
        (aux blew up to ~990 cells and boxed the robot in).  The persistence +
        dead-zone protection below is the actual floating-wall fix; the footprint
        is all that A*/DWA need.  Per frame, in order:

        1. **GLOBAL DECAY** -- every confident cell loses ``C.AUX_GLOBAL_DECAY``.
           This bounds total accumulation: a mark left behind (no longer seen,
           never driven over) fades over ~10-15 s, while a wall the robot keeps in
           view is re-reinforced faster than it decays.  It is small enough that a
           cell confirmed to the cap right before the dead-zone approach stays
           lethal through it.
        2. **SEE-THROUGH DECAY** -- along columns the camera confirms clear, lose
           ``C.AUX_DECAY``.  The decay ray STARTS at ``depth_min``, so a cell
           inside the < depth_min blind shell is NEVER decayed: a floating wall the
           robot has closed in on (and can no longer see) keeps its mark.
        3. **REINFORCE** -- add ``C.AUX_HIT_INC`` at each hit-column endpoint cell,
           but ONLY where the 2-D lidar is blind (no lidar wall within
           ``C.AUX_LIDAR_BLIND_CELLS``) -- the anti-smear gate that keeps normal
           full-height walls (already lidar-mapped) out of the aux layer.
        4. **THRESHOLD** -- ``self.aux = aux_hits >= C.AUX_MIN_HITS`` (capped at
           ``C.AUX_HIT_CAP`` so a confirmed wall rides out stray clear frames).
           
        Args:
            pose (Tuple[float, float, float]): Robot pose (x, y, yaw).
            bearings (np.ndarray): Angles in robot body frame for each depth column.
            hit_ranges (np.ndarray): Detected distances for each column.
            hit_mask (np.ndarray): Boolean mask of which columns hit an obstacle.
            clear_mask (np.ndarray): Boolean mask of clear columns.
            depth_min (float): Minimum range of the depth camera to protect the blind shell.
        """
        hits = self.aux_hits
        hflat = hits.reshape(-1)

        # --- step 1: gentle global decay (bounds long-term accumulation)
        nz = hflat > 0.0
        if nz.any():
            hflat[nz] = np.maximum(0.0, hflat[nz] - C.AUX_GLOBAL_DECAY)

        n_total = bearings.shape[0]
        if n_total == 0:
            self.aux = hits >= C.AUX_MIN_HITS
            return
        if n_total > C.AUX_COL_SUBSAMPLE:
            idx = np.linspace(0, n_total - 1, C.AUX_COL_SUBSAMPLE).astype(int)
        else:
            idx = np.arange(n_total)

        # lidar-blind gate (dilated occupied mask): keep depth marks only where
        # the 2-D lidar genuinely cannot see -- i.e. real floating/low panels.
        blind = self.occupied_mask()
        for _ in range(C.AUX_LIDAR_BLIND_CELLS):
            blind = _dilate8(blind)

        n_cells = self.n
        clear_lin = []
        occ_lin = []
        for i in idx:
            if hit_mask[i]:
                rng = float(hit_ranges[i])
                bearing = float(bearings[i])
                clear_to = max(depth_min, rng - C.AUX_CLEAR_MARGIN)
                clear_lin.append(self._ray_cells(pose, bearing, depth_min, clear_to))
                ang = pose[2] + bearing
                ex = pose[0] + rng * math.cos(ang)
                ey = pose[1] + rng * math.sin(ang)
                eix, eiy = self.world_to_grid(ex, ey)
                if self.in_bounds(eix, eiy) and not blind[eix, eiy]:
                    occ_lin.append(eix * n_cells + eiy)
            elif clear_mask[i]:
                clear_lin.append(self._ray_cells(pose, float(bearings[i]), depth_min, C.AUX_MAX_RANGE))

        # --- step 2: see-through decay (blind shell protected: rays start at depth_min)
        nonempty = [c for c in clear_lin if c.size]
        if nonempty:
            c = np.unique(np.concatenate(nonempty))
            hflat[c] = np.maximum(0.0, hflat[c] - C.AUX_DECAY)

        # --- step 3: reinforce the sparse hit-endpoint cells (deduped per frame)
        if occ_lin:
            o = np.unique(np.asarray(occ_lin, dtype=np.int64))
            hflat[o] = np.minimum(C.AUX_HIT_CAP, hflat[o] + C.AUX_HIT_INC)

        # --- step 4: boolean lethal layer
        self.aux = hits >= C.AUX_MIN_HITS

    def mark_free_disc(self, wx: float, wy: float, radius: float, *, clear_lidar: bool = True) -> None:
        """Force a true disc around a known-free point to a free log-odds.

        ALSO clears the aux layer in that disc: the robot is physically there,
        so that ground truth overrides any stale aux mark.  This directly
        addresses the Maze1 bug where the robot could box itself in with aux
        marks it could never erase by standing in/spinning through them.  Never
        touches the poison layer (poison is a real hazard regardless of the
        robot's current position).

        The clamp is confined to a RADIAL mask.  Writing the whole ``ceil``ed
        bounding box instead (the original form) reached ``r*res*sqrt(2)`` =
        0.177 m for a 0.102 m request -- past the 0.128 m footprint and past the
        planner's 0.12 m lethal band -- so a single call hard-erased saturated
        wall cells the robot was merely driving beside.  That, not the log-odds
        integration, is what made the map fade along the robot's own track.

        Args:
            wx (float): World X coordinate.
            wy (float): World Y coordinate.
            radius (float): Clearing radius in meters.
            clear_lidar (bool): When False, only the aux layer is cleared and the
                lidar log-odds ``L`` is left untouched.  Callers that run every
                control tick must use False: ``L`` is only ever rebuilt by
                ``integrate_scan``, which is gated behind a minimum travel, so a
                per-tick clamp on ``L`` erases far more than it restores.
        """
        cix, ciy = self.world_to_grid(wx, wy)
        r = int(math.ceil(radius / self.res))
        x0, x1 = max(0, cix - r), min(self.n, cix + r + 1)
        y0, y1 = max(0, ciy - r), min(self.n, ciy + r + 1)
        if x0 >= x1 or y0 >= y1:
            return
        gx = np.arange(x0, x1)[:, None] - cix
        gy = np.arange(y0, y1)[None, :] - ciy
        disc = (gx * gx + gy * gy) * (self.res * self.res) <= radius * radius
        if clear_lidar:
            sub = self.L[x0:x1, y0:y1]
            np.minimum(sub, C.L_FREE_THRESH - 0.1, out=sub, where=disc)
        aux_sub = self.aux[x0:x1, y0:y1]
        aux_sub[disc] = False
        hits_sub = self.aux_hits[x0:x1, y0:y1]
        hits_sub[disc] = 0.0   # reset confidence where the robot has driven

    def add_poison_points(self, pts_world: np.ndarray) -> None:
        """Confidence-gated poison stamping (self-correcting against drift).

        Call ONCE per perception frame with the green floor points projected this
        frame (may be empty).  Each step: decay the confidence of every
        not-yet-confirmed cell, then add ``C.POISON_HIT_INC`` to each cell seen
        this frame (once per cell, not per point, so confirmation is per-FRAME not
        per-pixel).  ``self.poison`` is the boolean lethal mask = confidence at or
        above ``C.POISON_MIN_HITS``.  A cell seen on consecutive close frames
        climbs to lethal and then to ``C.POISON_HIT_CAP`` (sticky); a one-off
        projection-drift mark peaks below the threshold and fades within a couple
        of frames, instead of smearing the map as the old write-only bitmap did.
        
        Args:
            pts_world (np.ndarray): Shape (M, 2) array of poison points in the world frame.
        """
        hits = self.poison_hits
        # 1) decay transient (not-yet-confirmed) confidence everywhere
        decayable = (hits > 0.0) & (hits < C.POISON_HIT_CAP)
        if decayable.any():
            hits[decayable] = np.maximum(0.0, hits[decayable] - C.POISON_DECAY)
        # 2) reinforce the cells seen this frame (deduplicated -> +INC per cell)
        if pts_world.shape[0] > 0:
            ix, iy = self.world_to_grid_arr(pts_world[:, 0], pts_world[:, 1])
            ok = self._inb_arr(ix, iy)
            if ok.any():
                flat = np.unique(ix[ok].astype(np.int64) * self.n + iy[ok].astype(np.int64))
                ux, uy = flat // self.n, flat % self.n
                hits[ux, uy] = np.minimum(C.POISON_HIT_CAP, hits[ux, uy] + C.POISON_HIT_INC)
        # 3) recompute the lethal mask
        self.poison = hits >= C.POISON_MIN_HITS

    # --------------------------------------------------------------- masks
    def occupied_mask(self) -> np.ndarray:
        """Returns boolean mask of lidar-occupied cells."""
        return self.L > C.L_OCC_THRESH

    def free_mask(self) -> np.ndarray:
        """Returns boolean mask of known-free cells."""
        return self.L < C.L_FREE_THRESH

    def unknown_mask(self) -> np.ndarray:
        """Returns boolean mask of unobserved cells."""
        return (~self.occupied_mask()) & (~self.free_mask())

    # ------------------------------------------------- mapped obstacles query
    def poison_points_near(self, wx: float, wy: float, radius: float) -> np.ndarray:
        """World (M,2) of poison-cell centres within ``radius`` of (wx,wy).

        Args:
            wx (float): Search centre X in world coords.
            wy (float): Search centre Y in world coords.
            radius (float): Search radius in meters.

        Returns:
            np.ndarray: Array of (x, y) coordinates of poison cells.
        """
        return self._mask_points_near(self.poison, wx, wy, radius)

    def aux_points_near(self, wx: float, wy: float, radius: float) -> np.ndarray:
        """World (M,2) of aux-occupied cell centres within ``radius`` of (wx,wy).
        
        Args:
            wx (float): Search centre X in world coords.
            wy (float): Search centre Y in world coords.
            radius (float): Search radius in meters.

        Returns:
            np.ndarray: Array of (x, y) coordinates of auxiliary obstacle cells.
        """
        return self._mask_points_near(self.aux, wx, wy, radius)

    def _mask_points_near(self, mask: np.ndarray, wx: float, wy: float, radius: float) -> np.ndarray:
        """Extracts world coordinates of mask true pixels within radius."""
        if not mask.any():
            return np.empty((0, 2))
        cix, ciy = self.world_to_grid(wx, wy)
        r = int(math.ceil(radius / self.res))
        x0, x1 = max(0, cix - r), min(self.n, cix + r + 1)
        y0, y1 = max(0, ciy - r), min(self.n, ciy + r + 1)
        sub = mask[x0:x1, y0:y1]
        lx, ly = np.nonzero(sub)
        if lx.size == 0:
            return np.empty((0, 2))
        wx_ = self.ox + (x0 + lx + 0.5) * self.res
        wy_ = self.oy + (y0 + ly + 0.5) * self.res
        return np.stack([wx_, wy_], axis=1)

    def is_poison_world(self, wx: float, wy: float) -> bool:
        """Check if a world coordinate lands in a poison cell.
        
        Args:
            wx (float): World X coordinate.
            wy (float): World Y coordinate.

        Returns:
            bool: True if poison, False otherwise.
        """
        ix, iy = self.world_to_grid(wx, wy)
        return self.in_bounds(ix, iy) and bool(self.poison[ix, iy])

    # ------------------------------------------------------------ scan match
    def _score_field(self) -> np.ndarray:
        """Float field for scan matching: occupied=1, 1-ring=0.6, 2-ring=0.3.

        Deliberately lidar-only (does NOT include aux): the depth-aux layer is
        noisier and only used for collision avoidance / costmap, never for
        localisation, so it cannot corrupt the scan-matched pose.
        
        Returns:
            np.ndarray: Score field grid.
        """
        occ = self.occupied_mask()
        field = np.zeros((self.n, self.n), dtype=np.float32)
        field[occ] = 1.0
        d1 = _dilate8(occ) & ~occ
        field[d1] = 0.6
        d2 = _dilate8(_dilate8(occ)) & ~_dilate8(occ)
        field[d2] = 0.3
        return field

    def scan_match(self, pred_pose: Tuple[float, float, float], pts_body: np.ndarray) -> Tuple[Tuple[float, float, float], float]:
        """Correlative (x,y,yaw) correction of ``pred_pose`` against the map.

        Returns ``(corrected_pose, hit_fraction)``.  Falls back to ``pred_pose``
        when too few beams land on mapped walls (avoids corrupting a good odom).

        Args:
            pred_pose (Tuple[float, float, float]): Dead-reckoning predicted pose.
            pts_body (np.ndarray): Current lidar hit points in body frame.

        Returns:
            Tuple[Tuple[float, float, float], float]: The corrected pose and matching fraction [0.0, 1.0].
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
                        best = (sc, dx, dy, dth)  # type: ignore
        score, dx, dy, dth = best
        # denominator must be the SCORED beam count, not the full cloud: `score`
        # sums over `pts` (capped at SM_MAX_BEAMS), so dividing by `m` caps
        # hit_frac at SM_MAX_BEAMS/m and silently rejects good matches whenever
        # the lidar has more beams than SM_MAX_BEAMS.
        hit_frac = score / max(pts.shape[0], 1)
        if hit_frac < C.SM_MIN_HIT_FRAC:
            return pred_pose, hit_frac
        return (px + dx, py + dy, wrap_angle(pth + dth)), hit_frac

    # -------------------------------------------------------------- costmap
    def build_costmap(self) -> Tuple[np.ndarray, np.ndarray]:
        """Return ``(cost float32, lethal bool)`` over the whole grid.

        ``cost`` is additive A* penalty per cell (0 in open free space).  Unknown
        cells are traversable (optimistic) so frontier goals stay reachable; the
        live-lidar DWA layer guarantees real wall clearance regardless.  Aux
        (depth-derived low-obstacle) cells are folded into the SAME hard/soft
        obstacle bands as lidar-occupied cells -- both represent "a wall", aux
        is just the one the lidar plane missed.
        
        Returns:
            Tuple[np.ndarray, np.ndarray]: The cost field and lethal mask.
        """
        occ = self.occupied_mask() | self.aux
        res = self.res

        hard_b = max(1, int(round(C.HARD_OBS_DIST / res)))
        pref_b = max(hard_b + 1, int(round(C.CENTER_PREF_RANGE / res)))
        d_obs = _distance_bands(occ, pref_b + 1)

        lethal = d_obs <= hard_b
        cost = np.zeros((self.n, self.n), dtype=np.float32)
        # center-seeking gradient: cost falls off with clearance out to pref_b so
        # A* rides the medial axis (maximum clearance) of every corridor rather
        # than hugging the first non-lethal cell.  This is what aligns the path
        # to the absolute centre of the tight 0.39 m gaps.  (Replaces the old
        # fixed-width soft halo, which went flat in the middle of a narrow gap
        # and so gave A* no reason to prefer dead-centre.)
        grad_zone = (d_obs > hard_b) & (d_obs <= pref_b)
        frac = (pref_b - d_obs.astype(np.float32)) / max(1, (pref_b - hard_b))
        cost[grad_zone] += C.COST_OBS_WEIGHT * frac[grad_zone]

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
