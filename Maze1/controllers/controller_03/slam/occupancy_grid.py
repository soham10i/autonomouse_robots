"""Phase B — clean log-odds occupancy grid.

Design goals (these directly fix the failures of the old controller):

* **Thin walls.**  Free space is cleared by ray-sampling every traversed cell,
  and each cell is updated *at most once per scan* (de-duplicated) so beams that
  overlap don't over-count.  The free/occupied increments are close in
  magnitude, so a single spurious hit is erased after a couple of clear passes
  instead of becoming permanent.

* **No purple blobs.**  The depth-camera auxiliary layer is *hit-count gated*
  (a cell must be seen ``AUX_MIN_HITS`` times before it is treated as an
  obstacle) and *decayed* (cleared wherever the lidar later proves the cell is
  free).  Together these stop the floating-wall layer from smearing into solid
  regions.

Index convention: ``L[ix, iy]`` — axis 0 is x (column), axis 1 is y (row).
"""
from __future__ import annotations

import math

import numpy as np

import settings as S


class OccupancyGrid:
    def __init__(self):
        self.res = S.GRID_RESOLUTION
        self.cells = S.GRID_CELLS
        self.origin = np.asarray(S.GRID_ORIGIN, dtype=np.float64)

        shape = (self.cells, self.cells)
        self.L = np.zeros(shape, dtype=np.float32)          # lidar log-odds
        # Depth obstacle layer (floating walls the lidar is blind to): a SEPARATE
        # log-odds grid, marked AND raytrace-cleared by the depth camera only, and
        # NEVER touched by the lidar (which reports a floating wall's cell as free
        # because it passes underneath).  This is the ROS costmap_2d obstacle-layer
        # / depthimage_to_laserscan pattern — the proper fix that replaces the old
        # sticky-aux / reconcile / decay / dead-end-barrier hacks.
        self.depth_L = np.zeros(shape, dtype=np.float32)
        self.aux_count = np.zeros(shape, dtype=np.int16)    # (legacy, unused)
        self.poison = np.zeros(shape, dtype=bool)           # lethal, never cleared
        # Learned floating-wall barriers (e.g. WallMedium(3) over the blue
        # pillar): a confirmed obstacle the robot proved it cannot pass — lethal
        # for planning, and NEVER cleared by any aux logic (unlike aux, which
        # the footprint/escape clears wipe).  This is what stops A* from
        # re-routing under a floating wall the dead-end detector already learned.
        self.barrier = np.zeros(shape, dtype=bool)
        # Accurate floating-wall footprint projected down from the 3D voxel cloud
        # (mapping.cloud_map.CloudMap.project_to_2d).  Unlike the one-view thin
        # depth scan, this is the accumulated wall surface collapsed to its true
        # cell outline — it is unioned into the lethal/costmap layers so the
        # planner avoids tumbled/tilted floating walls at their real location.
        self.cloud_obs = np.zeros(shape, dtype=bool)
        # The cloud projection accumulates without clearing, so it is kept OUT of
        # the planner by default (render-only) — it ran away and trapped the
        # robot.  Flip via S.CLOUD_OBS_TO_PLANNER once it has a clearing scheme.
        self.use_cloud_obs = bool(S.CLOUD_OBS_TO_PLANNER)

        self._dirty = True

    # ------------------------------------------------------- cloud obstacle
    def set_cloud_obs(self, mask):
        """Replace the cloud-projected floating-wall mask (from CloudMap).

        Cells the robot's own footprint clears (``mark_free_disc``) are kept
        clear so a stale projection where the robot is standing cannot box it in.
        """
        mask = np.asarray(mask, dtype=bool)
        if mask.shape == self.cloud_obs.shape:
            self.cloud_obs = mask
            self._dirty = True

    # --------------------------------------------------------------- coords
    def world_to_grid(self, wx, wy):
        """Scalar world -> (ix, iy) cell indices (may be out of bounds)."""
        ix = int(math.floor((wx - self.origin[0]) / self.res))
        iy = int(math.floor((wy - self.origin[1]) / self.res))
        return ix, iy

    def grid_to_world(self, ix, iy):
        """Cell centre in world coordinates."""
        wx = self.origin[0] + (ix + 0.5) * self.res
        wy = self.origin[1] + (iy + 0.5) * self.res
        return wx, wy

    def world_to_grid_arr(self, wx, wy):
        """Vectorised world -> indices with an in-bounds mask."""
        ix = np.floor((np.asarray(wx) - self.origin[0]) / self.res).astype(np.int32)
        iy = np.floor((np.asarray(wy) - self.origin[1]) / self.res).astype(np.int32)
        m = (ix >= 0) & (ix < self.cells) & (iy >= 0) & (iy < self.cells)
        return ix, iy, m

    def in_bounds(self, ix, iy):
        return 0 <= ix < self.cells and 0 <= iy < self.cells

    # ---------------------------------------------------------- integration
    def integrate_scan(self, pose, hits_world):
        """Fold one lidar scan into the grid.

        ``hits_world`` is an ``(M, 2)`` array of world-frame endpoints for the
        beams that returned a finite, in-range hit.  Free space is carved along
        each ray; the endpoint cell is marked occupied.
        """
        hits_world = np.asarray(hits_world, dtype=np.float64)
        if hits_world.shape[0] == 0:
            return
        rx, ry, _ = pose

        # --- occupied endpoints (one update per cell this scan) ---
        ex, ey, em = self.world_to_grid_arr(hits_world[:, 0], hits_world[:, 1])
        ex, ey = ex[em], ey[em]
        endpoints = hits_world[em]
        if ex.size == 0:
            return

        # --- free cells along every ray, vectorised sampling ---
        free_ix, free_iy = self._free_cells_along_rays(rx, ry, endpoints)
        self._apply_updates(free_ix, free_iy, ex, ey)

    def _free_cells_along_rays(self, rx, ry, endpoints, min_dist=0.0):
        """Cells traversed (free) by straight rays from (rx, ry) to each endpoint,
        stopping one cell short of the endpoint.  Cells closer than ``min_dist``
        are skipped — used by the depth integrator so it never clears inside the
        camera's blind zone (where it cannot actually observe free space)."""
        endpoints = np.asarray(endpoints, dtype=np.float64).reshape(-1, 2)
        if endpoints.shape[0] == 0:
            return np.empty(0, np.int32), np.empty(0, np.int32)
        dx = endpoints[:, 0] - rx
        dy = endpoints[:, 1] - ry
        dist = np.hypot(dx, dy)
        good = dist > self.res
        dx, dy, dist = dx[good], dy[good], dist[good]
        if dist.size == 0:
            return np.empty(0, np.int32), np.empty(0, np.int32)
        ux = dx / dist
        uy = dy / dist
        step = self.res
        n_max = int(math.ceil(float(dist.max()) / step))
        t_grid = (np.arange(1, n_max + 1) * step).astype(np.float64)
        sx = rx + ux[:, None] * t_grid[None, :]
        sy = ry + uy[:, None] * t_grid[None, :]
        valid = (t_grid[None, :] < (dist[:, None] - self.res)) & (t_grid[None, :] >= min_dist)
        fix, fiy, fm = self.world_to_grid_arr(sx[valid], sy[valid])
        return fix[fm], fiy[fm]

    def _apply_updates(self, free_ix, free_iy, occ_ix, occ_iy):
        """De-duplicate then apply L_FREE to free cells and L_OCC to endpoints.

        Free is applied first so that a cell which is both traversed and hit in
        the same scan ends up occupied (the endpoint wins).
        """
        L = self.L
        if free_ix.size:
            lin = free_ix.astype(np.int64) * self.cells + free_iy
            lin = np.unique(lin)
            fix = (lin // self.cells).astype(np.int32)
            fiy = (lin % self.cells).astype(np.int32)
            np.add.at(L, (fix, fiy), S.L_FREE)
        if occ_ix.size:
            lin = occ_ix.astype(np.int64) * self.cells + occ_iy
            lin = np.unique(lin)
            oix = (lin // self.cells).astype(np.int32)
            oiy = (lin % self.cells).astype(np.int32)
            np.add.at(L, (oix, oiy), S.L_OCC)
        np.clip(L, S.L_MIN, S.L_MAX, out=L)
        self._dirty = True

    # -------------------------------------------------- depth obstacle layer
    def integrate_depth_rays(self, pose, hits_world, free_ends_world):
        """Fold one depth-derived scan into the SEPARATE depth obstacle layer.

        ``hits_world``      : (M, 2) world endpoints where the depth cam sees an
                              in-band (collision-height) obstacle -> marked occupied
                              and free-cleared along the ray up to it.
        ``free_ends_world`` : (K, 2) world endpoints for bearings with NO in-band
                              obstacle -> the robot-height tunnel is clear, so the
                              ray is free-cleared (this is the raytrace CLEARING
                              that makes the layer self-correct — a floating wall
                              that moves out of view, or a false hit, gets erased).

        The lidar never writes here, so a real floating wall the lidar marks free
        stays marked occupied in this layer.

        Two corrections that matter for low/floating walls:
        * **Mark only where the lidar is NOT already a wall** — this layer is for
          obstacles the lidar MISSES (floating/low walls); copying every normal
          wall here just smears the map and over-inflates passages.
        * **Never clear inside the depth blind zone** (< ``DEPTH_OBS_MIN_RANGE``):
          as the robot closes on a low slab the depth sees PAST it and would
          otherwise erase the mark right before colliding.
        """
        rx, ry, _ = pose
        hits_world = np.asarray(hits_world, dtype=np.float64).reshape(-1, 2)
        free_ends_world = np.asarray(free_ends_world, dtype=np.float64).reshape(-1, 2)

        ex, ey, em = self.world_to_grid_arr(hits_world[:, 0], hits_world[:, 1])
        ex, ey = ex[em], ey[em]
        # keep only hits the lidar does NOT already see as a wall (floating/low)
        if ex.size:
            occ = self.occupied_mask()
            n = max(1, S.DEPTH_MARK_NEAR_WALL_CELLS)
            try:
                from scipy.ndimage import binary_dilation
                yy, xx = np.ogrid[-n:n + 1, -n:n + 1]
                near_wall = binary_dilation(occ, structure=(xx * xx + yy * yy) <= n * n)
            except Exception:
                near_wall = occ
            keep = ~near_wall[ex, ey]
            ex, ey = ex[keep], ey[keep]

        # free cells from BOTH hit rays and clear-only rays, but NOT inside the
        # depth blind zone (cells closer than the sensor min range are unobserved)
        all_ends = np.vstack([hits_world, free_ends_world]) if free_ends_world.size \
            else hits_world
        fix, fiy = self._free_cells_along_rays(rx, ry, all_ends,
                                               min_dist=S.DEPTH_OBS_MIN_RANGE)

        L = self.depth_L
        if fix.size:
            lin = np.unique(fix.astype(np.int64) * self.cells + fiy)
            np.add.at(L, ((lin // self.cells).astype(np.int32),
                          (lin % self.cells).astype(np.int32)), S.DEPTH_L_FREE)
        if ex.size:
            lin = np.unique(ex.astype(np.int64) * self.cells + ey)
            np.add.at(L, ((lin // self.cells).astype(np.int32),
                          (lin % self.cells).astype(np.int32)), S.DEPTH_L_OCC)
        np.clip(L, S.DEPTH_L_MIN, S.DEPTH_L_MAX, out=L)
        self._dirty = True

    def depth_obs_mask(self):
        return self.depth_L >= S.DEPTH_OCC_THRESH

    # ----------------------------------------------------------- aux layer
    def mark_aux_points(self, pts_xy):
        """Increment the depth-cam hit count at ``pts_xy`` (M, 2) world points."""
        pts_xy = np.asarray(pts_xy, dtype=np.float64)
        if pts_xy.shape[0] == 0:
            return
        ix, iy, m = self.world_to_grid_arr(pts_xy[:, 0], pts_xy[:, 1])
        ix, iy = ix[m], iy[m]
        if ix.size == 0:
            return
        # Only mark cells the lidar does NOT already see as occupied.  The aux
        # layer exists for FLOATING walls the lidar misses; marking normal walls
        # (already lidar-occupied) just over-paints and narrows passages — this
        # was the runaway purple smear shrinking the corridors.
        keep = self.L[ix, iy] < S.L_OCC_THRESH
        ix, iy = ix[keep], iy[keep]
        if ix.size:
            np.add.at(self.aux_count, (ix, iy), 1)
            np.clip(self.aux_count, 0, S.AUX_CAP, out=self.aux_count)
            self._dirty = True

    def mark_aux_disc(self, wx, wy, radius, sticky=True):
        """Stamp a STICKY aux-obstacle disc (used by the virtual bumper).

        Floating walls are invisible to the lidar, so the bumper marks them here
        at a hit count above the sticky threshold so decay never erases them.
        """
        count = S.AUX_STICKY_HITS + 1 if sticky else S.AUX_MIN_HITS
        ix_c, iy_c = self.world_to_grid(wx, wy)
        n = max(1, int(math.ceil(radius / self.res)))
        r2 = (radius / self.res) ** 2
        for dx in range(-n, n + 1):
            for dy in range(-n, n + 1):
                if dx * dx + dy * dy > r2:
                    continue
                ix, iy = ix_c + dx, iy_c + dy
                if self.in_bounds(ix, iy):
                    self.aux_count[ix, iy] = max(int(self.aux_count[ix, iy]), count)
        self._dirty = True

    def reconcile_aux_with_walls(self, radius=None):
        """Clear aux marks that merely SHADOW a real (lidar-seen) wall.

        This is the "clearing observation" the aux layer was missing (ROS
        ``costmap_2d`` marks AND clears; we only marked).  A genuine floating
        wall is invisible to the lidar — there is no lidar-occupied cell near
        it.  But for ordinary full-height walls the depth camera ALSO sees the
        chassis-height band, and pose drift / depth quantisation spray that aux
        onto the free cells just in FRONT of the wall.  Once those cross
        ``AUX_STICKY_HITS`` they are never decayed (decay skips sticky cells),
        so they permanently bulge the inflated corner and squeeze the planned
        centre-line into the opposite wall — the wide-purple-corner that wedged
        the robot.

        Since the lidar already represents that wall thinly and accurately, any
        aux within ``radius`` of a lidar-occupied cell is redundant and is
        cleared (even sticky ones).  Isolated aux — true floating walls and
        bumper stamps, which have NO lidar-occupied neighbour — is preserved.
        Returns the number of cells cleared.
        """
        if not self.aux_count.any():
            return 0
        occ = self.occupied_mask()
        if not occ.any():
            return 0
        r = S.AUX_WALL_CLEAR_RADIUS if radius is None else radius
        n = max(1, int(math.ceil(r / self.res)))
        try:
            from scipy.ndimage import binary_dilation
            y, x = np.ogrid[-n:n + 1, -n:n + 1]
            struct = (x * x + y * y) <= n * n
            near_wall = binary_dilation(occ, structure=struct)
        except Exception:
            near_wall = occ
        clear = (self.aux_count > 0) & near_wall
        c = int(clear.sum())
        if c:
            self.aux_count[clear] = 0
            self._dirty = True
        return c

    def decay_aux_where_free(self):
        """Clear *low-confidence* aux marks where the lidar proves the cell free.

        This removes depth-noise ghost walls behind obstacles the robot passed,
        but it MUST NOT erase confirmed floating walls — the lidar always reports
        a floating wall's cell as free, so anything at/above ``AUX_STICKY_HITS``
        (well-seen by depth, or bumper-stamped) is preserved.  Returns the count
        cleared.
        """
        cleared = ((self.aux_count > 0)
                   & (self.aux_count < S.AUX_STICKY_HITS)
                   & self.free_mask())
        n = int(cleared.sum())
        if n:
            self.aux_count[cleared] = 0
            self._dirty = True
        return n

    # ------------------------------------------------------------- poison
    def mark_poison_points(self, wx, wy):
        """Mark each world point's cell as poison (lethal, never cleared)."""
        ix, iy, m = self.world_to_grid_arr(wx, wy)
        ix, iy = ix[m], iy[m]
        if ix.size == 0:
            return 0
        before = int(self.poison.sum())
        self.poison[ix, iy] = True
        self._dirty = True
        return int(self.poison.sum()) - before

    def mark_barrier_disc(self, wx, wy, radius):
        """Stamp a PERMANENT learned-obstacle disc (a floating wall the robot
        proved it cannot pass).  Lethal for planning, never cleared.  Poison is
        left untouched."""
        ix_c, iy_c = self.world_to_grid(wx, wy)
        n = max(1, int(math.ceil(radius / self.res)))
        r2 = (radius / self.res) ** 2
        for dx in range(-n, n + 1):
            for dy in range(-n, n + 1):
                if dx * dx + dy * dy > r2:
                    continue
                ix, iy = ix_c + dx, iy_c + dy
                if self.in_bounds(ix, iy):
                    self.barrier[ix, iy] = True
        self._dirty = True

    def is_poison_world(self, wx, wy):
        ix, iy = self.world_to_grid(wx, wy)
        if not self.in_bounds(ix, iy):
            return False
        return bool(self.poison[ix, iy])

    def mark_free_disc(self, wx, wy, radius):
        """Force a world disc to free (keeps the robot's own cell plannable).

        This is called at the robot's OWN pose, so the disc is provably free —
        the robot is physically standing there.  We therefore clear ALL aux in
        it, *including sticky* marks.  The old code preserved sticky aux here,
        which meant a robot boxed in by FALSE sticky aux could never erode the
        box and span forever in recovery — exactly the permanent loop seen in
        the runs.  The virtual bumper stamps real floating walls *ahead* of the
        robot (``BUMPER_MARK_AHEAD`` > this radius), so a wall it actually hit is
        not erased by its own footprint clear.  Poison is preserved — it is
        mission failure and must never be erased.
        """
        ix_c, iy_c = self.world_to_grid(wx, wy)
        n = max(1, int(math.ceil(radius / self.res)))
        r2 = (radius / self.res) ** 2
        for dx in range(-n, n + 1):
            for dy in range(-n, n + 1):
                if dx * dx + dy * dy > r2:
                    continue
                ix, iy = ix_c + dx, iy_c + dy
                if not self.in_bounds(ix, iy) or self.poison[ix, iy]:
                    continue
                self.L[ix, iy] = S.L_MIN
                self.aux_count[ix, iy] = 0
                # A cell the robot is physically standing in cannot hold a
                # floating wall — wipe any (stale/displaced) depth or cloud mark
                # so a mis-projected slab can never box the robot in on its pose.
                self.depth_L[ix, iy] = S.DEPTH_L_MIN
                self.cloud_obs[ix, iy] = False
        self._dirty = True

    def clear_aux_disc(self, wx, wy, radius):
        """Clear ALL aux (incl. sticky) in a world disc; poison untouched.

        Escape hatch for a robot boxed in by false aux after repeated
        recoveries: it has been spinning freely in place, so the immediate area
        is navigable and the surrounding aux is almost certainly spurious.
        Returns the number of cells cleared.
        """
        ix_c, iy_c = self.world_to_grid(wx, wy)
        n = max(1, int(math.ceil(radius / self.res)))
        r2 = (radius / self.res) ** 2
        cleared = 0
        for dx in range(-n, n + 1):
            for dy in range(-n, n + 1):
                if dx * dx + dy * dy > r2:
                    continue
                ix, iy = ix_c + dx, iy_c + dy
                if self.in_bounds(ix, iy) and self.aux_count[ix, iy] > 0:
                    self.aux_count[ix, iy] = 0
                    cleared += 1
        if cleared:
            self._dirty = True
        return cleared

    # -------------------------------------------------------------- masks
    def occupied_mask(self):
        return self.L >= S.L_OCC_THRESH

    def free_mask(self):
        return self.L <= S.L_FREE_THRESH

    def unknown_mask(self):
        return ~self.occupied_mask() & ~self.free_mask()

    def aux_mask(self):
        return self.aux_count >= S.AUX_MIN_HITS

    def lethal_mask(self):
        m = (self.occupied_mask() | self.depth_obs_mask()
             | self.poison | self.barrier)
        if getattr(self, "use_cloud_obs", False):
            m = m | self.cloud_obs
        return m

    def is_lethal_world(self, wx, wy):
        """Cheap point lethal test (wall / floating-wall / poison / barrier).

        Off-map counts as lethal (never reverse off the known map).  Used by the
        recovery reverse so the robot never backs into poison or a wall."""
        ix, iy = self.world_to_grid(wx, wy)
        if not self.in_bounds(ix, iy):
            return True
        return bool(self.L[ix, iy] >= S.L_OCC_THRESH
                    or self.depth_L[ix, iy] >= S.DEPTH_OCC_THRESH
                    or self.poison[ix, iy] or self.barrier[ix, iy])

    def segment_clear(self, x0, y0, x1, y1, stop_short=0.0):
        """True if the straight world segment (x0,y0)->(x1,y1) crosses NO lethal
        cell, ignoring the last ``stop_short`` metres before the endpoint.

        Used to confirm a pillar only when the robot has a CLEAR GROUND PATH to it
        — otherwise the depth camera, looking UNDER a floating wall, "sees" the
        pillar and the robot wrongly declares it reached while a wall it cannot
        drive through sits between them.  ``stop_short`` excludes the pillar's own
        (occupied) cell so the target itself doesn't count as a blocker."""
        dx, dy = x1 - x0, y1 - y0
        dist = math.hypot(dx, dy)
        reach = max(0.0, dist - stop_short)
        if reach <= self.res:
            return True
        ux, uy = dx / dist, dy / dist
        lethal = self.lethal_mask()
        t = self.res
        while t <= reach:
            ix, iy = self.world_to_grid(x0 + ux * t, y0 + uy * t)
            if self.in_bounds(ix, iy) and lethal[ix, iy]:
                return False
            t += self.res
        return True

    def nearest_lethal_dist(self, wx, wy, max_r=0.5):
        """Distance (m) from a world point to the nearest lethal cell within
        ``max_r`` (``inf`` if none).  Windowed so it is cheap to call per tick.

        The recovery uses this to decide whether there is room to ROTATE in place
        without a corner scrubbing a wall — the trapping walls are frequently
        inside the 2-D lidar's 0.2 m blind zone, so the live scan can't measure
        this but the map can."""
        lethal = self.lethal_mask()
        ic, jc = self.world_to_grid(wx, wy)
        n = max(1, int(math.ceil(max_r / self.res)))
        i0, i1 = max(0, ic - n), min(self.cells, ic + n + 1)
        j0, j1 = max(0, jc - n), min(self.cells, jc + n + 1)
        sub = lethal[i0:i1, j0:j1]
        if not sub.any():
            return float("inf")
        ii, jj = np.where(sub)
        cx = self.origin[0] + (i0 + ii + 0.5) * self.res
        cy = self.origin[1] + (j0 + jj + 0.5) * self.res
        return float(np.min(np.hypot(cx - wx, cy - wy)))

    def occupancy_prob(self):
        return 1.0 - 1.0 / (1.0 + np.exp(self.L))

    def inflated_lethal(self, radius):
        lethal = self.lethal_mask()
        n = max(1, int(math.ceil(radius / self.res)))
        try:
            from scipy.ndimage import binary_dilation
            y, x = np.ogrid[-n:n + 1, -n:n + 1]
            struct = (x * x + y * y) <= n * n
            return binary_dilation(lethal, structure=struct)
        except Exception:
            return lethal

    # ------------------------------------------------------------- costmap
    def costmap(self):
        """Return ``(cost, hard_lethal)`` for A*.

        ``cost`` is finite soft cost in a halo around obstacles/poison and
        ``inf`` inside the hard-lethal radius; ``hard_lethal`` is the bool mask
        of cells the planner must never enter.
        """
        try:
            from scipy.ndimage import distance_transform_edt
            obs = self.occupied_mask() | self.depth_obs_mask() | self.barrier
            if getattr(self, "use_cloud_obs", False):
                obs = obs | self.cloud_obs
            dist_obs = distance_transform_edt(~obs) * self.res
            dist_poi = distance_transform_edt(~self.poison) * self.res
        except Exception:
            obs = self.occupied_mask() | self.depth_obs_mask() | self.barrier
            if getattr(self, "use_cloud_obs", False):
                obs = obs | self.cloud_obs
            dist_obs = np.where(obs, 0.0, 10.0).astype(np.float32)
            dist_poi = np.where(self.poison, 0.0, 10.0).astype(np.float32)

        hard_obs = max(S.ROBOT_RADIUS - S.HARD_OBS_MARGIN, 0.04)
        soft_obs = hard_obs + S.SOFT_OBS_HALO
        hard_poi = max(S.ROBOT_RADIUS - S.HARD_POISON_MARGIN, hard_obs)
        soft_poi = hard_poi + S.SOFT_POISON_HALO

        cost = np.zeros_like(dist_obs, dtype=np.float32)

        band = (dist_obs >= hard_obs) & (dist_obs < soft_obs)
        if band.any():
            t = (dist_obs[band] - hard_obs) / max(soft_obs - hard_obs, 1e-6)
            cost[band] += S.COST_OBS_WEIGHT * np.exp(-S.INFLATION_ALPHA * t)

        band = (dist_poi >= hard_poi) & (dist_poi < soft_poi)
        if band.any():
            t = (dist_poi[band] - hard_poi) / max(soft_poi - hard_poi, 1e-6)
            cost[band] += S.COST_POISON_WEIGHT * np.exp(-S.INFLATION_ALPHA * t)

        hard_lethal = (dist_obs < hard_obs) | (dist_poi < hard_poi)
        cost[hard_lethal] = np.inf
        return cost, hard_lethal
