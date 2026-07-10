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

import numpy as np

import config as C
from geometry import transform_points, wrap_angle


# --------------------------------------------------------------------------- #
#  Lidar range image -> body-frame points
# --------------------------------------------------------------------------- #
class LidarModel:
    """Beam ``i`` points at ``theta_i = fov/2 - i*fov/(N-1)`` (Webots convention).

    The lidar sits on the robot's z-axis, so its x/y frame == the body frame.
    """

    def __init__(self, n_beams, fov, r_min=None, r_max=None):
        self.n = int(n_beams)
        self.fov = float(fov)
        self.r_min = C.LIDAR_RANGE_MIN if r_min is None else max(r_min, C.LIDAR_RANGE_MIN)
        self.r_max = C.LIDAR_RANGE_MAX if r_max is None else min(r_max, 30.0)
        start = self.fov * 0.5
        step = self.fov / max(self.n - 1, 1)
        self.angles = start - step * np.arange(self.n)
        self._cos = np.cos(self.angles)
        self._sin = np.sin(self.angles)

    def ranges_to_body(self, ranges):
        """Return ``(pts_body (M,2), ranges_valid (M,))`` for in-range beams."""
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
def _dilate8(mask):
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


def _distance_bands(mask, max_band):
    """Approx. distance (in cells, Chebyshev) from each cell to nearest True."""
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
    """2-D log-odds occupancy map with poison/auxiliary obstacle layers, costmap construction and lidar scan-matching."""
    def __init__(self):
        self.res = C.GRID_RESOLUTION
        self.n = C.GRID_CELLS
        self.ox, self.oy = C.GRID_ORIGIN
        self.L = np.zeros((self.n, self.n), dtype=np.float32)
        self.poison = np.zeros((self.n, self.n), dtype=bool)
        # per-cell poison confidence (see add_poison_points): a cell is lethal
        # poison only once its confidence reaches C.POISON_MIN_HITS, so transient
        # projection-drift splashes decay away instead of sticking forever.
        self.poison_hits = np.zeros((self.n, self.n), dtype=np.float32)
        # depth-camera auxiliary obstacle layer (NEW for Maze4) -- see module
        # docstring: boolean only, no counters, cleared by raytrace every tick.
        self.aux = np.zeros((self.n, self.n), dtype=bool)
        # per-cell depth-obstacle CONFIDENCE (see integrate_depth_obstacles): the
        # boolean `aux` above is just `aux_hits >= C.AUX_MIN_HITS`.  Confidence
        # persists across the depth blind zone so floating walls are not forgotten
        # the instant the robot gets too close to see them.
        self.aux_hits = np.zeros((self.n, self.n), dtype=np.float32)
        # CONFIRMED-occupied protection (the "keep accurate close points" fix):
        # a cell the lidar has seen as a wall from CLOSE, accurate range (and on
        # enough scans to be solid) is latched here so a free-space ray that merely
        # GRAZES past it to a farther endpoint cannot carve it out -- the thin-wall
        # erosion that emptied the map.  But the latch is SELF-CLEANING (not
        # permanent): a cell the lidar repeatedly sees THROUGH is not a real wall
        # (drift smear / stale latch) and is released after OCC_CONFIRM_RELEASE free
        # passes; occ_free_strikes counts those passes and is reset whenever the cell
        # is re-seen as occupied.  A real wall is never crossed by a free ray, so it
        # never accrues strikes and stays latched.  Also cleared where the robot
        # physically drives (mark_free_disc).  Permanence (the first version) made
        # pivot drift-smear accumulate forever and caged the robot.
        self.occ_confirmed = np.zeros((self.n, self.n), dtype=bool)
        self.occ_free_strikes = np.zeros((self.n, self.n), dtype=np.float32)

    # ----------------------------------------------------- coordinate maths
    def world_to_grid(self, wx, wy):
        ix = int((wx - self.ox) / self.res)
        iy = int((wy - self.oy) / self.res)
        return ix, iy

    def world_to_grid_arr(self, wx, wy):
        ix = ((np.asarray(wx) - self.ox) / self.res).astype(np.int32)
        iy = ((np.asarray(wy) - self.oy) / self.res).astype(np.int32)
        return ix, iy

    def grid_to_world(self, ix, iy):
        return (self.ox + (ix + 0.5) * self.res, self.oy + (iy + 0.5) * self.res)

    def in_bounds(self, ix, iy):
        return 0 <= ix < self.n and 0 <= iy < self.n

    def _inb_arr(self, ix, iy):
        return (ix >= 0) & (ix < self.n) & (iy >= 0) & (iy < self.n)

    def _ray_cells(self, pose, bearing_body, r0, r1):
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
    def integrate_scan(self, pose, pts_body, ranges):
        """Ray-trace free space and stamp occupied endpoints from one scan."""
        if pts_body.shape[0] == 0:
            return
        x, y, th = pose
        pts_world = transform_points(pts_body, x, y, th)
        rix, riy = self.world_to_grid(x, y)

        free_lin = []
        occ_lin = []
        occ_close = []   # endpoints seen from CLOSE, accurate range -> candidates to confirm
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
                    lin = eix * n_cells + eiy
                    occ_lin.append(lin)
                    if rng < C.OCC_CONFIRM_RANGE:
                        occ_close.append(lin)

        Lflat = self.L.reshape(-1)
        conf_flat = self.occ_confirmed.reshape(-1)
        strike_flat = self.occ_free_strikes.reshape(-1)
        if free_lin:
            f = np.unique(np.concatenate(free_lin))
            is_conf = conf_flat[f]
            fn = f[~is_conf]                 # unconfirmed cells -> normal free update
            if fn.size:
                np.add.at(Lflat, fn, C.L_FREE)
            fc = f[is_conf]                  # confirmed cells crossed by a free ray
            if fc.size:
                # KEEP-THE-POINTS, but SELF-CLEANING: a confirmed wall shrugs off a
                # few grazing free rays, yet a cell the lidar sees THROUGH on many
                # scans (drift smear / stale latch) is released so it cannot cage the
                # robot.  Real walls are never crossed by free rays -> never released.
                strike_flat[fc] += 1.0
                rel = fc[strike_flat[fc] >= C.OCC_CONFIRM_RELEASE]
                if rel.size:
                    conf_flat[rel] = False
                    strike_flat[rel] = 0.0
                    np.add.at(Lflat, rel, C.L_FREE)
        if occ_lin:
            o = np.unique(np.asarray(occ_lin, dtype=np.int64))
            np.add.at(Lflat, o, C.L_OCC)
            strike_flat[o] = 0.0             # re-seen as a wall -> reset accumulated doubt
        np.clip(self.L, C.L_MIN, C.L_MAX, out=self.L)
        # LATCH: a close-range cell that has accumulated to a solid log-odds level
        # (seen as a wall on several scans, not one noisy beam) becomes confirmed.
        if occ_close:
            oc = np.unique(np.asarray(occ_close, dtype=np.int64))
            oc = oc[Lflat[oc] >= C.L_CONFIRM_LEVEL]
            if oc.size:
                conf_flat[oc] = True
                strike_flat[oc] = 0.0

    def integrate_depth_obstacles(self, pose, bearings, hit_ranges, hit_mask, clear_mask, depth_min):
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
        # NOTE: an earlier build latched a SATURATED aux cell into occ_confirmed so a
        # floating wall would persist through the < depth_min blind shell.  Reverted:
        # depth blobs are noisy, the latch was permanent (occ_confirmed only releases
        # under the robot footprint / on lidar see-through, and the lidar sees STRAIGHT
        # UNDER a floating wall so it would never strike-release there), so it dropped a
        # ~50-cell permanent blob that caged the robot.  The aux confidence layer below
        # already persists the floating wall across the blind shell without permanence.

    def mark_free_disc(self, wx, wy, radius):
        """Force a small disc around a known-free point to a free log-odds.

        ALSO clears the aux AND poison layers in that disc: the robot is
        physically standing there, so the cell is demonstrably NOT a hazard --
        ground truth overrides any stale aux/poison mark.  Clearing aux fixes the
        Maze1 self-boxing bug; clearing poison removes any phantom green cell the
        projection put under the robot's own footprint (a real poison patch is
        re-confirmed by the camera the instant the robot leaves it, so a genuine
        hazard is not lost -- only the small footprint the robot just proved safe).
        """
        cix, ciy = self.world_to_grid(wx, wy)
        r = int(math.ceil(radius / self.res))
        x0, x1 = max(0, cix - r), min(self.n, cix + r + 1)
        y0, y1 = max(0, ciy - r), min(self.n, ciy + r + 1)
        if x0 >= x1 or y0 >= y1:
            return
        sub = self.L[x0:x1, y0:y1]
        np.minimum(sub, C.L_FREE_THRESH - 0.1, out=sub)
        self.aux[x0:x1, y0:y1] = False
        self.aux_hits[x0:x1, y0:y1] = 0.0   # reset confidence where the robot has driven
        self.poison[x0:x1, y0:y1] = False
        self.poison_hits[x0:x1, y0:y1] = 0.0
        self.occ_confirmed[x0:x1, y0:y1] = False  # robot drove here -> release the latch
        self.occ_free_strikes[x0:x1, y0:y1] = 0.0

    def add_poison_points(self, pts_world):
        """Confidence-gated poison stamping (self-correcting against drift).

        Call ONCE per perception frame with the green floor points projected this
        frame (may be empty).  Each step: decay the confidence of every
        not-yet-confirmed cell, then add ``C.POISON_HIT_INC`` to each cell seen
        this frame (once per cell, not per point, so confirmation is per-FRAME not
        per-pixel).  ``self.poison`` is the boolean lethal mask = confidence at or
        above ``C.POISON_MIN_HITS``.

        Two-tier decay so NO cell is permanent (the "pois -> 822, never decays"
        fix):
          * GLOBAL decay (``C.POISON_GLOBAL_DECAY``) drains EVERY nonzero cell,
            including already-lethal/capped ones, so a patch the robot no longer
            sees eventually fades; a patch still in view is re-topped faster than
            it bleeds and stays lethal.
          * TRANSIENT decay (``C.POISON_DECAY``) additionally drains cells that
            are NOT yet lethal, so a one-off projection mark must be re-seen on
            consecutive frames to cross the threshold (it peaks low and is gone in
            ~2 frames).  Above the lethal line a cell is only globally decayed.
        """
        hits = self.poison_hits
        # 1a) global decay everywhere (bounds permanence -- capped cells too)
        nz = hits > 0.0
        if nz.any():
            hits[nz] = np.maximum(0.0, hits[nz] - C.POISON_GLOBAL_DECAY)
        # 1b) extra transient decay on cells not yet lethal (consecutive-frame gate)
        transient = (hits > 0.0) & (hits < C.POISON_MIN_HITS)
        if transient.any():
            hits[transient] = np.maximum(0.0, hits[transient] - C.POISON_DECAY)
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
    def occupied_mask(self):
        return (self.L > C.L_OCC_THRESH) | self.occ_confirmed

    def free_mask(self):
        return self.L < C.L_FREE_THRESH

    def unknown_mask(self):
        return (~self.occupied_mask()) & (~self.free_mask())

    # ------------------------------------------------- mapped obstacles query
    def poison_points_near(self, wx, wy, radius):
        """World (M,2) of poison-cell centres within ``radius`` of (wx,wy)."""
        return self._mask_points_near(self.poison, wx, wy, radius)

    def aux_points_near(self, wx, wy, radius):
        """World (M,2) of aux-occupied cell centres within ``radius`` of (wx,wy)."""
        return self._mask_points_near(self.aux, wx, wy, radius)

    def _mask_points_near(self, mask, wx, wy, radius):
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

    def is_poison_world(self, wx, wy):
        ix, iy = self.world_to_grid(wx, wy)
        return self.in_bounds(ix, iy) and bool(self.poison[ix, iy])

    # ------------------------------------------------------------ scan match
    def _score_field(self):
        """Float field for scan matching: occupied=1, 1-ring=0.6, 2-ring=0.3.

        Deliberately lidar-only (does NOT include aux): the depth-aux layer is
        noisier and only used for collision avoidance / costmap, never for
        localisation, so it cannot corrupt the scan-matched pose.
        """
        occ = self.occupied_mask()
        field = np.zeros((self.n, self.n), dtype=np.float32)
        field[occ] = 1.0
        d1 = _dilate8(occ) & ~occ
        field[d1] = 0.6
        d2 = _dilate8(_dilate8(occ)) & ~_dilate8(occ)
        field[d2] = 0.3
        return field

    def scan_match(self, pred_pose, pts_body):
        """Correlative (x,y,yaw) correction of ``pred_pose`` against the map.

        Returns ``(corrected_pose, hit_fraction)``.  Falls back to ``pred_pose``
        when too few beams land on mapped walls (avoids corrupting a good odom).
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
        correction = math.hypot(dx, dy)

        # Conservative acceptance (de-drift WITHOUT jitter): accept a correction
        # only when the match is confident (enough beams land on mapped walls) AND
        # the shift it asks for is small.  The old "unconditional accept above
        # SM_TRUST_HIT_FRAC" branch is gone -- a confident match that wants a large
        # jump is a mis-association (or a momentary alias in a low-information
        # corridor), and accepting it is exactly what slid/smeared the pose before.
        # Bounding EVERY accepted correction to SM_MARGINAL_MAX_CORR lets the
        # matcher nudge out slow odometry drift a little each step while never
        # teleporting the belief pose.
        if hit_frac >= C.SM_MIN_HIT_FRAC and correction <= C.SM_MARGINAL_MAX_CORR:
            return (px + dx, py + dy, wrap_angle(pth + dth)), hit_frac

        return pred_pose, hit_frac

    # -------------------------------------------------------------- costmap
    def build_costmap(self):
        """Return ``(cost float32, lethal bool)`` over the whole grid.

        ``cost`` is additive A* penalty per cell (0 in open free space).  Unknown
        cells are traversable (optimistic) so frontier goals stay reachable; the
        live-lidar DWA layer guarantees real wall clearance regardless.

        Aux (depth-derived floating-wall) cells get their OWN, WIDER hard/soft
        bands (C.AUX_HARD_OBS_DIST / C.AUX_CENTER_PREF_RANGE) instead of being
        folded into the lidar-wall bands: aux confidence is sparser (2-frame
        confirm) and less precisely localised than a lidar-mapped wall, so A*
        should start curving away from it sooner and keep more clearance,
        rather than threading it as tightly as a solid, precisely-mapped wall.
        The two obstacle sources are combined by taking the lethal UNION and,
        in the shared soft zone, the MAX of their two cost contributions.
        """
        res = self.res
        occ_lidar = self.occupied_mask()
        occ_aux = self.aux

        hard_b = max(1, int(round(C.HARD_OBS_DIST / res)))
        pref_b = max(hard_b + 1, int(round(C.CENTER_PREF_RANGE / res)))
        aux_hard_b = max(hard_b, int(round(C.AUX_HARD_OBS_DIST / res)))
        aux_pref_b = max(aux_hard_b + 1, int(round(C.AUX_CENTER_PREF_RANGE / res)))

        d_lidar = _distance_bands(occ_lidar, pref_b + 1)
        lethal = d_lidar <= hard_b
        cost = np.zeros((self.n, self.n), dtype=np.float32)
        # center-seeking gradient: cost falls off with clearance out to pref_b so
        # A* rides the medial axis (maximum clearance) of every corridor rather
        # than hugging the first non-lethal cell.  This is what aligns the path
        # to the absolute centre of the tight 0.39 m gaps.  (Replaces the old
        # fixed-width soft halo, which went flat in the middle of a narrow gap
        # and so gave A* no reason to prefer dead-centre.)
        grad_zone = (d_lidar > hard_b) & (d_lidar <= pref_b)
        frac = (pref_b - d_lidar.astype(np.float32)) / max(1, (pref_b - hard_b))
        cost[grad_zone] += C.COST_OBS_WEIGHT * frac[grad_zone]

        if occ_aux.any():
            d_aux = _distance_bands(occ_aux, aux_pref_b + 1)
            lethal |= d_aux <= aux_hard_b
            aux_grad = (d_aux > aux_hard_b) & (d_aux <= aux_pref_b)
            aux_frac = (aux_pref_b - d_aux.astype(np.float32)) / max(1, (aux_pref_b - aux_hard_b))
            cost[aux_grad] = np.maximum(cost[aux_grad], C.COST_OBS_WEIGHT * aux_frac[aux_grad])

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
