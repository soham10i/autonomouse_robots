"""Unified mapping module.

Layers
------
1. ``L``                 — log-odds occupancy from LiDAR (primary obstacle source).
2. ``aux_obstacle``      — boolean cells set by the depth-camera projection
                            (catches floating / low walls the 2D lidar misses).
3. ``poison``            — boolean cells set when the green floor patch is
                            seen.  *Lethal*; never cleared once set.

The class also provides:
    - ``lethal_mask()``    occupied ∪ aux ∪ poison
    - ``inflated_lethal()`` lethal expanded by the robot footprint
    - ``costmap()``        soft cost (distance from obstacles) for A*
    - ``mark_*``           helpers for poison / aux obstacles / forced occupancy

Storage convention: ``L[ix, iy]`` — index 0 is x (column), index 1 is y (row).
This keeps world-coordinate semantics clean.
"""
import math
import os

import numpy as np

import config as C
from utils import bresenham, world_to_grid, grid_to_world


class Grid2D:
    def __init__(self):
        self.cells = C.GRID_CELLS
        self.res = C.GRID_RESOLUTION
        self.origin = np.array(C.GRID_ORIGIN, dtype=np.float64)
        shape = (self.cells, self.cells)
        self.L = np.zeros(shape, dtype=np.float32)
        self.aux_obstacle = np.zeros(shape, dtype=bool)
        self.poison = np.zeros(shape, dtype=bool)

        self._cached_lethal = None
        self._cached_inflated = None
        self._cached_costmap = None
        self._dirty = True

    # ----------------------------- coords -------------------------------

    def w2i(self, wx, wy):
        return world_to_grid(wx, wy, self.origin, self.res)

    def i2w(self, ix, iy):
        return grid_to_world(ix, iy, self.origin, self.res)

    def in_bounds(self, ix, iy):
        return 0 <= ix < self.cells and 0 <= iy < self.cells

    def world_to_idx_arr(self, wx_arr, wy_arr):
        ix = np.floor((wx_arr - self.origin[0]) / self.res).astype(np.int32)
        iy = np.floor((wy_arr - self.origin[1]) / self.res).astype(np.int32)
        m = (ix >= 0) & (ix < self.cells) & (iy >= 0) & (iy < self.cells)
        return ix, iy, m

    # ----------------------------- LiDAR --------------------------------

    def integrate_scan(self, pose, hits_world):
        """Update log-odds: free along each ray, occupied at the hit."""
        if hits_world.shape[0] == 0:
            return
        rx, ry, _ = pose
        sx, sy = self.w2i(rx, ry)
        ix_arr, iy_arr, mask = self.world_to_idx_arr(hits_world[:, 0], hits_world[:, 1])
        ix_arr = ix_arr[mask]
        iy_arr = iy_arr[mask]
        if ix_arr.size == 0:
            return

        self._dirty = True
        L = self.L
        cells = self.cells
        for tx, ty in zip(ix_arr.tolist(), iy_arr.tolist()):
            line = bresenham(sx, sy, tx, ty)
            # mark all but the endpoint as free
            for fx, fy in line[:-1]:
                if 0 <= fx < cells and 0 <= fy < cells:
                    v = L[fx, fy] + C.L_FREE
                    L[fx, fy] = v if v > C.L_MIN else C.L_MIN
            if 0 <= tx < cells and 0 <= ty < cells:
                v = L[tx, ty] + C.L_OCC
                L[tx, ty] = v if v < C.L_MAX else C.L_MAX

    # ----------------------------- aux depth ----------------------------

    def mark_aux_obstacle_points(self, pts_world_xy):
        """``pts_world_xy`` (M, 2) — chassis-height points from depth cam."""
        if pts_world_xy is None or pts_world_xy.shape[0] == 0:
            return
        ix, iy, m = self.world_to_idx_arr(pts_world_xy[:, 0], pts_world_xy[:, 1])
        ix = ix[m]
        iy = iy[m]
        if ix.size == 0:
            return
        self.aux_obstacle[ix, iy] = True
        self._dirty = True

    # ----------------------------- poison -------------------------------

    def mark_poison(self, wx, wy, radius=C.POISON_DETECT_RADIUS):
        ix_c, iy_c = self.w2i(wx, wy)
        n = int(math.ceil(radius / self.res)) + 1
        r2 = (radius / self.res) ** 2
        for dx in range(-n, n + 1):
            for dy in range(-n, n + 1):
                if dx * dx + dy * dy <= r2:
                    ix = ix_c + dx
                    iy = iy_c + dy
                    if self.in_bounds(ix, iy):
                        self.poison[ix, iy] = True
        self._dirty = True

    def is_poison_world(self, wx, wy):
        ix, iy = self.w2i(wx, wy)
        if not self.in_bounds(ix, iy):
            return False
        return bool(self.poison[ix, iy])

    # ----------------------------- forced occ ---------------------------

    def mark_inflation(self, wx, wy, radius=0.30):
        """Force a circular world region to L_MAX (legacy — used sparingly)."""
        ix_c, iy_c = self.w2i(wx, wy)
        n = int(math.ceil(radius / self.res)) + 1
        r2 = (radius / self.res) ** 2
        for dx in range(-n, n + 1):
            for dy in range(-n, n + 1):
                if dx * dx + dy * dy <= r2:
                    ix = ix_c + dx
                    iy = iy_c + dy
                    if self.in_bounds(ix, iy):
                        self.L[ix, iy] = C.L_MAX
        self._dirty = True

    # ----------------------------- masks --------------------------------

    def occupancy_prob(self):
        return 1.0 - 1.0 / (1.0 + np.exp(self.L))

    def occupied_mask(self):
        return self.L >= C.L_THRESH_OCC

    def free_mask(self):
        return self.L <= C.L_THRESH_FREE

    def unknown_mask(self):
        return np.abs(self.L) < 0.05

    def lethal_mask(self):
        if not self._dirty and self._cached_lethal is not None:
            return self._cached_lethal
        lethal = self.occupied_mask() | self.aux_obstacle | self.poison
        self._cached_lethal = lethal
        return lethal

    def inflated_lethal(self, radius=None):
        """Boolean mask: lethal cells dilated by ``radius`` (m)."""
        if radius is None:
            radius = C.INFLATION_RADIUS
        lethal = self.lethal_mask()
        n = max(1, int(math.ceil(radius / self.res)))
        try:
            from scipy.ndimage import binary_dilation
            structure = self._disc_struct(n)
            return binary_dilation(lethal, structure=structure)
        except Exception:
            # Pure-Python fallback (slow but correct).
            inflated = lethal.copy()
            r2 = n * n
            ix, iy = np.where(lethal)
            for cx, cy in zip(ix.tolist(), iy.tolist()):
                for dx in range(-n, n + 1):
                    for dy in range(-n, n + 1):
                        if dx * dx + dy * dy <= r2:
                            xx, yy = cx + dx, cy + dy
                            if self.in_bounds(xx, yy):
                                inflated[xx, yy] = True
            return inflated

    def _disc_struct(self, n):
        size = 2 * n + 1
        y, x = np.ogrid[-n:n + 1, -n:n + 1]
        return (x * x + y * y) <= n * n

    def costmap(self):
        """Costmap for A* with separate hard-lethal vs soft halo.

        Hard lethal (cost == inf): cells where the robot footprint cannot fit.
        Threshold is tight (``ROBOT_RADIUS - HARD_OBS_MARGIN``) so corridors
        only marginally wider than the robot remain plannable.

        Soft halo: above the hard threshold, an exponential cost in the
        ``[HARD_OBS, HARD_OBS + SOFT_OBS_HALO]`` band biases A* toward the
        middle of free space.

        Poison is treated similarly with stricter thresholds.

        Returns ``(cost, hard_lethal)`` where ``hard_lethal`` is a bool mask
        (cells the planner must not enter).
        """
        try:
            from scipy.ndimage import distance_transform_edt
            obs_mask = self.occupied_mask() | self.aux_obstacle
            dist_obs = distance_transform_edt(~obs_mask) * self.res
            dist_poison = distance_transform_edt(~self.poison) * self.res
        except Exception:
            obs_mask = self.occupied_mask() | self.aux_obstacle
            dist_obs = self._approx_distance_transform(obs_mask)
            dist_poison = self._approx_distance_transform(self.poison)

        hard_obs = max(C.ROBOT_RADIUS - C.HARD_OBS_MARGIN, 0.04)
        soft_obs = hard_obs + C.SOFT_OBS_HALO
        hard_poi = max(C.ROBOT_RADIUS - C.HARD_POISON_MARGIN, hard_obs)
        soft_poi = hard_poi + C.SOFT_POISON_HALO

        cost = np.zeros_like(dist_obs, dtype=np.float32)

        # soft halo around obstacles (only for cells outside the hard radius)
        m_obs_soft = (dist_obs >= hard_obs) & (dist_obs < soft_obs)
        if m_obs_soft.any():
            t = (dist_obs[m_obs_soft] - hard_obs) / max(soft_obs - hard_obs, 1e-6)
            cost[m_obs_soft] += C.ASTAR_INFLATION_WEIGHT * np.exp(-C.INFLATION_ALPHA * t)

        # soft halo around poison
        m_poi_soft = (dist_poison >= hard_poi) & (dist_poison < soft_poi)
        if m_poi_soft.any():
            t = (dist_poison[m_poi_soft] - hard_poi) / max(soft_poi - hard_poi, 1e-6)
            cost[m_poi_soft] += C.ASTAR_POISON_INFLATION_WEIGHT * np.exp(
                -C.INFLATION_ALPHA * t
            )

        hard_lethal = (dist_obs < hard_obs) | (dist_poison < hard_poi)
        cost[hard_lethal] = np.inf
        return cost, hard_lethal

    # ----------------------------- carve free ---------------------------

    def mark_free_disc(self, wx, wy, radius):
        """Force a circular world region to free (log-odds = L_MIN, no aux).

        Used to guarantee the robot's own cell is plannable: the integration
        of LiDAR endpoints at very short range can otherwise leave the robot
        sitting on a 'lethal' cell after a near-collision."""
        ix_c, iy_c = self.w2i(wx, wy)
        n = max(1, int(math.ceil(radius / self.res)))
        r2 = (radius / self.res) ** 2
        for dx in range(-n, n + 1):
            for dy in range(-n, n + 1):
                if dx * dx + dy * dy > r2:
                    continue
                ix = ix_c + dx
                iy = iy_c + dy
                if not self.in_bounds(ix, iy):
                    continue
                if self.poison[ix, iy]:
                    continue   # never erase poison
                self.L[ix, iy] = C.L_MIN
                self.aux_obstacle[ix, iy] = False
        self._dirty = True

    def _approx_distance_transform(self, mask):
        """Cheap 2-pass chamfer (pixels). Used only if scipy is missing."""
        cells = self.cells
        d = np.where(mask, 0.0, np.inf).astype(np.float32)
        # forward
        for x in range(cells):
            for y in range(cells):
                v = d[x, y]
                if x > 0:
                    v = min(v, d[x - 1, y] + 1)
                if y > 0:
                    v = min(v, d[x, y - 1] + 1)
                if x > 0 and y > 0:
                    v = min(v, d[x - 1, y - 1] + math.sqrt(2))
                d[x, y] = v
        # backward
        for x in range(cells - 1, -1, -1):
            for y in range(cells - 1, -1, -1):
                v = d[x, y]
                if x < cells - 1:
                    v = min(v, d[x + 1, y] + 1)
                if y < cells - 1:
                    v = min(v, d[x, y + 1] + 1)
                if x < cells - 1 and y < cells - 1:
                    v = min(v, d[x + 1, y + 1] + math.sqrt(2))
                d[x, y] = v
        return d * self.res

    # ----------------------------- diagnostics --------------------------

    def save_png(self, path, pose=None, path_world=None, frontier=None,
                 standoff_world=None, state_text=None):
        try:
            import matplotlib
            matplotlib.use("Agg")
            import matplotlib.pyplot as plt
        except Exception as e:
            print("[grid_2d] matplotlib missing, skipping PNG:", e)
            return
        p = self.occupancy_prob()
        rgb = np.ones((self.cells, self.cells, 3), dtype=np.float32)
        unk = self.unknown_mask()
        rgb[unk] = (0.78, 0.78, 0.78)
        occ = self.occupied_mask()
        rgb[occ] = (0.05, 0.05, 0.05)
        rgb[self.aux_obstacle & ~occ] = (0.45, 0.20, 0.50)
        rgb[self.poison] = (0.05, 0.65, 0.05)

        fig, ax = plt.subplots(figsize=(6, 6), dpi=140)
        extent = [
            self.origin[0],
            self.origin[0] + self.cells * self.res,
            self.origin[1],
            self.origin[1] + self.cells * self.res,
        ]
        ax.imshow(rgb.transpose(1, 0, 2), origin="lower", extent=extent)

        if pose is not None:
            x, y, th = pose
            ax.plot([x], [y], "r.", markersize=7)
            ax.plot([x, x + 0.22 * math.cos(th)], [y, y + 0.22 * math.sin(th)],
                    "r-", lw=1.4)
        if path_world is not None and len(path_world) > 1:
            xs = [pp[0] for pp in path_world]
            ys = [pp[1] for pp in path_world]
            ax.plot(xs, ys, "-", color="#1f77b4", lw=1.6, label="path")
        if frontier is not None:
            ax.plot([frontier[0]], [frontier[1]], "x", color="#e67e22",
                    markersize=10, label="frontier")
        if standoff_world is not None:
            ax.plot([standoff_world[0]], [standoff_world[1]], "*",
                    color="#f1c40f", markersize=14, label="standoff")
        if state_text:
            ax.text(0.02, 0.98, state_text, transform=ax.transAxes, fontsize=9,
                    va="top", ha="left",
                    bbox=dict(boxstyle="round", facecolor="white", alpha=0.7))

        ax.set_xlabel("x [m]")
        ax.set_ylabel("y [m]")
        ax.set_aspect("equal")
        os.makedirs(os.path.dirname(path) or ".", exist_ok=True)
        fig.savefig(path, bbox_inches="tight")
        plt.close(fig)
