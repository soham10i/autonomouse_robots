"""3D voxel cloud map — continuous depth accumulation + 3D->2D projection.

The depth camera is back-projected to world-frame ``(x, y, z)`` points every
frame and hashed into a sparse voxel grid (one hit-count per occupied voxel).
This is the same idea as the ``teleop_mapping`` 3D scene, brought into the
autonomous controller so the map keeps a real 3D model of the maze while it
explores.

Two products come out of the accumulated cloud:

* **3D render** (``points``) — voxel-centre world points + counts, height-coloured
  by the exporter for a PLY / 3D scatter.
* **2D obstacle footprint** (``project_to_2d``) — the voxels whose height is in
  the robot's collision band ``[CLOUD_OBS_Z_MIN, CLOUD_OBS_Z_MAX]`` collapsed
  straight down to grid cells.  Because every view of a wall is accumulated, a
  tumbled/tilted floating slab projects to its TRUE footprint, not the single
  near-face point a one-shot thin scan produced (which mapped ~0.15 m off and
  wedged the robot).  Voxels purely above ``ROBOT_HEIGHT`` are drive-under
  passages and are excluded.

Pure NumPy + a Python dict — no Webots, no Open3D — so it is unit-testable.
"""
from __future__ import annotations

import numpy as np

import settings as S


# Packing: voxel integer coords are offset to be non-negative and packed into a
# single int64 key (21 bits per axis => +/- 2^20 voxels, ~ +/-31 km at 0.03 m).
_OFF = 1 << 20
_BITS = 21
_MASK = (1 << _BITS) - 1


class CloudMap:
    def __init__(self, voxel=None, z_min=None, z_max=None):
        self.voxel = float(S.CLOUD_VOXEL_M if voxel is None else voxel)
        self.z_min = float(S.CLOUD_Z_MIN if z_min is None else z_min)
        self.z_max = float(S.CLOUD_Z_MAX if z_max is None else z_max)
        self._cnt: dict[int, int] = {}     # packed voxel key -> accumulated hits

    # --------------------------------------------------------------- packing
    @staticmethod
    def _pack(v):
        """(M, 3) int voxel coords -> (M,) int64 keys."""
        vx = v[:, 0].astype(np.int64) + _OFF
        vy = v[:, 1].astype(np.int64) + _OFF
        vz = v[:, 2].astype(np.int64) + _OFF
        return (vx << (2 * _BITS)) | (vy << _BITS) | vz

    @staticmethod
    def _unpack(keys):
        """(M,) int64 keys -> (M, 3) int voxel coords."""
        keys = np.asarray(keys, dtype=np.int64)
        vz = (keys & _MASK) - _OFF
        vy = ((keys >> _BITS) & _MASK) - _OFF
        vx = ((keys >> (2 * _BITS)) & _MASK) - _OFF
        return np.stack([vx, vy, vz], axis=1)

    # ----------------------------------------------------------- accumulation
    def add_points_world(self, pts):
        """Fold world-frame ``(M, 3)`` points into the voxel hash.

        Returns the number of points integrated (after height/finiteness gating).
        """
        pts = np.asarray(pts, dtype=np.float64).reshape(-1, 3)
        if pts.shape[0] == 0:
            return 0
        m = (np.isfinite(pts).all(axis=1)
             & (pts[:, 2] > self.z_min) & (pts[:, 2] < self.z_max))
        pts = pts[m]
        if pts.shape[0] == 0:
            return 0
        v = np.floor(pts / self.voxel).astype(np.int64)
        keys = self._pack(v)
        uniq, counts = np.unique(keys, return_counts=True)
        d = self._cnt
        for k, c in zip(uniq.tolist(), counts.tolist()):
            d[k] = d.get(k, 0) + c
        if len(d) > S.CLOUD_MAX_VOXELS:
            self._evict()
        return int(pts.shape[0])

    def _evict(self):
        """Drop the lowest-count half of the voxels when over the cap (keeps the
        well-confirmed structure, sheds transient noise)."""
        keys = np.fromiter(self._cnt.keys(), dtype=np.int64)
        cnt = np.fromiter(self._cnt.values(), dtype=np.int64)
        keep_n = S.CLOUD_MAX_VOXELS // 2
        order = np.argsort(cnt)[::-1][:keep_n]
        self._cnt = {int(k): int(c) for k, c in zip(keys[order], cnt[order])}

    # ------------------------------------------------------------- accessors
    def __len__(self):
        return len(self._cnt)

    def points(self):
        """Return ``(pts_world (N, 3), counts (N,))`` voxel-centre world points."""
        if not self._cnt:
            return np.empty((0, 3)), np.empty((0,), dtype=np.int64)
        keys = np.fromiter(self._cnt.keys(), dtype=np.int64)
        cnt = np.fromiter(self._cnt.values(), dtype=np.int64)
        v = self._unpack(keys)
        pts = (v.astype(np.float64) + 0.5) * self.voxel
        return pts, cnt

    # ----------------------------------------------------------- projection
    def project_to_2d(self, grid, z_min=None, z_max=None, min_hits=None):
        """Collapse collision-band voxels into a ``(cells, cells)`` bool mask.

        Only voxels with height in ``[z_min, z_max]`` and at least ``min_hits``
        accumulated hits project down — that is the floating-wall / wall footprint
        the planner must avoid.  Voxels purely above the band are drive-under
        passages and never appear.
        """
        z0 = S.CLOUD_OBS_Z_MIN if z_min is None else z_min
        z1 = S.CLOUD_OBS_Z_MAX if z_max is None else z_max
        mh = S.CLOUD_OBS_MIN_HITS if min_hits is None else min_hits
        mask = np.zeros((grid.cells, grid.cells), dtype=bool)
        pts, cnt = self.points()
        if pts.shape[0] == 0:
            return mask
        sel = (pts[:, 2] > z0) & (pts[:, 2] < z1) & (cnt >= mh)
        pts = pts[sel]
        if pts.shape[0] == 0:
            return mask
        ix, iy, m = grid.world_to_grid_arr(pts[:, 0], pts[:, 1])
        ix, iy = ix[m], iy[m]
        if ix.size:
            mask[ix, iy] = True
        return mask
