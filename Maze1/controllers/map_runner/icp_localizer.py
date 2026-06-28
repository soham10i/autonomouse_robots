"""ICP scan-to-scan localizer for correcting odometry drift.

This module implements 2D Iterative Closest Point (ICP) scan matching to
correct cumulative odometry errors in real-time.  It aligns consecutive
LiDAR scans against each other and computes a pose correction.

Algorithm (per call to ``correct()``):
    1. Transform the current local scan into world frame using odometry.
    2. Run ICP to align the current world-frame scan against the previous
       world-frame scan (or accumulated reference cloud).
    3. Compute the drift = (ICP motion) − (odometry motion).
    4. Apply a blended correction to the robot pose.

The reference cloud is **accumulated** from past scans (up to
``MAX_REF_POINTS``) and periodically refreshed, giving more structure
for ICP to lock onto than a single previous scan.

Math
----
    E(R, t) = Σ_i || R · p_i + t − q_i ||²

    Centroid:  p̄ = mean(p),  q̄ = mean(q)
    Centered:  p' = p − p̄,   q' = q − q̄
    Cross-cov: H  = Σ p'_i · q'_i^T
    SVD:       U, S, V^T = svd(H)
    Rotation:  R  = V · U^T   (negate last col of V if det < 0)
    Translate: t  = q̄ − R · p̄

References
----------
- PythonRobotics ICP: github.com/AtsushiSakai/PythonRobotics
- KISS-ICP: github.com/PRBonn/kiss-icp
- slam_navigator/icp.py in this project (original reference)
"""
import math
import numpy as np

try:
    from scipy.spatial import KDTree
    _HAS_SCIPY = True
except ImportError:
    _HAS_SCIPY = False

# ── Configuration ──────────────────────────────────────────────────────
ICP_MAX_ITER = 25           # max ICP iterations per call
ICP_CONVERGE_TOL = 1e-4     # convergence threshold on mean-error change
ICP_MAX_PAIR_DIST = 0.5     # reject point pairs farther than this (m)
ICP_MIN_POINTS = 15         # minimum scan points for ICP to run
ICP_BLEND = 0.5             # how much of the ICP correction to apply
ICP_MAX_CORRECTION_M = 0.15 # reject corrections larger than this (m)
ICP_MAX_CORRECTION_RAD = math.radians(8.0)  # reject angular corrections larger than this
MAX_REF_POINTS = 4000       # cap on accumulated reference cloud size
REF_VOXEL_SIZE = 0.06       # voxel downsample size for reference cloud (m)


def _wrap(a):
    """Wrap angle to (-π, π]."""
    while a > math.pi:
        a -= 2.0 * math.pi
    while a <= -math.pi:
        a += 2.0 * math.pi
    return a


def _voxel_downsample_2d(pts, voxel):
    """Keep one point per 2D voxel cell."""
    if pts.shape[0] == 0:
        return pts
    keys = np.floor(pts / voxel).astype(np.int64)
    _, idx = np.unique(keys, axis=0, return_index=True)
    return pts[idx]


# ── Pure-numpy fallback for KDTree nearest-neighbour search ────────────
def _brute_nearest(src, tgt):
    """For each point in src, find the nearest point in tgt.

    Returns (distances, indices) — same interface as KDTree.query().
    O(N*M), used only when scipy is unavailable.
    """
    # Chunked to avoid memory blow-up with large clouds
    chunk = 500
    dists_all = np.empty(src.shape[0])
    idx_all = np.empty(src.shape[0], dtype=np.intp)
    for i in range(0, src.shape[0], chunk):
        s = src[i:i + chunk]
        diff = s[:, None, :] - tgt[None, :, :]  # (chunk, M, 2)
        d2 = (diff * diff).sum(axis=2)           # (chunk, M)
        j = np.argmin(d2, axis=1)
        dists_all[i:i + chunk] = np.sqrt(d2[np.arange(len(j)), j])
        idx_all[i:i + chunk] = j
    return dists_all, idx_all


def _icp_2d(source, target, max_iter=ICP_MAX_ITER, tol=ICP_CONVERGE_TOL,
            max_pair_dist=ICP_MAX_PAIR_DIST):
    """Align source points onto target points using ICP.

    Parameters
    ----------
    source : (N, 2) ndarray — current scan points
    target : (M, 2) ndarray — reference points
    max_iter : int
    tol : float — convergence threshold on mean error change
    max_pair_dist : float — reject pairs farther than this (m)

    Returns
    -------
    R : (2, 2) rotation matrix
    t : (2,)   translation vector
    converged : bool
    mean_error : float
    n_matched : int — number of inlier correspondences
    """
    if source.shape[0] < 3 or target.shape[0] < 3:
        return np.eye(2), np.zeros(2), False, float("inf"), 0

    src = source.copy().astype(np.float64)
    R_total = np.eye(2)
    t_total = np.zeros(2)
    prev_error = float("inf")

    # Build search structure once
    if _HAS_SCIPY:
        tree = KDTree(target.astype(np.float64))
    else:
        tgt_f = target.astype(np.float64)

    for _ in range(max_iter):
        # Step 1: Nearest-neighbour correspondences
        if _HAS_SCIPY:
            dists, indices = tree.query(src)
        else:
            dists, indices = _brute_nearest(src, tgt_f)

        # Reject outlier pairs
        mask = dists < max_pair_dist
        n_matched = int(mask.sum())
        if n_matched < 3:
            break

        matched_src = src[mask]
        matched_tgt = target[indices[mask]].astype(np.float64)

        # Step 2: Centroids
        p_bar = matched_src.mean(axis=0)
        q_bar = matched_tgt.mean(axis=0)

        # Step 3: Center
        p_c = matched_src - p_bar
        q_c = matched_tgt - q_bar

        # Step 4: Cross-covariance H = Σ p' · q'^T
        H = p_c.T @ q_c

        # Step 5: SVD
        U, S, Vt = np.linalg.svd(H)

        # Step 6: Optimal rotation R = V · U^T
        R = Vt.T @ U.T
        if np.linalg.det(R) < 0:
            Vt[-1, :] *= -1
            R = Vt.T @ U.T

        # Step 7: Optimal translation
        t = q_bar - R @ p_bar

        # Apply to source
        src = (R @ src.T).T + t

        # Accumulate total
        R_total = R @ R_total
        t_total = R @ t_total + t

        # Convergence check
        mean_error = float(np.mean(dists[mask]))
        if abs(prev_error - mean_error) < tol:
            return R_total, t_total, True, mean_error, n_matched
        prev_error = mean_error

    return R_total, t_total, False, prev_error, n_matched


class IcpLocalizer:
    """Scan-to-scan ICP localizer with accumulated reference cloud.

    Usage::

        loc = IcpLocalizer()
        # each tick (or every Nth tick):
        corrected_pose, info = loc.correct(odom_pose, scan_local)
    """

    def __init__(self):
        self._ref_cloud = None          # (M, 2) accumulated reference in world frame
        self._prev_odom_pose = None     # (x, y, θ) at last ICP call
        self._corrections_applied = 0
        self._total_drift_x = 0.0
        self._total_drift_y = 0.0
        self._total_drift_th = 0.0
        print("[icp] IcpLocalizer initialized "
              f"(scipy={'yes' if _HAS_SCIPY else 'NO — using brute-force fallback'})",
              flush=True)

    @property
    def corrections_applied(self):
        return self._corrections_applied

    def _local_to_world(self, scan_local, pose):
        """Transform (N, 2) local-frame points to world frame."""
        x, y, th = pose
        c, s = math.cos(th), math.sin(th)
        wx = x + c * scan_local[:, 0] - s * scan_local[:, 1]
        wy = y + s * scan_local[:, 0] + c * scan_local[:, 1]
        return np.stack([wx, wy], axis=1)

    def correct(self, odom_pose, scan_local):
        """Run ICP correction against accumulated reference cloud.

        Parameters
        ----------
        odom_pose : tuple (x, y, θ) — current dead-reckoned pose
        scan_local : (N, 2) ndarray — current LiDAR scan in robot frame

        Returns
        -------
        corrected_pose : tuple (x, y, θ)
        info : dict with keys:
            applied (bool), converged (bool), mean_error (float),
            n_matched (int), correction (dx, dy, dθ),
            ref_cloud_size (int)
        """
        info = {
            "applied": False, "converged": False,
            "mean_error": float("inf"), "n_matched": 0,
            "correction": (0.0, 0.0, 0.0), "ref_cloud_size": 0,
        }

        if scan_local is None or scan_local.shape[0] < ICP_MIN_POINTS:
            self._maybe_init_ref(odom_pose, scan_local)
            return odom_pose, info

        # Transform current scan to world using odometry
        scan_world = self._local_to_world(scan_local, odom_pose)

        # First call — initialize reference cloud
        if self._ref_cloud is None:
            self._ref_cloud = _voxel_downsample_2d(scan_world, REF_VOXEL_SIZE)
            self._prev_odom_pose = odom_pose
            info["ref_cloud_size"] = self._ref_cloud.shape[0]
            return odom_pose, info

        # Run ICP: align current world-scan against reference
        R, t, converged, mean_err, n_matched = _icp_2d(
            scan_world, self._ref_cloud,
        )

        info["converged"] = converged
        info["mean_error"] = mean_err
        info["n_matched"] = n_matched
        info["ref_cloud_size"] = self._ref_cloud.shape[0]

        # Extract pose correction from ICP result
        dth = math.atan2(R[1, 0], R[0, 0])
        c, s = math.cos(dth), math.sin(dth)
        ox, oy = odom_pose[0], odom_pose[1]

        # The ICP result (R, t) aligns world-frame points to world-frame points.
        # The true corrected position is the transform applied to the robot's position:
        full_cx = c * ox - s * oy + float(t[0])
        full_cy = s * ox + c * oy + float(t[1])
        full_cth = _wrap(odom_pose[2] + dth)

        # Calculate the pure delta so we can apply the blend factor
        delta_x = full_cx - ox
        delta_y = full_cy - oy
        delta_th = dth

        # Safety gates: reject implausibly large corrections
        if (converged and mean_err < 0.12
                and abs(delta_x) < ICP_MAX_CORRECTION_M
                and abs(delta_y) < ICP_MAX_CORRECTION_M
                and abs(delta_th) < ICP_MAX_CORRECTION_RAD):

            # Apply blended correction
            cx = ox + delta_x * ICP_BLEND
            cy = oy + delta_y * ICP_BLEND
            cth = _wrap(odom_pose[2] + delta_th * ICP_BLEND)

            corrected = (cx, cy, cth)
            info["applied"] = True
            info["correction"] = (delta_x * ICP_BLEND, delta_y * ICP_BLEND, delta_th * ICP_BLEND)

            self._corrections_applied += 1
            self._total_drift_x += abs(delta_x * ICP_BLEND)
            self._total_drift_y += abs(delta_y * ICP_BLEND)
            self._total_drift_th += abs(delta_th * ICP_BLEND)

            # Accumulate the corrected scan into the reference cloud
            corrected_scan_world = self._local_to_world(scan_local, corrected)
            self._accumulate_ref(corrected_scan_world)
            self._prev_odom_pose = corrected
            return corrected, info

        # ICP didn't converge or correction was too large — still accumulate
        self._accumulate_ref(scan_world)
        self._prev_odom_pose = odom_pose
        return odom_pose, info

    def _maybe_init_ref(self, pose, scan_local):
        """Initialize reference on first call even if scan is small."""
        if self._ref_cloud is None and scan_local is not None and scan_local.shape[0] > 0:
            scan_world = self._local_to_world(scan_local, pose)
            self._ref_cloud = _voxel_downsample_2d(scan_world, REF_VOXEL_SIZE)
            self._prev_odom_pose = pose

    def _accumulate_ref(self, scan_world):
        """Add new world-frame points to the reference cloud with voxel downsampling."""
        combined = np.vstack([self._ref_cloud, scan_world])
        self._ref_cloud = _voxel_downsample_2d(combined, REF_VOXEL_SIZE)

        # Cap size to prevent unbounded growth
        if self._ref_cloud.shape[0] > MAX_REF_POINTS:
            # Keep the most recent points (they're more relevant)
            self._ref_cloud = self._ref_cloud[-MAX_REF_POINTS:]

    def drift_stats(self):
        """Return cumulative drift correction statistics."""
        return {
            "corrections": self._corrections_applied,
            "total_drift_x": self._total_drift_x,
            "total_drift_y": self._total_drift_y,
            "total_drift_th_deg": math.degrees(self._total_drift_th),
        }
