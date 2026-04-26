"""Final mapping export: clean 2D PNG + downsampled 3D PLY.

Kept dependency-light on purpose — no Open3D required.  Save a PLY and
open it in Open3D, MeshLab, CloudCompare, or Blender.
"""
import math
import os
import numpy as np

import config as C


def _voxel_downsample(points, voxel=0.04):
    """Keep one point per voxel of size `voxel` (meters)."""
    if points.shape[0] == 0:
        return points
    keys = np.floor(points / voxel).astype(np.int64)
    _, unique_idx = np.unique(keys, axis=0, return_index=True)
    return points[unique_idx]


def _height_to_rgb(z, z_lo=-0.05, z_hi=0.30):
    """Rainbow-ish colormap on height (blue low → red high)."""
    t = np.clip((z - z_lo) / max(z_hi - z_lo, 1e-6), 0.0, 1.0)
    r = np.clip(1.5 - np.abs(4 * t - 3), 0, 1)
    g = np.clip(1.5 - np.abs(4 * t - 2), 0, 1)
    b = np.clip(1.5 - np.abs(4 * t - 1), 0, 1)
    rgb = np.stack([r, g, b], axis=-1)
    return (rgb * 255.0).astype(np.uint8)


def export_ply(path, points_world, voxel=0.04):
    """Write a height-colored PLY. points_world: (N, 3) world-frame array.

    After voxel downsampling and with a ~5 m maze the cloud is <100k points,
    which opens instantly in Open3D / MeshLab.
    """
    if points_world is None or points_world.shape[0] == 0:
        print("[export] no 3D points to write")
        return
    pts = _voxel_downsample(points_world, voxel=voxel)
    colors = _height_to_rgb(pts[:, 2])
    os.makedirs(os.path.dirname(path) or ".", exist_ok=True)
    with open(path, "w") as f:
        f.write("ply\nformat ascii 1.0\n")
        f.write(f"element vertex {pts.shape[0]}\n")
        f.write("property float x\nproperty float y\nproperty float z\n")
        f.write("property uchar red\nproperty uchar green\nproperty uchar blue\n")
        f.write("end_header\n")
        for (x, y, z), (r, g, b) in zip(pts.tolist(), colors.tolist()):
            f.write(f"{x:.4f} {y:.4f} {z:.4f} {r} {g} {b}\n")
    print(f"[export] wrote {pts.shape[0]} points -> {path}")


def export_final_png(path, grid, pose_history=None, pillar_positions=None,
                     show_aux=True, show_poison=True, title=None):
    """Higher-quality final map.

    Layers (drawn back-to-front):
        - white     known free
        - light grey unknown
        - dark grey  aux-obstacle (depth-cam: floating / overhead walls)
        - black     occupied (lidar log-odds threshold)
        - light green poison patches
    Plus the trajectory in blue, start dot, and pillar markers.

    Falls back to a no-op on missing matplotlib so the controller doesn't
    crash on minimal Python installs."""
    try:
        import matplotlib
        matplotlib.use("Agg")
        import matplotlib.pyplot as plt
    except Exception as e:
        print("[export] matplotlib missing, skipping final PNG:", e)
        return

    p = grid.occupancy_prob()
    occ = (p > 0.65)
    unk = (np.abs(grid.L) < 0.05)
    aux = grid.aux_obstacle.astype(bool) if show_aux else np.zeros_like(occ)
    poi = grid.poison.astype(bool) if show_poison else np.zeros_like(occ)

    # cheap 3x3 morphological close to deblur the walls
    try:
        from scipy.ndimage import binary_closing
        occ = binary_closing(occ, iterations=1)
    except Exception:
        pass

    rgb = np.ones((grid.cells, grid.cells, 3), dtype=np.float32)
    rgb[unk] = (0.82, 0.82, 0.82)               # light grey unknown
    rgb[aux & ~occ] = (0.45, 0.45, 0.45)        # mid-grey aux
    rgb[occ] = (0.05, 0.05, 0.05)               # black occupied
    rgb[poi] = (0.55, 0.85, 0.55)               # green poison

    fig, ax = plt.subplots(figsize=(6, 6), dpi=160)
    extent = [
        grid.origin[0],
        grid.origin[0] + grid.cells * grid.res,
        grid.origin[1],
        grid.origin[1] + grid.cells * grid.res,
    ]
    ax.imshow(rgb.transpose(1, 0, 2), origin="lower", extent=extent)

    if pose_history is not None and len(pose_history) > 1:
        xs = [pp[0] for pp in pose_history]
        ys = [pp[1] for pp in pose_history]
        ax.plot(xs, ys, "-", color="#1f77b4", lw=1.2, label="trajectory")
        ax.plot([xs[0]], [ys[0]], "o", color="#1f77b4", markersize=6, label="start")

    if pillar_positions:
        marker_colors = {"blue": "#1f77b4", "yellow": "#f1c40f", "green": "#2ecc71"}
        for name, pos in pillar_positions.items():
            if pos is None:
                continue
            # Reject NaN sentinels coming from npz loads.
            if not (np.isfinite(pos[0]) and np.isfinite(pos[1])):
                continue
            ax.plot([pos[0]], [pos[1]], marker="*", markersize=18,
                    markerfacecolor=marker_colors.get(name, "r"),
                    markeredgecolor="k", label=f"{name} pillar")

    ax.set_title(title or "Final occupancy + trajectory + landmarks")
    ax.set_xlabel("x [m]")
    ax.set_ylabel("y [m]")
    ax.legend(loc="lower right", fontsize=8)
    ax.set_aspect("equal")
    ax.grid(True, color="0.85", lw=0.3)
    os.makedirs(os.path.dirname(path) or ".", exist_ok=True)
    fig.savefig(path, bbox_inches="tight")
    plt.close(fig)
    print(f"[export] wrote final map -> {path}")
