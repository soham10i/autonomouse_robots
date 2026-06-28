"""Visualisation + map export (matplotlib, optional at runtime).

Renders the SLAM occupancy grid with the live trajectory, current plan,
frontiers, goal, and pillar landmarks — the kind of frontier-exploration view
the reference videos show in RViz.  Also writes the binary ``map.npz`` so a
later mission run can warm-start from a finished map.

All functions degrade to a no-op (with a printed warning) if matplotlib is
unavailable, so the controller never crashes on a minimal install.
"""
from __future__ import annotations

import math
import os

import numpy as np

import settings as S


def _grid_to_rgb(grid):
    """White free, grey unknown, purple aux, black occupied, green poison."""
    cells = grid.cells
    rgb = np.ones((cells, cells, 3), dtype=np.float32)        # free = white
    rgb[grid.unknown_mask()] = (0.80, 0.80, 0.80)
    rgb[grid.depth_obs_mask() & ~grid.occupied_mask()] = (0.45, 0.20, 0.50)
    # cloud-projected floating walls (3D->2D) in teal, under the lidar walls
    rgb[grid.cloud_obs & ~grid.occupied_mask()] = (0.10, 0.55, 0.55)
    rgb[grid.occupied_mask()] = (0.05, 0.05, 0.05)
    rgb[grid.barrier] = (0.90, 0.35, 0.05)       # learned floating-wall barrier
    rgb[grid.poison] = (0.10, 0.65, 0.10)
    return rgb


def _extent(grid):
    return [grid.origin[0], grid.origin[0] + grid.cells * grid.res,
            grid.origin[1], grid.origin[1] + grid.cells * grid.res]


def save_snapshot(path, grid, pose=None, path_world=None, frontiers=None,
                  goal=None, pillars=None, state_text=None):
    try:
        import matplotlib
        matplotlib.use("Agg")
        import matplotlib.pyplot as plt
    except Exception as e:
        print("[viz] matplotlib unavailable:", e)
        return
    rgb = _grid_to_rgb(grid)
    fig, ax = plt.subplots(figsize=(6, 6), dpi=130)
    ax.imshow(rgb.transpose(1, 0, 2), origin="lower", extent=_extent(grid),
              interpolation="nearest")   # crisp cells, no antialias blur

    if path_world:
        xs = [p[0] for p in path_world]
        ys = [p[1] for p in path_world]
        ax.plot(xs, ys, "-", color="#1f77b4", lw=1.6)
    if frontiers:
        fx = [f.centroid_world[0] for f in frontiers]
        fy = [f.centroid_world[1] for f in frontiers]
        ax.plot(fx, fy, "x", color="#e67e22", markersize=7)
    if goal is not None:
        ax.plot([goal[0]], [goal[1]], "*", color="#f1c40f", markersize=15,
                markeredgecolor="k")
    if pillars:
        colors = {"blue": "#1f77b4", "yellow": "#f1c40f"}
        for name, pos in pillars.items():
            if pos is None:
                continue
            ax.plot([pos[0]], [pos[1]], "P", color=colors.get(name, "r"),
                    markersize=12, markeredgecolor="k")
    if pose is not None:
        x, y, th = pose
        ax.plot([x], [y], "r.", markersize=8)
        ax.plot([x, x + 0.25 * math.cos(th)], [y, y + 0.25 * math.sin(th)],
                "r-", lw=1.5)
    if state_text:
        ax.text(0.02, 0.98, state_text, transform=ax.transAxes, fontsize=9,
                va="top", ha="left",
                bbox=dict(boxstyle="round", facecolor="white", alpha=0.75))
    ax.set_xlabel("x [m]")
    ax.set_ylabel("y [m]")
    ax.set_aspect("equal")
    os.makedirs(os.path.dirname(path) or ".", exist_ok=True)
    fig.savefig(path, bbox_inches="tight")
    plt.close(fig)


def save_final_map(path, grid, pose_history=None, pillars=None):
    try:
        import matplotlib
        matplotlib.use("Agg")
        import matplotlib.pyplot as plt
    except Exception as e:
        print("[viz] matplotlib unavailable:", e)
        return
    rgb = _grid_to_rgb(grid)
    fig, ax = plt.subplots(figsize=(6, 6), dpi=160)
    ax.imshow(rgb.transpose(1, 0, 2), origin="lower", extent=_extent(grid),
              interpolation="nearest")
    if pose_history and len(pose_history) > 1:
        xs = [p[0] for p in pose_history]
        ys = [p[1] for p in pose_history]
        ax.plot(xs, ys, "-", color="#1f77b4", lw=1.2, label="trajectory")
        ax.plot([xs[0]], [ys[0]], "o", color="#1f77b4", markersize=6, label="start")
    if pillars:
        colors = {"blue": "#1f77b4", "yellow": "#f1c40f"}
        for name, pos in pillars.items():
            if pos is None:
                continue
            ax.plot([pos[0]], [pos[1]], "*", color=colors.get(name, "r"),
                    markersize=18, markeredgecolor="k", label=f"{name} pillar")
    ax.set_title("mak_02 SLAM map + trajectory + landmarks")
    ax.set_xlabel("x [m]")
    ax.set_ylabel("y [m]")
    ax.legend(loc="lower right", fontsize=8)
    ax.set_aspect("equal")
    os.makedirs(os.path.dirname(path) or ".", exist_ok=True)
    fig.savefig(path, bbox_inches="tight")
    plt.close(fig)
    print("[viz] wrote final map ->", path)


def _height_to_rgb(z, z_lo=-0.05, z_hi=0.40):
    """Blue (low) -> red (high) colormap on height, for the 3D cloud."""
    t = np.clip((z - z_lo) / max(z_hi - z_lo, 1e-6), 0.0, 1.0)
    r = np.clip(1.5 - np.abs(4 * t - 3), 0, 1)
    g = np.clip(1.5 - np.abs(4 * t - 2), 0, 1)
    b = np.clip(1.5 - np.abs(4 * t - 1), 0, 1)
    return np.stack([r, g, b], axis=-1)


def save_ply(path, cloud, voxel=None):
    """Write the accumulated 3D voxel cloud as a height-coloured ASCII PLY.

    ``cloud`` is a ``CloudMap``; opens in MeshLab / CloudCompare / Open3D.
    """
    try:
        pts, _ = cloud.points()
    except Exception as e:
        print("[viz] ply: bad cloud:", e)
        return
    if pts.shape[0] == 0:
        print("[viz] ply: no 3D points to write")
        return
    colors = (_height_to_rgb(pts[:, 2]) * 255.0).astype(np.uint8)
    os.makedirs(os.path.dirname(path) or ".", exist_ok=True)
    with open(path, "w") as f:
        f.write("ply\nformat ascii 1.0\n")
        f.write(f"element vertex {pts.shape[0]}\n")
        f.write("property float x\nproperty float y\nproperty float z\n")
        f.write("property uchar red\nproperty uchar green\nproperty uchar blue\n")
        f.write("end_header\n")
        for (x, y, z), (r, g, b) in zip(pts.tolist(), colors.tolist()):
            f.write(f"{x:.4f} {y:.4f} {z:.4f} {r} {g} {b}\n")
    print(f"[viz] wrote {pts.shape[0]} cloud points -> {path}", flush=True)


def save_cloud_3d_png(path, cloud, pose_history=None, pillars=None, title=None):
    """3D scatter render of the voxel cloud (matplotlib), height-coloured, with
    the trajectory and pillar markers — the 3D map view like teleop_mapping."""
    try:
        import matplotlib
        matplotlib.use("Agg")
        import matplotlib.pyplot as plt
        from mpl_toolkits.mplot3d import Axes3D  # noqa: F401 (registers 3d)
    except Exception as e:
        print("[viz] matplotlib/3d unavailable:", e)
        return
    pts, cnt = cloud.points()
    if pts.shape[0] == 0:
        print("[viz] 3d: no cloud points")
        return
    cols = _height_to_rgb(pts[:, 2])
    fig = plt.figure(figsize=(8, 6), dpi=150)
    ax = fig.add_subplot(111, projection="3d")
    ax.scatter(pts[:, 0], pts[:, 1], pts[:, 2], c=cols, s=2, marker="s",
               depthshade=False)
    if pose_history and len(pose_history) > 1:
        xs = [p[0] for p in pose_history]
        ys = [p[1] for p in pose_history]
        ax.plot(xs, ys, [0.0] * len(xs), "-", color="#1f77b4", lw=1.4)
    if pillars:
        pc = {"blue": "#1f77b4", "yellow": "#f1c40f"}
        for name, pos in (pillars or {}).items():
            if pos is None:
                continue
            ax.scatter([pos[0]], [pos[1]], [0.15], marker="*", s=180,
                       color=pc.get(name, "r"), edgecolor="k")
    ax.set_xlabel("x [m]")
    ax.set_ylabel("y [m]")
    ax.set_zlabel("z [m]")
    ax.set_zlim(0.0, max(0.4, float(pts[:, 2].max())))
    ax.set_title(title or "mak_02 3D voxel cloud")
    try:
        ax.set_box_aspect((1, 1, 0.35))
    except Exception:
        pass
    os.makedirs(os.path.dirname(path) or ".", exist_ok=True)
    fig.savefig(path, bbox_inches="tight")
    plt.close(fig)
    print(f"[viz] wrote 3D cloud render ({pts.shape[0]} pts) -> {path}", flush=True)


def save_npz(path, grid, pose_history=None, pillars=None, start_pose=None):
    """Persist the map for warm-starting a later mission run."""
    def _pos(p):
        return (np.array(p, dtype=np.float32) if p is not None
                else np.array([np.nan, np.nan], dtype=np.float32))
    os.makedirs(os.path.dirname(path) or ".", exist_ok=True)
    pillars = pillars or {}
    np.savez_compressed(
        path,
        occupied=grid.occupied_mask(),
        free=grid.free_mask(),
        unknown=grid.unknown_mask(),
        depth_obs=grid.depth_obs_mask(),
        poison=grid.poison,
        barrier=grid.barrier,
        log_odds=grid.L,
        resolution=np.float32(grid.res),
        origin=np.asarray(grid.origin, dtype=np.float32),
        cells=np.int32(grid.cells),
        pose_history=np.asarray(pose_history or [], dtype=np.float32),
        start_pose=np.asarray(start_pose if start_pose is not None
                              else [np.nan, np.nan, np.nan], dtype=np.float32),
        pillar_blue=_pos(pillars.get("blue")),
        pillar_yellow=_pos(pillars.get("yellow")),
    )
    print("[viz] wrote map.npz ->", path)
