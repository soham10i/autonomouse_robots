"""Debug visualisation helpers — kept light so they can run every loop.

* ``dump_grid`` — full annotated occupancy / poison / aux PNG.
* ``dump_costmap`` — heatmap of the planner cost (helps debug stuck plans).
* ``dump_camera_masks`` — colour mask debug images for blue/yellow/green.
"""
import os
import math
import numpy as np


def dump_grid(path, grid, pose=None, path_world=None, frontier=None,
              standoff_world=None, state_text=None):
    """Thin wrapper around ``grid.save_png`` so callers don't import grid_2d."""
    grid.save_png(
        path, pose=pose, path_world=path_world,
        frontier=frontier, standoff_world=standoff_world,
        state_text=state_text,
    )


def dump_costmap(path, grid, cost):
    try:
        import matplotlib
        matplotlib.use("Agg")
        import matplotlib.pyplot as plt
    except Exception:
        return
    finite = np.where(np.isfinite(cost), cost, np.nan)
    finite_max = np.nanmax(finite) if np.any(np.isfinite(cost)) else 1.0
    img = np.where(np.isfinite(cost), cost, finite_max + 5.0)

    fig, ax = plt.subplots(figsize=(5, 5), dpi=120)
    extent = [
        grid.origin[0], grid.origin[0] + grid.cells * grid.res,
        grid.origin[1], grid.origin[1] + grid.cells * grid.res,
    ]
    ax.imshow(img.T, origin="lower", cmap="magma", extent=extent)
    ax.set_title("planner costmap (purple = inf / lethal)")
    ax.set_xlabel("x [m]")
    ax.set_ylabel("y [m]")
    ax.set_aspect("equal")
    os.makedirs(os.path.dirname(path) or ".", exist_ok=True)
    fig.savefig(path, bbox_inches="tight")
    plt.close(fig)


def dump_camera_masks(path_prefix, perception, robot_io):
    """Save HSV masks for blue/yellow/green to disk for tuning."""
    if perception is None or robot_io is None or robot_io.camera is None:
        return
    try:
        import matplotlib
        matplotlib.use("Agg")
        import matplotlib.pyplot as plt
    except Exception:
        return
    raw = robot_io.camera.getImage()
    if raw is None:
        return
    h = robot_io.camera.getHeight()
    w = robot_io.camera.getWidth()
    img = np.frombuffer(raw, dtype=np.uint8).reshape(h, w, 4)[:, :, :3]
    rgb = img[:, :, ::-1]   # BGR → RGB

    try:
        import cv2
        hsv = cv2.cvtColor(img, cv2.COLOR_BGR2HSV)
        from config import HSV_BLUE, HSV_YELLOW, HSV_GREEN
        masks = [
            ("blue", cv2.inRange(hsv, np.array(HSV_BLUE[0], dtype=np.uint8),
                                 np.array(HSV_BLUE[1], dtype=np.uint8))),
            ("yellow", cv2.inRange(hsv, np.array(HSV_YELLOW[0], dtype=np.uint8),
                                   np.array(HSV_YELLOW[1], dtype=np.uint8))),
            ("green", cv2.inRange(hsv, np.array(HSV_GREEN[0], dtype=np.uint8),
                                  np.array(HSV_GREEN[1], dtype=np.uint8))),
        ]
    except Exception:
        return

    fig, axes = plt.subplots(1, 4, figsize=(12, 3), dpi=110)
    axes[0].imshow(rgb); axes[0].set_title("RGB"); axes[0].axis("off")
    for ax, (name, m) in zip(axes[1:], masks):
        ax.imshow(m, cmap="gray"); ax.set_title(name); ax.axis("off")
    os.makedirs(os.path.dirname(path_prefix) or ".", exist_ok=True)
    fig.savefig(path_prefix + "_masks.png", bbox_inches="tight")
    plt.close(fig)
