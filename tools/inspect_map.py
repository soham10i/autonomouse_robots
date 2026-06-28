"""Sanity-check a teleop ``map.npz`` outside Webots.

Usage:
    python tools/inspect_map.py Maze1/controllers/teleop_mapping/maps/map.npz

Prints summary statistics and writes ``map_inspect.png`` next to the input
file. Useful before Phase 3: confirms that the binary layers, world origin,
resolution, and pillar positions all line up before A* consumes them.
"""
from __future__ import annotations

import os
import sys

import numpy as np


def _print_summary(npz, path):
    occ = npz["occupied"]
    free = npz["free"]
    unk = npz["unknown"]
    aux = npz["aux_obstacle"]
    poi = npz["poison"]
    res = float(npz["resolution"])
    origin = npz["origin"]
    cells = int(npz["cells"])
    pose_history = npz["pose_history"]

    print(f"path                : {path}")
    print(f"resolution          : {res:.4f} m/cell")
    print(f"origin (world x,y)  : ({origin[0]:+.3f}, {origin[1]:+.3f}) m")
    print(f"grid                : {cells} x {cells} cells "
          f"({cells * res:.2f} x {cells * res:.2f} m)")
    print(f"occupied cells      : {int(occ.sum()):>6}")
    print(f"free cells          : {int(free.sum()):>6}")
    print(f"unknown cells       : {int(unk.sum()):>6}")
    print(f"aux obstacle cells  : {int(aux.sum()):>6}")
    print(f"poison cells        : {int(poi.sum()):>6}")
    print(f"pose history points : {len(pose_history)}")
    def _pillar_line(prefix, key, conf_key):
        if key not in npz.files:
            return
        p = npz[key]
        if not np.isfinite(p[0]):
            print(f"{prefix:20s}: NOT SEEN")
            return
        confirmed = bool(npz[conf_key]) if conf_key in npz.files else False
        tag = "CONFIRMED" if confirmed else "seen-only (never reached)"
        print(f"{prefix:20s}: ({p[0]:+.2f}, {p[1]:+.2f})  [{tag}]")

    _pillar_line("blue pillar", "pillar_blue", "pillar_blue_confirmed")
    _pillar_line("yellow pillar", "pillar_yellow", "pillar_yellow_confirmed")


def _render(npz, png_path):
    try:
        import matplotlib
        matplotlib.use("Agg")
        import matplotlib.pyplot as plt
    except Exception as e:
        print(f"[inspect] matplotlib not available, skipping render: {e}")
        return

    occ = npz["occupied"]
    aux = npz["aux_obstacle"]
    unk = npz["unknown"]
    poi = npz["poison"]
    res = float(npz["resolution"])
    origin = npz["origin"]
    cells = int(npz["cells"])
    pose_history = npz["pose_history"]

    rgb = np.ones((cells, cells, 3), dtype=np.float32)
    rgb[unk] = (0.82, 0.82, 0.82)
    rgb[aux & ~occ] = (0.45, 0.45, 0.45)
    rgb[occ] = (0.05, 0.05, 0.05)
    rgb[poi] = (0.55, 0.85, 0.55)

    extent = [origin[0], origin[0] + cells * res, origin[1], origin[1] + cells * res]
    fig, ax = plt.subplots(figsize=(6, 6), dpi=140)
    ax.imshow(rgb.transpose(1, 0, 2), origin="lower", extent=extent)

    if pose_history.size > 0:
        ax.plot(pose_history[:, 0], pose_history[:, 1], "-",
                color="#1f77b4", lw=1.0, label="trajectory")
        ax.plot([pose_history[0, 0]], [pose_history[0, 1]], "o",
                color="#1f77b4", markersize=6, label="start")

    if "pillar_blue" in npz.files and np.isfinite(npz["pillar_blue"][0]):
        b = npz["pillar_blue"]
        ax.plot(b[0], b[1], "*", markersize=18, markerfacecolor="#1f77b4",
                markeredgecolor="k", label="blue pillar")
    if "pillar_yellow" in npz.files and np.isfinite(npz["pillar_yellow"][0]):
        y = npz["pillar_yellow"]
        ax.plot(y[0], y[1], "*", markersize=18, markerfacecolor="#f1c40f",
                markeredgecolor="k", label="yellow pillar")

    ax.set_title("map.npz inspection")
    ax.set_xlabel("x [m]")
    ax.set_ylabel("y [m]")
    ax.legend(loc="lower right", fontsize=8)
    ax.set_aspect("equal")
    ax.grid(True, color="0.85", lw=0.3)
    fig.savefig(png_path, bbox_inches="tight")
    plt.close(fig)
    print(f"[inspect] wrote {png_path}")


def main(argv):
    if len(argv) < 2:
        print(__doc__)
        sys.exit(1)
    path = argv[1]
    if not os.path.isfile(path):
        print(f"[inspect] no such file: {path}")
        sys.exit(2)
    npz = np.load(path)
    _print_summary(npz, path)
    out_png = os.path.join(os.path.dirname(path) or ".", "map_inspect.png")
    _render(npz, out_png)


if __name__ == "__main__":
    main(sys.argv)
