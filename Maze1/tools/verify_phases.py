"""Offline phase verifier.

Each Webots phase writes its outputs to disk; this tool inspects them
without launching Webots so you can iterate fast.

Usage
-----
    python3 verify_phases.py phase1
        Load the teleop_mapping outputs (map.npz + scene.ply) and report:
          - pillar positions
          - poison cell count + bounding box
          - free / occupied / unknown coverage
          - PLY wall point bounding box (used by Phase 3)
        Saves an annotated PNG: phase1_check.png

    python3 verify_phases.py phase3
        Run the same A* the map_runner would run (offline) and save
        phase3_plan.png with start, goals, walls, and the two paths.

    python3 verify_phases.py all
        Run both checks back-to-back.

Run from the project root or from anywhere — paths are resolved relative
to this file's location.
"""
import os
import sys
import math
import argparse

import numpy as np

_HERE = os.path.dirname(os.path.abspath(__file__))
_REPO = os.path.normpath(os.path.join(_HERE, "..", ".."))
TELEOP_MAPS = os.path.normpath(os.path.join(
    _HERE, "..", "controllers", "teleop_mapping", "maps"))
RUNNER_DIR = os.path.normpath(os.path.join(
    _HERE, "..", "controllers", "map_runner"))
NAV_DIR = os.path.normpath(os.path.join(
    _HERE, "..", "controllers", "maze_navigator"))


def _import_runner():
    """Import map_runner without executing its main()."""
    sys.path.insert(0, RUNNER_DIR)
    sys.path.insert(0, NAV_DIR)
    import importlib.util
    spec = importlib.util.spec_from_file_location(
        "map_runner_mod", os.path.join(RUNNER_DIR, "map_runner.py"))
    mod = importlib.util.module_from_spec(spec)
    # Stub robot_io / odometry so map_runner imports don't try to talk to
    # Webots. We only need the planning utilities.
    sys.modules.setdefault("controller", _make_controller_stub())
    spec.loader.exec_module(mod)
    return mod


def _make_controller_stub():
    """Minimal stub of webots' `controller` module. Only used so the
    map_runner top-level imports succeed; we never instantiate Robot."""
    import types
    m = types.ModuleType("controller")

    class _Stub:
        def __init__(self, *a, **kw):
            pass

        def __getattr__(self, name):
            return _Stub()

        def __call__(self, *a, **kw):
            return _Stub()
    for name in ("Robot", "Keyboard", "Camera", "RangeFinder", "Lidar",
                 "Motor", "PositionSensor", "InertialUnit", "Gyro",
                 "Accelerometer", "Compass", "DistanceSensor"):
        setattr(m, name, _Stub)
    return m


# ---------------------------------------------------------------------------
# Phase 1: inspect map.npz + scene.ply
# ---------------------------------------------------------------------------

def phase1():
    npz_path = os.path.join(TELEOP_MAPS, "map.npz")
    ply_path = os.path.join(TELEOP_MAPS, "scene.ply")
    if not os.path.isfile(npz_path):
        print(f"[phase1] FAIL: {npz_path} not found. Run teleop_mapping first.")
        return False
    if not os.path.isfile(ply_path):
        print(f"[phase1] WARN: {ply_path} not found (Phase 3 will fail).")

    data = np.load(npz_path, allow_pickle=False)
    occ = data["occupied"]
    free = data["free"]
    unk = data["unknown"]
    aux = data["aux_obstacle"]
    poison = data["poison"]
    res = float(data["resolution"])
    origin = data["origin"]
    cells = int(data["cells"])
    blue = data["pillar_blue"]
    yellow = data["pillar_yellow"]
    poses = data["pose_history"]

    print("=" * 60)
    print("PHASE 1 — teleop_mapping output check")
    print("=" * 60)
    total = cells * cells
    print(f"Grid: {cells}x{cells} @ {res}m  origin={tuple(origin)}")
    print(f"  occupied : {int(occ.sum()):6d} cells  ({100*occ.sum()/total:5.1f}%)")
    print(f"  free     : {int(free.sum()):6d} cells  ({100*free.sum()/total:5.1f}%)")
    print(f"  unknown  : {int(unk.sum()):6d} cells  ({100*unk.sum()/total:5.1f}%)")
    print(f"  aux_obs  : {int(aux.sum()):6d} cells")
    print(f"  poison   : {int(poison.sum()):6d} cells "
          f"({100*poison.sum()/total:5.2f}%)")

    # Poison sanity check — a real green tile is ~50x50cm (~150 cells).
    # Anything > 800 cells means the centroid+disc bug is back.
    poison_count = int(poison.sum())
    if poison_count > 800:
        print(f"  [WARN] poison footprint is suspiciously large "
              f"({poison_count} cells). Per-pixel projection should "
              f"give roughly 100-400 cells for one ~50cm patch.")
    elif poison_count == 0:
        print(f"  [WARN] no poison cells — green floor never detected.")
    else:
        print(f"  [OK]   poison footprint reasonable.")

    print(f"\nPillars:")
    print(f"  blue   = {tuple(blue)}  "
          f"{'(NOT detected)' if not np.isfinite(blue[0]) else ''}")
    print(f"  yellow = {tuple(yellow)}  "
          f"{'(NOT detected)' if not np.isfinite(yellow[0]) else ''}")

    print(f"\nTrajectory: {len(poses)} samples, "
          f"x∈[{poses[:,0].min():+.2f}, {poses[:,0].max():+.2f}], "
          f"y∈[{poses[:,1].min():+.2f}, {poses[:,1].max():+.2f}]")

    if os.path.isfile(ply_path):
        with open(ply_path) as f:
            lines = f.readlines()
        h = next(i for i, l in enumerate(lines) if l.strip() == "end_header") + 1
        pts = []
        for l in lines[h:]:
            p = l.split()
            if len(p) >= 3:
                pts.append((float(p[0]), float(p[1]), float(p[2])))
        pts = np.array(pts)
        wall = pts[(pts[:, 2] > 0.03) & (pts[:, 2] < 0.55)]
        print(f"\nPLY: {len(pts)} total, {len(wall)} wall-height points")
        print(f"  X∈[{pts[:,0].min():+.2f}, {pts[:,0].max():+.2f}]")
        print(f"  Y∈[{pts[:,1].min():+.2f}, {pts[:,1].max():+.2f}]")
        print(f"  Z∈[{pts[:,2].min():+.2f}, {pts[:,2].max():+.2f}]")

    # Save annotated visualization
    try:
        import matplotlib
        matplotlib.use("Agg")
        import matplotlib.pyplot as plt
        fig, ax = plt.subplots(figsize=(8, 8), dpi=140)
        rgb = np.ones((cells, cells, 3))
        rgb[unk] = (0.78, 0.78, 0.78)
        rgb[occ] = (0.05, 0.05, 0.05)
        rgb[aux & ~occ] = (0.45, 0.20, 0.50)
        rgb[poison] = (0.05, 0.65, 0.05)
        extent = [origin[0], origin[0] + cells * res,
                  origin[1], origin[1] + cells * res]
        ax.imshow(rgb.transpose(1, 0, 2), origin="lower", extent=extent)
        ax.plot(poses[:, 0], poses[:, 1], "-", color="#1f77b4", lw=1.0,
                alpha=0.7, label="trajectory")
        if np.isfinite(blue[0]):
            ax.plot(blue[0], blue[1], "*", color="dodgerblue", ms=22,
                    markeredgecolor="black", label=f"blue {tuple(np.round(blue,2))}")
        if np.isfinite(yellow[0]):
            ax.plot(yellow[0], yellow[1], "*", color="gold", ms=22,
                    markeredgecolor="black", label=f"yellow {tuple(np.round(yellow,2))}")
        if len(poses):
            ax.plot(poses[0, 0], poses[0, 1], "o", color="limegreen", ms=14,
                    markeredgecolor="black", label="start")
        ax.set_aspect("equal")
        ax.set_xlabel("x [m]")
        ax.set_ylabel("y [m]")
        ax.set_title(f"Phase 1 verification — poison={poison_count} cells")
        ax.legend(loc="upper left", fontsize=9)
        out = os.path.join(TELEOP_MAPS, "phase1_check.png")
        fig.savefig(out, bbox_inches="tight")
        plt.close(fig)
        print(f"\n[phase1] saved: {out}")
    except Exception as e:
        print(f"[phase1] viz failed: {e}")
    return True


# ---------------------------------------------------------------------------
# Phase 3: simulate the map_runner plan offline
# ---------------------------------------------------------------------------

def phase3():
    print("=" * 60)
    print("PHASE 3 — map_runner plan check (offline)")
    print("=" * 60)
    runner = _import_runner()

    ply_path = os.path.join(TELEOP_MAPS, "scene.ply")
    npz_path = os.path.join(TELEOP_MAPS, "map.npz")
    if not os.path.isfile(ply_path):
        print(f"[phase3] FAIL: {ply_path} missing.")
        return False

    all_pts = runner.load_ply(ply_path)
    wall_pts = runner.extract_walls(all_pts)
    print(f"PLY: {len(all_pts)} total, {len(wall_pts)} wall-height points")

    poison_g = None
    if os.path.isfile(npz_path):
        npz = np.load(npz_path, allow_pickle=False)
        blue = (float(npz["pillar_blue"][0]), float(npz["pillar_blue"][1]))
        yellow = (float(npz["pillar_yellow"][0]), float(npz["pillar_yellow"][1]))
        if "poison" in npz.files:
            psrc = npz["poison"].astype(bool)
            sr = float(npz["resolution"])
            sox, soy = float(npz["origin"][0]), float(npz["origin"][1])
            sn = int(npz["cells"])
            nx_g = int((runner.X_MAX - runner.X_MIN) / runner.RES)
            ny_g = int((runner.Y_MAX - runner.Y_MIN) / runner.RES)
            poison_g = np.zeros((nx_g, ny_g), dtype=bool)
            si, sj = np.where(psrc)
            for a, b in zip(si.tolist(), sj.tolist()):
                wx = sox + (a + 0.5) * sr
                wy = soy + (b + 0.5) * sr
                ix = int((wx - runner.X_MIN) / runner.RES)
                iy = int((wy - runner.Y_MIN) / runner.RES)
                if 0 <= ix < nx_g and 0 <= iy < ny_g:
                    poison_g[ix, iy] = True
    else:
        blue = (2.456, 0.443)
        yellow = (0.779, -0.851)

    # Robot start: assume world (0, 0) since odometry zero-points there.
    rx, ry, rth = 0.0, 0.0, 0.0
    print(f"start  = ({rx:+.2f}, {ry:+.2f})")
    print(f"blue   = {blue}")
    print(f"yellow = {yellow}")

    grid = runner.Grid(wall_pts, poison_g)
    grid.inflate()
    grid.carve(rx, ry)
    grid.carve(blue[0], blue[1])
    grid.carve(yellow[0], yellow[1])

    pb = runner.astar(grid, (rx, ry), blue)
    if pb is None:
        print("[phase3] FAIL: A* could not find START -> BLUE")
        return False
    pb = runner.simplify(pb)
    pb_len = sum(math.hypot(b[0]-a[0], b[1]-a[1]) for a, b in zip(pb[:-1], pb[1:]))
    print(f"START -> BLUE  : {len(pb)} waypoints, {pb_len:.2f}m")

    # Adaptive inflation step-down for leg 2 (mirrors map_runner).
    direct = math.hypot(blue[0]-yellow[0], blue[1]-yellow[1])
    py = None
    chosen_inf = None
    for inf_r in [runner.INFLATE, 2, 1]:
        g2 = runner.Grid(wall_pts, poison_g)
        g2.inflate(r=inf_r)
        g2.carve(blue[0], blue[1])
        g2.carve(yellow[0], yellow[1])
        cand = runner.astar(g2, blue, yellow)
        if cand is None:
            continue
        cand = runner.simplify(cand)
        cl = sum(math.hypot(b[0]-a[0], b[1]-a[1])
                 for a, b in zip(cand[:-1], cand[1:]))
        if cl > direct * 2.5:
            continue
        py = cand
        chosen_inf = inf_r
        break

    if py is None:
        print("[phase3] FAIL: A* could not find BLUE -> YELLOW at any inflation")
    else:
        py_len = sum(math.hypot(b[0]-a[0], b[1]-a[1])
                     for a, b in zip(py[:-1], py[1:]))
        print(f"BLUE  -> YELLOW: {len(py)} waypoints, {py_len:.2f}m "
              f"(inflate={chosen_inf})")

    # Save plan visualization
    out = os.path.join(RUNNER_DIR, "maps", "phase3_plan.png")
    os.makedirs(os.path.dirname(out), exist_ok=True)
    runner.save_map(wall_pts, (rx, ry, rth), blue, yellow, pb, py, out)
    return py is not None


def main():
    p = argparse.ArgumentParser()
    p.add_argument("phase", choices=["phase1", "phase3", "all"])
    args = p.parse_args()
    ok = True
    if args.phase in ("phase1", "all"):
        ok &= phase1()
    if args.phase in ("phase3", "all"):
        ok &= phase3()
    sys.exit(0 if ok else 1)


if __name__ == "__main__":
    main()
