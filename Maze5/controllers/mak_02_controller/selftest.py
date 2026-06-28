"""Webots-free self-test of the mak_02 Maze5 navigation pipeline.

Exercises mapping -> scan-match -> costmap -> A* -> frontier -> DWA -> perception
on synthetic data and asserts sane behaviour.  Run with any Python that has
NumPy:

    python3 selftest.py
"""
from __future__ import annotations

import math
import os
import sys

import numpy as np

sys.path.insert(0, os.path.dirname(os.path.abspath(__file__)))

import config as C
import astar
import frontier as FR
import local_planner as LP
from mapping import LidarModel, OccupancyGrid
from perception import Perception

PASS, FAIL = "PASS", "FAIL"
_fails = []


def check(name, cond):
    print(f"  [{PASS if cond else FAIL}] {name}")
    if not cond:
        _fails.append(name)


def _room_ranges(lm, half=1.7, gap=(0.6, 1.2)):
    out = []
    for a in lm.angles:
        if gap[0] < a < gap[1]:
            out.append(11.5)
            continue
        c, s = math.cos(a), math.sin(a)
        ts = []
        for n_, off in ((c, half), (c, -half), (s, half), (s, -half)):
            if abs(n_) > 1e-6:
                t = off / n_
                if t > 0:
                    x, y = t * c, t * s
                    if -half - 1e-6 <= x <= half + 1e-6 and -half - 1e-6 <= y <= half + 1e-6:
                        ts.append(t)
        out.append(min(ts) if ts else 11.5)
    return np.array(out, dtype=np.float32)


def main():
    print("mak_02 selftest")
    lm = LidarModel(400, 2 * math.pi, 0.2, 12.0)
    grid = OccupancyGrid()
    pose = (0.0, 0.0, 0.0)
    for _ in range(4):
        r = _room_ranges(lm)
        pts, rr = lm.ranges_to_body(r)
        grid.integrate_scan(pose, pts, rr)
    check("occupancy has walls", grid.occupied_mask().sum() > 100)
    check("occupancy has free space", grid.free_mask().sum() > 500)

    corr, hit = grid.scan_match((0.05, -0.04, 0.0), pts)
    moved = math.hypot(corr[0], corr[1])
    check("scan-match correction is bounded", moved <= C.SM_LIN_HALF + 1e-6)

    # poison patch + costmap
    pp = np.array([[0.5 + 0.02 * i, 0.02 * j] for i in range(-3, 4) for j in range(-3, 4)])
    grid.add_poison_points(pp)
    cost, lethal = grid.build_costmap()
    check("poison created lethal cells", lethal.sum() > 0)
    check("poison cell is lethal", lethal[grid.world_to_grid(0.5, 0.0)])

    # A* to a free goal, avoiding poison
    start = grid.world_to_grid(0.0, 0.0)
    goal = grid.world_to_grid(-1.2, 0.9)
    cells = astar.plan(cost, lethal, start, goal)
    check("A* found a path", cells is not None and len(cells) > 1)
    if cells:
        simp = astar.simplify(cells, lethal)
        check("path simplification shortens", len(simp) <= len(cells))
        crosses = any(lethal[ix, iy] for ix, iy in cells)
        check("A* path avoids lethal cells", not crosses)

    # frontiers exist at the gap
    cands = FR.find_frontiers(grid, lethal, (0.0, 0.0), (0.0, 0.0))
    check("frontiers detected at the opening", len(cands) >= 1)

    # DWA: open space straight, smoothness bounded
    d = LP.DWAPlanner(0.032)
    EMPTY = np.empty((0, 2))
    v = w = 0.0
    for _ in range(40):
        v, w = d.compute((0, 0, 0), (2.0, 0.0), EMPTY, EMPTY)
    check("DWA drives forward in open space", v > 0.3 and abs(w) < 0.1)
    d = LP.DWAPlanner(0.032)
    prev = (0.0, 0.0)
    mdw = 0.0
    for _ in range(30):
        v, w = d.compute((0, 0, 0), (0.0, 2.0), EMPTY, EMPTY)
        mdw = max(mdw, abs(w - prev[1]))
        prev = (v, w)
    check("DWA angular rate is jerk-limited", mdw <= C.A_W * 0.032 + 1e-6)
    v, w = LP.DWAPlanner(0.032).compute((0, 0, 0), (1.0, 0.0), EMPTY, EMPTY, green_block=True)
    check("green_block forbids forward motion", v <= 0.0)

    # perception: blue pillar + green floor
    per = Perception(640, 480, 1.04)
    img = np.zeros((480, 640, 3), dtype=np.uint8)
    img[150:330, 300:340] = (255, 30, 0)      # BGR blue blob
    img[360:480, 250:400] = (0, 200, 0)       # green floor strip
    dets = per.update_pillars(img, None, (0.0, 0.0, 0.0))
    check("blue pillar detected", dets["blue"] is not None)
    check("blue pillar world position set", per.pillar_world["blue"] is not None)
    gw = per.green_floor_world(img, (0.0, 0.0, 0.0))
    check("green floor projected to world points", gw.shape[0] > 0)
    check("green reflex fires on poison ahead", per.green_reflex(img))

    print()
    if _fails:
        print(f"FAILED ({len(_fails)}): {_fails}")
        sys.exit(1)
    print("ALL CHECKS PASSED")


if __name__ == "__main__":
    main()
