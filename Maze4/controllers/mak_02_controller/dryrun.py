"""Webots-free closed-loop validation of the carrot + DWA driver.

The self-test (``selftest.py``) checks each module in isolation on a single
frame.  This harness closes the loop: it builds synthetic corridors, generates a
realistic body-frame lidar cloud from their walls every tick, runs the REAL
``choose_carrot`` + ``DWAPlanner.compute`` pipeline, integrates the returned
``(v, w)`` with the control timestep, and measures what the robot actually does
over hundreds of ticks.

This is the "orchestration loop" that lets the planner be tuned without the
simulator: it directly encodes the bug the user reported -- "the bot doesn't
move ahead and followed only few centimetre" -- as a hard assertion on AVERAGE
SPEED and DISTANCE TRAVELLED.  With the old multiplicative heading x front x
side slow-down caps this fails (the robot crawls); with the Maze1-proven
clearance-rejection policy it passes.

Run:  python3 dryrun.py
"""
from __future__ import annotations

import math
import os
import sys
from typing import Any, Dict, List, Optional, Tuple

import numpy as np

sys.path.insert(0, os.path.dirname(os.path.abspath(__file__)))

import config as C
import local_planner as LP
from geometry import inverse_transform_points


# --------------------------------------------------------------------------- #
#  Synthetic corridor world (walls as dense point clouds)
# --------------------------------------------------------------------------- #
def corridor_walls(centerline: List[Tuple[float, float]], half_width: float, spacing: float = 0.02) -> np.ndarray:
    """World wall points (N,2) for a corridor of ``half_width`` around a polyline.

    For each densely-sampled point on the centreline, drop one wall point on
    each side along the local normal.  This approximates the two corridor walls
    (including an L-bend) the way a real lidar would see them, minus occlusion
    -- which is irrelevant for the local planner's nearest-point clearance test.

    Args:
        centerline (List[Tuple[float, float]]): The path polyline coordinates.
        half_width (float): The lateral distance from the centerline to the walls.
        spacing (float, optional): Distance between sampled wall points. Defaults to 0.02.

    Returns:
        np.ndarray: An (N, 2) array of wall coordinates.
    """
    pts = []
    cl = np.asarray(centerline, dtype=float)
    for a, b in zip(cl[:-1], cl[1:]):
        seg = b - a
        L = float(np.hypot(*seg))
        if L < 1e-9:
            continue
        t = seg / L
        n = np.array([-t[1], t[0]])           # left normal
        k = max(2, int(L / spacing))
        for s in np.linspace(0.0, L, k):
            c = a + t * s
            pts.append(c + n * half_width)
            pts.append(c - n * half_width)
    return np.asarray(pts, dtype=float)


def polyline_points(polyline: List[Tuple[float, float]], spacing: float = 0.02) -> np.ndarray:
    """Dense (N,2) points sampled along an explicit wall polyline.

    Args:
        polyline (List[Tuple[float, float]]): The vertices of the polyline.
        spacing (float, optional): Distance between sampled points. Defaults to 0.02.

    Returns:
        np.ndarray: An (N, 2) array of coordinates densely sampled along the polyline.
    """
    pts = []
    pl = np.asarray(polyline, dtype=float)
    for a, b in zip(pl[:-1], pl[1:]):
        seg = b - a
        L = float(np.hypot(*seg))
        k = max(2, int(L / spacing))
        for s in np.linspace(0.0, L, k):
            pts.append(a + (seg / max(L, 1e-9)) * s)
    return np.asarray(pts, dtype=float)


def l_corridor(corner: Tuple[float, float], leg_in: float, leg_out: float, half_width: float) -> np.ndarray:
    """Walls (N,2) for a clean left-turning L-corridor of full width 2*half_width.

    Built as two MITERED wall polylines (inner + outer) so the convex inner
    corner is a single point, not the self-intersecting notch a per-segment
    normal offset produces.  Travels +x into ``corner`` then +y out of it.

    Args:
        corner (Tuple[float, float]): The internal turning point coordinates (x, y).
        leg_in (float): The length of the inbound leg leading into the corner.
        leg_out (float): The length of the outbound leg leaving the corner.
        half_width (float): The lateral distance from the centerline to the walls.

    Returns:
        np.ndarray: An (N, 2) array of wall coordinates representing the corridor boundaries.
    """
    cx, cy = corner
    hw = half_width
    inner = [(cx - leg_in, cy + hw), (cx - hw, cy + hw), (cx - hw, cy + leg_out)]
    outer = [(cx - leg_in, cy - hw), (cx + hw, cy - hw), (cx + hw, cy + leg_out)]
    return np.concatenate([polyline_points(inner), polyline_points(outer)], axis=0)


# --------------------------------------------------------------------------- #
#  Geometry helper
# --------------------------------------------------------------------------- #
def dist_to_polyline(px: float, py: float, poly: List[Tuple[float, float]]) -> float:
    """Perpendicular distance from (px,py) to a polyline (the path centreline).

    Args:
        px (float): Target point x-coordinate.
        py (float): Target point y-coordinate.
        poly (List[Tuple[float, float]]): The polyline vertices.

    Returns:
        float: Minimum distance from the point to the nearest line segment on the polyline.
    """
    best = float("inf")
    for a, b in zip(poly[:-1], poly[1:]):
        ax, ay = a
        bx, by = b
        dx, dy = bx - ax, by - ay
        L2 = dx * dx + dy * dy
        t = 0.0 if L2 < 1e-12 else max(0.0, min(1.0, ((px - ax) * dx + (py - ay) * dy) / L2))
        qx, qy = ax + t * dx, ay + t * dy
        best = min(best, math.hypot(px - qx, py - qy))
    return best


# --------------------------------------------------------------------------- #
#  One closed-loop episode
# --------------------------------------------------------------------------- #
def simulate(name: str, start_pose: Tuple[float, float, float], path_world: List[Tuple[float, float]],
             walls: np.ndarray, *, max_ticks: int = 900, goal_tol: float = 0.22,
             v_cap: Optional[float] = None) -> Dict[str, Any]:
    """Runs a single scenario simulation of the DWA local planner.

    Args:
        name (str): Identifier for the scenario.
        start_pose (Tuple[float, float, float]): Initial pose ``(x, y, theta)``.
        path_world (List[Tuple[float, float]]): The target path to follow.
        walls (np.ndarray): The synthetic point-cloud walls of the corridor.
        max_ticks (int, optional): The maximum number of control ticks to simulate. Defaults to 900.
        goal_tol (float, optional): Distance tolerance to consider the goal reached. Defaults to 0.22.
        v_cap (Optional[float], optional): Maximum velocity limit. Defaults to None.

    Returns:
        Dict[str, Any]: Performance metrics from the simulation including completion status, distance,
        average speed, and clearance to walls.
    """
    dt = 0.032
    dwa = LP.DWAPlanner(ctrl_dt=dt)
    x, y, th = start_pose
    goal = path_world[-1]

    dist_travelled = 0.0
    min_clear = float("inf")
    max_dev = 0.0
    v_hist = []
    reached = False
    ticks = 0

    for ticks in range(1, max_ticks + 1):
        # body-frame lidar cloud: walls within sensor range, like the real lidar
        body = inverse_transform_points(walls, x, y, th)
        rng = np.hypot(body[:, 0], body[:, 1])
        body = body[rng < C.LIDAR_RANGE_MAX]

        # true min clearance of the robot CENTRE to any wall (collision metric)
        if body.shape[0]:
            min_clear = min(min_clear, float(np.min(np.hypot(body[:, 0], body[:, 1]))))

        carrot, near_goal = LP.choose_carrot(path_world, (x, y, th), max(0.0, v_hist[-1] if v_hist else 0.0))
        if carrot is None:
            break
        v, w = dwa.compute((x, y, th), carrot, body, np.empty((0, 2)), v_cap=v_cap)
        v_hist.append(v)

        # integrate the unicycle model with the control timestep
        th += w * dt
        nx = x + v * math.cos(th) * dt
        ny = y + v * math.sin(th) * dt
        dist_travelled += math.hypot(nx - x, ny - y)
        x, y = nx, ny
        max_dev = max(max_dev, dist_to_polyline(x, y, path_world))

        if math.hypot(goal[0] - x, goal[1] - y) <= goal_tol:
            reached = True
            break

    elapsed = ticks * dt
    avg_speed = dist_travelled / max(elapsed, 1e-6)
    return {
        "name": name, "reached": reached, "ticks": ticks, "elapsed": elapsed,
        "dist": dist_travelled, "avg_speed": avg_speed, "min_clear": min_clear,
        "max_dev": max_dev,
        "final_dist_to_goal": math.hypot(goal[0] - x, goal[1] - y),
        "final_pose": (x, y, th),
    }


# --------------------------------------------------------------------------- #
#  Scenarios + assertions
# --------------------------------------------------------------------------- #
def _report(r: Dict[str, Any]) -> None:
    """Print a summary of the simulated episode's results.

    Args:
        r (Dict[str, Any]): The simulation metrics returned by `simulate`.
    """
    print(f"  [{r['name']}] reached={r['reached']} "
          f"dist={r['dist']:.2f}m avg_v={r['avg_speed']:.3f}m/s "
          f"min_clear={r['min_clear']:.3f}m max_dev={r['max_dev']:.3f}m "
          f"t={r['elapsed']:.1f}s goal_err={r['final_dist_to_goal']:.2f}m")


def main() -> None:
    """Run all dry-run scenarios and validate assertions.

    Exits with code 1 if any scenarios fail to meet requirements.
    """
    print("mak_02 Maze4 closed-loop dry-run (carrot + DWA)")
    fails: List[str] = []

    def check(name: str, cond: bool) -> None:
        """Evaluate a boolean condition and append to failures if False."""
        print(f"    [{'PASS' if cond else 'FAIL'}] {name}")
        if not cond:
            fails.append(name)

    # ---- Scenario 1: straight 4 m corridor, comfortable 0.50 m width --------
    L = 4.0
    path = [(0.0, 0.0), (L, 0.0)]
    walls = corridor_walls([(-0.3, 0.0), (L + 0.3, 0.0)], half_width=0.25)
    r = simulate("straight-wide", (0.0, 0.0, 0.0), path, walls)
    _report(r)
    check("straight-wide reaches the goal", r["reached"])
    # the regression that pins the crawl bug: real cruising speed, not cm/s
    check("straight-wide cruises (avg_v > 0.20 m/s)", r["avg_speed"] > 0.20)
    check("straight-wide never clips a wall (min_clear > 0.10 m)", r["min_clear"] > 0.10)

    # ---- Scenario 2: the REAL Maze4 bottleneck — 0.388 m gap to BLUE ---------
    # half_width 0.194 == ground-truth tightest clearance; robot half-width 0.116
    # => only 0.078 m slack per side.  Must stay centred to pass.
    walls_n = corridor_walls([(-0.3, 0.0), (L + 0.3, 0.0)], half_width=0.194)
    r = simulate("real-gap-0.388", (0.0, 0.0, 0.0), path, walls_n)
    _report(r)
    check("real-gap reaches the goal", r["reached"])
    check("real-gap still moves (avg_v > 0.12 m/s, no crawl)", r["avg_speed"] > 0.12)
    check("real-gap stays centred (max_dev < 0.06 m)", r["max_dev"] < 0.06)
    check("real-gap never clips a wall (min_clear > 0.116 m)", r["min_clear"] > 0.116)

    # ---- Scenario 2b: start 0.05 m off-centre — must recover to centreline ----
    r = simulate("real-gap-offset", (0.0, 0.06, 0.0), path, walls_n)
    _report(r)
    check("off-centre start recovers and reaches goal", r["reached"])
    check("off-centre start never clips a wall (min_clear > 0.112 m)", r["min_clear"] > 0.112)

    # ---- Scenario 3: L-corner corridor (tests pivot + turn-through) ----------
    # corner at (2.4, 0); robot enters along +x, exits along +y.
    path_L = [(0.0, 0.0), (2.4, 0.0), (2.4, 2.2)]
    walls_L = l_corridor((2.4, 0.0), leg_in=2.7, leg_out=2.4, half_width=0.26)
    r = simulate("corner", (0.0, 0.0, 0.0), path_L, walls_L, max_ticks=1600)
    _report(r)
    check("corner reaches the goal around the bend", r["reached"])
    check("corner makes real progress (dist > 3.5 m)", r["dist"] > 3.5)
    check("corner never clips a wall (min_clear > 0.09 m)", r["min_clear"] > 0.09)

    # ---- Scenario 4: must turn from a 90deg-off start (pivot gate) -----------
    r = simulate("pivot-start", (0.0, 0.0, math.pi / 2), path, walls)
    _report(r)
    check("pivot-start turns to face the goal and reaches it", r["reached"])

    print()
    if fails:
        print(f"DRY-RUN FAILED ({len(fails)}): {fails}")
        sys.exit(1)
    print("ALL DRY-RUN SCENARIOS PASSED")


if __name__ == "__main__":
    main()
