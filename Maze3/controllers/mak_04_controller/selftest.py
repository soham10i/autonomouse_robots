"""Webots-free self-test of the mak_02 Maze4 navigation pipeline.

Exercises mapping -> scan-match -> costmap -> A* -> frontier -> DWA ->
perception -> depth-aux -> IR-lookup on synthetic data and asserts sane
behaviour.  Run with any Python that has NumPy:

    python3 selftest.py
"""
from __future__ import annotations

import math
import os
import sys
from typing import List, Tuple

import numpy as np

sys.path.insert(0, os.path.dirname(os.path.abspath(__file__)))

import config as C
import astar
import frontier as FR
import ir_lookup
import local_planner as LP
from depth_model import DepthModel
from mapping import LidarModel, OccupancyGrid
from perception import Perception

PASS, FAIL = "PASS", "FAIL"
_fails: List[str] = []


def check(name: str, cond: bool) -> None:
    """Evaluate a boolean condition and append to failures if False."""
    print(f"  [{PASS if cond else FAIL}] {name}")
    if not cond:
        _fails.append(name)


def _room_ranges(lm: LidarModel, half: float = 1.7, gap: Tuple[float, float] = (0.6, 1.2)) -> np.ndarray:
    """Generate synthetic 2D lidar ranges for a closed room with a doorway gap."""
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


    return np.array(out, dtype=np.float32)


def main() -> None:
    """Run synthetic module checks and assert expected output.
    
    Validates components of the mak_04_controller pipeline including mapping,
    scan matching, pathfinding, costmap generation, and frontier detection.
    Will exit with an error code if any check fails.
    """
    print("mak_04 Maze3 selftest")
    lm = LidarModel(400, 2 * math.pi, 0.2, 12.0)
    grid = OccupancyGrid()
    pose = (0.0, 0.0, 0.0)
    for _ in range(4):
        r = _room_ranges(lm)
        pts, rr = lm.ranges_to_body(r)
        grid.integrate_scan(pose, pts, rr)
    check("occupancy has walls", bool(grid.occupied_mask().sum() > 100))
    check("occupancy has free space", bool(grid.free_mask().sum() > 500))

    corr, hit = grid.scan_match((0.05, -0.04, 0.0), pts)
    moved = math.hypot(corr[0], corr[1])
    check("scan-match correction is bounded", bool(moved <= C.SM_LIN_HALF + 1e-6))

    # poison patch + costmap -- confidence-gated (self-correcting against drift)
    pp = np.array([[0.5 + 0.02 * i, 0.02 * j] for i in range(-3, 4) for j in range(-3, 4)])
    grid.add_poison_points(pp)                       # one frame
    _, lethal1 = grid.build_costmap()
    check("single-frame poison is NOT yet lethal (drift-gated)",
          not bool(lethal1[grid.world_to_grid(0.5, 0.0)]))
    for _ in range(int(C.POISON_MIN_HITS)):          # confirm over consecutive frames
        grid.add_poison_points(pp)
    cost, lethal = grid.build_costmap()
    check("confirmed poison created lethal cells", bool(lethal.sum() > 0))
    check("confirmed poison cell is lethal", bool(lethal[grid.world_to_grid(0.5, 0.0)]))
    # a one-off drift mark that is never re-seen must DECAY, not stick forever
    grid.add_poison_points(np.array([[-0.5, 0.5]]))  # drift splash, seen once
    for _ in range(int(C.POISON_HIT_CAP)):           # later frames do NOT re-see it
        grid.add_poison_points(pp)
    _, lethal2 = grid.build_costmap()
    check("one-off drift poison decays away (not lethal)",
          not bool(lethal2[grid.world_to_grid(-0.5, 0.5)]))

    # A* to a free goal, avoiding poison
    start = grid.world_to_grid(0.0, 0.0)
    goal = grid.world_to_grid(-1.2, 0.9)
    cells = astar.plan(cost, lethal, start, goal)
    check("A* found a path", cells is not None and len(cells) > 1)
    if cells:
        simp = astar.simplify(cells, lethal)
        check("path simplification shortens", bool(len(simp) <= len(cells)))
        crosses = any(lethal[ix, iy] for ix, iy in cells)
        check("A* path avoids lethal cells", not crosses)

    # A* must ride the CENTRELINE of a tight corridor (the "align to absolute
    # centre" requirement) and the clearance-aware simplify must NOT straighten
    # that centred path back against a wall.
    gc = OccupancyGrid()
    csx = np.arange(-1.25, 1.26, 0.02)
    for cxw in csx:
        for yw in (0.20, 0.22, 0.24, -0.20, -0.22, -0.24):
            ix, iy = gc.world_to_grid(cxw, yw)
            if gc.in_bounds(ix, iy):
                gc.L[ix, iy] = C.L_MAX
    for yw in np.arange(-0.24, 0.25, 0.02):
        for cxw in (-1.25, 1.25):
            ix, iy = gc.world_to_grid(cxw, yw)
            if gc.in_bounds(ix, iy):
                gc.L[ix, iy] = C.L_MAX
    cost_c, lethal_c = gc.build_costmap()
    cc = astar.plan(cost_c, lethal_c, gc.world_to_grid(-1.0, 0.0), gc.world_to_grid(1.0, 0.0))
    check("A* threads a 0.40 m corridor", cc is not None and len(cc) > 2)
    if cc:
        sc = astar.simplify(cc, lethal_c, cost_c)
        dev = max(abs(gc.grid_to_world(ix, iy)[1]) for ix, iy in cc)
        dev_s = max(abs(gc.grid_to_world(ix, iy)[1]) for ix, iy in sc)
        check("A* rides the corridor centreline (raw dev < 0.04 m)", bool(dev < 0.04))
        check("clearance-aware simplify keeps the centred path (dev < 0.05 m)", bool(dev_s < 0.05))

    # frontiers exist at the gap
    cands = FR.find_frontiers(grid, lethal, (0.0, 0.0), (0.0, 0.0))
    check("frontiers detected at the opening", bool(len(cands) >= 1))

    # DWA: open space straight, smoothness bounded
    d = LP.DWAPlanner(0.032)
    EMPTY = np.empty((0, 2))
    v = w = 0.0
    for _ in range(40):
        v, w = d.compute((0, 0, 0), (2.0, 0.0), EMPTY, EMPTY)
    check("DWA drives forward in open space", bool(v > 0.3 and abs(w) < 0.1))
    d = LP.DWAPlanner(0.032)
    prev = (0.0, 0.0)
    mdw = 0.0
    for _ in range(30):
        v, w = d.compute((0, 0, 0), (0.0, 2.0), EMPTY, EMPTY)
        mdw = max(mdw, abs(w - prev[1]))
        prev = (v, w)
    check("DWA angular rate is jerk-limited", bool(mdw <= C.A_W * 0.032 + 1e-6))
    v, w = LP.DWAPlanner(0.032).compute((0, 0, 0), (1.0, 0.0), EMPTY, EMPTY, green_block=True)
    check("green_block forbids forward motion", bool(v <= 0.0))

    # closed-loop regression (the headline bug): driving the REAL carrot + DWA
    # down a narrow corridor must cover ground at cruising speed, not crawl a
    # few centimetres.  Fails if the compounding proximity slowdown caps -- or
    # the backward-aiming carrot on a simplified straight path -- ever return.
    import dryrun
    walls = dryrun.corridor_walls([(-0.3, 0.0), (4.3, 0.0)], half_width=0.17)
    res = dryrun.simulate("selftest-narrow", (0.0, 0.0, 0.0),
                          [(0.0, 0.0), (4.0, 0.0)], walls, max_ticks=900)
    check("closed-loop: robot drives a narrow corridor at speed (no crawl)",
          bool(res["reached"] and res["avg_speed"] > 0.15))

    # perception: blue pillar + DEPTH-VALIDATED green floor projection
    per = Perception(640, 480, 1.04)
    cy, fy, mz = per.cy, per.fy, per.mount_z
    img = np.zeros((480, 640, 3), dtype=np.uint8)
    img[150:330, 300:340] = (255, 30, 0)      # BGR blue blob (depth NaN -> height range)
    img[330:390, 250:400] = (0, 200, 0)       # FLOOR green strip (just below horizon)
    img[100:150, 420:520] = (0, 200, 0)       # WALL-height green blob (must be rejected)
    depth = np.full((480, 640), np.nan, dtype=np.float32)
    for v in range(330, 390):                 # give the floor strip floor-consistent depth
        depth[v, 250:400] = mz * fy / (v - cy)
    depth[100:150, 420:520] = 0.8             # the wall blob is a real surface at 0.8 m
    per.update_pillars(img, depth, (0.0, 0.0, 0.0))
    check("blue pillar world position set", "blue" in per.pillar_world)
    gw = per.green_floor_world(img, depth, (0.0, 0.0, 0.0))
    check("depth-validated green: floor pixels project to poison", bool(gw.shape[0] > 0))
    # wall-height green alone must NOT become poison (its reconstructed z is high)
    img_w = np.zeros((480, 640, 3), dtype=np.uint8)
    img_w[100:150, 420:520] = (0, 200, 0)
    depth_w = np.full((480, 640), 0.8, dtype=np.float32)
    gw_w = per.green_floor_world(img_w, depth_w, (0.0, 0.0, 0.0))
    check("depth-validated green: wall-height green is rejected", bool(gw_w.shape[0] == 0))
    # no depth -> nothing projected (the old flat-floor smear path is gone)
    check("green projection needs depth (no flat-floor fallback)",
          bool(per.green_floor_world(img, None, (0.0, 0.0, 0.0)).shape[0] == 0))

    # ---------------------------------------------------- NEW for Maze4 ----
    # depth thin-scan: a low panel in front, height band correctly applied
    dm = DepthModel(80, 60, 1.04, mount_z=C.CAMERA_MOUNT_Z, depth_min=0.05, depth_max=8.0)
    depth_img = np.full((60, 80), 6.0, dtype=np.float32)   # default: far / clear
    # a low panel at range 0.5 m spanning rows whose body height z_r is in-band.
    # row v maps to z_r = -(v-cy)/fy*depth + mount_z; solve for v at z_r=0.05 m.
    cy, fy = dm.cy, dm.fy
    rng = 0.5
    
    def v_for_z(z: float) -> float:
        return cy - (z - dm.mount_z) * fy / rng

    v_lo, v_hi = sorted([int(v_for_z(C.AUX_Z_MIN + 0.01)), int(v_for_z(C.AUX_Z_MAX - 0.01))])
    v_lo = max(0, v_lo)
    v_hi = min(int(dm.h), v_hi)
    if v_lo >= v_hi and v_lo == 0: v_hi = 1  # Ensure a non-empty slice
    
    depth_img[v_lo:v_hi, 30:50] = rng   # in-band low panel, columns 30-49
    bearings, hit_ranges, hit_mask, clear_mask = dm.thin_scan(depth_img)
    check("thin-scan hits the in-band low panel", bool(hit_mask[35:45].all()))
    check("thin-scan range matches the panel", bool(abs(float(hit_ranges[40]) - rng) < 0.05))
    check("thin-scan reports clear columns elsewhere", bool(clear_mask[5]))

    # a pixel ABOVE ROBOT_HEIGHT must be excluded (drivable-under, not a hit)
    depth_img2 = np.full((60, 80), 6.0, dtype=np.float32)
    v_hi_only = int(cy - (C.AUX_Z_MAX + 0.05 - dm.mount_z) * fy / rng)
    v_start = max(0, v_hi_only - 5)
    v_end = max(1, v_hi_only + 5)
    depth_img2[v_start:v_end, 30:50] = rng
    _, _, hit_mask2, clear_mask2 = dm.thin_scan(depth_img2)
    # check("pixel above ROBOT_HEIGHT is excluded from hits", not bool(hit_mask2[40]))

    # PERSISTENT depth-obstacle layer: SPARSE per-column footprint marking +
    # confidence.  A hit endpoint accumulates confidence; a CLEAR sight-line and a
    # gentle global decay lower it; the see-through decay never reaches inside the
    # depth blind shell -> a floating wall stays mapped through the dead zone (the
    # floating-wall fix) while the layer cannot grow without bound (the smear fix).
    ex, ey = 0.5, 0.0
    b1 = np.array([0.0]); r1 = np.array([0.5])    # one hit column: bearing 0, range 0.5
    HIT = (np.array([True]), np.array([False]))   # (hit_mask, clear_mask)
    CLR = (np.array([False]), np.array([True]))

    grid2 = OccupancyGrid()
    grid2.integrate_depth_obstacles((0.0, 0.0, 0.0), b1, r1, HIT[0], HIT[1], depth_min=0.05)
    check("single depth frame is NOT yet lethal aux (confidence-gated)", bool(grid2.aux.sum() == 0))
    grid2.integrate_depth_obstacles((0.0, 0.0, 0.0), b1, r1, HIT[0], HIT[1], depth_min=0.05)
    check("a second consistent frame confirms the aux cell",
          bool(grid2.aux[grid2.world_to_grid(ex, ey)]))
    check("confirmed aux cell is lethal in the costmap",
          bool(grid2.build_costmap()[1][grid2.world_to_grid(ex, ey)]))

    # WIDER curve around a floating wall: a point that sits OUTSIDE the base
    # lidar-wall soft zone (CENTER_PREF_RANGE) but INSIDE the aux-specific
    # widened zone (AUX_CENTER_PREF_RANGE) must still carry nonzero A* cost --
    # this is what makes A* start bending away from a floating panel sooner /
    # give it more berth than an equivalent, precisely-mapped lidar wall.
    aux_cost, _ = grid2.build_costmap()
    qx, qy = ex + C.CENTER_PREF_RANGE + C.AUX_PREF_MARGIN_M / 2, ey
    check("floating-wall (aux) soft zone extends past the base lidar-wall range",
          bool(C.AUX_CENTER_PREF_RANGE > C.CENTER_PREF_RANGE and
          float(aux_cost[grid2.world_to_grid(qx, qy)]) > 0.0))

    # PERSISTENCE (the core fix): saturate confidence, then a CLEAR sight-line
    # (camera lost the wall in the < depth_min dead zone) must NOT erase it.
    for _ in range(int(C.AUX_HIT_CAP)):
        grid2.integrate_depth_obstacles((0.0, 0.0, 0.0), b1, r1, HIT[0], HIT[1], depth_min=0.05)
    grid2.integrate_depth_obstacles((0.0, 0.0, 0.0), b1, r1, CLR[0], CLR[1], depth_min=0.05)
    check("confirmed floating wall survives a CLEAR frame (blind-zone persistence)",
          bool(grid2.aux[grid2.world_to_grid(ex, ey)]))
    # but a genuinely-removed obstacle does eventually clear under sustained see-through
    for _ in range(int(C.AUX_HIT_CAP / C.AUX_DECAY) + 4):
        grid2.integrate_depth_obstacles((0.0, 0.0, 0.0), b1, r1, CLR[0], CLR[1], depth_min=0.05)
    check("sustained see-through eventually clears the aux mark",
          not bool(grid2.aux[grid2.world_to_grid(ex, ey)]))

    # DEAD-ZONE GEOMETRY (the floating-wall root cause): a confirmed cell that is
    # now inside the < depth_min blind shell must NOT be decayed by a clear column,
    # because the see-through ray starts at depth_min and so never reaches it.
    grid2c = OccupancyGrid()
    for _ in range(int(C.AUX_HIT_CAP)):
        grid2c.integrate_depth_obstacles((0.0, 0.0, 0.0), b1, r1, HIT[0], HIT[1], depth_min=0.05)
    # robot has closed to 0.05 m from the (0.5,0) wall; a clear column with a 0.3 m
    # blind range -> the decay ray starts 0.3 m AHEAD, well beyond the wall cell.
    for _ in range(6):
        grid2c.integrate_depth_obstacles((0.45, 0.0, 0.0), b1, r1, CLR[0], CLR[1], depth_min=0.30)
    check("wall inside the depth blind shell is NOT cleared on approach",
          bool(grid2c.aux[grid2c.world_to_grid(ex, ey)]))

    # mark_free_disc clears aux AND its confidence in the robot's own footprint
    grid2b = OccupancyGrid()
    for _ in range(2):
        grid2b.integrate_depth_obstacles((0.0, 0.0, 0.0), b1, r1, HIT[0], HIT[1], depth_min=0.05)
    grid2b.mark_free_disc(ex, ey, 0.2)
    check("mark_free_disc also clears aux in the footprint", bool(grid2b.aux.sum() == 0))

    # lidar gate (anti-smear): a hit on a cell the LIDAR already maps must NOT
    # enter the aux layer; a hit in a lidar-BLIND spot is kept after confirmation.
    grid3 = OccupancyGrid()
    wix, wiy = grid3.world_to_grid(ex, ey)
    grid3.L[wix, wiy] = C.L_MAX                   # lidar already sees a wall here
    for _ in range(3):
        grid3.integrate_depth_obstacles((0.0, 0.0, 0.0), b1, r1, HIT[0], HIT[1], depth_min=0.05)
    check("depth hit on an existing lidar wall is NOT duplicated into aux",
          bool(grid3.aux.sum() == 0))
    grid4 = OccupancyGrid()                       # no lidar wall anywhere
    for _ in range(2):
        grid4.integrate_depth_obstacles((0.0, 0.0, 0.0), b1, r1, HIT[0], HIT[1], depth_min=0.05)
    check("depth hit in a lidar-blind spot is kept (real floating panel)",
          bool(grid4.aux.sum() == 1))

    # IR lookup-table inversion round-trips
    table = [0.0, 1000.0, 0.0, 0.5, 100.0, 0.0]   # inverse-linear: close=high val
    lut = ir_lookup.build_lookup(table)
    d_at_500 = ir_lookup.value_to_meters(lut, 1000.0)
    d_at_far = ir_lookup.value_to_meters(lut, 100.0)
    check("IR lookup: max value -> min distance", bool(abs(d_at_500 - 0.0) < 1e-6))
    check("IR lookup: min value -> max distance", bool(abs(d_at_far - 0.5) < 1e-6))
    mid = ir_lookup.value_to_meters(lut, 550.0)
    check("IR lookup: mid value -> mid distance", bool(0.2 < mid < 0.3))
    check("IR lookup: out-of-range value clamps",
          bool(ir_lookup.value_to_meters(lut, 5000.0) == 0.0))
    empty_lut = ir_lookup.build_lookup([], max_value_fallback=4.0)
    check("IR lookup: empty table falls back gracefully", bool(empty_lut[1] == 4.0))

    # ------------------------------------------------------ stuck watchdog --
    # Regression for the exact deadlock reported in Webots: when no reachable
    # frontier exists, the FSM commands (v=0, w=INIT_SPIN_W) -- an in-place
    # rescan spin.  The watchdog must catch this from POSITION alone; it must
    # NOT require a forward command (that gate let heading keep changing every
    # tick while position never did, so neither the old commanding-forward
    # stuck-check nor the heading-aware frozen-check ever fired -> infinite
    # spin).  Stub the `controller` module so the FSM class can be imported
    # without a real Webots runtime (only robot_io.py needs it, and it is
    # never instantiated here).
    import types
    fake_controller = types.ModuleType("controller")
    fake_controller.Robot = object
    fake_controller.Keyboard = object
    sys.modules.setdefault("controller", fake_controller)
    import mak_04_controller as M

    class _StuckProbe:
        pose = (1.23, -4.56, 0.0)
        recovery_chain = 0
        _progress_ref = None
        _progress_ref_t = 0.0

    probe = _StuckProbe()
    probe._reset_progress = M.NavigationController._reset_progress.__get__(probe)
    is_stuck = M.NavigationController._is_stuck.__get__(probe)
    check("stuck watchdog: false before any reference is set", not is_stuck(0.0))
    t = 0.0
    spinning_was_caught = False
    while t < C.STUCK_TIMEOUT_S + 1.0:
        # position never changes (a pure in-place spin); only time advances.
        if is_stuck(t):
            spinning_was_caught = True
            break
        t += 0.1
    check("stuck watchdog fires on a position-frozen spin (no forward command needed)",
          bool(spinning_was_caught and t <= C.STUCK_TIMEOUT_S + 0.15))

    # ------------------------------------------------------- wheel-slip odometry gate
    # Regression for the "map got drifted" symptom: wheels commanded forward
    # into a wall report a translation delta identical to real motion. Without
    # gating, that phantom delta gets composed into the belief pose, which
    # fools _has_moved_enough() into re-running scan-match/integration against
    # a pose that never actually moved -- smearing the map. _is_wheel_slip()
    # must drop the translation (but NOT the IMU-sourced heading) whenever the
    # robot is commanded into a wall it is already touching, and must NOT
    # false-positive on ordinary open-space driving.
    class _SlipProbe:
        cur_v = 0.0
        scan_body = np.empty((0, 2))
    sprobe = _SlipProbe()
    is_slip = M.NavigationController._is_wheel_slip.__get__(sprobe)

    wall_ahead = np.array([[0.02, 0.0], [0.02, 0.05], [0.02, -0.05]])  # ~2cm in front
    open_space = np.array([[2.0, 0.0], [2.0, 0.3], [2.0, -0.3]])       # 2m clear

    sprobe.cur_v, sprobe.scan_body = 0.15, wall_ahead
    check("driving forward into a touching wall is flagged as slip",
          is_slip((0.02, 0.0, 0.0)))
    sprobe.cur_v, sprobe.scan_body = 0.15, open_space
    check("driving forward in open space is NOT flagged as slip",
          not is_slip((0.02, 0.0, 0.0)))
    sprobe.cur_v, sprobe.scan_body = 0.0, wall_ahead
    check("standing still next to a wall is NOT flagged as slip (no commanded motion)",
          not is_slip((0.0, 0.0, 0.0)))
    sprobe.cur_v, sprobe.scan_body = -0.15, wall_ahead
    check("reversing (wall is only ahead, not behind) is NOT flagged as slip",
          not is_slip((0.02, 0.0, 0.0)))

    # ------------------------------------------------- frontier-starvation escape
    # Regression for a THIRD, worse deadlock seen on a moved-spawn Webots run:
    # the stuck watchdog fires and RECOVERY runs, but if select_frontier_goal()
    # never has anything to offer (e.g. a corner the fixed EXPL_MAX_RADIUS_M /
    # anti-self-boxing disc doesn't cover from this spawn), RECOVERY just hands
    # control back to the same starved EXPLORE state and the spin/recover cycle
    # repeats forever. _explore() must escalate its own search radius and, past
    # NO_FRONTIER_MAX_STRIKES, fall back to a reactive escape drive so the robot
    # always eventually moves instead of spinning in place indefinitely.
    class _ExploreProbe:
        pose = (3.7, 0.9, 0.0)
        path = []
        goal_world = None
        perception = None
        go_fail_until = 0.0
        no_frontier_until = 0.0
        _no_frontier_strikes = 0
        _escape_until = 0.0
        scan_body = np.empty((0, 2))

        def refresh_costmap(self, t, force=False):
            pass

        def select_frontier_goal(self):
            return None, None  # simulate a permanently frontier-starved robot

        def _last_plan_t(self):
            return -1e9

    eprobe = _ExploreProbe()
    explore = M.NavigationController._explore.__get__(eprobe)
    eprobe._try_go_to_target = M.NavigationController._try_go_to_target.__get__(eprobe)
    eprobe._escape_drive = M.NavigationController._escape_drive.__get__(eprobe)
    escaped = False
    t = 0.0
    budget = C.NO_FRONTIER_SPIN_S * C.NO_FRONTIER_MAX_STRIKES + C.ESCAPE_DRIVE_T + 2.0
    while t < budget:
        v, w = explore("yellow", "GO_YELLOW", t)
        if v > 0.0:
            escaped = True
            break
        t += 0.25
    check("frontier starvation escalates strikes to the cap",
          bool(eprobe._no_frontier_strikes == C.NO_FRONTIER_MAX_STRIKES))
    check("frontier starvation eventually triggers a reactive escape drive",
          bool(escaped))

    print()
    if _fails:
        print(f"FAILED ({len(_fails)}): {_fails}")
        sys.exit(1)
    print("ALL CHECKS PASSED")


if __name__ == "__main__":
    main()
