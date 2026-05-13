"""Phase 3 — Map-runner with hybrid PLY+NPZ walls + pillar localization + pure-pursuit.

Architecture
------------
1. Walls come from a HYBRID of two sources, merged via boolean OR:
   - ``map.npz['occupied'] | map.npz['aux_obstacle']`` — 2D LiDAR log-odds
   - ``scene.ply`` height-filtered (0.03 < z < 0.55) — 3D depth camera
   The 2D LiDAR misses short/glass walls; the depth camera catches them.
   Merging both ensures no wall is ever invisible to the path planner.
2. At startup the robot spins in place and observes both pillars. The two
   correspondences {(blue_obs, blue_map), (yellow_obs, yellow_map)} solve
   exactly for the rigid SE(2) transform from the saved-map frame into the
   current odom frame. All wall/poison/pillar data are then resampled into
   the current frame, so the runner tolerates being placed anywhere in the
   maze (not just at the teleop start pose).
3. A* uses an EDT-based corridor-centering cost so paths prefer the middle
   of free space. After A*, the path passes through (RDP -> nudge sharp
   turns to corridor centers -> insert safety midpoints where simplified
   segments come within 16 cm of any wall -> split long segments).
4. Execution is closed-loop pure pursuit (PathFollower from
   maze_navigator/local_control.py) instead of ROT-then-DRV, so the robot
   never has to rotate in place at corner waypoints — eliminates the
   chassis-sweep wall-clip bug.
"""
import os
import sys
import math
import heapq

import numpy as np

_HERE = os.path.dirname(os.path.abspath(__file__))
sys.path.insert(0, os.path.normpath(os.path.join(_HERE, "..", "maze_navigator")))

import config as C
from robot_io import RobotIO
from odometry import Odometry
from lidar_scan import ScanProcessor
from perception import PillarDetector
from local_control import PathFollower

# ── grid extent (current odom frame) ────────────────────────────────────
RES          = 0.04
X_MIN, X_MAX = -3.0, 4.0
Y_MIN, Y_MAX = -5.0, 3.0

# ── A* / costmap ────────────────────────────────────────────────────────
POISON_COST       = 50.0   # cost multiplier for poison cells
INFLATE           = 4      # base wall inflation (4 cells = 16 cm)
WALL_PROX_WEIGHT  = 10.0   # EDT corridor-centering weight in A*
WALL_PROX_CELLS   = 12     # EDT influence radius (cells) → 48 cm

# ── path simplification / safety (Issue 2 fix) ──────────────────────────
RDP_TOL           = 0.04   # path simplification tolerance (m)
NUDGE_ANGLE_DEG   = 20.0   # turn ≥ this triggers corridor-center nudge
NUDGE_SEARCH_R    = 6      # nudge search radius (cells) → 24 cm
SAFETY_MIN_DIST   = 4      # min wall distance for simplified segments → 16 cm
MAX_SEG_LEN       = 0.30   # split waypoints longer than this (m)
CARVE_R           = 0.22   # free-disc radius around start/goals (m)

# ── pillar localization (Issue 3 fix) ───────────────────────────────────
LOCALIZE_W        = 0.6    # spin rate during localization (rad/s)
LOCALIZE_REVS     = 1.25   # how many full revs to spin
LOCALIZE_RNG_MIN  = 0.20
LOCALIZE_RNG_MAX  = 4.0
LOCALIZE_MIN_OBS  = 4

# ── pure-pursuit execution ──────────────────────────────────────────────
EXEC_TIMEOUT_S    = 90.0   # per-leg timeout safety


def _wrap(a):
    while a >  math.pi: a -= 2 * math.pi
    while a < -math.pi: a += 2 * math.pi
    return a


# ═══════════════════════════════════════════════════════════════════════
#  1. Wall / poison loaders (Issue 1 fix — npz layers, not PLY scatter)
# ═══════════════════════════════════════════════════════════════════════

def _resample_mask(src_mask, src_origin, src_res, R=None, t=None):
    """Resample a saved-frame boolean mask into the runner grid.

    If ``(R, t)`` are provided, source cell centers are first transformed
    by ``p_odom = R @ p_saved + t`` (rigid 2D transform from pillar
    localization) before binning into the runner grid.
    """
    nx_g = int((X_MAX - X_MIN) / RES)
    ny_g = int((Y_MAX - Y_MIN) / RES)
    out = np.zeros((nx_g, ny_g), dtype=bool)

    ii, jj = np.where(src_mask)
    if len(ii) == 0:
        return out

    sx = src_origin[0] + (ii + 0.5) * src_res
    sy = src_origin[1] + (jj + 0.5) * src_res
    if R is not None and t is not None:
        ox = R[0, 0] * sx + R[0, 1] * sy + t[0]
        oy = R[1, 0] * sx + R[1, 1] * sy + t[1]
    else:
        ox, oy = sx, sy

    ix = ((ox - X_MIN) / RES).astype(int)
    iy = ((oy - Y_MIN) / RES).astype(int)
    m = (ix >= 0) & (ix < nx_g) & (iy >= 0) & (iy < ny_g)
    out[ix[m], iy[m]] = True
    return out


def load_walls_from_npz(npz, R=None, t=None):
    """Build a clean wall mask from ``occupied | aux_obstacle``."""
    src = npz["occupied"].astype(bool)
    if "aux_obstacle" in npz.files:
        src = src | npz["aux_obstacle"].astype(bool)
    return _resample_mask(
        src,
        (float(npz["origin"][0]), float(npz["origin"][1])),
        float(npz["resolution"]),
        R, t,
    )


def load_walls_from_ply(ply_path, R=None, t=None):
    """Project 3D point cloud wall points (0.03 < z < 0.55) to a 2D grid.

    The depth camera captures walls that the 2D LiDAR misses (short walls,
    glass, overhead structures). The result is dilated by 1 cell to fill
    sub-voxel gaps from the 4 cm voxel downsampling.

    Math per point:
        1. Height filter:  keep if 0.03 < z < 0.55
        2. Optional SE(2) transform:  p_odom = R @ p_map + t
        3. Grid binning:  ix = floor((x - X_MIN) / RES),
                          iy = floor((y - Y_MIN) / RES)
        4. 1-cell morphological dilation to close voxel gaps
    """
    nx_g = int((X_MAX - X_MIN) / RES)
    ny_g = int((Y_MAX - Y_MIN) / RES)

    if not os.path.isfile(ply_path):
        print(f"[walls] PLY not found: {ply_path}")
        return np.zeros((nx_g, ny_g), dtype=bool)

    with open(ply_path) as f:
        lines = f.readlines()
    h = next(i for i, l in enumerate(lines) if l.strip() == 'end_header') + 1
    raw = []
    for l in lines[h:]:
        p = l.split()
        if len(p) >= 3:
            raw.append((float(p[0]), float(p[1]), float(p[2])))
    pts = np.array(raw)
    if len(pts) == 0:
        return np.zeros((nx_g, ny_g), dtype=bool)

    # 1. Height filter — keep only wall-height points
    z = pts[:, 2]
    walls = pts[(z > 0.03) & (z < 0.55)]
    if len(walls) == 0:
        return np.zeros((nx_g, ny_g), dtype=bool)

    # 2. Optional SE(2) rigid transform
    if R is not None and t is not None:
        ox = R[0, 0] * walls[:, 0] + R[0, 1] * walls[:, 1] + t[0]
        oy = R[1, 0] * walls[:, 0] + R[1, 1] * walls[:, 1] + t[1]
    else:
        ox, oy = walls[:, 0], walls[:, 1]

    # 3. Grid binning
    out = np.zeros((nx_g, ny_g), dtype=bool)
    ix = ((ox - X_MIN) / RES).astype(int)
    iy = ((oy - Y_MIN) / RES).astype(int)
    m = (ix >= 0) & (ix < nx_g) & (iy >= 0) & (iy < ny_g)
    out[ix[m], iy[m]] = True

    # 4. Morphological dilation (fills sub-voxel gaps)
    try:
        from scipy.ndimage import binary_dilation
        out = binary_dilation(out, iterations=1)
    except Exception:
        pass

    n_wall_pts = len(walls)
    n_cells = int(out.sum())
    print(f"[walls] PLY: {n_wall_pts} wall points → {n_cells} grid cells")
    return out


def load_walls_hybrid(npz, ply_path, R=None, t=None):
    """Merge NPZ + PLY walls:  any cell that EITHER source marks as wall
    is treated as a wall.  This ensures no wall is ever invisible to A*.

    NPZ captures:  LiDAR-confirmed flat walls (log-odds > threshold)
    PLY captures:  depth-camera walls including short/glass/overhead
    """
    w_npz = load_walls_from_npz(npz, R, t)
    w_ply = load_walls_from_ply(ply_path, R, t)
    merged = w_npz | w_ply
    n_npz = int(w_npz.sum())
    n_ply = int(w_ply.sum())
    n_merged = int(merged.sum())
    print(f"[walls] NPZ={n_npz}  PLY={n_ply}  merged={n_merged}  "
          f"(PLY added {n_merged - n_npz} walls NPZ missed)")
    return merged


def load_poison_from_npz(npz, R=None, t=None):
    if "poison" not in npz.files:
        nx_g = int((X_MAX - X_MIN) / RES)
        ny_g = int((Y_MAX - Y_MIN) / RES)
        return np.zeros((nx_g, ny_g), dtype=bool)
    return _resample_mask(
        npz["poison"].astype(bool),
        (float(npz["origin"][0]), float(npz["origin"][1])),
        float(npz["resolution"]),
        R, t,
    )


# ═══════════════════════════════════════════════════════════════════════
#  2. Grid with EDT corridor-centering
# ═══════════════════════════════════════════════════════════════════════

class Grid:
    def __init__(self, wall_mask, poison_mask=None):
        self.wall = wall_mask.copy()
        self.nx, self.ny = self.wall.shape
        self.poison = (poison_mask.copy() if poison_mask is not None
                       else np.zeros_like(self.wall))
        self.dist_to_wall = None

    def w2i(self, x, y):
        return int((x - X_MIN) / RES), int((y - Y_MIN) / RES)

    def i2w(self, ix, iy):
        return X_MIN + (ix + 0.5) * RES, Y_MIN + (iy + 0.5) * RES

    def ok(self, ix, iy):
        return 0 <= ix < self.nx and 0 <= iy < self.ny

    def inflate(self, r=INFLATE):
        orig = self.wall.copy()
        for di in range(-r, r + 1):
            for dj in range(-r, r + 1):
                if 0 < math.hypot(di, dj) <= r:
                    self.wall |= np.roll(np.roll(orig, di, 0), dj, 1)
        self._compute_distance_field()
        print(f"[grid] inflate(r={r}): {self.wall.sum()} wall cells")

    def _compute_distance_field(self):
        try:
            from scipy.ndimage import distance_transform_edt
            self.dist_to_wall = distance_transform_edt(~self.wall)
        except Exception:
            self.dist_to_wall = None  # corridor-centering disabled

    def carve(self, x, y, rm=CARVE_R):
        rc = int(math.ceil(rm / RES))
        cx, cy = self.w2i(x, y)
        for di in range(-rc, rc + 1):
            for dj in range(-rc, rc + 1):
                if math.hypot(di, dj) <= rc:
                    ix, iy = cx + di, cy + dj
                    if self.ok(ix, iy):
                        self.wall[ix, iy] = False
                        self.poison[ix, iy] = False
        self._compute_distance_field()

    def cost(self, ix, iy):
        if not self.ok(ix, iy) or self.wall[ix, iy]:
            return float('inf')
        base = POISON_COST if self.poison[ix, iy] else 1.0
        if self.dist_to_wall is not None:
            d = self.dist_to_wall[ix, iy]
            if d < WALL_PROX_CELLS:
                base += WALL_PROX_WEIGHT * (1.0 - d / WALL_PROX_CELLS)
        return base


# ═══════════════════════════════════════════════════════════════════════
#  3. A*
# ═══════════════════════════════════════════════════════════════════════

def astar(grid, start, goal):
    si, sj = grid.w2i(*start)
    gi, gj = grid.w2i(*goal)
    if not grid.ok(si, sj) or not grid.ok(gi, gj):
        return None
    h = lambda i, j: math.hypot(i - gi, j - gj)
    openq = [(h(si, sj), 0.0, si, sj)]
    gscore = {(si, sj): 0.0}
    parent = {}
    closed = set()
    while openq:
        _, g, ci, cj = heapq.heappop(openq)
        if (ci, cj) in closed:
            continue
        closed.add((ci, cj))
        if ci == gi and cj == gj:
            path = []
            n = (gi, gj)
            while n in parent:
                path.append(grid.i2w(*n))
                n = parent[n]
            path.append(grid.i2w(si, sj))
            path.reverse()
            return path
        for di, dj in [(-1,0),(1,0),(0,-1),(0,1),(-1,-1),(-1,1),(1,-1),(1,1)]:
            ni, nj = ci + di, cj + dj
            c = grid.cost(ni, nj)
            if c == float('inf') or (ni, nj) in closed:
                continue
            ng = g + math.hypot(di, dj) * c
            if ng < gscore.get((ni, nj), float('inf')):
                gscore[(ni, nj)] = ng
                parent[(ni, nj)] = (ci, cj)
                heapq.heappush(openq, (ng + h(ni, nj), ng, ni, nj))
    return None


# ═══════════════════════════════════════════════════════════════════════
#  4. Path simplification + safety pass (Issue 2 fix)
# ═══════════════════════════════════════════════════════════════════════

def _rdp(pts, eps):
    if len(pts) <= 2:
        return pts
    s, e = np.array(pts[0]), np.array(pts[-1])
    lv = e - s
    ll = np.linalg.norm(lv)
    if ll < 1e-9:
        return [pts[0], pts[-1]]
    lu = lv / ll
    dmax, idx = 0, 0
    for i in range(1, len(pts) - 1):
        v = np.array(pts[i]) - s
        d = abs(v[0] * lu[1] - v[1] * lu[0])
        if d > dmax:
            dmax, idx = d, i
    if dmax > eps:
        return _rdp(pts[:idx + 1], eps)[:-1] + _rdp(pts[idx:], eps)
    return [pts[0], pts[-1]]


def _nudge_to_center(pts, grid,
                     search_r=NUDGE_SEARCH_R,
                     angle_thresh_deg=NUDGE_ANGLE_DEG):
    """Move sharp-turn waypoints toward the local corridor center."""
    if grid.dist_to_wall is None:
        return pts
    MAX_CORRIDOR_DIST = 8
    result = [pts[0]]
    thresh = math.radians(angle_thresh_deg)
    for i in range(1, len(pts) - 1):
        a1 = math.atan2(pts[i][1] - pts[i - 1][1], pts[i][0] - pts[i - 1][0])
        a2 = math.atan2(pts[i + 1][1] - pts[i][1], pts[i + 1][0] - pts[i][0])
        if abs(_wrap(a2 - a1)) > thresh:
            ix, iy = grid.w2i(*pts[i])
            best_d = 0
            best_ix, best_iy = ix, iy
            for di in range(-search_r, search_r + 1):
                for dj in range(-search_r, search_r + 1):
                    if math.hypot(di, dj) > search_r:
                        continue
                    ni, nj = ix + di, iy + dj
                    if grid.ok(ni, nj) and not grid.wall[ni, nj]:
                        d = grid.dist_to_wall[ni, nj]
                        if d > best_d and d < MAX_CORRIDOR_DIST:
                            best_d = d
                            best_ix, best_iy = ni, nj
            result.append(grid.i2w(best_ix, best_iy))
        else:
            result.append(pts[i])
    result.append(pts[-1])
    return result


def _ensure_segment_clearance(pts, original_path, grid, min_dist=SAFETY_MIN_DIST):
    """Insert an A*-original midpoint where a simplified segment passes
    within ``min_dist`` cells of any wall (rotation-sweep safety)."""
    if grid.dist_to_wall is None:
        return pts
    result = [pts[0]]
    for i in range(1, len(pts)):
        dx = pts[i][0] - pts[i - 1][0]
        dy = pts[i][1] - pts[i - 1][1]
        length = math.hypot(dx, dy)
        n_samples = max(3, int(length / (RES * 0.5)))
        unsafe = False
        for k in range(n_samples + 1):
            t = k / n_samples
            x = pts[i - 1][0] + t * dx
            y = pts[i - 1][1] + t * dy
            ix, iy = grid.w2i(x, y)
            if not grid.ok(ix, iy) or grid.wall[ix, iy]:
                unsafe = True
                break
            if grid.dist_to_wall[ix, iy] < min_dist:
                unsafe = True
                break
        if unsafe and original_path:
            mid_x = (pts[i - 1][0] + pts[i][0]) / 2
            mid_y = (pts[i - 1][1] + pts[i][1]) / 2
            best = None
            best_score = float('inf')
            for op in original_path:
                oix, oiy = grid.w2i(op[0], op[1])
                if grid.ok(oix, oiy) and grid.dist_to_wall[oix, oiy] >= min_dist:
                    d = math.hypot(op[0] - mid_x, op[1] - mid_y)
                    if d < best_score:
                        best_score = d
                        best = op
            if best:
                result.append(best)
        result.append(pts[i])
    return result


def _split_long_segments(pts, max_seg=MAX_SEG_LEN):
    result = [pts[0]]
    for i in range(1, len(pts)):
        dx = pts[i][0] - pts[i - 1][0]
        dy = pts[i][1] - pts[i - 1][1]
        seg_len = math.hypot(dx, dy)
        if seg_len > max_seg:
            n_splits = int(math.ceil(seg_len / max_seg))
            for k in range(1, n_splits):
                t = k / n_splits
                result.append((pts[i - 1][0] + t * dx,
                               pts[i - 1][1] + t * dy))
        result.append(pts[i])
    return result


def simplify(path, grid):
    if len(path) <= 2:
        return path
    original = list(path)
    pts = _rdp(path, RDP_TOL)
    pts = _nudge_to_center(pts, grid)
    pts = _ensure_segment_clearance(pts, original, grid)
    pts = _split_long_segments(pts)
    return pts


# ═══════════════════════════════════════════════════════════════════════
#  5. Visualization
# ═══════════════════════════════════════════════════════════════════════

def save_map(grid_for_viz, pose, blue, yellow, pb=None, py=None,
             out_path="maps/final_map.png"):
    try:
        import matplotlib
        matplotlib.use("Agg")
        import matplotlib.pyplot as plt
    except Exception:
        return
    fig, ax = plt.subplots(1, 1, figsize=(12, 12))

    wi, wj = np.where(grid_for_viz.wall)
    wx_w = X_MIN + (wi + 0.5) * RES
    wy_w = Y_MIN + (wj + 0.5) * RES
    ax.scatter(wx_w, wy_w, s=1, c='black', alpha=0.6,
               label='Walls (npz)', zorder=2)

    if grid_for_viz.poison.any():
        pi, pj = np.where(grid_for_viz.poison)
        px = X_MIN + (pi + 0.5) * RES
        py_p = Y_MIN + (pj + 0.5) * RES
        ax.scatter(px, py_p, s=4, c='green', alpha=0.3,
                   label='Poison', zorder=3)

    rx, ry, _ = pose
    ax.plot(rx, ry, '^', color='limegreen', ms=16, zorder=10,
            markeredgecolor='black', markeredgewidth=1.5)
    ax.annotate(f"START\n({rx:.2f}, {ry:.2f})", xy=(rx, ry), fontsize=10,
                color='green', fontweight='bold',
                xytext=(rx - 0.5, ry - 0.3),
                arrowprops=dict(arrowstyle='->', color='green'))
    ax.plot(blue[0], blue[1], 'o', color='dodgerblue', ms=16, zorder=10,
            markeredgecolor='black', markeredgewidth=1.5)
    ax.annotate(f"BLUE\n({blue[0]:.2f}, {blue[1]:.2f})", xy=blue,
                fontsize=10, color='blue', fontweight='bold',
                xytext=(blue[0] + 0.2, blue[1] + 0.3),
                arrowprops=dict(arrowstyle='->', color='blue'))
    ax.plot(yellow[0], yellow[1], 'o', color='gold', ms=16, zorder=10,
            markeredgecolor='black', markeredgewidth=1.5)
    ax.annotate(f"YELLOW\n({yellow[0]:.2f}, {yellow[1]:.2f})", xy=yellow,
                fontsize=10, color='goldenrod', fontweight='bold',
                xytext=(yellow[0] - 0.8, yellow[1] - 0.3),
                arrowprops=dict(arrowstyle='->', color='goldenrod'))

    for p, cl, lb in [(pb, "dodgerblue", "→BLUE"),
                       (py, "orangered", "→YELLOW")]:
        if not p:
            continue
        xs = [q[0] for q in p]
        ys = [q[1] for q in p]
        pl = sum(math.hypot(b[0] - a[0], b[1] - a[1])
                 for a, b in zip(p[:-1], p[1:]))
        ax.plot(xs, ys, '-o', color=cl, lw=2.5, ms=6, alpha=0.9,
                label=f"A* {lb} ({pl:.2f}m)", zorder=8)
        for i, (wx, wy) in enumerate(p):
            ax.annotate(f"W{i}", xy=(wx, wy), fontsize=7, color=cl,
                        xytext=(3, 3), textcoords='offset points')

    ax.set_xlabel("x [m]", fontsize=12)
    ax.set_ylabel("y [m]", fontsize=12)
    ax.set_title("Walls (npz['occupied']) + EDT corridor-centered A*",
                 fontsize=14)
    ax.legend(fontsize=11, loc='upper left')
    ax.set_aspect('equal')
    ax.grid(True, alpha=0.3, ls='--')

    allx = [rx, blue[0], yellow[0]]
    ally = [ry, blue[1], yellow[1]]
    if pb:
        allx += [q[0] for q in pb]
        ally += [q[1] for q in pb]
    if py:
        allx += [q[0] for q in py]
        ally += [q[1] for q in py]
    margin = 0.5
    ax.set_xlim(min(allx) - margin, max(allx) + margin)
    ax.set_ylim(min(ally) - margin, max(ally) + margin)

    fig.tight_layout()
    fig.savefig(out_path, dpi=150)
    plt.close(fig)
    print(f"[viz] saved {out_path}")


# ═══════════════════════════════════════════════════════════════════════
#  6. Controller
# ═══════════════════════════════════════════════════════════════════════

class MapRunner:
    def __init__(self):
        self.io = RobotIO()
        self.dt = self.io.timestep * 1e-3
        self.odom = Odometry()
        self.scan = (ScanProcessor(self.io.lidar)
                     if self.io.lidar is not None else None)
        self.sim_time = 0.0
        self.maps_dir = os.path.join(_HERE, "maps")
        os.makedirs(self.maps_dir, exist_ok=True)

    def _step(self):
        if self.io.step() == -1:
            return False
        self.sim_time += self.dt
        wl, wr = self.io.read_encoders()
        if wl is not None:
            self.odom.update(wl, wr, self.io.read_yaw(), self.dt)
        return True

    # ----------------------------- localization -------------------------

    def localize_via_pillars(self, map_blue, map_yellow):
        """Spin in place; observe both pillars; solve rigid (R, t)
        transform from saved-map frame into current odom frame.

        Returns (R, t) on success, None on failure.
        """
        print("\n[localize] spinning to observe pillars...")
        if self.io.camera is None or self.io.depth is None:
            print("[localize] camera/depth missing — cannot localize")
            return None
        detector = PillarDetector(self.io.camera, self.io.depth)
        obs = {"blue": [], "yellow": []}
        target_acc = 2 * math.pi * LOCALIZE_REVS
        last_th = self.odom.pose()[2]
        spun = 0.0
        while spun < target_acc:
            self.io.set_cmd(0.0, LOCALIZE_W)
            if not self._step():
                return None
            cur_th = self.odom.pose()[2]
            spun += abs(_wrap(cur_th - last_th))
            last_th = cur_th
            dets = detector.detect()
            rx, ry, rth = self.odom.pose()
            for color in ("blue", "yellow"):
                det = dets.get(color)
                if det is None:
                    continue
                rng = det.get("range", float("nan"))
                if not (np.isfinite(rng) and
                        LOCALIZE_RNG_MIN < rng < LOCALIZE_RNG_MAX):
                    continue
                bx = rx + rng * math.cos(rth + det["bearing"])
                by = ry + rng * math.sin(rth + det["bearing"])
                obs[color].append((bx, by))
        self.io.stop()
        for _ in range(5):
            self._step()

        print(f"[localize] obs: blue={len(obs['blue'])}, "
              f"yellow={len(obs['yellow'])}")
        if (len(obs["blue"]) < LOCALIZE_MIN_OBS or
                len(obs["yellow"]) < LOCALIZE_MIN_OBS):
            print("[localize] insufficient obs — using identity transform")
            return None

        obs_blue   = np.median(obs["blue"],   axis=0)
        obs_yellow = np.median(obs["yellow"], axis=0)
        map_b = np.array(map_blue,   dtype=np.float64)
        map_y = np.array(map_yellow, dtype=np.float64)

        v_obs = obs_yellow - obs_blue
        v_map = map_y      - map_b
        if np.linalg.norm(v_map) < 1e-6:
            print("[localize] degenerate map vector — using identity")
            return None
        dtheta = (math.atan2(v_obs[1], v_obs[0]) -
                  math.atan2(v_map[1], v_map[0]))
        c, s = math.cos(dtheta), math.sin(dtheta)
        R = np.array([[c, -s], [s, c]])
        t = ((obs_blue + obs_yellow) / 2 -
             R @ ((map_b + map_y) / 2))

        proj_b = R @ map_b + t
        proj_y = R @ map_y + t
        err_b = float(np.linalg.norm(obs_blue - proj_b))
        err_y = float(np.linalg.norm(obs_yellow - proj_y))
        print(f"[localize] dtheta={math.degrees(dtheta):+7.2f}°  "
              f"dxy=({t[0]:+.3f},{t[1]:+.3f})")
        print(f"[localize] residual: blue={err_b:.3f}m  yellow={err_y:.3f}m")
        return R, t

    # ----------------------------- execution ----------------------------

    def _exec_path(self, path_world, label):
        """Closed-loop pure-pursuit drive over the simplified A* path.

        Replaces the old open-loop ROT-then-DRV: the robot never rotates
        in place at a corner waypoint, so the chassis cannot sweep into
        an adjacent wall during a turn (the W7→W8 bug).
        """
        print(f"\n[exec] ── {label} (pure-pursuit) ──")
        follower = PathFollower()
        follower.set_path(path_world, final_tol=C.WAYPOINT_REACH_TOL)
        timeout_t = self.sim_time + EXEC_TIMEOUT_S
        last_log_t = self.sim_time

        while follower.has_path():
            if not self._step():
                return False
            if self.sim_time > timeout_t:
                print(f"[exec] {label} TIMEOUT after {EXEC_TIMEOUT_S}s")
                self.io.stop()
                return False
            ranges = np.zeros(0)
            angles = np.zeros(0)
            if self.scan is not None:
                _, ranges = self.scan.scan_world(self.odom.pose())
                angles = self.scan._angles
            v, w, status = follower.step(self.odom.pose(), ranges, angles)
            self.io.set_cmd(v, w)
            if status == "arrived":
                break
            if (self.sim_time - last_log_t) > 1.0:
                rx, ry, _ = self.odom.pose()
                rem = follower.remaining_distance(self.odom.pose())
                print(f"[exec]   t={self.sim_time:6.2f}s "
                      f"pos=({rx:+.2f},{ry:+.2f}) rem={rem:.2f}m "
                      f"v={v:+.2f} w={w:+.2f} {status}")
                last_log_t = self.sim_time

        self.io.stop()
        for _ in range(3):
            self._step()
        rx, ry, _ = self.odom.pose()
        print(f"[exec] {label} done @ ({rx:+.2f},{ry:+.2f})")
        return True

    # ----------------------------- main loop ----------------------------

    def run(self):
        print("\n" + "=" * 60)
        print("  MAP RUNNER — npz walls + pillar localization + pure-pursuit")
        print("=" * 60 + "\n")

        # Warmup so encoders + IMU stabilise
        for _ in range(10):
            if not self._step():
                return

        npz_path = os.path.normpath(
            os.path.join(_HERE, "..", "teleop_mapping", "maps", "map.npz"))
        if not os.path.isfile(npz_path):
            print(f"[plan] ERROR: {npz_path} not found — run teleop first")
            self.io.stop()
            return
        npz = np.load(npz_path, allow_pickle=False)

        map_blue   = (float(npz["pillar_blue"][0]),
                      float(npz["pillar_blue"][1]))
        map_yellow = (float(npz["pillar_yellow"][0]),
                      float(npz["pillar_yellow"][1]))
        if not (np.isfinite(map_blue[0]) and np.isfinite(map_yellow[0])):
            print("[plan] ERROR: pillar positions in npz are NaN — re-run teleop")
            self.io.stop()
            return
        print(f"[plan] map BLUE   @ ({map_blue[0]:+.3f}, {map_blue[1]:+.3f})")
        print(f"[plan] map YELLOW @ ({map_yellow[0]:+.3f}, {map_yellow[1]:+.3f})")

        # ── Issue 3 fix: localize via pillars ──
        result = self.localize_via_pillars(map_blue, map_yellow)
        if result is None:
            R = np.eye(2)
            t = np.zeros(2)
        else:
            R, t = result

        blue   = tuple(R @ np.array(map_blue)   + t)
        yellow = tuple(R @ np.array(map_yellow) + t)
        ply_path = os.path.normpath(
            os.path.join(_HERE, "..", "teleop_mapping", "maps", "scene.ply"))
        wall_mask   = load_walls_hybrid(npz, ply_path, R, t)
        poison_mask = load_poison_from_npz(npz, R, t)
        print(f"[plan] walls: {wall_mask.sum()} cells; "
              f"poison: {poison_mask.sum()} cells")
        print(f"[plan] BLUE   (odom) @ ({blue[0]:+.3f}, {blue[1]:+.3f})")
        print(f"[plan] YELLOW (odom) @ ({yellow[0]:+.3f}, {yellow[1]:+.3f})")

        rx, ry, rth = self.odom.pose()
        print(f"[plan] start  @ ({rx:+.3f}, {ry:+.3f}, "
              f"{math.degrees(rth):+.1f}°)")

        # ── A* leg 1: START → BLUE (adaptive inflation) ──
        print(f"\n[plan] A*: START → BLUE")
        direct1 = math.hypot(rx - blue[0], ry - blue[1])
        pb = None
        grid_viz = None
        for inf_r in [INFLATE, 3, 2, 1]:
            grid1 = Grid(wall_mask, poison_mask)
            grid1.inflate(r=inf_r)
            grid1.carve(rx, ry)
            grid1.carve(blue[0], blue[1])
            grid1.carve(yellow[0], yellow[1])
            pb_raw = astar(grid1, (rx, ry), blue)
            if pb_raw is None:
                print(f"  inflate={inf_r}: no path, trying smaller...")
                continue
            pb = simplify(pb_raw, grid1)
            pl = sum(math.hypot(b[0] - a[0], b[1] - a[1])
                     for a, b in zip(pb[:-1], pb[1:]))
            if pl > direct1 * 5.0:
                print(f"  inflate={inf_r}: path {pl:.2f}m > 5x direct, "
                      f"trying tighter...")
                pb = None
                continue
            print(f"  ✓ inflate={inf_r}: {len(pb)} waypoints, {pl:.2f}m")
            grid_viz = grid1
            break
        if pb is None:
            print("[plan] FAILED — no path to BLUE!")
            self.io.stop()
            return

        # ── A* leg 2: BLUE → YELLOW ──
        print(f"\n[plan] A*: BLUE → YELLOW")
        direct2 = math.hypot(blue[0] - yellow[0], blue[1] - yellow[1])
        py = None
        for inf_r in [INFLATE, 3, 2, 1]:
            grid2 = Grid(wall_mask, poison_mask)
            grid2.inflate(r=inf_r)
            grid2.carve(blue[0], blue[1])
            grid2.carve(yellow[0], yellow[1])
            py_raw = astar(grid2, blue, yellow)
            if py_raw is None:
                print(f"  inflate={inf_r}: no path, trying smaller...")
                continue
            py = simplify(py_raw, grid2)
            pl2 = sum(math.hypot(b[0] - a[0], b[1] - a[1])
                      for a, b in zip(py[:-1], py[1:]))
            if pl2 > direct2 * 8.0:
                print(f"  inflate={inf_r}: path {pl2:.2f}m > 8x direct, "
                      f"trying tighter...")
                py = None
                continue
            print(f"  ✓ inflate={inf_r}: {len(py)} waypoints, {pl2:.2f}m")
            break
        if py is None:
            print("[plan] WARNING — no path to YELLOW; only leg 1 will run")

        save_map(grid_viz, self.odom.pose(), blue, yellow, pb, py,
                 os.path.join(self.maps_dir, "final_map.png"))

        # ── execute leg 1 ──
        print(f"\n{'=' * 60}\n  EXECUTE LEG 1: START → BLUE\n{'=' * 60}")
        t0 = self.sim_time
        if not self._exec_path(pb, "LEG 1 → BLUE"):
            return
        x, y, _ = self.odom.pose()
        err1 = math.hypot(x - blue[0], y - blue[1])
        print(f"\n[result] LEG 1: {self.sim_time - t0:.1f}s  "
              f"pos=({x:+.2f},{y:+.2f})  err to BLUE = {err1:.3f}m")

        # ── execute leg 2 ──
        if py:
            print(f"\n{'=' * 60}\n  EXECUTE LEG 2: BLUE → YELLOW\n{'=' * 60}")
            t1 = self.sim_time
            if not self._exec_path(py, "LEG 2 → YELLOW"):
                return
            x, y, _ = self.odom.pose()
            err2 = math.hypot(x - yellow[0], y - yellow[1])
            print(f"\n[result] LEG 2: {self.sim_time - t1:.1f}s  "
                  f"pos=({x:+.2f},{y:+.2f})  err to YELLOW = {err2:.3f}m")

        self.io.stop()
        print(f"\n{'═' * 60}\n  MISSION COMPLETE\n{'═' * 60}\n")


def main():
    MapRunner().run()


if __name__ == "__main__":
    main()
