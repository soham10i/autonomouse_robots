"""Maze2 standalone autonomous explorer — no Maze1 dependencies.

Uses VFH (Vector Field Histogram) reactive navigation with frontier-based
exploration. LiDAR is the primary sensor for both mapping and navigation.
"""
import os, sys, math, time
import numpy as np
from controller import Robot, Keyboard

# ── Robot hardware constants ─────────────────────────────────────────────
WHEEL_R = 0.043
WHEEL_SEP = 0.220
ROBOT_R = 0.125
V_MAX = 0.40
W_MAX = 2.0

MOTOR_NAMES = ("fl_wheel_joint", "fr_wheel_joint",
               "rl_wheel_joint", "rr_wheel_joint")
ENCODER_NAMES = ("front left wheel motor sensor", "front right wheel motor sensor",
                 "rear left wheel motor sensor", "rear right wheel motor sensor")

# ── Grid constants ───────────────────────────────────────────────────────
GRID_RES = 0.04
GRID_SIZE = 10.0
GRID_CELLS = int(GRID_SIZE / GRID_RES)
GRID_ORIGIN = np.array([-5.0, -5.0])

# ── Navigation constants ────────────────────────────────────────────────
SAFETY_DIST = 0.14          # min distance to obstacles (must fit tight corridors)
FWD_BRAKE_DIST = 0.18       # hard-brake if forward clearance < this
FWD_SLOW_DIST = 0.45        # start slowing down
NUM_SECTORS = 72            # VFH sectors (5° each)
SECTOR_WIDTH = 2 * math.pi / NUM_SECTORS

_HERE = os.path.dirname(os.path.abspath(__file__))
MAPS_DIR = os.path.join(_HERE, "maps")
os.makedirs(MAPS_DIR, exist_ok=True)


class OccGrid:
    """Simple log-odds occupancy grid."""
    def __init__(self):
        self.L = np.zeros((GRID_CELLS, GRID_CELLS), dtype=np.float32)
        self.poison = np.zeros((GRID_CELLS, GRID_CELLS), dtype=bool)

    def w2i(self, wx, wy):
        return (int((wx - GRID_ORIGIN[0]) / GRID_RES),
                int((wy - GRID_ORIGIN[1]) / GRID_RES))

    def i2w(self, ix, iy):
        return (GRID_ORIGIN[0] + (ix + 0.5) * GRID_RES,
                GRID_ORIGIN[1] + (iy + 0.5) * GRID_RES)

    def _inbounds(self, ix, iy):
        return 0 <= ix < GRID_CELLS and 0 <= iy < GRID_CELLS

    def integrate(self, pose, hits_xy):
        """Raytrace free space + mark endpoints occupied."""
        if hits_xy.shape[0] == 0:
            return
        rx, ry, _ = pose
        sx, sy = self.w2i(rx, ry)
        ix = np.floor((hits_xy[:, 0] - GRID_ORIGIN[0]) / GRID_RES).astype(int)
        iy = np.floor((hits_xy[:, 1] - GRID_ORIGIN[1]) / GRID_RES).astype(int)
        m = (ix >= 0) & (ix < GRID_CELLS) & (iy >= 0) & (iy < GRID_CELLS)
        for tx, ty in zip(ix[m], iy[m]):
            for fx, fy in _bresenham(sx, sy, tx, ty)[:-1]:
                if self._inbounds(fx, fy):
                    self.L[fx, fy] = max(self.L[fx, fy] - 0.4, -2.0)
            self.L[tx, ty] = min(self.L[tx, ty] + 0.85, 3.5)

    def occupied(self):  return self.L >= 0.85
    def free(self):      return self.L <= -0.4
    def unknown(self):   return np.abs(self.L) < 0.05

    def find_frontiers(self, pose, blacklist):
        """Return (wx, wy) of best frontier or None."""
        free = self.free()
        unk = self.unknown()
        # Frontier = free cell adjacent to unknown
        pad = np.zeros_like(unk)
        pad[1:] |= unk[:-1]; pad[:-1] |= unk[1:]
        pad[:, 1:] |= unk[:, :-1]; pad[:, :-1] |= unk[:, 1:]
        front = free & pad
        if not front.any():
            return None
        rx, ry, rth = pose
        try:
            from scipy.ndimage import label
            lab, n = label(front)
        except ImportError:
            return self._simple_frontier(front, pose, blacklist)
        if n == 0:
            return None
        sizes = np.bincount(lab.ravel()); sizes[0] = 0
        best, best_score = None, -1e9
        for k in range(1, n + 1):
            if sizes[k] < 4:
                continue
            ixs, iys = np.where(lab == k)
            cx, cy = float(ixs.mean()), float(iys.mean())
            wx, wy = self.i2w(int(cx), int(cy))
            d = math.hypot(wx - rx, wy - ry)
            if d < 0.3:
                continue
            if any(math.hypot(wx - bx, wy - by) < 0.4 for bx, by in blacklist):
                continue
            score = sizes[k] / (1.0 + d)
            if score > best_score:
                best_score = score
                best = (wx, wy)
        return best

    def _simple_frontier(self, front, pose, blacklist):
        ixs, iys = np.where(front)
        if len(ixs) == 0:
            return None
        rx, ry, _ = pose
        best, best_d = None, 1e9
        for i in range(0, len(ixs), max(1, len(ixs)//50)):
            wx, wy = self.i2w(int(ixs[i]), int(iys[i]))
            d = math.hypot(wx - rx, wy - ry)
            if d < 0.3: continue
            if any(math.hypot(wx-bx, wy-by) < 0.4 for bx, by in blacklist):
                continue
            if d < best_d:
                best_d = d; best = (wx, wy)
        return best


def _bresenham(x0, y0, x1, y1):
    dx, dy = abs(x1-x0), abs(y1-y0)
    sx = 1 if x0 < x1 else -1
    sy = 1 if y0 < y1 else -1
    err = dx - dy; x, y = x0, y0; out = []
    while True:
        out.append((x, y))
        if x == x1 and y == y1: break
        e2 = 2 * err
        if e2 > -dy: err -= dy; x += sx
        if e2 < dx:  err += dx; y += sy
    return out


class Navigator:
    """VFH-based reactive navigator with wall-following fallback."""

    def __init__(self):
        self.goal = None          # (wx, wy) or None
        self._wall_follow_dir = 1.0  # +1 = left wall, -1 = right wall

    def set_goal(self, xy):
        self.goal = xy

    def compute_cmd(self, pose, ranges, angles, tick=0):
        """Return (v, w) using VFH gap selection."""
        rx, ry, rth = pose
        ra = np.asarray(ranges, dtype=np.float32)
        an = np.asarray(angles, dtype=np.float64)
        do_log = (tick % 64 == 0)  # diagnostic log every 64 ticks

        # Build polar obstacle histogram
        histogram = np.full(NUM_SECTORS, 999.0)
        valid = np.isfinite(ra) & (ra > 0.05) & (ra < 4.0)
        if valid.any():
            for r, a in zip(ra[valid], an[valid]):
                # Sector index (wrap to [0, NUM_SECTORS))
                sector = int((a + math.pi) / SECTOR_WIDTH) % NUM_SECTORS
                if r < histogram[sector]:
                    histogram[sector] = r

        # Determine blocked sectors (obstacle closer than safety distance)
        blocked = histogram < SAFETY_DIST

        # Inflate blocked sectors by robot angular width
        inflated = blocked.copy()
        for i in range(NUM_SECTORS):
            if blocked[i]:
                r = max(histogram[i], 0.05)
                spread = int(math.ceil(math.asin(min(0.99, ROBOT_R / r)) / SECTOR_WIDTH))
                for j in range(-spread, spread + 1):
                    inflated[(i + j) % NUM_SECTORS] = True

        # Goal bearing (in sector space)
        if self.goal is not None:
            goal_angle = math.atan2(self.goal[1] - ry, self.goal[0] - rx) - rth
        else:
            goal_angle = 0.0  # default: forward
        goal_sector = int((goal_angle + math.pi) / SECTOR_WIDTH) % NUM_SECTORS

        # Find gaps (runs of unblocked sectors)
        gaps = []
        start = None
        for i in range(NUM_SECTORS * 2):  # wrap-around search
            idx = i % NUM_SECTORS
            if not inflated[idx]:
                if start is None: start = i
            else:
                if start is not None:
                    gaps.append((start, i - 1))
                    start = None
        if start is not None:
            gaps.append((start, NUM_SECTORS * 2 - 1))

        # Remove duplicate wrap-around gaps
        seen = set()
        unique_gaps = []
        for s, e in gaps:
            key = (s % NUM_SECTORS, e % NUM_SECTORS)
            if key not in seen:
                seen.add(key)
                unique_gaps.append((s, e))
        gaps = unique_gaps

        n_blocked = int(inflated.sum())
        if do_log:
            print(f"[vfh] blocked={n_blocked}/{NUM_SECTORS} gaps={len(gaps)}", flush=True)

        if not gaps:
            print(f"[nav] fully blocked ({n_blocked}/{NUM_SECTORS}), spinning", flush=True)
            self._wall_follow_dir *= -1  # alternate spin direction
            return 0.0, self._wall_follow_dir * W_MAX * 0.5

        # Score each gap: prefer alignment with goal + wider gaps
        best_gap = None
        best_score = -1e9
        for s, e in gaps:
            width = e - s + 1
            if width < 2:
                continue
            center_sector = (s + e) // 2
            center_angle = (center_sector * SECTOR_WIDTH) - math.pi
            # Alignment with goal
            diff = abs(((center_sector - goal_sector + NUM_SECTORS//2) % NUM_SECTORS) - NUM_SECTORS//2)
            alignment = 1.0 - diff / (NUM_SECTORS / 2)
            # Gap width score
            width_score = min(width, 20) / 20.0
            # Min range in gap (depth)
            gap_ranges = [histogram[j % NUM_SECTORS] for j in range(s, e+1)]
            depth = min(min(gap_ranges), 4.0) / 4.0
            score = alignment * 3.0 + width_score * 1.5 + depth * 0.5
            if score > best_score:
                best_score = score
                best_gap = (s, e, center_angle)

        if best_gap is None:
            if do_log:
                print("[nav] no usable gap (all too narrow)", flush=True)
            return 0.0, self._wall_follow_dir * W_MAX * 0.5

        gap_s, gap_e, steer_angle = best_gap
        gap_width = gap_e - gap_s + 1

        # Steering
        steer = _wrap(steer_angle)
        if abs(steer) > 1.2:
            return 0.0, _clamp(2.0 * steer, -W_MAX, W_MAX)

        w = _clamp(2.5 * steer, -W_MAX, W_MAX)

        # Speed: forward clearance — use narrow cone + 10th percentile
        # (not min) to reject outlier returns from wall edges
        fwd_mask = valid & (np.abs(an) < math.radians(15))
        if fwd_mask.any():
            fwd_vals = np.sort(ra[fwd_mask])
            idx = max(0, int(len(fwd_vals) * 0.10))  # 10th percentile
            fwd_clear = float(fwd_vals[idx])
        else:
            fwd_clear = float("inf")

        if do_log:
            print(f"[vfh] steer={math.degrees(steer):.0f}° "
                  f"fwd={fwd_clear:.2f}m gap_w={gap_width}", flush=True)

        if fwd_clear < FWD_BRAKE_DIST:
            # Even when braking, creep forward slowly if gap exists
            v = 0.04 if gap_width > 4 else 0.0
            return v, _clamp(2.0 * steer, -W_MAX, W_MAX)

        if fwd_clear < FWD_SLOW_DIST:
            scale = (fwd_clear - FWD_BRAKE_DIST) / max(FWD_SLOW_DIST - FWD_BRAKE_DIST, 0.01)
            v = V_MAX * max(0.20, scale) * math.cos(steer)
        else:
            v = V_MAX * math.cos(steer)

        # Never return v=0 when we found a good gap — anti-spin-lock
        v = max(0.05, _clamp(v, 0.0, V_MAX))
        return v, w


def _wrap(a):
    while a > math.pi: a -= 2*math.pi
    while a <= -math.pi: a += 2*math.pi
    return a

def _clamp(v, lo, hi):
    return max(lo, min(hi, v))


class AutoExplorer:
    def __init__(self):
        self.robot = Robot()
        self.dt_ms = int(self.robot.getBasicTimeStep())
        self.dt = self.dt_ms * 1e-3

        # Motors
        self.motors = [self.robot.getDevice(n) for n in MOTOR_NAMES]
        for m in self.motors:
            m.setPosition(float("inf"))
            m.setVelocity(0.0)

        # Encoders
        self.encoders = [self.robot.getDevice(n) for n in ENCODER_NAMES]
        for e in self.encoders:
            if e: e.enable(self.dt_ms)

        # LiDAR
        self.lidar = self.robot.getDevice("laser")
        self.lidar.enable(self.dt_ms)
        self.lidar.enablePointCloud()
        self.lidar_n = self.lidar.getHorizontalResolution()
        fov = self.lidar.getFov()
        self._lidar_angles = (fov/2) - (fov / max(self.lidar_n-1, 1)) * np.arange(self.lidar_n)

        # IMU
        self.imu = self.robot.getDevice("imu inertial_unit")
        if self.imu: self.imu.enable(self.dt_ms)

        # Keyboard
        self.kb = self.robot.getKeyboard()
        self.kb.enable(self.dt_ms)

        # Odometry state
        self.x, self.y, self.theta = 0.0, 0.0, 0.0
        self._prev_enc = None

        # Mapping & navigation
        self.grid = OccGrid()
        self.nav = Navigator()
        self.frontier_blacklist = []
        self.cur_goal = None
        self.pose_history = []

        # Stuck detection
        self._stuck_ref = None
        self._stuck_t = 0.0

        # Timing
        self.sim_time = 0.0
        self.tick = 0
        self._last_png_t = -10.0

        print(f"[explorer] started | dt={self.dt_ms}ms | lidar_n={self.lidar_n}", flush=True)

    def _set_cmd(self, v, w):
        wr = (2*v + w*WHEEL_SEP) / (2*WHEEL_R)
        wl = (2*v - w*WHEEL_SEP) / (2*WHEEL_R)
        mx = V_MAX / WHEEL_R * 1.4
        wl, wr = _clamp(wl, -mx, mx), _clamp(wr, -mx, mx)
        self.motors[0].setVelocity(wl); self.motors[2].setVelocity(wl)
        self.motors[1].setVelocity(wr); self.motors[3].setVelocity(wr)

    def _stop(self):
        for m in self.motors: m.setVelocity(0.0)

    def _update_odom(self):
        vals = [e.getValue() if e else 0 for e in self.encoders]
        wl = 0.5 * (vals[0] + vals[2])
        wr = 0.5 * (vals[1] + vals[3])
        if self._prev_enc is None:
            self._prev_enc = (wl, wr)
            if self.imu:
                rpy = self.imu.getRollPitchYaw()
                self.theta = rpy[2]
            return
        dl = (wl - self._prev_enc[0]) * WHEEL_R
        dr = (wr - self._prev_enc[1]) * WHEEL_R
        self._prev_enc = (wl, wr)
        ds = 0.5 * (dl + dr)
        dth = (dr - dl) / WHEEL_SEP
        mid_th = self.theta + 0.5 * dth
        self.x += ds * math.cos(mid_th)
        self.y += ds * math.sin(mid_th)
        if self.imu:
            rpy = self.imu.getRollPitchYaw()
            self.theta = rpy[2]
        else:
            self.theta = _wrap(self.theta + dth)

    def _pose(self):
        return (self.x, self.y, self.theta)

    def _lidar_hits(self, pose):
        """Return (M, 2) world-frame hit points."""
        ra = np.asarray(self.lidar.getRangeImage(), dtype=np.float32)
        rmin, rmax = max(self.lidar.getMinRange(), 0.05), min(self.lidar.getMaxRange(), 8.0)
        valid = np.isfinite(ra) & (ra > rmin) & (ra < rmax)
        a = self._lidar_angles[valid]
        r = ra[valid]
        x, y, th = pose
        wx = x + r * np.cos(th + a)
        wy = y + r * np.sin(th + a)
        return np.stack([wx, wy], axis=1), ra

    def _check_stuck(self, pose):
        x, y, _ = pose
        if self._stuck_ref is None:
            self._stuck_ref = (x, y)
            self._stuck_t = self.sim_time
            return False
        d = math.hypot(x - self._stuck_ref[0], y - self._stuck_ref[1])
        if d > 0.12:
            self._stuck_ref = (x, y)
            self._stuck_t = self.sim_time
            return False
        return (self.sim_time - self._stuck_t) > 5.0

    def _recovery(self):
        """Reverse then spin."""
        print(f"[recovery] t={self.sim_time:.1f}s reversing + spinning", flush=True)
        t0 = self.sim_time
        # Reverse 0.8s
        while self.robot.step(self.dt_ms) != -1:
            self.sim_time += self.dt
            self._set_cmd(-0.12, 0.0)
            if self.sim_time - t0 > 0.8: break
        # Spin 1.0s
        spin_dir = 1.0 if np.random.random() > 0.5 else -1.0
        t0 = self.sim_time
        while self.robot.step(self.dt_ms) != -1:
            self.sim_time += self.dt
            self._set_cmd(0.0, spin_dir * 1.5)
            if self.sim_time - t0 > 1.0: break
        self._stop()
        self._stuck_ref = None
        self._stuck_t = self.sim_time

    def _save_png(self, pose, suffix=""):
        if self.sim_time - self._last_png_t < 3.0:
            return
        self._last_png_t = self.sim_time
        try:
            import matplotlib; matplotlib.use("Agg")
            import matplotlib.pyplot as plt
        except: return
        p = 1.0 - 1.0 / (1.0 + np.exp(self.grid.L))
        rgb = np.ones((GRID_CELLS, GRID_CELLS, 3), dtype=np.float32)
        rgb[self.grid.unknown()] = 0.78
        rgb[self.grid.occupied()] = 0.05
        ext = [GRID_ORIGIN[0], GRID_ORIGIN[0]+GRID_SIZE,
               GRID_ORIGIN[1], GRID_ORIGIN[1]+GRID_SIZE]
        fig, ax = plt.subplots(figsize=(6,6), dpi=120)
        ax.imshow(rgb.transpose(1,0,2), origin="lower", extent=ext)
        if self.pose_history:
            xs = [p[0] for p in self.pose_history]
            ys = [p[1] for p in self.pose_history]
            ax.plot(xs, ys, "b-", lw=0.8)
        ax.plot(pose[0], pose[1], "r.", ms=6)
        if self.cur_goal:
            ax.plot(self.cur_goal[0], self.cur_goal[1], "x", color="orange", ms=10)
        ax.set_title(f"t={self.sim_time:.1f}s unk={int(self.grid.unknown().sum())}")
        ax.set_aspect("equal")
        name = f"explore_{int(self.sim_time):04d}{suffix}.png"
        fig.savefig(os.path.join(MAPS_DIR, name), bbox_inches="tight")
        plt.close(fig)

    def _finalize(self):
        print("[explorer] finalizing...", flush=True)
        pose = self._pose()
        self._save_png(pose, suffix="_final")
        # Save map.npz
        try:
            np.savez_compressed(os.path.join(MAPS_DIR, "map.npz"),
                occupied=self.grid.occupied(),
                free=self.grid.free(),
                unknown=self.grid.unknown(),
                resolution=np.float32(GRID_RES),
                origin=GRID_ORIGIN.astype(np.float32),
                cells=np.int32(GRID_CELLS),
                pose_history=np.array(self.pose_history, dtype=np.float32))
            print(f"[explorer] wrote map.npz "
                  f"occ={int(self.grid.occupied().sum())} "
                  f"free={int(self.grid.free().sum())} "
                  f"unk={int(self.grid.unknown().sum())}", flush=True)
        except Exception as e:
            print(f"[explorer] map.npz failed: {e}", flush=True)
        # Final map PNG
        try:
            import matplotlib; matplotlib.use("Agg")
            import matplotlib.pyplot as plt
            rgb = np.ones((GRID_CELLS, GRID_CELLS, 3), dtype=np.float32)
            rgb[self.grid.unknown()] = 0.82
            rgb[self.grid.occupied()] = 0.05
            ext = [GRID_ORIGIN[0], GRID_ORIGIN[0]+GRID_SIZE,
                   GRID_ORIGIN[1], GRID_ORIGIN[1]+GRID_SIZE]
            fig, ax = plt.subplots(figsize=(6,6), dpi=160)
            ax.imshow(rgb.transpose(1,0,2), origin="lower", extent=ext)
            if self.pose_history:
                xs = [p[0] for p in self.pose_history]
                ys = [p[1] for p in self.pose_history]
                ax.plot(xs, ys, "b-", lw=1.0)
                ax.plot(xs[0], ys[0], "go", ms=8, label="start")
            ax.set_title("Final Map"); ax.set_aspect("equal")
            ax.legend(fontsize=8); ax.grid(True, color="0.85", lw=0.3)
            fig.savefig(os.path.join(MAPS_DIR, "final_map.png"), bbox_inches="tight")
            plt.close(fig)
            print("[explorer] wrote final_map.png", flush=True)
        except Exception as e:
            print(f"[explorer] final_map.png failed: {e}", flush=True)

    def run(self):
        no_frontier_t = 0.0
        replan_t = -10.0
        print("[explorer] entering main loop", flush=True)

        try:
            while self.robot.step(self.dt_ms) != -1:
                self.sim_time += self.dt
                self.tick += 1
                self._update_odom()
                pose = self._pose()

                if self.tick % 5 == 0:
                    self.pose_history.append(pose)

                # Check keyboard
                k = self.kb.getKey()
                if k == ord('Q') or k == ord('q'):
                    print("[explorer] Q pressed, stopping", flush=True)
                    break

                # LiDAR scan + mapping
                hits, raw_ranges = self._lidar_hits(pose)
                self.grid.integrate(pose, hits)

                # Carve free around robot
                ix, iy = self.grid.w2i(pose[0], pose[1])
                n = max(1, int(ROBOT_R * 0.7 / GRID_RES))
                for dx in range(-n, n+1):
                    for dy in range(-n, n+1):
                        if dx*dx + dy*dy <= n*n:
                            cx, cy = ix+dx, iy+dy
                            if self.grid._inbounds(cx, cy):
                                self.grid.L[cx, cy] = min(self.grid.L[cx, cy], -0.4)

                # Frontier-based goal selection (every 2s or when no goal)
                if (self.sim_time - replan_t > 2.0) or self.cur_goal is None:
                    replan_t = self.sim_time
                    f = self.grid.find_frontiers(pose, self.frontier_blacklist)
                    if f is not None:
                        # Check if we reached the old goal
                        if self.cur_goal is not None:
                            d_goal = math.hypot(pose[0]-self.cur_goal[0],
                                                pose[1]-self.cur_goal[1])
                            if d_goal < 0.4:
                                print(f"[explorer] reached goal, picking new", flush=True)
                        self.cur_goal = f
                        self.nav.set_goal(f)
                        no_frontier_t = self.sim_time
                        if self.tick % 20 == 0:
                            print(f"[t={self.sim_time:.1f}s] goal=({f[0]:+.2f},{f[1]:+.2f}) "
                                  f"unk={int(self.grid.unknown().sum())} "
                                  f"pose=({pose[0]:+.2f},{pose[1]:+.2f},{pose[2]:+.2f})",
                                  flush=True)
                    else:
                        if self.sim_time - no_frontier_t > 20.0:
                            print("[explorer] no frontiers for 20s, DONE", flush=True)
                            break

                # Stuck detection + recovery
                if self._check_stuck(pose):
                    if self.cur_goal is not None:
                        self.frontier_blacklist.append(self.cur_goal)
                        self.frontier_blacklist = self.frontier_blacklist[-20:]
                        self.cur_goal = None
                    self._recovery()
                    continue

                # VFH navigation
                v, w = self.nav.compute_cmd(pose, raw_ranges, self._lidar_angles, tick=self.tick)
                self._set_cmd(v, w)

                # Periodic PNG
                self._save_png(pose)

                # Periodic log
                if self.tick % 32 == 0:
                    unk = int(self.grid.unknown().sum())
                    goal_str = (f"({self.cur_goal[0]:+.2f},{self.cur_goal[1]:+.2f})"
                                if self.cur_goal else "None")
                    print(f"[t={self.sim_time:6.1f}s] "
                          f"pose=({pose[0]:+.2f},{pose[1]:+.2f},{pose[2]:+.2f}) "
                          f"unk={unk:>5d} goal={goal_str} "
                          f"bl={len(self.frontier_blacklist)}", flush=True)

        except KeyboardInterrupt:
            print("[explorer] interrupted", flush=True)
        finally:
            self._stop()
            self._finalize()


def main():
    explorer = AutoExplorer()
    explorer.run()

if __name__ == "__main__":
    main()
