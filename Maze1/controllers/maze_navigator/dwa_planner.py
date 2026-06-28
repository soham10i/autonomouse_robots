"""Dynamic Window Approach (DWA) local planner — v3 (paper-faithful).

Implements Fox, Burgard & Thrun (1997) faithfully:

    V_r = V_s  ∩  V_d  ∩  V_a

where:
    V_s  — physical limits  [v_min, v_max] × [-w_max, w_max]
    V_d  — dynamic window   velocities reachable in one Δt given accel limits
    V_a  — admissible       velocities from which the robot can stop before
                            hitting the nearest obstacle on that arc

Objective:
    G(v, ω) = σ( α·heading(v,ω) + β·dist(v,ω) + γ·velocity(v,ω) )

All three sub-scores are independently normalised to [0,1] before weighting,
and σ is applied to the weighted sum (sigmoid, saturating at ±6 to keep the
output numerically bounded).

Maze1 weights (calibrated from .wbt geometry):
    Corridors are ~0.45 m wide with 0.25 m robot → only 0.20 m margin each
    side.  Clearance must dominate to avoid grazing walls.
      α = 0.55  (heading toward goal)
      β = 0.35  (clearance from walls)
      γ = 0.10  (forward speed preference)

API mirrors SmoothDriver so callers can swap them without any other changes.
"""
import math

import numpy as np

import config as C
from utils import wrap_angle, clamp


# ---------------------------------------------------------------------------
# DWA tuning constants  (maze-specific values; all overridable via config.py)
# ---------------------------------------------------------------------------

def _cfg(name, default):
    """Pull from config if present, fall back to local default."""
    return getattr(C, name, default)


# Control cycle — called every this many simulation ticks.
# 8 × 32 ms = 256 ms ≈ the 250 ms used in the DWA paper.
DWA_CALL_EVERY_N_TICKS: int   = _cfg("DWA_CALL_EVERY_N_TICKS", 8)

# Trajectory simulation
DWA_DT: float                 = _cfg("DWA_DT_S", 0.256)        # one control cycle
DWA_HORIZON_S: float          = _cfg("DWA_HORIZON_S", 1.2)     # scoring rollout
DWA_N_STEPS: int              = max(1, int(DWA_HORIZON_S / DWA_DT))

# Velocity sample counts
DWA_V_SAMPLES: int            = _cfg("DWA_V_SAMPLES", 9)        # v ∈ [0, v_max]
DWA_W_SAMPLES: int            = _cfg("DWA_W_SAMPLES", 21)       # ω ∈ [-w_max, w_max]

# --- Dynamic Window (V_d) ---
# Acceleration limits.  Kept moderate (0.50 m/s²) even though Webots motors
# respond instantly — this prevents huge velocity jumps that confuse odometry.
DWA_V_ACCEL: float            = _cfg("DWA_V_ACCEL", 0.50)       # m/s²
DWA_W_ACCEL: float            = _cfg("DWA_W_ACCEL", 1.80)       # rad/s²

# --- Admissible Velocities (V_a) ---
# Braking decelerations used to compute the stopping distance.
# Must be ≥ 0 — set equal to ACCEL for a symmetric model.
DWA_V_BRAKE: float            = _cfg("DWA_V_BRAKE", 0.50)       # m/s²
DWA_W_BRAKE: float            = _cfg("DWA_W_BRAKE", 1.80)       # rad/s²

# --- Scoring weights (α, β, γ from the paper) ---
# Maze1-calibrated: tight corridors → clearance dominates.
DWA_ALPHA: float              = _cfg("DWA_ALPHA", 0.55)  # heading
DWA_BETA: float               = _cfg("DWA_BETA",  0.35)  # clearance
DWA_GAMMA: float              = _cfg("DWA_GAMMA", 0.10)  # velocity

# Safety / goal
DWA_MIN_CLEARANCE_M: float    = _cfg("DWA_MIN_CLEAR_M", 0.13)
DWA_GOAL_TOLERANCE: float     = 0.30

# Minimum forward speed when heading error < 60° — prevents spin-lock.
DWA_MIN_FWD_IF_ALIGNED: float = 0.08


# ---------------------------------------------------------------------------
class DWAPlanner:
    """Drop-in replacement for SmoothDriver, using paper-faithful DWA.

    Call ``set_current_velocity(v, w)`` every tick with the last commanded
    velocity so the Dynamic Window can contract/expand correctly.
    """

    def __init__(self):
        self.path: list = []
        self.idx: int = 0
        self.final_tol: float = DWA_GOAL_TOLERANCE

        # Current velocity state — updated by set_current_velocity()
        self._v_cur: float = 0.0
        self._w_cur: float = 0.0

        # Last commanded output — held between DWA calls (rate limiter)
        self._last_v: float = 0.0
        self._last_w: float = 0.0
        self._last_status: str = "follow"
        self._tick: int = 0          # internal tick counter for rate-limiter

    # ----------------------------- public API ------------------------------

    def set_path(self, path_world, final_tol=None):
        self.path = list(path_world) if path_world else []
        self.idx = 0
        self.final_tol = (
            final_tol if final_tol is not None else DWA_GOAL_TOLERANCE
        )
        self._last_v = 0.0
        self._last_w = 0.0
        self._last_status = "follow"

    def has_path(self) -> bool:
        return len(self.path) >= 1

    def remaining_distance(self, pose) -> float:
        if not self.path:
            return 0.0
        x, y, _ = pose
        return math.hypot(self.path[-1][0] - x, self.path[-1][1] - y)

    def set_current_velocity(self, v: float, w: float):
        """Feed back the last commanded velocity so V_d is accurate."""
        self._v_cur = float(v)
        self._w_cur = float(w)

    def step(self, pose, ranges=None, angles=None, aux_clear=None,
             cur_speed=None):
        """Return ``(v, w, status)`` — status ∈ {follow, blocked, arrived}.

        Rate-limited: the full DWA optimisation only runs every
        ``DWA_CALL_EVERY_N_TICKS`` calls; the last command is held between
        calls so the caller's loop overhead doesn't matter.
        """
        if not self.has_path():
            return 0.0, 0.0, "arrived"

        self._tick += 1

        rx, ry, rth = pose

        # Advance waypoint index past reached waypoints.
        while self.idx < len(self.path) - 1:
            wx, wy = self.path[self.idx]
            if math.hypot(wx - rx, wy - ry) < C.WAYPOINT_REACH_TOL:
                self.idx += 1
            else:
                break

        # Final waypoint reached?
        last_x, last_y = self.path[-1]
        rem = math.hypot(last_x - rx, last_y - ry)
        if rem < self.final_tol:
            self._last_v = 0.0
            self._last_w = 0.0
            self._last_status = "arrived"
            return 0.0, 0.0, "arrived"

        # Rate-limiter: skip full optimisation between ticks.
        if self._tick % DWA_CALL_EVERY_N_TICKS != 1:
            return self._last_v, self._last_w, self._last_status

        # --- Full DWA optimisation ---
        target = self._lookahead_point(rx, ry)
        goal_bearing = math.atan2(target[1] - ry, target[0] - rx)
        heading_err = wrap_angle(goal_bearing - rth)
        path_bearing = self._path_bearing_at_lookahead()

        # Convert lidar scan to world-frame obstacle points.
        obs_xy = self._lidar_to_xy(ranges, angles, rth, rx, ry)
        if aux_clear is not None and np.isfinite(aux_clear):
            # Treat aux_clear as a point directly ahead at that distance.
            ox = rx + float(aux_clear) * math.cos(rth)
            oy = ry + float(aux_clear) * math.sin(rth)
            extra = np.array([[ox, oy]], dtype=np.float32)
            obs_xy = (np.vstack([obs_xy, extra])
                      if obs_xy.shape[0] > 0 else extra)

        # Scale v_max down near the goal.
        v_max = C.V_MAX
        if rem < 0.5:
            v_max *= clamp(rem / 0.5, 0.3, 1.0)

        # ---- V_d: Dynamic Window ----------------------------------------
        # Velocities reachable from (v_cur, w_cur) in one DWA_DT step.
        v_lo = max(0.0,        self._v_cur - DWA_V_ACCEL * DWA_DT)
        v_hi = min(v_max,      self._v_cur + DWA_V_ACCEL * DWA_DT)
        w_lo = max(-C.W_MAX,   self._w_cur - DWA_W_ACCEL * DWA_DT)
        w_hi = min(C.W_MAX,    self._w_cur + DWA_W_ACCEL * DWA_DT)

        vs = np.linspace(v_lo, v_hi, DWA_V_SAMPLES)
        ws = np.linspace(w_lo, w_hi, DWA_W_SAMPLES)

        # Collect raw sub-scores for normalisation (σ step).
        candidates = []  # list of (v, w, heading_raw, dist_raw, vel_raw)

        for v_cand in vs:
            for w_cand in ws:
                # ---- V_a: Admissibility -----------------------------------
                # The robot must be able to stop before any obstacle on this arc.
                # Forward-simulate to find minimum clearance.
                traj = self._simulate(rx, ry, rth, v_cand, w_cand)
                min_dist = self._min_obstacle_dist(traj, obs_xy)

                # Admissibility: reject if speed > sqrt(2·d·decel) for either axis.
                v_adm = math.sqrt(2.0 * max(min_dist, 0.0) * DWA_V_BRAKE)
                w_adm = math.sqrt(2.0 * max(min_dist, 0.0) * DWA_W_BRAKE)
                if v_cand > v_adm or abs(w_cand) > w_adm:
                    continue  # V_a: not admissible

                # Hard collision guard (belt-and-suspenders).
                if min_dist < DWA_MIN_CLEARANCE_M:
                    continue

                # ---- Sub-scores (raw, pre-normalisation) -----------------
                # heading: alignment of final end-pose with goal direction
                end_x, end_y, end_th = traj[-1]
                h_to_goal = math.atan2(target[1] - end_y, target[0] - end_x)
                h_err = abs(wrap_angle(h_to_goal - end_th))
                heading_raw = math.pi - h_err  # ∈ [0, π] — higher is better

                # dist: clearance distance (paper uses "dist" = closest obstacle)
                dist_raw = min_dist  # metres

                # velocity: forward speed
                vel_raw = v_cand    # m/s

                candidates.append((v_cand, w_cand, heading_raw, dist_raw, vel_raw))

        if not candidates:
            # All trajectories inadmissible or in collision — spin toward goal.
            spin_w = clamp(2.0 * heading_err, -C.W_MAX, C.W_MAX)
            self._last_v, self._last_w, self._last_status = 0.0, spin_w, "blocked"
            return 0.0, spin_w, "blocked"

        # ---- σ-normalisation (paper section 2.2) -------------------------
        # Normalise each raw component across all surviving candidates so
        # that no single dimension dominates purely by scale.
        h_vals  = [c[2] for c in candidates]
        d_vals  = [c[3] for c in candidates]
        v_vals  = [c[4] for c in candidates]

        h_max = max(h_vals) or 1e-9
        d_max = max(d_vals) or 1e-9
        v_max_cand = max(v_vals) or 1e-9

        best_score = -math.inf
        best_v, best_w = candidates[0][0], candidates[0][1]

        for v_cand, w_cand, heading_raw, dist_raw, vel_raw in candidates:
            h_n = heading_raw / h_max       # [0, 1]
            d_n = dist_raw    / d_max       # [0, 1]
            vel_n = vel_raw   / v_max_cand  # [0, 1]

            # Optional path-alignment component (extra, not in paper).
            if path_bearing is not None and v_cand > 0.01:
                end_x, end_y, _ = self._simulate(rx, ry, rth, v_cand, w_cand)[-1]
                move_dir = math.atan2(end_y - ry, end_x - rx)
                path_err = abs(wrap_angle(move_dir - path_bearing))
                path_n = 1.0 - path_err / math.pi
            else:
                path_n = 0.5

            raw_score = (DWA_ALPHA * h_n
                         + DWA_BETA  * d_n
                         + DWA_GAMMA * vel_n
                         + 0.05 * path_n)   # tiny path-alignment nudge

            # σ: sigmoid to bound the score — avoids explosion near walls.
            score = 1.0 / (1.0 + math.exp(-clamp(raw_score * 3.0, -6.0, 6.0)))

            if score > best_score:
                best_score = score
                best_v = v_cand
                best_w = w_cand

        # Anti-spin-lock: if goal is roughly ahead but DWA chose v≈0, nudge.
        if abs(heading_err) < math.radians(60) and best_v < DWA_MIN_FWD_IF_ALIGNED:
            best_v = DWA_MIN_FWD_IF_ALIGNED

        best_v = clamp(best_v, 0.0, C.V_MAX)
        best_w = clamp(best_w, -C.W_MAX, C.W_MAX)

        self._last_v, self._last_w, self._last_status = best_v, best_w, "follow"
        return best_v, best_w, "follow"

    # ----------------------------- internals ------------------------------

    def _simulate(self, x, y, th, v, w):
        """Forward-simulate constant (v, ω) arc for DWA_HORIZON_S."""
        traj = [(x, y, th)]
        cx, cy, cth = x, y, th
        for _ in range(DWA_N_STEPS):
            cx  += v * math.cos(cth) * DWA_DT
            cy  += v * math.sin(cth) * DWA_DT
            cth  = wrap_angle(cth + w * DWA_DT)
            traj.append((cx, cy, cth))
        return traj

    def _lidar_to_xy(self, ranges, angles, rth, rx, ry):
        """Convert lidar polar scan to world-frame (N, 2) obstacle points."""
        if ranges is None or angles is None or len(ranges) == 0:
            return np.empty((0, 2), dtype=np.float32)
        ra = np.asarray(ranges, dtype=np.float32)
        an = np.asarray(angles, dtype=np.float64)
        valid = np.isfinite(ra) & (ra > 0.05) & (ra < 4.0)
        if not valid.any():
            return np.empty((0, 2), dtype=np.float32)
        r = ra[valid]
        a = an[valid]
        wx = rx + r * np.cos(rth + a)
        wy = ry + r * np.sin(rth + a)
        return np.stack([wx, wy], axis=1).astype(np.float32)

    def _min_obstacle_dist(self, traj, obs_xy):
        """Minimum distance from any trajectory point to any obstacle point."""
        if obs_xy.shape[0] == 0:
            return float("inf")
        worst = float("inf")
        # Check every 3rd point to keep the inner loop fast.
        for i in range(0, len(traj), 3):
            tx, ty, _ = traj[i]
            dx = obs_xy[:, 0] - tx
            dy = obs_xy[:, 1] - ty
            d_min = float(np.sqrt((dx * dx + dy * dy).min()))
            if d_min < worst:
                worst = d_min
            if worst < DWA_MIN_CLEARANCE_M:
                return worst   # early exit
        return worst

    def _lookahead_point(self, rx, ry):
        """Return the immediate next waypoint on the path.

        Previous implementation searched for a point ≥ L metres from the
        robot, which caused corner-cutting on the heavily-simplified A*
        path (only 5 waypoints for 3.4 m → 0.85 m spacing).  Now we
        simply return ``self.path[self.idx]`` so the DWA follows the A*
        route faithfully around corners.

        If the robot is already very close to the next waypoint, advance
        one further to prevent erratic micro-turns at waypoint crossings.
        """
        i = self.idx
        while i < len(self.path) - 1:
            wx, wy = self.path[i]
            if math.hypot(wx - rx, wy - ry) >= C.WAYPOINT_REACH_TOL:
                return wx, wy
            i += 1
        return self.path[-1]

    def _path_bearing_at_lookahead(self):
        """Direction of the A* path at the current lookahead segment."""
        if len(self.path) < 2 or self.idx >= len(self.path) - 1:
            return None
        ax, ay = self.path[self.idx]
        bx, by = self.path[min(self.idx + 1, len(self.path) - 1)]
        if math.hypot(bx - ax, by - ay) < 1e-4:
            return None
        return math.atan2(by - ay, bx - ax)
