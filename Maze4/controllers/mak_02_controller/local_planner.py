"""Local control: pure-pursuit carrot + DWA driver on LIVE lidar.

Pure pursuit only chooses *where* to aim (a carrot point on the global A* path).
The DWA layer decides *how* to get there using the live lidar scan, so it stays
clear of walls and centres itself in corridors even if the global map has drift.
All obstacle geometry is handled in the robot BODY frame.
"""
from __future__ import annotations

import math

import numpy as np

import config as C
from geometry import inverse_transform_points, wrap_angle


# --------------------------------------------------------------------------- #
#  Pure pursuit — pick the carrot on the path
# --------------------------------------------------------------------------- #
def choose_carrot(path_world, pose, v_cur):
    """Return the look-ahead point (wx, wy) on ``path_world`` and a goal flag.

    Arc-length pure pursuit: project the robot onto the path POLYLINE (nearest
    point on the nearest *segment*, not the nearest vertex), then walk ``look``
    metres forward ALONG the path from that projection and interpolate the
    carrot there.  This is the robust formulation — the earlier "first vertex
    >= look away, starting from the closest vertex" shortcut aimed BACKWARD once
    the robot passed the start of a line-of-sight-simplified straight segment
    (A* collapses an open corridor to just [start, goal], so the start vertex
    stays the closest while sitting >= look behind the robot -> the carrot
    snapped back to the start and the robot pivoted around).  Projecting along
    the segment guarantees the carrot is always ahead of the robot's progress.
    """
    if not path_world:
        return None, True
    x, y = pose[0], pose[1]
    look = C.LOOKAHEAD_BASE + C.LOOKAHEAD_K_V * max(0.0, v_cur)
    look = max(C.LOOKAHEAD_MIN, min(C.LOOKAHEAD_MAX, look))
    goal = path_world[-1]
    if len(path_world) == 1:
        return goal, math.hypot(goal[0] - x, goal[1] - y) <= C.LOOKAHEAD_MAX

    # 1) nearest point on the polyline -> (segment index, projection point)
    best_d2 = float("inf")
    seg_i = 0
    proj = path_world[0]
    for i in range(len(path_world) - 1):
        ax, ay = path_world[i]
        bx, by = path_world[i + 1]
        dx, dy = bx - ax, by - ay
        seg_len2 = dx * dx + dy * dy
        if seg_len2 < 1e-12:
            t = 0.0
            pxs, pys = ax, ay
        else:
            t = ((x - ax) * dx + (y - ay) * dy) / seg_len2
            t = max(0.0, min(1.0, t))
            pxs, pys = ax + t * dx, ay + t * dy
        d2 = (x - pxs) ** 2 + (y - pys) ** 2
        if d2 < best_d2:
            best_d2 = d2
            seg_i = i
            proj = (pxs, pys)

    # 2) walk `look` metres forward along the path from that projection
    remaining = look
    cx, cy = proj
    carrot = goal
    for i in range(seg_i, len(path_world) - 1):
        bx, by = path_world[i + 1]
        seg = math.hypot(bx - cx, by - cy)
        if seg >= remaining:
            f = remaining / seg if seg > 1e-9 else 0.0
            carrot = (cx + (bx - cx) * f, cy + (by - cy) * f)
            break
        remaining -= seg
        cx, cy = bx, by

    # NOTE: do NOT override `carrot` with the raw `goal` here.
    # Within LOOKAHEAD_MAX the old code forced carrot = goal (the pillar world
    # position), which drew a straight line from the robot through whatever wall
    # happened to separate them — exactly the "goes straight into a wall" failure
    # visible in GO_YELLOW.  The path-interpolated carrot already converges to the
    # final waypoint as the robot approaches; no override is needed.
    near_goal = math.hypot(goal[0] - x, goal[1] - y) <= C.LOOKAHEAD_MAX
    return carrot, near_goal


# --------------------------------------------------------------------------- #
#  DWA local planner
# --------------------------------------------------------------------------- #
class DWAPlanner:
    """Dynamic-Window-Approach local planner: samples feasible (v, w) rollouts against the live obstacle cloud and returns the best safe command toward the carrot."""
    def __init__(self, ctrl_dt=0.032):
        self.prev_v = 0.0
        self.prev_w = 0.0
        self.ctrl_dt = ctrl_dt   # real time between commands (Webots timestep)

    def _obstacles_body(self, lidar_body, poison_body):
        parts = []
        if lidar_body is not None and lidar_body.shape[0] > 0:
            pts = lidar_body
            if pts.shape[0] > C.DWA_LIDAR_SUBSAMPLE:
                idx = np.linspace(0, pts.shape[0] - 1, C.DWA_LIDAR_SUBSAMPLE).astype(int)
                pts = pts[idx]
            parts.append(pts)
        if poison_body is not None and poison_body.shape[0] > 0:
            parts.append(poison_body)
        if not parts:
            return np.empty((0, 2))
        return np.concatenate(parts, axis=0)

    def _side_clearance(self, obs, long_window=0.30):
        """Nearest lateral obstacle alongside the robot (|x| < window), or inf.

        Drives the single tight-gap speed factor: in a genuinely narrow passage
        the robot eases off for precise centring.  Unlike the deleted compounding
        caps this is ONE floored factor, active only below DWA_TIGHT_GAP_DIST.
        """
        if obs.shape[0] == 0:
            return float("inf")
        beside = np.abs(obs[:, 0]) < long_window
        if not beside.any():
            return float("inf")
        return float(np.min(np.abs(obs[beside, 1])))

    def compute(self, pose, carrot_world, lidar_body, poison_body,
                green_block=False, allow_reverse=False, v_cap=None):
        """Return a smooth (v, w) command toward the carrot.

        Candidate (v, w) pairs span the FULL feasible window (trajectory-rollout
        / move_base-style local planner).  Trajectories that pass within
        ``DWA_SAFE_RADIUS`` of any live-lidar or mapped-poison point are rejected;
        the best survivor is then rate-limited against the previous command using
        the real control period for jerk-free, smooth motion.
        """
        obs = self._obstacles_body(lidar_body, poison_body)
        carrot_b = inverse_transform_points(
            np.array([[carrot_world[0], carrot_world[1]]]), pose[0], pose[1], pose[2])[0]
        cx, cy = float(carrot_b[0]), float(carrot_b[1])
        carrot_bearing = math.atan2(cy, cx)
        carrot_dist = math.hypot(cx, cy)

        # ----------------------------------------------------------------- #
        #  Speed policy (Maze1-proven): NO compounding proximity slowdown.
        #
        #  The previous Maze4 build scaled the forward ceiling by THREE
        #  multiplicative factors — heading-misalignment x front-cone x
        #  side-wall proximity.  In a corridor whose side walls sit ~0.12 m
        #  away every factor fired at once and they COMPOUNDED to a near-zero
        #  crawl: the robot "moved only a few centimetres".  Maze1's postmortem
        #  (control/local_planner.py) deleted exactly this cap, with the note
        #  that an omnidirectional slowdown "made the robot crawl down every
        #  corridor ... and pinned it".  Here speed is governed ONLY by the
        #  per-rollout clearance rejection in the scoring loop below — the
        #  canonical DWA mechanism: a fast straight rollout that would approach
        #  a wall is rejected, so the planner keeps full speed when the path
        #  AHEAD is clear and naturally selects a slower/curved rollout only
        #  when it must.  ONE clean, non-compounding gate remains: pivot in
        #  place when the carrot is far off the nose, so the skid-steer rotates
        #  to face it instead of carving a wide wall-clipping arc (the role
        #  Maze1's pure-pursuit `PP_BIG_HEADING_STOP` played).
        # ----------------------------------------------------------------- #
        v_top = C.V_MAX if v_cap is None else min(C.V_MAX, v_cap)
        if green_block:
            v_top = 0.0   # never push forward onto poison
        # single tight-gap speed factor (floored -> never a crawl): ease off only
        # when a side wall is genuinely close, for precise centreline threading.
        side = self._side_clearance(obs)
        if side < C.DWA_TIGHT_GAP_DIST:
            sfrac = (side - C.DWA_SAFE_RADIUS) / max(1e-3, C.DWA_TIGHT_GAP_DIST - C.DWA_SAFE_RADIUS)
            v_top *= max(C.DWA_TIGHT_GAP_VFRAC, min(1.0, sfrac))
        pivot = (abs(carrot_bearing) > C.DWA_PIVOT_BEARING
                 and carrot_dist > C.DWA_PIVOT_MIN_DIST)

        dt = C.DWA_STEP_S
        if v_top > 1e-3 and not pivot:
            v_samples = list(np.linspace(0.0, v_top, C.DWA_V_SAMPLES))
        else:
            v_samples = [0.0]
        if allow_reverse or green_block:
            v_samples = sorted(set(v_samples + [-0.12]))
        w_samples = np.linspace(-C.W_MAX, C.W_MAX, C.DWA_W_SAMPLES)

        n_steps = max(2, int(C.DWA_HORIZON_S / dt))
        best_score = -1e18
        best = None

        # pre-calculate initial clearance to allow squeezing out of violated margins
        initial_clear = float("inf")
        if obs.shape[0] > 0:
            initial_clear = float(np.min(np.hypot(obs[:, 0], obs[:, 1])))

        for v in v_samples:
            for w in w_samples:
                # rollout in body frame from origin
                x = y = th = 0.0
                min_clear = float("inf")
                collide = False
                for _ in range(n_steps):
                    x += v * math.cos(th) * dt
                    y += v * math.sin(th) * dt
                    th += w * dt
                    if obs.shape[0] > 0:
                        d = float(np.min(np.hypot(obs[:, 0] - x, obs[:, 1] - y)))
                        if d < min_clear:
                            min_clear = d
                        # Paralyze-fix: if inside safety margin, allow trajectories that maintain or improve clearance
                        if d < C.DWA_SAFE_RADIUS and d < initial_clear - 1e-5:
                            collide = True
                            break
                if collide:
                    continue
                # scoring (all terms ~normalised to [0,1])
                end_bear = math.atan2(cy - y, cx - x)
                head_align = 0.5 * (1.0 + math.cos(wrap_angle(th - end_bear)))
                end_dist = math.hypot(cx - x, cy - y)
                dist_term = 1.0 - min(1.0, end_dist / max(0.3, carrot_dist + 0.3))
                clear_term = min(1.0, min_clear / C.DWA_CLEAR_CAP) if math.isfinite(min_clear) else 1.0
                speed_term = max(0.0, v) / C.V_MAX
                straight_pen = abs(w) / C.W_MAX
                score = (C.DWA_W_HEADING * head_align
                         + C.DWA_W_DIST * dist_term
                         + C.DWA_W_CLEAR * clear_term
                         + C.DWA_W_SPEED * speed_term
                         - C.DWA_W_STRAIGHT * straight_pen)
                if score > best_score:
                    best_score = score
                    best = (float(v), float(w))

        if best is None:
            # fully boxed in: spin in place toward the carrot / freer side
            w_dir = 1.0 if carrot_bearing >= 0 else -1.0
            best = (0.0, w_dir * C.DWA_SEARCH_W)

        # rate-limit against the previous command using the REAL control period
        # (Webots timestep) so velocity changes are smooth/jerk-free per tick.
        dc = self.ctrl_dt
        v_cmd = max(self.prev_v - C.A_V * dc, min(self.prev_v + C.A_V * dc, best[0]))
        w_cmd = max(self.prev_w - C.A_W * dc, min(self.prev_w + C.A_W * dc, best[1]))
        self.prev_v, self.prev_w = v_cmd, w_cmd
        return v_cmd, w_cmd

    def reset(self):
        self.prev_v = self.prev_w = 0.0


# --------------------------------------------------------------------------- #
#  Reactive helpers for recovery
# --------------------------------------------------------------------------- #
def rear_clearance(lidar_body, half_width=0.18):
    """Min distance to an obstacle in the rear cone (x < 0), or inf."""
    if lidar_body is None or lidar_body.shape[0] == 0:
        return float("inf")
    behind = (lidar_body[:, 0] < -0.02) & (np.abs(lidar_body[:, 1]) < half_width)
    if not behind.any():
        return float("inf")
    p = lidar_body[behind]
    return float(np.min(np.hypot(p[:, 0], p[:, 1])))


def freer_side(lidar_body):
    """+1 if the left half-plane has more room than the right, else -1."""
    if lidar_body is None or lidar_body.shape[0] == 0:
        return 1.0
    left = lidar_body[lidar_body[:, 1] > 0]
    right = lidar_body[lidar_body[:, 1] < 0]
    dl = float(np.min(np.hypot(left[:, 0], left[:, 1]))) if left.shape[0] else float("inf")
    dr = float(np.min(np.hypot(right[:, 0], right[:, 1]))) if right.shape[0] else float("inf")
    return 1.0 if dl >= dr else -1.0


def front_clearance(lidar_body, half_width=0.16):
    """Min distance to an obstacle straight ahead (x > 0 cone), or inf."""
    if lidar_body is None or lidar_body.shape[0] == 0:
        return float("inf")
    ahead = (lidar_body[:, 0] > 0.0) & (np.abs(lidar_body[:, 1]) < half_width)
    if not ahead.any():
        return float("inf")
    p = lidar_body[ahead]
    return float(np.min(p[:, 0]))
