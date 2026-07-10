"""Local navigation algorithms: Pure Pursuit and Dynamic Window Approach (DWA).

Provides reactive path-following logic. Pure Pursuit translates a global polyline
path into a short-range "carrot" target. The DWA layer samples kinematic rollouts
against raw Lidar data to select the optimal safe velocity vector, ensuring
collision avoidance independent of potential SLAM drift.
"""
from __future__ import annotations

import math
from typing import Optional

import numpy as np

import config as C
from geometry import inverse_transform_points, wrap_angle


# --------------------------------------------------------------------------- #
#  Pure pursuit — pick the carrot on the path
# --------------------------------------------------------------------------- #
def choose_carrot(
    path_world: list[tuple[float, float]],
    pose: tuple[float, float, float],
    v_cur: float,
) -> tuple[Optional[tuple[float, float]], bool]:
    """Computes a look-ahead target point on the global path.

    Utilizes arc-length pure pursuit: the robot's current pose is projected onto
    the nearest path segment, and a target point is interpolated a dynamic
    distance forward along the polyline. The lookahead distance scales linearly
    with current velocity.

    Args:
        path_world (list[tuple[float, float]]): The A* path as a sequence of `(x, y)` world coordinates.
        pose (tuple[float, float, float]): Current robot pose `(x, y, theta)`.
        v_cur (float): Current forward linear velocity in m/s.

    Returns:
        tuple[Optional[tuple[float, float]], bool]: A tuple `(carrot_point, near_goal)`.
            `carrot_point` is the `(wx, wy)` coordinate of the target, or None if the path is empty.
            `near_goal` is True if the final destination is within the lookahead horizon.
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

    near_goal = math.hypot(goal[0] - x, goal[1] - y) <= C.LOOKAHEAD_MAX
    if near_goal:
        carrot = goal
    return carrot, near_goal


# --------------------------------------------------------------------------- #
#  DWA local planner
# --------------------------------------------------------------------------- #
class DWAPlanner:
    """Dynamic-Window-Approach local planner.

    Samples feasible (v, w) kinematic rollouts against the live obstacle cloud.
    Scores candidates based on clearance, goal heading alignment, and forward
    progress to return the optimal safe control command.

    Attributes:
        prev_v (float): The previously issued linear velocity command (m/s).
        prev_w (float): The previously issued angular velocity command (rad/s).
        ctrl_dt (float): The fundamental control loop period (Webots timestep) in seconds.
    """

    def __init__(self, ctrl_dt: float = 0.032) -> None:
        """Initializes the DWA Planner with zeroed state.

        Args:
            ctrl_dt (float, optional): Control loop timestep in seconds. Defaults to 0.032.
        """
        self.prev_v = 0.0
        self.prev_w = 0.0
        self.ctrl_dt = ctrl_dt   # real time between commands (Webots timestep)

    def _obstacles_body(self, lidar_body: Optional[np.ndarray], poison_body: Optional[np.ndarray]) -> np.ndarray:
        """Concatenates lidar points and poison map points into a single obstacle array.

        Args:
            lidar_body (Optional[np.ndarray]): Lidar hits in body frame `(N, 2)`.
            poison_body (Optional[np.ndarray]): Poison zone boundaries in body frame `(M, 2)`.

        Returns:
            np.ndarray: Combined array of obstacle points `(K, 2)`.
        """
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

    def _nearest_dist(self, obs: np.ndarray) -> float:
        """Finds the Euclidean distance to the nearest obstacle.

        Args:
            obs (np.ndarray): Obstacle point cloud `(N, 2)`.

        Returns:
            float: Distance to the closest point in meters, or infinity if empty.
        """
        if obs.shape[0] == 0:
            return float("inf")
        return float(np.min(np.hypot(obs[:, 0], obs[:, 1])))

    def _side_clearance(self, obs: np.ndarray, long_window: float = 0.30) -> float:
        """Calculates the lateral clearance to the nearest side wall.

        Identifies obstacles physically beside the robot to apply narrow-gap
        speed reductions.

        Args:
            obs (np.ndarray): Obstacle point cloud `(N, 2)`.
            long_window (float, optional): Longitudinal distance threshold defining "beside". Defaults to 0.30.

        Returns:
            float: The minimum lateral (Y-axis) distance in meters, or infinity.
        """
        if obs.shape[0] == 0:
            return float("inf")
        beside = np.abs(obs[:, 0]) < long_window
        if not beside.any():
            return float("inf")
        return float(np.min(np.abs(obs[beside, 1])))

    def compute(
        self,
        pose: tuple[float, float, float],
        carrot_world: tuple[float, float],
        lidar_body: Optional[np.ndarray],
        poison_body: Optional[np.ndarray],
        green_block: bool = False,
        allow_reverse: bool = False,
        v_cap: Optional[float] = None,
    ) -> tuple[float, float]:
        """Calculates the optimal local motion command to reach the carrot point.

        Samples `(v, w)` pairs, rolls out trajectories assuming constant velocity,
        rejects paths colliding with `lidar_body` or `poison_body`, and scores
        the remainder. The chosen command is rate-limited by dynamic constraints.

        Args:
            pose (tuple[float, float, float]): Current robot pose `(x, y, theta)`.
            carrot_world (tuple[float, float]): The pursuit target `(wx, wy)`.
            lidar_body (Optional[np.ndarray]): Live Lidar obstacles in body frame `(N, 2)`.
            poison_body (Optional[np.ndarray]): Mapped poison boundaries in body frame `(M, 2)`.
            green_block (bool, optional): If True, halts all forward motion. Defaults to False.
            allow_reverse (bool, optional): If True, includes reverse velocity samples. Defaults to False.
            v_cap (Optional[float], optional): Hard upper limit on linear velocity. Defaults to None.

        Returns:
            tuple[float, float]: The bounded velocity command `(v, w)` in m/s and rad/s.
        """
        obs = self._obstacles_body(lidar_body, poison_body)
        carrot_b = inverse_transform_points(
            np.array([[carrot_world[0], carrot_world[1]]]), pose[0], pose[1], pose[2])[0]
        cx, cy = float(carrot_b[0]), float(carrot_b[1])
        carrot_bearing = math.atan2(cy, cx)
        carrot_dist = math.hypot(cx, cy)

        # ----------------------------------------------------------------- #
        #  Speed policy: NO compounding proximity slowdown. Heading-misalign
        #  x front-cone x side-wall factors used to multiply together and
        #  compound to a near-zero crawl near any corner or pillar -- the
        #  robot would stall short of the pillar reach threshold and loop in
        #  RECOVERY forever without net progress. Speed is governed ONLY by
        #  the per-rollout clearance rejection in the scoring loop below --
        #  a fast straight rollout that would approach a wall is rejected, so
        #  the planner keeps full speed when the path AHEAD is clear. ONE
        #  clean, non-compounding gate remains: pivot in place when the
        #  carrot is far off the nose, so the skid-steer rotates to face it
        #  instead of carving a wide wall-clipping arc.
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

        # pre-calculate initial clearance to allow squeezing out of violated margins:
        # if the robot's CURRENT position is already inside DWA_SAFE_RADIUS of an
        # obstacle (e.g. standing right at a pillar/corner standoff point), a naive
        # "reject anything under SAFE_RADIUS" check rejects even v=0 forever (the
        # rollout never leaves the violated position) -> permanent freeze. Only
        # reject a rollout that makes the violated clearance WORSE.
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

    def reset(self) -> None:
        """Clears the internal velocity state to allow instantaneous stops."""
        self.prev_v = self.prev_w = 0.0


# --------------------------------------------------------------------------- #
#  Reactive helpers for recovery
# --------------------------------------------------------------------------- #
def rear_clearance(lidar_body: Optional[np.ndarray], half_width: float = 0.18) -> float:
    """Determines the minimum obstacle distance in the robot's rear cone.

    Args:
        lidar_body (Optional[np.ndarray]): Lidar hits in body frame `(N, 2)`.
        half_width (float, optional): The lateral corridor width to check. Defaults to 0.18.

    Returns:
        float: Minimum distance in meters behind the robot, or infinity if clear.
    """
    if lidar_body is None or lidar_body.shape[0] == 0:
        return float("inf")
    behind = (lidar_body[:, 0] < -0.02) & (np.abs(lidar_body[:, 1]) < half_width)
    if not behind.any():
        return float("inf")
    p = lidar_body[behind]
    return float(np.min(np.hypot(p[:, 0], p[:, 1])))


def freer_side(lidar_body: Optional[np.ndarray]) -> float:
    """Determines which lateral hemisphere has fewer proximal obstacles.

    Used during recovery to decide the optimal in-place rotation direction.

    Args:
        lidar_body (Optional[np.ndarray]): Lidar hits in body frame `(N, 2)`.

    Returns:
        float: `1.0` if the left side is clearer, `-1.0` if the right side is clearer.
    """
    if lidar_body is None or lidar_body.shape[0] == 0:
        return 1.0
    left = lidar_body[lidar_body[:, 1] > 0]
    right = lidar_body[lidar_body[:, 1] < 0]
    dl = float(np.min(np.hypot(left[:, 0], left[:, 1]))) if left.shape[0] else float("inf")
    dr = float(np.min(np.hypot(right[:, 0], right[:, 1]))) if right.shape[0] else float("inf")
    return 1.0 if dl >= dr else -1.0


def front_clearance(lidar_body: Optional[np.ndarray], half_width: float = 0.16) -> float:
    """Determines the minimum obstacle distance straight ahead of the robot.

    Used by the state machine to aggressively halt progress if an obstacle
    (like a pillar) is physically imminent.

    Args:
        lidar_body (Optional[np.ndarray]): Lidar hits in body frame `(N, 2)`.
        half_width (float, optional): The lateral corridor width to check. Defaults to 0.16.

    Returns:
        float: Minimum straight-ahead distance in meters, or infinity if clear.
    """
    if lidar_body is None or lidar_body.shape[0] == 0:
        return float("inf")
    ahead = (lidar_body[:, 0] > 0.0) & (np.abs(lidar_body[:, 1]) < half_width)
    if not ahead.any():
        return float("inf")
    p = lidar_body[ahead]
    return float(np.min(p[:, 0]))
