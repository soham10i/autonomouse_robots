"""Smooth (VFH-style) local navigator.

A drop-in alternative to ``PathFollower`` (pure pursuit + collision veto)
designed for robust corridor-and-gap driving in cluttered mazes. Instead
of slavishly tracking A* waypoints — which sit on grid cells and may be
only a few centimetres from a wall — this driver:

  - reads the full 360° lidar polar histogram every tick;
  - inflates each obstacle by the angle it would subtend on a robot of
    radius SD_SAFETY_DIST, so adjacent sectors get marked as blocked;
  - finds runs of un-blocked sectors in the forward hemisphere and
    estimates each gap's chord-width in metres;
  - scores gaps by alignment with the A*-derived goal bearing, width,
    and depth, then picks the best;
  - steers toward the gap's CENTRE — not toward a waypoint — and
    modulates forward speed by ``cos(steer)``, the gap's width, and the
    forward clearance.

The A* path is still consumed but only as a *global hint* — the
lookahead point defines the goal bearing. Local lateral adjustment is
purely lidar-driven, which is how competition autonomous cars stay
centred in narrow tracks.

API mirrors ``PathFollower`` so callers can swap them.
"""
import math

import numpy as np

import config as C
from utils import wrap_angle, clamp


# ---------------------------------------------------------------------------
# Driver tuning. Most cross-controller tunables live in ``config.py``; the
# ones below govern only the gap-selection logic and stay local.
# ---------------------------------------------------------------------------

# Inflate each obstacle by the angle subtended by a circle of this radius.
# = ROBOT_RADIUS (0.125 m) + 4 cm safety margin.
SD_SAFETY_DIST = 0.165

# Reject gaps whose chord-width is below this (m). 0.20 m chassis-width +
# 4 cm margin = 0.24 m — i.e. just barely fits the bot squared with the
# corridor.
SD_MIN_GAP_WIDTH_M = 0.24

# Forward hemisphere considered for gap selection. ±110° lets the bot peek
# slightly behind so it can pick a clean exit if the only gap is to the
# side or rear-side.
SD_FORWARD_HALF_ANGLE = math.radians(110)

# Distance beyond which a lidar return is treated as "free" (effectively
# no obstacle in the histogram). Lidar's hardware max is ~8 m; we cap at
# 4 m because anything past that won't be in the next planning window.
SD_MAX_RANGE = 4.0

# Low-pass filter on the steering angle to suppress chatter when adjacent
# gaps swap "best" status as the bot moves. Higher = more responsive.
# 0.55 is a good middle: turns commit in ~5 ticks (~160 ms) but two adjacent
# gaps still don't make the heading flip-flop.
SD_STEER_LPF = 0.55

# When the chosen gap is essentially forward AND wide, stop low-passing —
# go straight without lag. This kills the "wiggle near goal" pattern.
SD_STEER_FORWARD_TOL = math.radians(8.0)   # gap within ±8° of forward
SD_STEER_FORWARD_GAP_W = 0.40              # gap ≥ 0.40 m wide


class SmoothDriver:
    """Drop-in alternative to ``PathFollower`` with VFH-style steering."""

    def __init__(self):
        self.path = []
        self.idx = 0
        self.final_tol = C.WAYPOINT_REACH_TOL
        self._steer_filtered = 0.0
        self._last_gap = None       # diagnostic — last (angle, width, range)

    # ----------------------------- API ----------------------------------

    def set_path(self, path_world, final_tol=None):
        self.path = list(path_world) if path_world else []
        self.idx = 0
        self.final_tol = (
            final_tol if final_tol is not None else C.WAYPOINT_REACH_TOL
        )
        self._steer_filtered = 0.0

    def has_path(self):
        return len(self.path) >= 1

    def remaining_distance(self, pose):
        if not self.path:
            return 0.0
        x, y, _ = pose
        return math.hypot(self.path[-1][0] - x, self.path[-1][1] - y)

    def step(self, pose, ranges, angles, aux_clear=None, cur_speed=None):
        """Return ``(v, w, status)`` where ``status`` is
        ``"follow" | "blocked" | "arrived"``.

        ``ranges`` / ``angles`` are the full lidar scan; we use *all* of
        it for gap selection (not just front/side cones).
        """
        if not self.has_path():
            return 0.0, 0.0, "arrived"

        rx, ry, rth = pose

        # 1. Advance waypoint index past anything we've already passed.
        while self.idx < len(self.path) - 1:
            wx, wy = self.path[self.idx]
            if math.hypot(wx - rx, wy - ry) < C.WAYPOINT_REACH_TOL:
                self.idx += 1
            else:
                break

        # 2. Final-waypoint reach check.
        last_x, last_y = self.path[-1]
        rem = math.hypot(last_x - rx, last_y - ry)
        if rem < self.final_tol:
            return 0.0, 0.0, "arrived"

        # 3. Lookahead point → goal bearing in the robot frame.
        v_ref = cur_speed if cur_speed is not None else C.V_MAX
        L = clamp(C.LOOKAHEAD_BASE + C.LOOKAHEAD_K_V * abs(v_ref),
                  C.LOOKAHEAD_MIN, C.LOOKAHEAD_MAX)
        target = self._lookahead_point(rx, ry, L)
        goal_global = math.atan2(target[1] - ry, target[0] - rx)
        goal_bearing = wrap_angle(goal_global - rth)

        # 4. VFH gap selection.
        gap = find_best_gap(ranges, angles, goal_bearing,
                            safety_dist=SD_SAFETY_DIST,
                            min_gap_width_m=SD_MIN_GAP_WIDTH_M,
                            max_range=SD_MAX_RANGE)

        if gap is None:
            # Forward hemisphere is fully blocked (or all gaps too narrow).
            # Tell the caller — it should drive reactively / recover.
            self._last_gap = None
            return 0.0, 0.0, "blocked"

        gap_angle, gap_width_m, gap_min_range_m = gap
        self._last_gap = gap

        # 5. EMA-smooth the steering angle to kill flicker between adjacent
        #    gaps when the bot moves a few cm. Special case: if the chosen
        #    gap is essentially forward AND wide, bypass the filter entirely
        #    so we don't oscillate between two nearly-equivalent gaps near
        #    a goal.
        if (abs(gap_angle) < SD_STEER_FORWARD_TOL
                and gap_width_m >= SD_STEER_FORWARD_GAP_W):
            self._steer_filtered = gap_angle
        else:
            self._steer_filtered = (
                (1.0 - SD_STEER_LPF) * self._steer_filtered
                + SD_STEER_LPF * gap_angle
            )
        steer = self._steer_filtered

        # 6. Heading control.
        if abs(steer) > C.PF_BIG_HEADING_STOP:
            # Need to spin in place before committing to forward.
            w = clamp(C.PF_K_HEADING * steer, -C.W_MAX, C.W_MAX)
            return 0.0, w, "follow"

        if abs(steer) < C.PF_HEADING_DEADBAND:
            w = 0.0
        else:
            w = clamp(C.PF_K_HEADING * steer, -C.W_MAX, C.W_MAX)

        # 7. Speed shaping.
        # 7a. Forward clearance (lidar narrow forward cone + depth-cam aux).
        fwd = forward_clearance(ranges, angles)
        if aux_clear is not None and np.isfinite(aux_clear):
            fwd = min(fwd, float(aux_clear))

        if fwd < C.PF_FWD_BRAKE_DIST:
            return 0.0, w, "blocked"

        v_base = C.V_MAX * math.cos(steer)
        v_base = max(C.V_MIN_FORWARD, v_base)

        # 7b. Forward-clearance scaling.
        if fwd < C.PF_FWD_SLOW_DIST:
            scale = clamp(
                (fwd - C.PF_FWD_BRAKE_DIST)
                / max(C.PF_FWD_SLOW_DIST - C.PF_FWD_BRAKE_DIST, 1e-3),
                0.25, 1.0,
            )
            v_base *= scale

        # 7c. Gap-width scaling: the wider the chosen gap, the faster we
        #     drive. Saturates at 0.6 m so wide-open spaces still get full
        #     speed.
        gap_scale = clamp(gap_width_m / 0.6, 0.45, 1.0)
        v_base *= gap_scale

        # 7d. Near-final gentle braking.
        if rem < 0.6:
            v_base *= clamp(rem / 0.6, 0.3, 1.0)

        v = clamp(v_base, 0.0, C.V_MAX)
        return v, w, "follow"

    # ----------------------------- helpers ------------------------------

    def _lookahead_point(self, rx, ry, L):
        for i in range(self.idx, len(self.path)):
            wx, wy = self.path[i]
            if math.hypot(wx - rx, wy - ry) >= L:
                return wx, wy
        return self.path[-1]


# ---------------------------------------------------------------------------
# VFH primitives — exposed at module level for unit-testing and reuse.
# ---------------------------------------------------------------------------

def find_best_gap(ranges, angles, desired_bearing,
                  safety_dist=SD_SAFETY_DIST,
                  min_gap_width_m=SD_MIN_GAP_WIDTH_M,
                  max_range=SD_MAX_RANGE,
                  forward_half_angle=SD_FORWARD_HALF_ANGLE):
    """Return ``(centre_bearing_rad, world_width_m, min_range_in_gap_m)``
    for the best gap, or ``None`` if no usable gap exists.

    Algorithm
    ---------
    1. Mark each lidar sector as "blocked" if the return is closer than
       ``safety_dist``. Inflate the block by the angle subtended on a
       circle of radius ``safety_dist`` at the obstacle's range — this is
       the standard VFH+ inflation step.
    2. Mark medium-range obstacles with a smaller inflation (they bound
       gap edges but don't kill them).
    3. Restrict to the forward hemisphere and find runs of unblocked
       sectors.
    4. For each run, estimate the chord-width as ``2·r_min·sin(Δθ/2)``
       and reject anything below ``min_gap_width_m``.
    5. Score by ``(alignment, width, depth)`` and return the winner.
    """
    n = len(ranges)
    if n == 0 or angles is None or len(angles) != n:
        return None

    angles_arr = np.asarray(angles, dtype=np.float64)
    ranges_arr = np.asarray(ranges, dtype=np.float32)
    if n < 2:
        return None
    step = abs(float(angles_arr[1] - angles_arr[0]))
    if step <= 0:
        return None

    # Forward hemisphere mask.
    forward = np.abs(angles_arr) < forward_half_angle

    finite = np.isfinite(ranges_arr) & (ranges_arr > 1e-3)
    too_close = finite & (ranges_arr < safety_dist)
    # NOTE: "no return" or "past max_range" sectors are OPEN (free space),
    # not obstacles. We only treat them as non-contributing to the obstacle
    # inflation step below.

    blocked = np.zeros(n, dtype=bool)
    blocked |= too_close

    # Inflate close obstacles aggressively.
    close_idx = np.where(too_close)[0]
    for i in close_idx:
        r = float(ranges_arr[i])
        # Angular half-width of a circle of radius safety_dist at range r.
        # arcsin clamped because at very short range the inflation is huge.
        inflation = math.asin(min(0.99, safety_dist / max(r, safety_dist * 0.6)))
        k = max(1, int(math.ceil(inflation / step)))
        lo = max(0, i - k)
        hi = min(n, i + k + 1)
        blocked[lo:hi] = True

    # Inflate medium-range obstacles too — softer but still sized by their
    # subtended angle. This is what stops the bot from hugging a wall it's
    # safely past — that wall still pinches the gap edges.
    medium = finite & (ranges_arr >= safety_dist) & (ranges_arr < max_range)
    medium_idx = np.where(medium)[0]
    for i in medium_idx:
        r = float(ranges_arr[i])
        inflation = math.asin(min(0.99, safety_dist / r))
        k = max(0, int(math.ceil(inflation / step)) - 1)
        if k <= 0:
            continue
        lo = max(0, i - k)
        hi = min(n, i + k + 1)
        blocked[lo:hi] = True

    available = forward & ~blocked
    if not available.any():
        return None

    # Run-length encode the available mask.
    av_int = available.astype(np.int8)
    edges = np.diff(av_int)
    starts = list(np.where(edges == 1)[0] + 1)
    ends = list(np.where(edges == -1)[0])
    if available[0]:
        starts.insert(0, 0)
    if available[-1]:
        ends.append(n - 1)
    gaps = list(zip(starts, ends))

    best = None
    best_score = -math.inf
    for s, e in gaps:
        if e - s + 1 < 2:
            continue
        center = 0.5 * (angles_arr[s] + angles_arr[e])
        width_rad = abs(angles_arr[e] - angles_arr[s])
        rs = ranges_arr[s:e + 1]
        valid = np.isfinite(rs)
        r_min = float(rs[valid].min()) if valid.any() else max_range
        # Chord length: 2·r·sin(Δθ/2).
        width_world = 2.0 * r_min * math.sin(max(width_rad / 2.0, 1e-6))
        if width_world < min_gap_width_m:
            continue
        align = abs(wrap_angle(float(center) - desired_bearing))
        if align > math.radians(135):
            continue
        # Composite score:
        #   - alignment dominates (we want to reach the goal);
        #   - width is a strong tie-breaker;
        #   - depth (further-clear gaps) is a mild tie-breaker.
        score = (
            (1.0 - align / math.pi) * 3.0
            + min(width_world, 1.5) * 0.6
            + min(r_min, 2.0) * 0.2
        )
        if score > best_score:
            best_score = score
            best = (float(center), float(width_world), r_min)

    return best


def forward_clearance(ranges, angles, half_angle=math.radians(28.0)):
    """Min lidar range in a narrow forward cone. ``+inf`` if no return."""
    if ranges is None or len(ranges) == 0:
        return float("inf")
    ra = np.asarray(ranges, dtype=np.float32)
    an = np.asarray(angles, dtype=np.float64)
    valid = np.isfinite(ra) & (ra > 0.05) & (np.abs(an) < half_angle)
    if not valid.any():
        return float("inf")
    return float(np.min(ra[valid]))
