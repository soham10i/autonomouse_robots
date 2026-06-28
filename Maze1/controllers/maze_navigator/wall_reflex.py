"""Virtual-bumper IR proximity reflex.

A thin safety layer that sits between the active controller (FSM /
SmoothDriver / pure-pursuit follower) and the motor commands. It reads
the four corner IR rangers and, when one is closer to a wall than
``IR_REFLEX_TRIGGER``, modifies the planner's ``(v, w)`` to push the
chassis away.

Why this exists
---------------
The 2D lidar is at z = 0.10 m and only sees a horizontal slice of the
world. It can perfectly miss:
  - a tilted wall whose top edge is below the lidar
  - a low overhang at chassis height
  - a wall edge that's about to clip the front-left wheel during a
    sharp turn
The depth-cam catches some of these in the aux-obstacle layer but it
runs at planner cadence (often after the wheel has scraped). The four
IRs at the chassis corners are the closest-in safety net.

Design constraints
------------------
1. Pure function (with one small ``ReflexState`` for hysteresis) —
   no controller surgery, easy to disable.
2. Hysteresis: trigger at TRIGGER, release at RELEASE > TRIGGER. Stops
   chattering when the bot is hovering at the boundary.
3. Time-bounded: if a reflex hasn't cleared in ``MAX_HOLD_S`` it gives
   up — the bot is wedged and the existing recovery FSM should take
   over.
4. State-aware: reflex is disabled in INIT_SCAN (deliberate spinning)
   and RECOVERY (recovery is already manoeuvring deliberately close to
   walls).
5. Sign convention: ``+w`` is CCW (left turn), matching the rest of
   the codebase.

Trigger rules (priority order)
------------------------------
* ``fl < TRIGGER`` AND ``fr < TRIGGER``: bot is squarely facing a wall
  → return ``(0, 0, "wall-ahead")`` and let recovery take over.
* ``rl < TRIGGER`` AND ``rr < TRIGGER`` AND ``v < 0``: pinned at the
  back during a reverse → cancel the reverse, keep ``w``.
* ``fl < TRIGGER`` only: left-front about to clip → cap ``v`` and add
  a right-turn (negative w).
* ``fr < TRIGGER`` only: mirror.
* ``rl < TRIGGER`` only AND ``v > 0``: rear-left was caught during a
  right turn — ease the arc by adding a small left-turn correction.
* ``rr < TRIGGER`` only AND ``v > 0``: mirror.
"""
import math
import os

import config as C


# ---------------------------------------------------------------------------
# Tunables — env-var overridable so we can A/B without touching code.
# ---------------------------------------------------------------------------
def _get(name, default):
    v = os.environ.get(name)
    if v is None:
        return default
    try:
        return type(default)(v)
    except (TypeError, ValueError):
        return default


# Master enable flag — set ``IR_REFLEX_ENABLED=0`` to fully bypass the layer.
ENABLED = _get("IR_REFLEX_ENABLED", 1) == 1
# Trigger threshold: anything closer than this on a corner IR fires the reflex.
# Lowered from 0.20 → 0.15 because in narrow maze corridors a corner IR is
# *naturally* in the 0.18-0.22 m range during legitimate traversal — firing
# the reflex there caused the planner-vs-reflex thrash we saw in the log.
TRIGGER_M = float(_get("IR_REFLEX_TRIGGER", 0.15))
# Release threshold: hysteresis. Reflex stays active until ALL fronts are
# clear of this distance, preventing rapid re-engage chatter.
RELEASE_M = float(_get("IR_REFLEX_RELEASE", 0.20))
# Push-away angular velocity magnitude, rad/s. Strong enough to overpower
# the planner's heading command in tight spots.
W_MAG = float(_get("IR_REFLEX_W", 0.7))
# Forward speed cap while reflex is active.
V_CAP = float(_get("IR_REFLEX_V_CAP", 0.10))
# Hold time-out: if reflex hasn't cleared the obstacle in this long, we
# stop overriding so the recovery FSM gets a chance.
MAX_HOLD_S = float(_get("IR_REFLEX_MAX_HOLD_S", 1.0))
# IRs report a tiny distance when the obstacle is right at the sensor; below
# this number the reading is unreliable (sensor blind range). We treat
# anything < this as "nothing detected" (bot would have already collided).
SENSOR_BLIND_M = 0.03
# When the planner's commanded forward velocity is below this, the bot is
# rotating in place (or stationary) — there's no forward collision possible
# so the reflex just bypasses. Rationale: the original log showed the bot
# wedged with planner commanding ``v=0`` while the reflex kept adding
# rotational noise on top, which only made alignment worse without ever
# preventing a collision.
V_BYPASS_M_S = float(_get("IR_REFLEX_V_BYPASS", 0.05))

# State names where the reflex is NOT applied (planner is intentionally
# moving the bot near walls / spinning in place).
DISABLED_STATES = frozenset({"INIT", "INIT_SCAN", "RECOVERY", "DONE", "FAILED"})


class ReflexState:
    """Tiny mutable container the controller passes back into ``adjust``.

    Tracks hysteresis (active flag) and trigger time so we can bound
    how long a reflex holds before yielding to the recovery FSM. Also
    tallies counts per IR for the diagnostic line at finalise.
    """

    def __init__(self):
        self.active = False
        self.t_started = 0.0
        # Per-IR fire counts for the post-mission summary.
        self.counts = {"fl_range": 0, "fr_range": 0,
                       "rl_range": 0, "rr_range": 0,
                       "wall-ahead": 0, "pinned-back": 0}
        # Last reason logged (so the controller can de-dupe spam).
        self.last_why = None

    def summary(self):
        """Return a short string for the finalise log."""
        total = sum(self.counts.values())
        if total == 0:
            return "no fires"
        return ("fires=" + str(total) + "  "
                + "  ".join(f"{k}={v}" for k, v in self.counts.items() if v))


def _is_close(d, thr):
    """Closer than ``thr`` AND outside the sensor's blind range."""
    return (d is not None and not _isnan(d)
            and SENSOR_BLIND_M < d < thr)


def _isnan(x):
    try:
        return math.isnan(x)
    except (TypeError, ValueError):
        return False


def adjust(v, w, ir_m, state_name, sim_time, rs):
    """Apply the IR reflex to a planner-issued ``(v, w)`` command.

    Parameters
    ----------
    v, w : float
        Planner-issued velocity command (m/s, rad/s; +w = CCW).
    ir_m : dict
        ``{name: distance_m}`` from ``RobotIO.read_ir_m()``. Missing
        sensors map to NaN; missing keys are treated as "no detection".
    state_name : str
        Current FSM state — reflex is bypassed for ``DISABLED_STATES``.
    sim_time : float
        Sim-clock time, used for the MAX_HOLD timer.
    rs : ReflexState
        Mutable hysteresis tracker (one per controller instance).

    Returns
    -------
    (v_out, w_out, triggered, why)
        ``triggered`` is True iff a reflex rule fired this tick.
        ``why`` is a short tag for logging; "" when not triggered.
    """
    if not ENABLED or state_name in DISABLED_STATES:
        # Cool the hysteresis if we're disabled mid-reflex.
        rs.active = False
        return v, w, False, ""

    # If the planner isn't asking for forward motion, there's no forward
    # collision to prevent. The reflex purpose is to keep the bot from
    # scraping a wall while *driving toward* it; pure in-place rotation
    # near a wall isn't dangerous (the bot can't collide while v=0).
    # Skipping here also prevents the planner-vs-reflex tug-of-war we saw
    # when the path follower rotates in place to align with a goal.
    if abs(v) < V_BYPASS_M_S:
        rs.active = False
        return v, w, False, ""

    fl = ir_m.get("fl_range", float("nan"))
    fr = ir_m.get("fr_range", float("nan"))
    rl = ir_m.get("rl_range", float("nan"))
    rr = ir_m.get("rr_range", float("nan"))

    # Hysteresis: which threshold to use this tick.
    thr = RELEASE_M if rs.active else TRIGGER_M

    # ── Priority 1: squarely facing a wall ─────────────────────────────────
    if _is_close(fl, thr) and _is_close(fr, thr):
        if not rs.active:
            rs.active = True
            rs.t_started = sim_time
        rs.counts["wall-ahead"] += 1
        rs.last_why = "wall-ahead"
        return 0.0, 0.0, True, "wall-ahead"

    # ── Priority 2: pinned at the back during reverse ──────────────────────
    if _is_close(rl, thr) and _is_close(rr, thr) and v < 0.0:
        if not rs.active:
            rs.active = True
            rs.t_started = sim_time
        rs.counts["pinned-back"] += 1
        rs.last_why = "pinned-back"
        return 0.0, w, True, "pinned-back"

    # ── Priority 3a: front-left close → push right ─────────────────────────
    if _is_close(fl, thr) and not _is_close(fr, thr):
        if not rs.active:
            rs.active = True
            rs.t_started = sim_time
        rs.counts["fl_range"] += 1
        rs.last_why = "fl-close"
        v_out = max(min(v, V_CAP), 0.0)        # cap forward, no reverse
        w_out = w - W_MAG                       # negative w = right turn
        return v_out, w_out, True, "fl-close"

    # ── Priority 3b: front-right close → push left ─────────────────────────
    if _is_close(fr, thr) and not _is_close(fl, thr):
        if not rs.active:
            rs.active = True
            rs.t_started = sim_time
        rs.counts["fr_range"] += 1
        rs.last_why = "fr-close"
        v_out = max(min(v, V_CAP), 0.0)
        w_out = w + W_MAG
        return v_out, w_out, True, "fr-close"

    # ── Priority 4: rear corner close while moving forward ─────────────────
    # The rear caught a wall during a turn — ease the arc.
    if _is_close(rl, thr) and v > 0.0:
        rs.active = True
        rs.t_started = sim_time
        rs.counts["rl_range"] += 1
        rs.last_why = "rl-clip"
        return v, w + 0.5 * W_MAG, True, "rl-clip"   # nose left, ease the right turn
    if _is_close(rr, thr) and v > 0.0:
        rs.active = True
        rs.t_started = sim_time
        rs.counts["rr_range"] += 1
        rs.last_why = "rr-clip"
        return v, w - 0.5 * W_MAG, True, "rr-clip"

    # ── Nothing close → release reflex if it was active ───────────────────
    if rs.active:
        # All fronts above RELEASE_M (because we used thr=RELEASE_M above
        # and none of the above branches hit) → reflex truly clear.
        rs.active = False
    rs.last_why = None
    return v, w, False, ""


def time_bound_check(rs, sim_time):
    """Return True if a stuck reflex has held for ``MAX_HOLD_S``.

    The controller can use this to stop trusting the reflex and let the
    existing recovery FSM take over. Doesn't mutate ``rs`` — caller
    decides what to do.
    """
    if not rs.active:
        return False
    return (sim_time - rs.t_started) >= MAX_HOLD_S
