"""Central configuration — the single source of truth for the mak_02 controller.

This module is intentionally free of any Webots (`controller`) import so that
every algorithmic module that depends on it stays unit-testable *outside* the
simulator.  Hardware-facing code lives in :mod:`hardware.robot_io`.

The values mirror the Maze1 ROSbot world (Rosbot R2025a proto).  Anything that
might need per-map tuning lives here and nowhere else.
"""
from __future__ import annotations

import math

# ===========================================================================
# Robot geometry (Husarion ROSbot, Webots Rosbot proto)
# ===========================================================================
WHEEL_RADIUS = 0.043            # m
WHEEL_SEPARATION = 0.220        # m (track width)
ROBOT_RADIUS = 0.125            # m, circumscribed footprint radius
ROBOT_HEIGHT = 0.22             # m, top of chassis
LIDAR_MOUNT_Z = 0.10            # m, 2D lidar plane height
CAMERA_MOUNT_Z = 0.165          # m, depth/rgb camera optical centre height

# ===========================================================================
# Webots device names (resolved by hardware.robot_io)
# ===========================================================================
MOTOR_NAMES = ("fl_wheel_joint", "fr_wheel_joint", "rl_wheel_joint", "rr_wheel_joint")
ENCODER_NAMES = (
    "front left wheel motor sensor",
    "front right wheel motor sensor",
    "rear left wheel motor sensor",
    "rear right wheel motor sensor",
)
LIDAR_NAME = "laser"
RGB_CAMERA_NAME = "camera rgb"
DEPTH_NAME_CANDIDATES = ("camera depth", "range-finder", "camera range-finder", "depth")
IMU_NAME_CANDIDATES = ("imu inertial_unit", "imu inertial unit", "inertial unit", "imu")

# IR / ToF distance sensors (Husarion ROSbot VL53L0X x4).  The 2D lidar is BLIND
# closer than 0.2 m (its min range); these short-range sensors fill that blind
# zone so the robot stops clipping corners, and they catch low walls near the
# chassis.  Names are resolved leniently (a full device dump is printed at init
# so the exact names can be confirmed from the run log).
RANGE_SENSOR_NAMES = ("fl_range", "fr_range", "rl_range", "rr_range")
# Approx mount pose (x_fwd, y_left, theta) per sensor IN THE ROBOT FRAME — the
# front pair faces forward (+x), the rear pair faces backward.  These are the
# standard ROSbot corner mounts; tune if the device dump / behaviour disagrees.
RANGE_SENSOR_POSES = (
    (0.10, 0.05, 0.0),          # fl — front-left, facing +x
    (0.10, -0.05, 0.0),         # fr — front-right, facing +x
    (-0.10, 0.05, math.pi),     # rl — rear-left, facing -x
    (-0.10, -0.05, math.pi),    # rr — rear-right, facing -x
)
RANGE_SENSOR_MIN = 0.02         # m; ignore readings below
RANGE_SENSOR_MAX = 0.90         # m; ignore readings beyond (treat as no obstacle)

# ===========================================================================
# Velocity limits
# ===========================================================================
V_MAX = 0.22                    # m/s nominal forward speed cap
V_MIN = 0.04                    # m/s
W_MAX = 1.0                     # rad/s (lowered from 2.0: gentler turns, less
                                # overshoot into walls when escaping corners)
WHEEL_ANG_MAX = V_MAX / WHEEL_RADIUS * 1.5   # rad/s saturation per wheel

# ===========================================================================
# Occupancy grid (Phase B)
# ===========================================================================
GRID_RESOLUTION = 0.04          # m / cell
GRID_SIZE_M = 8.0               # square side length
GRID_CELLS = int(round(GRID_SIZE_M / GRID_RESOLUTION))   # 200
GRID_ORIGIN = (-4.0, -4.0)      # world coord of cell (0, 0) lower-left corner

# Log-odds update model.  Free clearing is deliberately close in magnitude to
# the occupied increment so that a stray hit does not become permanent the way
# it did in the old controller (L_FREE=-0.4 vs L_OCC=+0.85 there).
L_OCC = 0.65                    # add on a beam endpoint
L_FREE = -0.45                  # add on a traversed (free) cell
L_MIN = -2.5                    # clamp
L_MAX = 3.0                     # clamp
L_OCC_THRESH = 0.50             # L above this  => occupied (one hit commits)
L_FREE_THRESH = -0.40           # L below this  => free (one clear pass commits)
# unknown := not occupied and not free (never observed enough to commit either)

# Lidar beams below this range / above max are dropped before integration.
LIDAR_RANGE_MIN = 0.06          # m
LIDAR_RANGE_MAX = 8.0           # m

# ===========================================================================
# Depth-camera auxiliary obstacle layer (floating walls the 2D lidar misses)
# ===========================================================================
# A depth pixel counts as a "low/floating wall" obstacle only if its height is
# inside this band: above floor noise, at or below chassis top.  Pixels higher
# than ROBOT_HEIGHT are passages the robot drives under and must NOT be marked.
AUX_Z_MIN = 0.04                # m above floor
AUX_Z_MAX = ROBOT_HEIGHT        # m (== 0.22, high-float passages stay open)
AUX_MAX_RANGE = 1.5             # m, drop far/noisy depth pixels (was 2.0 — far
                                # depth smeared walls and narrowed passages)
AUX_MIN_HITS = 3                # a cell must be hit this many times before it
                                # becomes a confirmed aux obstacle (noise gate)
# Floating walls are INVISIBLE to the lidar, which keeps marking their cell
# "free" — so the lidar-free decay must NOT erase a well-confirmed floating wall.
# A cell seen this many times (or stamped by the virtual bumper) becomes STICKY
# and survives decay; only low-count (likely-noise) aux marks are cleared.
AUX_STICKY_HITS = 6
AUX_CAP = 12                    # clamp on the per-cell hit counter

# --- Depth obstacle layer (the principled floating-wall fix) ---------------
# A SEPARATE log-odds grid written ONLY by the depth camera (mark + raytrace
# clear), unioned with the lidar map for planning.  The lidar never writes here,
# so a floating wall the lidar reports "free" stays an obstacle; the depth's own
# raytrace clearing erases stale/false marks (self-correcting).  This replaces
# the sticky-aux / reconcile / decay / dead-end-barrier hacks.
DEPTH_L_OCC = 0.70             # add on a depth in-band obstacle endpoint
DEPTH_L_FREE = -0.50           # add on a depth-cleared (tunnel-free) cell
DEPTH_L_MIN = -2.0             # clamp
DEPTH_L_MAX = 3.0              # clamp
DEPTH_OCC_THRESH = 0.50        # depth_L above this => floating-wall obstacle
DEPTH_CLEAR_RANGE = 1.6        # m; clear the robot-height tunnel out to here when
                               # a bearing shows no in-band obstacle
DEPTH_OBS_MIN_RANGE = 0.6      # m; depth camera min range — NEVER clear closer
                               # than this (it can't see free space in the blind
                               # zone; clearing there erased low walls on approach)
DEPTH_MARK_NEAR_WALL_CELLS = 2 # skip depth marks within this many cells of a
                               # lidar wall: this layer is ONLY for obstacles the
                               # lidar misses (floating/low walls), not a copy of
                               # every normal wall (which smeared dobs to ~1000)
# Persistent MAP is LIDAR-ONLY by default: painting depth into the grid created
# the purple clutter that corrupted A*'s costmap.  Depth is still used live for
# floating-wall avoidance (DWA) + learned barriers — just not drawn into the map.
# Set True to restore the old depth-into-grid behaviour.
MAP_DEPTH_AUX_TO_GRID = False
# Live floating-wall avoidance: a depth point counts as a lidar-blind floating
# wall only if NO lidar-occupied cell is within this many cells of it (else it is
# a normal wall the lidar already maps and must not be double-counted into DWA).
LIVE_FLOATING_BLIND_CELLS = 2
# Clearing observation: aux within this distance of a real (lidar-occupied) wall
# is redundant — the lidar already maps that wall thinly — so it is cleared even
# if sticky.  Stops depth drift-spread from permanently bulging wall corners.
# Normal full-height walls also have material in the chassis-height band, so the
# depth cam paints their NEAR FACE; that shadow extends several cells in front of
# the lidar line, so this must be generous (0.08 was too small -> aux exploded to
# 500+ cells).  Genuine floating walls are >this from any lidar wall and survive.
AUX_WALL_CLEAR_RADIUS = 0.15    # m

# ===========================================================================
# Scan matching (Phase A — correlative scan matcher, coarse-to-fine)
# ===========================================================================
SM_ENABLED = True
SM_MAX_BEAMS = 140              # subsample the scan to this many beams
# Likelihood field: occupied cells are blurred by a Gaussian of this sigma so
# the matcher has a smooth score surface to climb.
SM_SIGMA_M = 0.06               # m (was 0.08: sharper field => tighter wall
                                # alignment => thinner walls, less drift lock-in)
# Coarse search half-window and step.
SM_COARSE_LIN_HALF = 0.16       # m, search +/- this in x and y
SM_COARSE_LIN_STEP = 0.04       # m
SM_COARSE_ANG_HALF = math.radians(8.0)   # rad
SM_COARSE_ANG_STEP = math.radians(2.0)   # rad
# Fine search half-window and step (centred on the coarse winner).
SM_FINE_LIN_HALF = 0.03         # m
SM_FINE_LIN_STEP = 0.01         # m
SM_FINE_ANG_HALF = math.radians(2.0)     # rad
SM_FINE_ANG_STEP = math.radians(0.5)     # rad
# gmapping-style update thresholds: only scan-match + integrate after the robot
# has moved at least this far / turned at least this much since the last update.
SM_UPDATE_LIN_M = 0.05          # m
SM_UPDATE_ANG_RAD = math.radians(5.0)    # rad
# Robust acceptance gate (prevents the "accepted a bad match -> pose jumped" GO_YELLOW
# stall in the certification report).  The raw score `norm` is the mean likelihood
# of the matched scan against the map (~fraction of beams landing on walls):
#   * norm >= SM_TRUST_SCORE_FRAC  -> STRONG, unambiguous match: always accept,
#     even a large correction (the map clearly explains the scan).
#   * SM_MIN_SCORE_FRAC <= norm < SM_TRUST_SCORE_FRAC -> MARGINAL/ambiguous (open
#     areas near the poison block score here): accept ONLY if the correction is
#     small (<= SM_MARGINAL_MAX_CORR_M), i.e. consistent with odometry.  A
#     marginal score that also wants to TELEPORT the pose is the failure mode —
#     reject it and dead-reckon on the (good) wheel+IMU odometry instead.
#   * norm < SM_MIN_SCORE_FRAC -> LOST: reject, dead-reckon.
# This is the standard correlative-scan-match outlier rejection (cartographer /
# gmapping downweight low-confidence matches rather than committing a jump).
SM_MIN_SCORE_FRAC = 0.30        # below this => lost, dead-reckon.  Kept low so the
                                # robot stays localized in FEATURE-POOR pockets (next
                                # to a lidar-invisible floating wall the scan has few
                                # matchable features and the score drops to ~0.4);
                                # the displacement gate below makes a low score safe
                                # — it can only nudge, never teleport, the pose.
SM_TRUST_SCORE_FRAC = 0.60      # at/above this => strong match, accept any correction
SM_MARGINAL_MAX_CORR_M = 0.10   # m; max correction accepted in the marginal band

# ===========================================================================
# Costmap / inflation (planning)
# ===========================================================================
HARD_OBS_MARGIN = 0.025         # lethal if dist < ROBOT_RADIUS - this (=> 0.10 m)
SOFT_OBS_HALO = 0.12            # m, soft cost band beyond the hard radius
HARD_POISON_MARGIN = -0.02      # poison is stricter than walls
SOFT_POISON_HALO = 0.20         # m
INFLATION_ALPHA = 3.0           # soft-cost exponential falloff
COST_OBS_WEIGHT = 6.0
COST_POISON_WEIGHT = 30.0

# ===========================================================================
# Frontier exploration (Phase C)
# ===========================================================================
FRONTIER_MIN_CLUSTER = 5        # cells; ignore clusters smaller than this
FRONTIER_MIN_DIST_M = 0.35      # m; ignore frontiers closer than this to robot
FRONTIER_BLACKLIST_RADIUS_M = 0.40   # m; failed goals suppress nearby frontiers
EXPL_MAX_RADIUS_M = 4.0         # m; never chase a frontier beyond this from start
# Utility = INFO_GAIN_W * (cluster cells) - DIST_W * (path length, m).
UTIL_INFO_GAIN_W = 1.0
UTIL_DIST_W = 1.6
INITIAL_SCAN_REVS = 1.05        # spin this many revolutions to seed the map
# After a failed attempt to reach a KNOWN-but-unreachable pillar, explore for
# this long (opening the route) before retrying — prevents the EXPLORE<->GO
# flip-flop when the pillar is walled off behind an unmapped passage.
GO_FAIL_COOLDOWN_S = 2.5
# Dead-end handling: a pillar can be VISIBLE but unreachable from the current
# side — e.g. the blue pillar sits behind WallMedium(3), a FLOATING wall (bottom
# edge 0.20 m, robot is 0.22 m tall) that the camera sees under but the robot
# cannot pass.  The lidar passes under it and marks the cells free, so A* keeps
# routing straight at the pillar -> GO fails -> flip-flop / wedged.  After this
# many GO attempts that make < GO_DEADEND_PROGRESS_M net approach, we LEARN the
# barrier (stamp sticky aux across the bearing to the pillar so A* reroutes),
# blacklist the approach, and explore for a route in from another side.
GO_DEADEND_FAILS = 3
GO_DEADEND_PROGRESS_M = 0.25      # m; net approach below this => attempt "failed"
GO_DEADEND_MAX_DIST = 1.30        # m; only stamp a barrier if stalled THIS close
                                  # to the pillar (else it's a nav stall, not a
                                  # real block — don't wall off good routes)
GO_DEADEND_COOLDOWN_S = 25.0      # s; suppress GO to that pillar after a dead-end
GO_DEADEND_BARRIER_AHEAD = ROBOT_RADIUS + 0.10   # m toward pillar to stamp barrier
GO_DEADEND_BARRIER_R = 0.10       # m; radius of each learned barrier disc
# Lateral offsets (m, perpendicular to the approach) at which barrier discs are
# stamped, so the learned barrier is a LINE that seals the whole passage/floating
# wall rather than a single dot A* can route around.  ±0.20 m + disc 0.10 +
# inflation ~0.10 => ~0.6 m sealed, wider than the robot.
GO_DEADEND_BARRIER_OFFSETS = (-0.20, -0.10, 0.0, 0.10, 0.20)
GO_DEADEND_BLACKLIST_R = 0.45     # m; suppress frontiers near the failed approach

# ===========================================================================
# Path following (pure pursuit)
# ===========================================================================
LOOKAHEAD_BASE = 0.28           # m
LOOKAHEAD_K_V = 0.40            # m per (m/s)
LOOKAHEAD_MIN = 0.16            # m
LOOKAHEAD_MAX = 0.65            # m
WAYPOINT_REACH_TOL = 0.16       # m
GOAL_REACH_TOL = 0.18           # m
PP_K_HEADING = 2.6              # rad/s per rad of heading error
PP_BIG_HEADING_STOP = 0.6       # rad; above this, spin in place (v=0)

# ===========================================================================
# Recovery
# ===========================================================================
STUCK_PROGRESS_MIN_M = 0.08     # m of progress expected per window
STUCK_TIMEOUT_S = 3.0           # s without progress => recovery (was 4.0)
RECOVERY_REVERSE_V = 0.20       # m/s (was 0.16 — back out of a wedge harder)
RECOVERY_REVERSE_T = 2.2        # s; MAX straight reverse (=> ~0.44 m).  The
                                # reverse now stops EARLY once there is room to
                                # rotate (see RECOVERY_ROTATE_CLEAR), so this is a
                                # cap that lets the robot back fully out of a tight
                                # channel before turning instead of K-turning in it.
RECOVERY_SPIN_W = 1.4           # rad/s
RECOVERY_SPIN_T = 1.0           # s
# Only rotate in place once the MAP says the nearest wall is at least this far —
# the robot is ~0.32 m on the diagonal, so in a channel narrower than ~2x this it
# would scrub a corner into the wall when the skid-steer drift swings it.  Backing
# out to a spot with this much clearance lets the turn absorb the drift.  Measured
# from the map (not the lidar — the trapping wall is often inside the 0.2 m lidar
# blind zone).
RECOVERY_ROTATE_CLEAR = 0.20    # m; required wall clearance before turning
RECOVERY_REAR_PROBE = 0.22      # m behind the robot to test for poison/wall when
                                # reversing (never back into poison or a wall)
RECOVERY_MAX_CHAIN = 3          # consecutive recoveries before global replan
RECOVERY_ESCAPE_CLEAR_R = 0.25  # m; on repeated recovery, wipe aux this far
                                # around the (freely-spinning) robot to break out
                                # of a false-aux box that has no reachable exit
# Rear-aware turning: before reversing (or swinging through a big turn) check the
# live scan behind the robot.  If a wall is within REAR_MIN_CLEAR in the rear
# cone, skip the reverse and rotate in place instead so the tail never backs into
# a wall.  Spins are biased toward whichever side has more room.
RECOVERY_REAR_MIN_CLEAR = 0.15  # m (legacy; recovery now reverses unless the rear
                                # is HARD-blocked — see RECOVERY_REAR_HARD)
RECOVERY_REAR_HARD = 0.12       # m; only veto the reverse if something is this
                                # close directly behind (else always back out)
RECOVERY_REAR_HALF_WIDTH = 0.16 # m, half-width of the rear collision cone

# ===========================================================================
# DWA local planner (the move_base local-planner equivalent)
# ===========================================================================
# Pure pursuit only TRACKS a path; this layer drives the robot using the LIVE
# lidar so it stays clear of walls and centres itself in corridors, exactly like
# move_base's DWA + inflation layer in the reference repo.  Candidate (v, w)
# pairs are rolled out and any trajectory passing closer than LP_SAFE_RADIUS to a
# lidar point is rejected; among the survivors the highest-scoring one wins.
LP_V_SAMPLES = 6                # forward-speed samples in [0, V_MAX]
LP_W_SAMPLES = 21               # angular samples in [-W_MAX, W_MAX] (odd => incl 0)
LP_HORIZON_S = 1.0              # trajectory rollout horizon (s)
LP_STEP_S = 0.2                 # rollout integration step (s)
# Hard reject radius = the robot's INSCRIBED radius (physical half-width, 0.10 m)
# and == the A* global-plan hard clearance (ROBOT_RADIUS - HARD_OBS_MARGIN =
# 0.10 m).  Two invariants make this the correct value, NOT a tuning knob:
#   1. It must be >= the half-width or the robot's SIDES graze walls (the old
#      0.08 was 2 cm below the half-width => guaranteed side-grazing; this is the
#      "drives up and touches the wall" bug from the certification report).
#   2. It must be <= the A* hard clearance or the local planner rejects the very
#      path A* produced (center cells are 0.10 m clear) -> self-induced stalls.
# Equality (0.10 == 0.10) satisfies both; the LP_W_CLEAR reward keeps the robot
# centred when there is room, so it only approaches 0.10 m in genuinely tight
# passages.  Using the larger circumscribed radius (0.125) would refuse gaps the
# 0.20 m-wide robot can thread straight, so we keep the inscribed value.
LP_SAFE_RADIUS = 0.10           # m; inscribed (half-width) radius == A* hard clearance
LP_CLEAR_CAP = 0.5              # m; clearance reward saturates here
# Scoring weights: heading drives progress, clearance centres + avoids walls.
LP_W_HEADING = 1.5              # alignment of final heading toward the carrot
LP_W_DIST = 1.0                # closeness of trajectory end to the carrot
LP_W_CLEAR = 0.4               # clearance from obstacles (corridor centring)
LP_W_VEL = 0.10                # mild preference for moving
LP_W_STRAIGHT = 0.15           # penalty on |w| -> damps the spin oscillation
LP_SEARCH_W = 0.8              # in-place spin (rad/s) when fully boxed in
# Proximity slowdown: scale the forward-speed window down as the nearest
# obstacle approaches, so the robot CRAWLS through tight spots and corners
# (less overshoot, and the scan matcher keeps up so it doesn't clip the inside
# corner from drift).  Full speed at LP_SLOWDOWN_DIST, ~V_MIN at LP_SAFE_RADIUS.
LP_SLOWDOWN_DIST = 0.45        # m (legacy; the global proximity slowdown was
                               # removed — speed is set by per-trajectory clearance)
LP_SUBSAMPLE = 90              # cap lidar points used for clearance (speed)
# DWA also avoids MAPPED aux/poison cells within this radius (floating walls and
# poison the live sensors can't see right now), not just the live lidar scan.
LP_MAPPED_OBS_RADIUS = 1.5     # m
# Poison is stricter than a wall: the DWA hard radius (0.10 m, inscribed) is
# SMALLER than the circumscribed footprint (0.125 m), so at 0.10 m the robot's
# corner would overlap a poison cell == mission failure.  We therefore DILATE the
# poison contribution to the DWA obstacle set by this many cells (0.04 m each) so
# the effective poison clearance becomes ~0.14 m >= the footprint radius, matching
# the A* poison hard clearance (ROBOT_RADIUS - HARD_POISON_MARGIN = 0.145 m).
LP_POISON_EXTRA_CELLS = 1      # cells of extra berth around poison for the DWA
# Floating walls (mapped depth_obs) get extra DWA berth too — the depth camera is
# blind < 0.6 m, so close in only the MAPPED layer can stop the robot driving
# under / into a floating wall.  Inflating it by this many cells (0.04 m each)
# keeps the robot clear even when right up against one.  Keep it small: the
# floating-wall passages are ~0.34 m wide, so 1 cell (=> ~0.14 m effective
# clearance) still fits the 0.25 m robot; 2 cells would start to choke them.
LP_DEPTH_OBS_EXTRA_CELLS = 1   # cells of extra berth around floating walls

# ===========================================================================
# Virtual bumper — learns invisible floating walls from collisions
# ===========================================================================
# If the robot commands forward motion but makes no progress for this long, an
# obstacle the sensors can't see is blocking it (a floating wall below/above the
# lidar plane and closer than the 0.6 m depth-cam minimum).  Stamp a STICKY aux
# obstacle just ahead so the map learns it and the planner routes around.
BUMPER_STALL_TIME_S = 0.6       # s of no progress while pushing forward
BUMPER_MIN_CMD_V = 0.10         # m/s; only arm while actually trying to drive
BUMPER_MIN_PROGRESS_M = 0.03    # m; less movement than this counts as blocked
BUMPER_MARK_AHEAD = ROBOT_RADIUS + 0.04   # m ahead of the robot to stamp
BUMPER_MARK_RADIUS = 0.06       # m radius of the stamped obstacle disc
# Only stamp a (permanent) barrier when localization is trustworthy.  A bump is
# detected as "commanding forward but not moving"; if the scan match is poor at
# that instant the pose itself is suspect, so the stamp would land at the WRONG
# place — a phantom wall that can permanently seal a real route (the (0.9,-1.1)
# barrier on the YELLOW approach corridor in the certification report).  When
# localization is poor we skip the stamp and let recovery unwedge instead.
BUMPER_MIN_SM_NORM = 0.45       # require this scan-match confidence before stamping

# ===========================================================================
# Perception — HSV thresholds (OpenCV convention, H in [0,179])
# ===========================================================================
HSV_BLUE = ((100, 120, 60), (130, 255, 255))
HSV_YELLOW = ((20, 120, 120), (35, 255, 255))
HSV_GREEN = ((40, 100, 60), (80, 255, 255))

PILLAR_RADIUS = 0.10            # m
PILLAR_HEIGHT = 0.30            # m
PILLAR_MIN_PIXELS = 20          # min coloured blob area (pixel count)
PILLAR_MAX_DETECT_RANGE = 4.5   # m
PILLAR_HEIGHT_MIN_FRAC = 0.30   # accept est_height in [min,max] * PILLAR_HEIGHT
PILLAR_HEIGHT_MAX_FRAC = 1.80
PILLAR_ASPECT_MAX = 2.5         # reject blobs wider than this (likely a wall)
PILLAR_OBS_AVG_N = 8            # running-mean window for the world position
PILLAR_OUTLIER_REJECT_M = 1.5   # m; discard detections this far from the mean
PILLAR_CONFIRM_DIST = 0.35      # m; odometry must reach within this to "arrive"
# Depth-verified pillar CONFIRMATION (so the robot only counts a pillar as reached
# after it has genuinely driven up to it — not after glimpsing it through/under a
# floating wall from afar with a biased estimate).  The depth camera is blind
# < 0.6 m, so the closest reliable confirmation band is just above that.  A
# detection whose depth range is in [MIN, CONFIRM_RANGE] and which agrees with the
# tracked estimate (within SNAP_TOL) counts as a close confirmation; after
# CONFIRM_HITS such readings the pillar is confirmed and its estimate is SNAPPED
# to the accurate close measurement.  Only a confirmed pillar can be "reached".
PILLAR_CONFIRM_MIN_RANGE = 0.50    # m; floor (depth is blind below ~0.6 anyway)
PILLAR_DEPTH_CONFIRM_RANGE = 0.95  # m; must see the pillar within this to confirm
PILLAR_CONFIRM_SNAP_TOL = 0.60     # m; close reading must be this near the estimate
PILLAR_CONFIRM_HITS = 3            # close readings required before confirming
# Confirmation also requires a CLEAR ground path to the pillar (no wall/floating
# wall between robot and pillar) — the depth cam sees the pillar UNDER a floating
# wall, so closeness alone wrongly "reaches" it.  The line-of-sight check ignores
# this much distance before the pillar so the pillar's own (occupied) cell and
# the cells right around it don't count as the blocker.
PILLAR_LOS_STOP_SHORT = 0.25       # m of the segment near the pillar to ignore
# When a pillar is known but not yet depth-confirmed, the robot drives to the
# nearest VIEWPOINT it can actually see the pillar from — a clear-LOS, non-lethal
# point at one of these radii (in the depth confirm band, >= the 0.6 m blind
# zone) around the pillar.  This stops it parking at an estimate that sits in an
# occluded pocket (blue is walled in, only visible from the south/west).
PILLAR_VIEW_RADII = (0.70, 0.80, 0.65, 0.88)   # m; viewpoint ring radii to try
PILLAR_VIEW_CANDIDATES = 24        # bearings sampled per radius
# A pillar blob whose left/right edge is within this many pixels of the image
# border is a PARTIAL view (pillar runs out of frame) — biased depth/bearing, so
# it must not CONFIRM the pillar (only a clean, fully-in-frame view may).
PILLAR_EDGE_MARGIN_PX = 4
# To CONFIRM, the pillar blob must also be at least this wide (px).  A 0.20 m
# pillar at the confirm range (<= 0.95 m) is >= ~120 px wide; a sliver seen
# through a gap is far narrower, and its centroid depth reads the occluding edge
# (the bias source).  70 px easily passes a full close view, rejects slivers.
PILLAR_CONFIRM_MIN_WIDTH_PX = 70

# ===========================================================================
# Mission orchestration (main FSM)
# ===========================================================================
INIT_SPIN_W = 1.2               # rad/s, in-place spin to seed the initial map
PLAN_REPLAN_PERIOD_S = 1.0      # s, replan cadence while navigating
FIELD_REBUILD_EVERY = 1         # rebuild likelihood field every N SLAM updates
PERCEPTION_EVERY_TICKS = 2      # run colour perception every N ticks
PILLAR_REACH_DIST = 0.45        # m, "arrived at pillar" when corrected pose within
GOAL_FAIL_BLACKLIST = True      # blacklist frontiers whose plan repeatedly fails
DONE_LINGER_S = 1.0             # s to keep the window open after DONE

# ===========================================================================
# Logging / visualisation
# ===========================================================================
LOG_EVERY_TICKS = 32
SNAPSHOT_PERIOD_S = 3.0
OUTPUT_DIRNAME = "maps"
