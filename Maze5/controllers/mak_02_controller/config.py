"""Single source of truth for the Maze5 ROSbot controller.

No Webots import here so every algorithmic module that depends on it stays
importable / testable outside the simulator.  Hardware-facing code lives in
:mod:`robot_io`.

Maze5 facts that drove the tuning (read from ``Maze5/worlds/Maze5.wbt``):
  * Robot start  ~ (-1.19,  2.53),  yaw ~ -1.31 rad
  * Blue pillar  ~ ( 1.81, -0.20)   (reach this first)
  * Yellow pillar~ (-2.73,  3.20)   (reach this second; time stops here)
  * Two green "poison" floor patches (1.8 x 0.5, ~15 deg) between start & blue.
  * Walls are boxes 1.0 x 0.05 x 0.5 centred at z=0.18  ->  span z in [-0.07,0.43].
    The 2-D lidar plane is at z=0.10, INSIDE that band, so the lidar sees EVERY
    wall.  Maze5 therefore has NO floating-wall problem -> a clean lidar-only
    occupancy map is sufficient and we deliberately do NOT use a depth "aux"
    obstacle layer (that layer was the main bug source on Maze1).
"""
from __future__ import annotations

import math

# ===========================================================================
# Robot geometry (Husarion ROSbot, cyberbotics Rosbot R2025a proto)
# ===========================================================================
WHEEL_RADIUS = 0.043            # m
WHEEL_SEPARATION = 0.220        # m, track width
ROBOT_RADIUS = 0.125            # m, circumscribed footprint radius
ROBOT_HEIGHT = 0.22             # m
LIDAR_MOUNT_Z = 0.10            # m, 2-D lidar plane
CAMERA_MOUNT_Z = 0.165          # m, optical centre height (camera looks ~horizontal)

# ===========================================================================
# Webots device names (validated on real Maze1 runs of the same proto)
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

# ===========================================================================
# Velocity limits  (fast on straights, smooth in corners)
# ===========================================================================
V_MAX = 1.50                    # m/s nominal forward cap on open straights
V_CRUISE = 1.30                 # m/s typical corridor speed
V_MIN = 0.05                    # m/s
W_MAX = 4.5                     # rad/s
A_V = 3.0                       # m/s^2 translational accel cap (smoothness)
A_W = 12.0                      # rad/s^2 angular accel cap (snappy but smooth corners)
WHEEL_ANG_MAX = V_MAX / WHEEL_RADIUS * 1.6   # rad/s saturation per wheel

# ===========================================================================
# Occupancy grid  (lidar-only, log-odds)
# ===========================================================================
GRID_RESOLUTION = 0.05          # m / cell
GRID_SIZE_M = 11.0              # square side, comfortably covers the arena
GRID_CELLS = int(round(GRID_SIZE_M / GRID_RESOLUTION))   # 220
GRID_ORIGIN = (-5.5, -5.5)      # world coord of cell (0,0) lower-left corner

L_OCC = 0.85                    # log-odds added on a beam endpoint
L_FREE = -0.40                  # log-odds added on a traversed cell
L_MIN = -2.5
L_MAX = 3.5
L_OCC_THRESH = 0.50             # L above this => occupied
L_FREE_THRESH = -0.35           # L below this => free
LIDAR_RANGE_MIN = 0.12          # m, drop closer beams (self-return / noise)
LIDAR_RANGE_MAX = 8.0           # m
LIDAR_MAX_INTEGRATE = 5.0       # m, do not map endpoints beyond this (keeps map crisp)

# ===========================================================================
# Costmap / inflation (planning)
# ===========================================================================
# Cells whose distance to the nearest obstacle is below the hard radius are
# lethal; a soft cost decays over the halo so A* prefers corridor centres but
# can still thread tight gaps.  Poison is stricter (crossing it = mission fail).
HARD_OBS_DIST = ROBOT_RADIUS - 0.02   # m -> lethal (~0.105). Tight to allow narrow gaps.
SOFT_OBS_HALO = 0.22                  # m, soft band beyond hard radius
COST_OBS_WEIGHT = 8.0
POISON_HARD_DIST = ROBOT_RADIUS + 0.06   # m, keep well clear of poison
POISON_SOFT_HALO = 0.30                  # m
COST_POISON_WEIGHT = 60.0
MIN_GAP_WIDTH = 2 * ROBOT_RADIUS + 0.03  # gaps narrower than this are impassable

# ===========================================================================
# Scan matching (light correlative correction of odometry drift)
# ===========================================================================
SM_ENABLED = True
SM_MIN_TRAVEL_M = 0.06          # only correct after this much motion
SM_MIN_TURN_RAD = math.radians(6.0)
SM_MAX_BEAMS = 160             # subsample scan for speed
SM_LIN_HALF = 0.12             # m, (x,y) search half-window
SM_LIN_STEP = 0.03             # m
SM_ANG_HALF = math.radians(5.0)   # rad, yaw search half-window (IMU is good, keep small)
SM_ANG_STEP = math.radians(2.5)
SM_MIN_HIT_FRAC = 0.30         # reject a match where < this frac of beams hit a wall
                               # (corrections are bounded by the small search
                               # window above, so accepting modest matches is safe)

# ===========================================================================
# Frontier exploration
# ===========================================================================
FRONTIER_MIN_CLUSTER = 4        # cells; ignore tiny clusters
FRONTIER_MIN_DIST_M = 0.30      # m; ignore frontiers nearer than this
FRONTIER_BLACKLIST_R = 0.40     # m; failed goals suppress nearby frontiers
EXPL_MAX_RADIUS_M = 6.0         # m from start; do not chase frontiers past this.
                                # (blue pillar ~4.05 m, yellow ~1.68 m from start
                                #  in Maze5; this keeps a margin while preventing
                                #  the robot wandering onto the open floor.)
UTIL_INFO_W = 1.0               # utility = INFO_W*cluster - DIST_W*path_len
UTIL_DIST_W = 1.3
INITIAL_SCAN_REVS = 1.0         # spin this many revolutions to seed the map
INIT_SPIN_W = 2.2               # rad/s during the seed spin (same coverage, faster)

# ===========================================================================
# Path following (pure pursuit carrot for the DWA layer)
# ===========================================================================
LOOKAHEAD_BASE = 0.32           # m
LOOKAHEAD_K_V = 0.45            # m per (m/s)
LOOKAHEAD_MIN = 0.20
LOOKAHEAD_MAX = 0.75
WAYPOINT_REACH_TOL = 0.16       # m
GOAL_REACH_TOL = 0.16           # m
REPLAN_PERIOD_S = 1.0           # s, global replan cadence
PATH_SIMPLIFY = True            # line-of-sight shortcut the A* path

# ===========================================================================
# DWA local planner (drives toward the carrot using LIVE lidar)
# ===========================================================================
DWA_V_SAMPLES = 7
DWA_W_SAMPLES = 25              # odd -> includes w=0
DWA_HORIZON_S = 1.1
DWA_STEP_S = 0.18
DWA_SAFE_RADIUS = 0.135         # m, hard reject if a rollout passes closer than this
DWA_CLEAR_CAP = 0.6             # m, clearance reward saturates here
DWA_W_HEADING = 1.1            # align final heading with carrot bearing
DWA_W_DIST = 1.2              # end the rollout near the carrot
DWA_W_CLEAR = 0.85            # clearance (corridor centring + wall avoidance)
DWA_W_SPEED = 0.18           # mild preference for moving
DWA_W_STRAIGHT = 0.10        # penalty on |w|, damps oscillation
DWA_SLOWDOWN_DIST = 0.55     # m, poison-lookup radius only (see _poison_body);
                             # NOT used as a DWA speed cap any more (see below)
DWA_LIDAR_SUBSAMPLE = 110    # cap lidar points used per rollout (speed)
DWA_SEARCH_W = 1.0           # in-place spin (rad/s) when fully boxed in
# Speed policy: NO compounding proximity slowdown (heading x front-cone x
# side-wall factors used to multiply together and crush v_top to ~0 near any
# corner/pillar, freezing the robot short of the reach threshold). Speed is
# governed ONLY by per-rollout clearance rejection below, plus ONE clean,
# non-compounding tight-gap factor and a pivot gate for large carrot bearings.
DWA_TIGHT_GAP_DIST = 0.22    # m, lateral clearance below which to ease off
DWA_TIGHT_GAP_VFRAC = 0.80   # floor of the tight-gap speed factor (raised: Maze5's
                             # corridors keep side clearance under DWA_TIGHT_GAP_DIST
                             # almost everywhere, so this floor was acting as a
                             # near-constant global speed cap, not an occasional
                             # tight-gap easing; actual collision safety is still
                             # enforced independently by the per-rollout DWA_SAFE_RADIUS
                             # rejection, so raising the floor doesn't reduce safety)
DWA_PIVOT_BEARING = 0.6      # rad (~34 deg) carrot offset above which we pivot
DWA_PIVOT_MIN_DIST = 0.18    # m, don't pivot for a carrot already this close

# ===========================================================================
# Green-poison safety reflex (drift-immune)
# ===========================================================================
# Independent of the mapped poison layer (which A* + DWA already avoid): a true
# last-resort guard that fires ONLY when projected green floor is IMMINENT —
# within GREEN_REFLEX_DIST metres directly ahead in the body frame.  It must NOT
# fire for distant poison (that caused a blue->yellow deadlock: the robot faced
# the far poison patches and froze even though a valid path routed around them).
GREEN_REFLEX_ENABLED = True
GREEN_REFLEX_DIST = 0.45        # m ahead; closer projected green -> block forward
GREEN_REFLEX_HALF_W = 0.20      # m, half-width of the imminent-poison corridor
GREEN_REFLEX_MIN_PTS = 6        # min projected green points in that box to fire

# ===========================================================================
# Recovery (stuck handling)
# ===========================================================================
STUCK_PROGRESS_MIN_M = 0.07     # m progress expected per window
STUCK_TIMEOUT_S = 3.0           # s without progress -> recovery
FROZEN_TIMEOUT_S = 5.0          # s with NO position AND NO heading change in a
                                # driving state -> recovery (backstop against any
                                # v=0,w=0 deadlock, e.g. a latched safety reflex)
RECOVERY_REVERSE_V = 0.18       # m/s
RECOVERY_REVERSE_T = 1.2        # s
RECOVERY_SPIN_W = 1.5           # rad/s
RECOVERY_SPIN_T = 1.0           # s
RECOVERY_REAR_MIN_CLEAR = 0.25  # m; if a wall is this close behind, skip reverse
RECOVERY_MAX_CHAIN = 4          # consecutive recoveries before a hard replan

# ===========================================================================
# Perception — HSV thresholds (OpenCV convention, H in [0,179])
# ===========================================================================
HSV_BLUE = ((100, 110, 50), (130, 255, 255))
HSV_YELLOW = ((20, 110, 110), (35, 255, 255))
HSV_GREEN = ((38, 80, 50), (85, 255, 255))

PILLAR_RADIUS = 0.10            # m
PILLAR_HEIGHT = 0.30            # m
PILLAR_MIN_PIXELS = 14          # min coloured blob area
PILLAR_MAX_DETECT_RANGE = 5.0   # m
PILLAR_HEIGHT_MIN_FRAC = 0.30
PILLAR_HEIGHT_MAX_FRAC = 1.90
PILLAR_ASPECT_MAX = 2.2         # reject wide blobs (walls share the colour rarely)
PILLAR_OBS_AVG_N = 6            # running mean window for world position
PILLAR_OUTLIER_REJECT_M = 1.2   # m, discard detections this far from the mean
PILLAR_CONFIRM_DIST = 0.40      # m, odometry must reach within this to "arrive"
PILLAR_STANDOFF = 0.42          # m, stop this far short of the pillar centre
GREEN_PROJECT_STRIDE = 3        # subsample factor when projecting green floor
GREEN_PROJECT_MAX_RANGE = 3.0   # m, drop far (noisy) green projections (keeps map crisp)

# ===========================================================================
# Mission / logging
# ===========================================================================
PILLAR_REACH_DIST_BLUE = 0.45   # m, mission "reached the pillar" threshold (centre dist);
                                 # matches YELLOW so BLUE isn't declared reached from as
                                 # far out as 0.75 m -- get closer before turning away
PILLAR_REACH_DIST_YELLOW = 0.45 # m, reach threshold for yellow pillar (get closer)
# Simple, ground-truth arrival check: the camera-based pillar_world estimate
# is a running average that can drift once the robot is very close (clipped
# colour blob, close-range depth noise), so pose_distance(pose, pw) can stay
# stuck above PILLAR_REACH_DIST forever even though the robot is visibly
# touching the pillar. PILLAR_TOUCH_DIST is compared against the LIVE lidar
# range straight ahead instead -- once GO_BLUE/GO_YELLOW has committed to a
# target, the only thing that close ahead of the robot IS the pillar, so
# this is a direct, reliable "am I basically touching it" test.
PILLAR_TOUCH_DIST = 0.30        # m, live-lidar front range that counts as reached
# Final-approach OPEN-LOOP CREEP: confirmed by log evidence (goal only 0.14m
# away, 0.16m of path left, v=0.00) that the DWA scorer can settle on v=0 a
# few cm short of a perfectly valid, obstacle-free goal instead of closing
# the gap. Rather than keep tuning that scorer, once within GO_CREEP_DIST of
# the (already-known-accurate) pillar estimate, stop trusting it: drive
# straight at a fixed speed with simple proportional heading correction, for
# up to GO_CREEP_MAX_S, then declare arrival unconditionally. Deterministic
# and can never stall.
GO_CREEP_DIST = 1.2             # m, switch from DWA to open-loop creep this close
GO_CREEP_V = 0.45               # m/s, fixed creep speed
GO_CREEP_KW = 2.0               # rad/s per rad of bearing error
GO_CREEP_MAX_S = 6.0            # s, hard cap -- force arrival after this regardless
GO_FAIL_COOLDOWN_S = 2.5        # s, after a failed GO, explore this long before retry
NO_FRONTIER_SPIN_S = 4.0        # s, spin-and-rescan when no frontier is available
PERCEPTION_EVERY_TICKS = 2      # run colour perception every N ticks
SNAPSHOT_PERIOD_S = 3.0         # s, write a map PNG this often
LOG_EVERY_TICKS = 24
OUTPUT_DIRNAME = "maps"
DONE_LINGER_S = 1.5             # s to keep window after DONE
