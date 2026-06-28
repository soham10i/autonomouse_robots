"""Single source of truth for tunable constants and Webots device names.

Anything that might need tuning per-map / per-robot lives here. The other
modules import from this file and never hard-code geometry or device names.
"""

# ----------------------------- robot geometry ----------------------------
ROBOT_WHEEL_RADIUS = 0.043
ROBOT_WHEEL_SEPARATION = 0.220
ROBOT_CHASSIS_HALF_WIDTH = 0.100
ROBOT_CHASSIS_HALF_LENGTH = 0.075
ROBOT_RADIUS = 0.125            # circumscribed footprint radius (m)
ROBOT_HEIGHT = 0.22
LIDAR_MOUNT_Z = 0.10
CAMERA_MOUNT_Z = 0.165

# ----------------------------- device names ------------------------------
MOTOR_NAMES = ("fl_wheel_joint", "fr_wheel_joint", "rl_wheel_joint", "rr_wheel_joint")
ENCODER_NAMES = (
    "front left wheel motor sensor",
    "front right wheel motor sensor",
    "rear left wheel motor sensor",
    "rear right wheel motor sensor",
)
LIDAR_NAME = "laser"
CAMERA_NAME = "camera rgb"
DEPTH_NAME_CANDIDATES = ("camera depth", "range-finder", "camera range-finder", "depth")
IMU_NAME_CANDIDATES = ("imu inertial_unit", "imu inertial unit", "inertial unit", "imu")
GYRO_NAME_CANDIDATES = ("imu gyro", "gyro")
ACCEL_NAME_CANDIDATES = ("imu accelerometer", "accelerometer")
COMPASS_NAME_CANDIDATES = ("imu compass", "compass")
IR_RANGE_NAMES = ("fl_range", "fr_range", "rl_range", "rr_range")

# ----------------------------- speed limits ------------------------------
V_MAX = 0.55                          # nominal top forward speed
V_MIN_FORWARD = 0.05
W_MAX = 2.2
WHEEL_ANG_MAX = V_MAX / ROBOT_WHEEL_RADIUS * 1.4

# ----------------------------- aux obstacle layer ------------------------
# The 2D lidar at LIDAR_MOUNT_Z = 0.10 m can NOT see floating walls whose
# top edge is below the lidar plane (the lidar rays pass over them). We
# therefore project depth-cam pixels in this height band into the aux
# obstacle layer. The lower bound has to be > 0 so floor pixels (z_r ≈ 0)
# are not flagged as obstacles.
#
# Maze1 has three kinds of floating walls:
#   1. Ground walls (bottom ≤ 0)        → always obstacle (lidar sees them)
#   2. Low-float  (0 < bottom < ROBOT_HEIGHT)  → obstacle (robot can't fit)
#   3. High-float (bottom ≥ ROBOT_HEIGHT)      → PASSAGE (robot drives under)
#
# Z_MAX must equal ROBOT_HEIGHT exactly so high-float passages are NOT
# flagged. Example: WallMedium(5) bottom=0.25 m > 0.22 m → passage.
# Previous value (ROBOT_HEIGHT + 0.05 = 0.27 m) incorrectly blocked it.
AUX_OBSTACLE_Z_MIN = 0.04             # m above floor — exclude floor noise
AUX_OBSTACLE_Z_MAX = ROBOT_HEIGHT      # m — only flag walls the bot hits
AUX_OBSTACLE_MAX_RANGE = 2.0          # m — depth pixels farther than this are dropped

# ----------------------------- 2D grid -----------------------------------
GRID_RESOLUTION = 0.04
GRID_SIZE_M = 8.0
GRID_CELLS = int(GRID_SIZE_M / GRID_RESOLUTION)
GRID_ORIGIN = (-4.0, -4.0)

# log-odds parameters
L_FREE = -0.4
L_OCC = 0.85
L_MIN = -2.0
L_MAX = 3.5
L_THRESH_OCC = 0.85
L_THRESH_FREE = -0.4

# ----------------------------- inflation / costmap -----------------------
# We keep TWO thresholds to allow the planner through narrow corridors that
# nominal "robot radius + safety" would mark as lethal.
#
#   HARD_OBS_MARGIN  : if distance from any obstacle is BELOW
#                       (ROBOT_RADIUS - HARD_OBS_MARGIN), the cell is lethal.
#                       Set very tight so passages slightly wider than the
#                       robot are usable.
#   SOFT_OBS_HALO    : extra distance above the hard threshold over which we
#                       apply a soft (finite) cost — A* prefers middle of
#                       free space but can still squeeze through.
#
# Poison gets stricter thresholds because crossing it is mission-failure.
HARD_OBS_MARGIN = 0.02                    # robot can pass within (radius-0.02)
SOFT_OBS_HALO = 0.12                      # soft halo width (m)
HARD_POISON_MARGIN = -0.02                # extra strictness vs robot radius
SOFT_POISON_HALO = 0.20                   # poison soft halo (m)
INFLATION_RADIUS = ROBOT_RADIUS + 0.04    # legacy (used by frontier filtering)
INFLATION_ALPHA = 3.0
ASTAR_INFLATION_WEIGHT = 6.0
ASTAR_POISON_INFLATION_WEIGHT = 30.0
POISON_INFLATION_RADIUS = ROBOT_RADIUS + 0.10
POISON_DETECT_RADIUS = 0.30               # half-width of the marked poison disc
MIN_CORRIDOR_WIDTH = 2 * ROBOT_RADIUS + 0.04  # below this corridor is rejected

# ----------------------------- HSV thresholds ----------------------------
HSV_BLUE = ((100, 120, 60), (130, 255, 255))
HSV_YELLOW = ((20, 120, 120), (35, 255, 255))
HSV_GREEN = ((40, 100, 60), (80, 255, 255))

# ----------------------------- pillar handling ---------------------------
PILLAR_RADIUS = 0.10
PILLAR_HEIGHT = 0.30                  # cylinder height (m) from Maze1.wbt
# Accept a detection if its estimated real-world height is within this range
# of PILLAR_HEIGHT.  Min 30% allows half-visible / occluded pillars;
# Max 180% rejects tall wall segments that happen to match the colour.
PILLAR_HEIGHT_MIN_FRAC = 0.30         # est_height >= 30% of PILLAR_HEIGHT
PILLAR_HEIGHT_MAX_FRAC = 1.80         # est_height <= 180% of PILLAR_HEIGHT
# Aspect ratio (width/height) of the blob in pixel space.
# A pillar (diam 0.2m, height 0.3m) has aspect ≈ 0.67.
# A wall segment is very wide → aspect >> 2.
PILLAR_ASPECT_MAX = 2.5               # reject blobs wider than this ratio
PILLAR_STANDOFF = 0.45                # target distance from pillar centre (m)
PILLAR_REACH_TOL = 0.55               # arrived if dist to pillar < this and seen
PILLAR_BEARING_TOL = 0.50             # |bearing| accepted for visual confirmation
PILLAR_AREA_FRAC = 0.18               # area-frac in image counts as "very close"
# Reduced from 60 to 15 pixels to catch the pillar even when it is very small/far
PILLAR_DETECT_MIN_PIXELS = 15
PILLAR_OBS_AVG_N = 8                  # smoothing window for pillar world-pos

# Max range at which we trust a depth-based pillar detection.
# Increased to 4.5m to allow detecting the blue pillar from further away.
PILLAR_MAX_DETECT_RANGE = 4.5

# Outlier rejection threshold for the pillar observation buffer (m).
# Increased to 2.0m to be more forgiving of odometry drift or noise.
PILLAR_OUTLIER_REJECT_M = 2.0

# Minimum number of accepted observations before we allow the confirmation
# distance check to fire. Reduced to 1 to confirm pillars instantly on first sight!
PILLAR_MIN_OBS_FOR_CONFIRM = 1

# Phase-2 "the bot actually got there" threshold.
# A pillar's tentative position (from the camera) is upgraded to *confirmed*
# only when the robot's own odometry has come within this distance of it.
# Floor: pillar_radius (0.10) + robot_radius (0.125) = 0.225 m geometric
# minimum, so this must be ≥ 0.225 m.
PILLAR_CONFIRM_DIST = 0.35

# Extra time we let exploration run AFTER frontier saturation when at least
# one pillar still hasn't been *seen* at all — gives a grace period for the
# bot to find the missing pillar before giving up.
EXPL_GRACE_AFTER_SATURATION_S = 15.0

# ----------------------------- planning ----------------------------------
ASTAR_DIAGONAL = True
ASTAR_REPLAN_EVERY_S = 1.5            # replanning cadence on stale paths
ASTAR_MAX_NODES = 80000               # safety cap
PILLAR_REPLAN_MOVE_M = 0.15           # only re-plan when target moves > this

# ----------------------------- path follower -----------------------------
LOOKAHEAD_BASE = 0.30
LOOKAHEAD_K_V = 0.40
LOOKAHEAD_MIN = 0.18
LOOKAHEAD_MAX = 0.70
WAYPOINT_REACH_TOL = 0.18

PF_K_HEADING = 3.0
PF_HEADING_DEADBAND = 0.06
PF_BIG_HEADING_STOP = 0.55            # above this heading error, halt v
PF_FWD_BRAKE_DIST = 0.28              # brake to 0 v if any source < this
PF_FWD_SLOW_DIST = 0.55               # progressively slow inside this band
PF_NARROW_SCALE_DIST = 0.22           # if either side < this, scale v down

# Sharp-corner stop-and-pivot (Layer B). Pure pursuit rounds off sharp path
# vertices, swinging the inside chassis corner toward the inner wall. At a
# vertex whose turn exceeds PF_PIVOT_TURN_DEG the follower instead drives up
# to the vertex, stops, pivots IN PLACE to the outgoing-segment heading,
# then resumes — exploiting the skid-steer's zero turning radius so the
# rectangle never sweeps into the corner.
PF_PIVOT_TURN_DEG = 55.0              # vertex turn sharper than this → pivot
PF_PIVOT_TRIGGER_DIST = 0.22          # begin pivot within this of the vertex (m)
PF_PIVOT_DONE_DEG = 8.0               # pivot complete when heading within this
PF_PIVOT_W = 1.2                      # pivot angular-speed cap (rad/s)
PF_PIVOT_W_MIN = 0.30                 # minimum pivot spin (rad/s) — no crawl

# ----------------------------- recovery ----------------------------------
STUCK_PROGRESS_MIN_M = 0.10
STUCK_TIMEOUT_S = 4.0
STUCK_OSC_W_THRESHOLD = 1.6           # rad/s repeated above this = oscillation
RECOVERY_REVERSE_M = 0.20
# Speed × time = nominal reverse distance. Was 0.12 m/s × 0.9 s = 10.8 cm,
# which wasn't enough to clear a wedged-into-a-corner pose. Bumped to
# 0.20 × 1.4 = 28 cm so the bot physically separates from the wall before
# the spin phase tries to rotate.
RECOVERY_REVERSE_V = 0.20             # m/s during the REVERSE phase
RECOVERY_REVERSE_T = 1.4
RECOVERY_SPIN_T = 1.1
RECOVERY_SPIN_W = 1.4
RECOVERY_MAX_CHAIN = 3                # consecutive recoveries before global replan

# ----------------------------- exploration -------------------------------
FRONTIER_MIN_CLUSTER = 6
FRONTIER_MIN_DIST = 0.40              # ignore frontiers nearer than this (m)
FRONTIER_BLACKLIST_RADIUS = 0.45
INITIAL_SCAN_REVS = 1.05              # how many full revs at startup

# Phase-2 exploration is bounded to a circle around the start pose so a stray
# opening in the maze boundary doesn't let the bot wander into the void
# outside the maze. The arena is ~5x5 m; this gives ~0.5 m of slack.
EXPL_MAX_RADIUS_FROM_START_M = 4.0

# Path-follower corridor-centering. When both side clearances are within
# PF_CENTER_TRIGGER_DIST and the bot's heading error is small, we add a
# small w correction that nudges the bot toward the centerline of the
# corridor. Stops the zigzag pattern in tight passages.
PF_CENTER_GAIN = 1.6                  # rad/s per metre of side asymmetry
PF_CENTER_TRIGGER_DIST = 0.55         # only apply when both sides closer
PF_CENTER_HEADING_LIMIT = 0.30        # only apply when |heading err| <
PF_CENTER_MAX_ASYM = 0.40             # ignore extreme asymmetry (junctions)

# ----------------------------- logging / dumps ---------------------------
LOG_EVERY_TICKS = 32
PNG_DUMP_PERIOD_S = 4.0
DEBUG_DUMP_PERIOD_S = 6.0
SCAN_MATCH_EVERY = 0                  # 0 = disabled (kept for future use)

# ----------------------------- DWA local planner -------------------------
# Calibrated from Maze1.wbt geometry:
#   Arena ~4.6×4.8 m — corridors formed by 0.50 m and 1.0 m wall segments,
#   each 0.05 m thick.  Narrowest passages: ≈ 0.45 m.
#   Robot width 0.25 m → only ≈ 0.20 m margin per side.
#   Blue  pillar @ (1.28, 0.83) — inside a 4-wall cluster.
#   Yellow pillar @ (-0.03, 0.31) — 0.46 m from the poison patch.
#
# Weight rationale (α + β + γ ≈ 1):
#   β (clearance) is raised to 0.35 because corridors are very tight and the
#   poison patch sits very close to the Yellow pillar approach path.
#   α (heading) stays at 0.55 so the planner still commits to the goal.
#   γ (velocity) at 0.10 prevents the planner from camping in place.
DWA_ALPHA            = 0.55    # heading alignment weight   (paper: 0.8)
DWA_BETA             = 0.35    # clearance weight           (paper: 0.1)
DWA_GAMMA            = 0.10    # velocity weight            (paper: 0.1)

# Control cycle ≈ 4 × 32 ms timestep ≈ 128 ms (faster than paper's 250 ms
# for better responsiveness in tight maze corridors).
DWA_CALL_EVERY_N_TICKS = 4

# Trajectory evaluation
DWA_DT_S             = 0.128   # one control step duration (s)  (4 × 32 ms)
DWA_HORIZON_S        = 1.2     # scoring rollout horizon (s)
DWA_V_SAMPLES        = 9       # sample count for v ∈ [v_lo, v_hi]
DWA_W_SAMPLES        = 21      # sample count for ω (odd → includes 0)

# Dynamic Window (V_d) — angular accel raised from 1.80 to 3.50 so the robot
# can commit to a sharp turn in ~0.6 s instead of ~1.2 s.
DWA_V_ACCEL          = 0.65    # translational accel limit (m/s²)
DWA_W_ACCEL          = 3.50    # rotational accel limit (rad/s²)

# Admissible Velocities (V_a) — braking decelerations.
DWA_V_BRAKE          = 0.65    # translational braking decel (m/s²)
DWA_W_BRAKE          = 3.50    # rotational braking decel (rad/s²)

# Safety floor — trajectory rejected if any point comes closer than this.
DWA_MIN_CLEAR_M      = 0.13    # m  (= ROBOT_RADIUS - 0.005 — tight but safe)
