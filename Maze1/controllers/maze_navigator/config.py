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
V_MAX = 0.40
V_MIN_FORWARD = 0.05
W_MAX = 2.2
WHEEL_ANG_MAX = V_MAX / ROBOT_WHEEL_RADIUS * 1.4

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
PILLAR_STANDOFF = 0.45                # target distance from pillar centre (m)
PILLAR_REACH_TOL = 0.55               # arrived if dist to pillar < this and seen
PILLAR_BEARING_TOL = 0.50             # |bearing| accepted for visual confirmation
PILLAR_AREA_FRAC = 0.18               # area-frac in image counts as "very close"
PILLAR_DETECT_MIN_PIXELS = 60
PILLAR_OBS_AVG_N = 5                  # smoothing window for pillar world-pos

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

PF_K_HEADING = 2.4
PF_HEADING_DEADBAND = 0.06
PF_BIG_HEADING_STOP = 0.55            # above this heading error, halt v
PF_FWD_BRAKE_DIST = 0.30              # brake to 0 v if any source < this
PF_FWD_SLOW_DIST = 0.65               # progressively slow inside this band
PF_NARROW_SCALE_DIST = 0.32           # if either side < this, scale v down

# ----------------------------- recovery ----------------------------------
STUCK_PROGRESS_MIN_M = 0.10
STUCK_TIMEOUT_S = 4.0
STUCK_OSC_W_THRESHOLD = 1.6           # rad/s repeated above this = oscillation
RECOVERY_REVERSE_M = 0.20
RECOVERY_REVERSE_T = 0.9
RECOVERY_SPIN_T = 1.1
RECOVERY_SPIN_W = 1.4
RECOVERY_MAX_CHAIN = 3                # consecutive recoveries before global replan

# ----------------------------- exploration -------------------------------
FRONTIER_MIN_CLUSTER = 6
FRONTIER_MIN_DIST = 0.40              # ignore frontiers nearer than this (m)
FRONTIER_BLACKLIST_RADIUS = 0.45
INITIAL_SCAN_REVS = 1.05              # how many full revs at startup

# ----------------------------- logging / dumps ---------------------------
LOG_EVERY_TICKS = 32
PNG_DUMP_PERIOD_S = 4.0
DEBUG_DUMP_PERIOD_S = 6.0
SCAN_MATCH_EVERY = 0                  # 0 = disabled (kept for future use)
