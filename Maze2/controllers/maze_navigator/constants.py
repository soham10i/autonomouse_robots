"""All tunable parameters for the maze navigator — single source of truth.

No magic numbers in other modules. Every constant is documented with:
  - what it controls
  - default value rationale
  - when you might want to change it

Reference: ROSbot specs from Cyberbotics
  https://www.cyberbotics.com/doc/guide/rosbot
"""

# =============================================================================
# 1. SIMULATION
# =============================================================================
TIME_STEP = 32          # ms — Webots basicTimeStep (must match world file)

# =============================================================================
# 2. ROBOT PHYSICAL PARAMETERS
# =============================================================================
WHEEL_RADIUS = 0.0425   # m — ROSbot wheel radius
TRACK_WIDTH  = 0.20     # m — distance between left and right wheels
ROBOT_RADIUS = 0.15     # m — conservative bounding circle for C-space inflation
                         #     (actual half-width ≈ 0.10m, but 0.15m adds safety margin)

# =============================================================================
# 3. MOTOR CONTROL
# =============================================================================
MAX_VELOCITY   = 26.0   # rad/s — absolute motor limit (from Webots proto)
BASE_SPEED     = 8.0    # rad/s — default forward speed during navigation
EXPLORE_SPEED  = 6.0    # rad/s — slower speed during exploration (safer)
TURN_SPEED     = 4.0    # rad/s — speed during in-place rotation
BACKUP_SPEED   = -3.0   # rad/s — reverse speed for emergency backup
BACKUP_DIST    = 0.15   # m    — how far to reverse during emergency backup

# PD path-following controller gains
KP_HEADING = 10.0       # proportional gain for heading correction
KD_HEADING =  2.0       # derivative gain (dampens oscillation)
SPEED_TURN_FACTOR = 0.4 # slow down to this fraction during sharp turns

# =============================================================================
# 4. SENSOR CONFIGURATION
# =============================================================================
# LiDAR
LIDAR_MIN_RANGE  = 0.10  # m — ignore readings closer (robot body reflections)
LIDAR_MAX_RANGE  = 8.0   # m — max reliable range of RPLidar A2
LIDAR_NUM_POINTS = 360   # expected points per scan (may differ in Webots)

# Camera processing interval
CAMERA_INTERVAL  = 8     # process camera every Nth timestep (saves CPU)

# Distance sensors (IR range finders)
EMERGENCY_STOP_DIST = 0.15  # m — hard stop if any sensor reads closer

# =============================================================================
# 5. OCCUPANCY GRID
# =============================================================================
GRID_SIZE       = 200    # cells per side → 200 × 200 grid
GRID_RESOLUTION = 0.05   # m/cell → 200 × 0.05 = 10m total coverage
GRID_ORIGIN     = 100    # cell index of world origin (0,0) → center of grid

# Log-odds parameters (Elfes 1989, Thrun ch.9)
LOG_ODDS_PRIOR  =  0.0   # initial belief: unknown (p = 0.5)
LOG_ODDS_OCC    =  0.7   # increment when a ray hits (strong evidence of wall)
LOG_ODDS_FREE   = -0.3   # decrement along ray path (weaker evidence of free)
LOG_ODDS_MAX    =  3.5   # clamp upper bound (prevents over-confidence)
LOG_ODDS_MIN    = -2.0   # clamp lower bound
OCC_THRESHOLD   =  0.5   # log-odds > this → cell classified as OCCUPIED
FREE_THRESHOLD  = -0.5   # log-odds < this → cell classified as FREE

# Inflation for path planning (C-space expansion)
# inflation_cells = ceil(ROBOT_RADIUS / GRID_RESOLUTION) = ceil(0.15/0.05) = 3
INFLATION_RADIUS_CELLS = 3

# Minimum new occupied cells before rebuilding inflation layer
INFLATION_REBUILD_THRESHOLD = 5

# Cell state constants (for readability)
CELL_UNKNOWN  = 0
CELL_FREE     = 1
CELL_OCCUPIED = 2

# =============================================================================
# 6. VISION / HSV THRESHOLDS
# =============================================================================
# Blue pillar: baseColor (0, 0, 1) in Webots → bright blue in HSV
BLUE_H_MIN, BLUE_H_MAX = 100, 135
BLUE_S_MIN = 100
BLUE_V_MIN =  80

# Yellow pillar: baseColor (1, 1, 0) → bright yellow
YELLOW_H_MIN, YELLOW_H_MAX = 18, 45
YELLOW_S_MIN = 100
YELLOW_V_MIN = 100

# Green poison ground: baseColor (0, 1, 0) → bright green
GREEN_H_MIN, GREEN_H_MAX = 35, 85
GREEN_S_MIN =  80
GREEN_V_MIN =  80

# Wood floating walls: RoughPine texture → brown/tan
WOOD_H_MIN, WOOD_H_MAX = 8, 30
WOOD_S_MIN =  40
WOOD_V_MIN =  60

# Minimum blob area (pixels) to count as a valid detection
MIN_PILLAR_AREA = 50
MIN_POISON_AREA = 200

# =============================================================================
# 7. EXPLORATION
# =============================================================================
FRONTIER_SIZE_BONUS = 0.5   # weight for cluster size in frontier scoring
MAX_FRONTIER_ATTEMPTS = 3   # blacklist frontier after this many failures
FRONTIER_STUCK_TIMEOUT = 8.0  # seconds — trigger recovery if no progress

# =============================================================================
# 8. NAVIGATION / GOAL REACHING
# =============================================================================
GOAL_REACHED_DIST    = 0.25  # m — pillar considered "reached" at this distance
PILLAR_APPROACH_DIST = 0.40  # m — slow down when this close to pillar
WAYPOINT_REACHED_DIST = 0.10 # m — advance to next waypoint at this distance

# =============================================================================
# 9. INITIAL SCAN
# =============================================================================
INITIAL_SCAN_ROTATIONS = 1.1  # full rotations during initial 360° scan
                               # slightly > 1.0 to ensure complete coverage

# =============================================================================
# 10. FSM PLAN FAILURE
# =============================================================================
PLAN_FAIL_COOLDOWN = 30  # timesteps to explore before retrying failed plan
