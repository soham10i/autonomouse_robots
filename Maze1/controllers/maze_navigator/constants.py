"""All tunable parameters for the maze navigator."""

import math

# --- Simulation ---
TIME_STEP = 32  # ms, must match world basicTimeStep

# --- Motor ---
MAX_VELOCITY = 26.0       # rad/s, ROSbot motor max
BASE_SPEED = 8.0          # rad/s, cruising speed during path following
EXPLORE_SPEED = 5.0       # rad/s, slower during exploration
TURN_SPEED = 3.0          # rad/s, speed for in-place rotation

# --- Robot physical ---
WHEEL_RADIUS = 0.043      # m
WHEEL_BASE = 0.20         # m, distance between left and right wheels
ROBOT_RADIUS = 0.15       # m, for obstacle inflation (conservative)

# --- Occupancy grid ---
GRID_RESOLUTION = 0.05    # m per cell (5cm)
GRID_WIDTH = 200          # cells (10m coverage)
GRID_HEIGHT = 200         # cells (10m coverage)

# Cell values
CELL_UNKNOWN = 0
CELL_FREE = 1
CELL_OCCUPIED = 2
CELL_POISON = 3
CELL_INFLATED = 4

# Log-odds parameters
LOG_ODDS_PRIOR = 0.0
LOG_ODDS_OCC = 0.7        # increment for occupied observation
LOG_ODDS_FREE = -0.3      # increment for free observation
LOG_ODDS_MAX = 5.0
LOG_ODDS_MIN = -5.0
LOG_ODDS_OCC_THRESH = 0.5 # above this -> occupied
LOG_ODDS_FREE_THRESH = -0.5  # below this -> free

# Inflation radius in cells
INFLATION_RADIUS = int(math.ceil(ROBOT_RADIUS / GRID_RESOLUTION))  # 3 cells

# --- LiDAR ---
LIDAR_MAX_RANGE = 8.0     # m, ignore readings beyond this
LIDAR_MIN_RANGE = 0.15    # m, ignore readings below this

# --- Camera ---
CAMERA_INTERVAL = 3       # process camera every N timesteps

# HSV ranges (OpenCV-style: H 0-180, S 0-255, V 0-255)
# Blue pillar
BLUE_H_MIN, BLUE_H_MAX = 100, 135
BLUE_S_MIN = 80
BLUE_V_MIN = 40

# Yellow pillar
YELLOW_H_MIN, YELLOW_H_MAX = 18, 45
YELLOW_S_MIN = 80
YELLOW_V_MIN = 80

# Green ground (poison)
GREEN_H_MIN, GREEN_H_MAX = 35, 85
GREEN_S_MIN = 50
GREEN_V_MIN = 30
GREEN_PIXEL_THRESHOLD = 0.10  # fraction of bottom-third pixels

# Wood/brown (floating walls)
WOOD_H_MIN, WOOD_H_MAX = 8, 30
WOOD_S_MIN = 30
WOOD_V_MIN = 40
WOOD_V_MAX = 220

# Pillar detection
MIN_PILLAR_AREA = 50          # minimum pixel area to count as pillar
PILLAR_APPROACH_DIST = 0.25   # m, switch to approach mode
GOAL_REACHED_DIST = 0.18      # m, declare pillar reached

# --- Path following (PD controller) ---
KP_FOLLOW = 10.0
KD_FOLLOW = 2.0
WAYPOINT_REACHED_DIST = 0.10  # m, move to next waypoint
SPEED_TURN_FACTOR = 0.4       # slow down when heading error is large

# --- Exploration ---
FRONTIER_MIN_SIZE = 3         # minimum cells in a frontier cluster
EXPLORE_TIMEOUT = 600         # seconds, give up exploring after this
INITIAL_SCAN_ROTATIONS = 1    # full rotations during initial scan

# --- Safety ---
EMERGENCY_STOP_DIST = 0.15   # m, stop immediately if obstacle this close
BACKUP_DIST = 0.15            # m, distance to reverse when avoiding
BACKUP_SPEED = -3.0           # rad/s, reverse speed
