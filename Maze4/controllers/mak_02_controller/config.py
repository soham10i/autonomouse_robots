"""Single source of truth for the Maze4 ROSbot controller.

No Webots import here so every algorithmic module that depends on it stays
importable / testable outside the simulator.  Hardware-facing code lives in
:mod:`robot_io`.

Maze4 facts that drove the tuning (read directly from ``Maze4/worlds/Maze4.wbt``,
verified with exact axis-angle rotation maths, not estimates):
  * Robot start  ~ (-2.228, 2.795), yaw 150 deg.
  * Blue pillar  ~ ( 0.44,  0.04)   (reach this first;  ~3.83 m from start)
  * Yellow pillar~ (-2.73,  3.13)   (reach this second; ~0.60 m from start;
    time stops here)
  * Cylinder pillars: explicit height 0.3 m, z=0.15 (fixed from the original
    buried-at-z=-0.6 default-height-2.0 m which created phantom obstacles in
    the lidar/costmap).  PILLAR_HEIGHT = 0.30 m.
  * One green "poison" floor patch (vs. two in Maze5) -- otherwise identical
    flat floor decal, no special handling needed.
  * 45 standing walls (18 WallShort + 27 WallMedium) are the normal full-height
    type: boxes 1.0/0.5 x 0.05 x 0.5 centred at z=0.18 -> span z in [-0.07,0.43],
    crossing the 2-D lidar plane (z=0.10) normally, exactly like Maze5.
  * UNLIKE Maze5: Maze4 has 4 tilted wall panels whose exact world z-span was
    computed via Rodrigues rotation of their box corners (NOT pass-under
    bridges -- confirmed against the world file; a genuine pass-under bridge,
    WallMedium(32) at z in [0.365,0.415], exists in Maze3, not here):
        WallShort(15)  z in [0.015, 0.065]  -- fully BELOW the lidar plane
        WallMedium(14) z in [0.085, 0.135]  -- straddles the lidar plane
        WallShort(16)  z in [0.115, 0.165]  -- fully ABOVE the lidar plane
        WallShort(18)  z in [0.155, 0.205]  -- fully ABOVE the lidar plane
    All 4 sit entirely inside the robot's body band [0, ROBOT_HEIGHT=0.22] so
    none are drivable-under; the 2-D lidar partially or fully misses them
    (WallShort(15) is invisible to it).  These are real, low, lidar-blind
    collision hazards -> Maze4 needs the depth-camera "aux" layer that Maze5
    deliberately omitted, PLUS the chassis-mounted IR sensors as a close-range
    backstop for WallShort(15) (~5 cm, below where the depth camera reliably
    resolves at close range).
"""
from __future__ import annotations

import math

# ===========================================================================
# Robot geometry (Husarion ROSbot, cyberbotics Rosbot R2025a proto)
# ===========================================================================
# Exact dimensions read from the Rosbot proto boundingObjects:
#   chassis box 0.20 (x) x 0.15 (y); wheels: anchors y=+/-0.110, wheel cylinder
#   height 0.035 (along y) offset -0.0115 -> OUTER wheel edge at y = +/-0.116 m.
# So the limiting dimensions are:
#   * passage half-width (straight threading)  = 0.116 m  (robot is 0.232 m wide)
#   * circumscribed radius (in-place rotation) = 0.128 m  (rear-wheel corner)
# Ground-truth Maze4 analysis: the tightest gaps the robot must thread are
# 0.388 m (to BLUE) and 0.418 m (to YELLOW) -> only ~0.07-0.09 m slack PER SIDE.
# Success therefore depends on riding the passage centreline; the costmap below
# is center-seeking and the DWA strongly centres on the live lidar.
WHEEL_RADIUS = 0.043            # m
WHEEL_SEPARATION = 0.220        # m, track width (wheel anchors at y=+/-0.110)
ROBOT_HALF_WIDTH = 0.116        # m, true outer half-width (wheels) -> straight-gap limit
ROBOT_RADIUS = 0.128            # m, circumscribed footprint radius (rotation clearance)
ROBOT_HEIGHT = 0.22             # m
LIDAR_MOUNT_Z = 0.10            # m, 2-D lidar plane (lidar at x=+0.02 on the proto)
CAMERA_MOUNT_Z = 0.165          # m, optical centre height (camera looks ~horizontal)

# ===========================================================================
# Webots device names (validated on real Maze1/Maze5 runs of the same proto)
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
# Chassis-mounted IR proximity sensors (~5-9 cm above the floor on the proto) --
# NEW for Maze4: a close-range backstop for the low floating panels above.
IR_RANGE_NAMES = ("fl_range", "fr_range", "rl_range", "rr_range")

# ===========================================================================
# Velocity limits  (precise: brisk on open straights, controlled in corridors)
# ===========================================================================
# Tuned for "precise speed": the DWA below no longer throttles v with
# compounding proximity caps (that crawl bug is gone), so V_MAX is the genuine
# top speed the planner runs at whenever the path ahead is clear.  Kept moderate
# (not 0.45) because Maze4's gaps are as narrow as MIN_GAP_WIDTH (~0.28 m) and a
# tighter top speed lets the per-rollout clearance rejection thread them cleanly
# instead of overshooting.  Easy to raise once a clean Webots run confirms it.
V_MAX = 0.35                    # m/s nominal forward cap on open straights
V_CRUISE = 0.28                 # m/s typical corridor / approach speed
V_MIN = 0.05                    # m/s
W_MAX = 2.2                     # rad/s
A_V = 0.9                       # m/s^2 translational accel cap (smoothness)
A_W = 6.0                       # rad/s^2 angular accel cap (snappy but smooth corners)
WHEEL_ANG_MAX = V_MAX / WHEEL_RADIUS * 1.6   # rad/s saturation per wheel

# ===========================================================================
# Occupancy grid  (lidar + depth-aux, log-odds for lidar / boolean for aux)
# ===========================================================================
GRID_RESOLUTION = 0.04          # m / cell (was 0.05; finer so the lethal-inflation
                                # rounding matches the 0.116 m footprint instead of
                                # under-cutting it, while a 0.39 m gap stays plannable)
GRID_SIZE_M = 11.0              # square side, comfortably covers the arena
GRID_CELLS = int(round(GRID_SIZE_M / GRID_RESOLUTION))   # 275
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
# Depth-camera auxiliary obstacle layer (floating/low walls the lidar misses)
# ===========================================================================
# NEW for Maze4.  A depth pixel counts as a low-obstacle hit only if its height
# is inside this band: above floor noise, at or below chassis top.  Pixels
# higher than ROBOT_HEIGHT are passages the robot could drive under (none exist
# in Maze4 today, but the classifier stays generically correct for any future
# map that does have one).
AUX_Z_MIN = 0.01                # m above floor -- lowered to catch WallShort(15) at z=0.015
AUX_Z_MAX = ROBOT_HEIGHT         # m (== 0.22); higher pixels are passable
AUX_MAX_RANGE = 1.0              # m; drop far/noisy depth pixels (tighter = fewer false aux marks)
AUX_CLEAR_MARGIN = 0.05          # m; stop raytrace-clearing this short of a hit
AUX_COL_SUBSAMPLE = 120          # cap depth-image columns rasterised per tick (speed)
# Keep NO sticky-hit counters and NO decay (those were Maze1's "purple
# thickening" source) -- but the depth layer MUST still be reconciled against
# the lidar.  The depth camera sees the chassis band of NORMAL full-height walls
# too, so stamping every depth hit duplicates the ENTIRE lidar wall map into the
# aux layer (observed: aux grew to ~920 cells, == the whole occ map), thickening
# every 0.39 m passage into a wall and boxing the robot in -- the depth's 0.6 m
# MIN RANGE then can't re-clear a wall the robot threads at ~0.19 m, so it never
# recovers.  Fix (ported from Maze1's _local_obstacles_body): a depth hit becomes
# an aux obstacle ONLY where the 2-D lidar is genuinely blind -- no lidar-occupied
# cell within AUX_LIDAR_BLIND_CELLS.  That keeps the real low panels (WallShort
# 15/16/18) while dropping the duplicate normal-wall marks.
AUX_LIDAR_BLIND_CELLS = 4        # skip a depth hit within this many cells of a lidar wall

# ===========================================================================
# Costmap / inflation (planning)
# ===========================================================================
# Cells within HARD_OBS_DIST of an obstacle are lethal.  Beyond that, the cost
# is CENTER-SEEKING: it decreases smoothly with clearance out to
# CENTER_PREF_RANGE, so A* rides the medial axis (maximum clearance) of every
# corridor instead of merely avoiding lethal cells and then being straightened
# back against a wall.  This is the "align to the absolute centre" behaviour.
#
# HARD_OBS_DIST now MATCHES the true passage half-width (0.116) instead of
# under-cutting it (the old 0.105 planned the wheels 0.011 m INTO the wall).  At
# the 0.04 m grid this rounds to 3 cells = 0.12 m, so a lethal-boundary cell
# still clears the wheels; the 0.388/0.418 m gaps keep a multi-cell free band
# down their centre.  Poison stays stricter (crossing it = mission fail).
HARD_OBS_DIST = ROBOT_HALF_WIDTH + 0.004  # m -> lethal (~0.12); matches the footprint
CENTER_PREF_RANGE = 0.30                  # m; center-seeking cost reaches this far
COST_OBS_WEIGHT = 10.0                    # strength of the centre pull
SIMPLIFY_MAX_COST = 3.5                   # only line-of-sight-shortcut the A* path
                                          # through cells at least this open (~>0.24 m
                                          # clearance); tight gaps keep their dense
                                          # centred path so the carrot tracks centre
# Due to camera projection drift, the poison appears closer to walls in the map 
# than in reality. We drastically reduce the poison threshold to let the robot 
# squeeze through these artificially blocked gaps without reversing.
POISON_HARD_DIST = 0.06                  # m, rounds to 2 cells (was 0.12m/3 cells)
POISON_SOFT_HALO = 0.08                  # m, reduced to prevent choking tight gaps
COST_POISON_WEIGHT = 35.0               # A* can route through costly-but-not-lethal zones near poison
MIN_GAP_WIDTH = 2 * ROBOT_HALF_WIDTH + 0.03  # ~0.262; gaps narrower than this are impassable

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
                                # (blue pillar ~3.83 m, yellow ~0.60 m from start
                                #  in Maze4; this keeps a margin while preventing
                                #  the robot wandering onto the open floor.)
UTIL_INFO_W = 1.0               # utility = INFO_W*cluster - DIST_W*path_len
UTIL_DIST_W = 1.3
INITIAL_SCAN_REVS = 1.0         # spin this many revolutions to seed the map
INIT_SPIN_W = 1.5               # rad/s during the seed spin

# ===========================================================================
# Path following (pure pursuit carrot for the DWA layer)
# ===========================================================================
LOOKAHEAD_BASE = 0.26           # m (shorter -> tracks the centreline through bends
LOOKAHEAD_K_V = 0.40            # m per (m/s)   instead of cutting the corner)
LOOKAHEAD_MIN = 0.18
LOOKAHEAD_MAX = 0.55
WAYPOINT_REACH_TOL = 0.16       # m
GOAL_REACH_TOL = 0.16           # m
REPLAN_PERIOD_S = 1.0           # s, global replan cadence
PATH_SIMPLIFY = True            # line-of-sight shortcut the A* path

# ===========================================================================
# DWA local planner (drives toward the carrot using LIVE lidar + aux)
# ===========================================================================
DWA_V_SAMPLES = 7
DWA_W_SAMPLES = 25              # odd -> includes w=0
DWA_HORIZON_S = 1.1
DWA_STEP_S = 0.18
# DWA_SAFE_RADIUS tracks the costmap's own HARD_OBS_DIST (~0.105) so the local
# planner threads the SAME gaps A* already considers passable.  It is now the
# ONLY thing that governs speed near obstacles: a rollout passing closer than
# this is rejected, so the planner keeps full V_MAX when the path ahead is clear
# and selects a slower/curved rollout only when a fast one would graze a wall.
# The old multiplicative heading x front x side slow-down caps were DELETED --
# they compounded to a near-zero crawl in narrow corridors (the "robot moves
# only a few centimetres" bug); see local_planner.DWAPlanner.compute().
DWA_SAFE_RADIUS = 0.12          # m, hard reject if a rollout passes closer than this
                                # (~ the 0.116 footprint; rollouts that would clip a
                                # wall with the real wheels are rejected, was 0.11)
DWA_CLEAR_CAP = 0.22           # m, clearance reward saturates here (small -> the
                               # centring term stays sensitive inside ~0.2 m gaps)
DWA_W_HEADING = 1.1            # align final heading with carrot bearing
DWA_W_DIST = 1.2              # end the rollout near the carrot
DWA_W_CLEAR = 1.3             # clearance == corridor CENTRING (raised: thread centre)
DWA_W_SPEED = 0.18           # mild preference for moving
DWA_W_STRAIGHT = 0.10        # penalty on |w|, damps oscillation
# Single, floored tight-gap speed factor (the ONLY proximity speed cap that
# survives — NOT the old compounding heading x front x side stack).  Below this
# lateral clearance the robot slows for precise centring; floored so it never
# crawls.  At 0.45 floor the worst-case tight-gap speed is 0.45*V_MAX ~= 0.16 m/s.
DWA_TIGHT_GAP_DIST = 0.22    # m, lateral clearance below which to ease off
DWA_TIGHT_GAP_VFRAC = 0.45   # floor of the tight-gap speed factor
# Pivot-in-place gate (the one clean, NON-compounding speed control that
# survives): when the carrot sits more than DWA_PIVOT_BEARING off the nose AND
# is farther than DWA_PIVOT_MIN_DIST, command v=0 and rotate to face it instead
# of carving a wide arc that clips a corridor wall.  0.6 rad matches Maze1's
# proven pure-pursuit PP_BIG_HEADING_STOP.
DWA_PIVOT_BEARING = 0.6      # rad (~34 deg) carrot offset above which we pivot
DWA_PIVOT_MIN_DIST = 0.18    # m, don't pivot for a carrot already this close
DWA_LIDAR_SUBSAMPLE = 110    # cap lidar points used per rollout (speed)
DWA_SEARCH_W = 1.0           # in-place spin (rad/s) when fully boxed in
DWA_SLOWDOWN_DIST = 0.38     # m, LEGACY: now only the radius for querying mapped
                             # poison/aux obstacles near the robot (no speed cap)
DWA_SIDE_SLOW_DIST = 0.20    # m, LEGACY/unused (side slow-down cap was removed)

# ===========================================================================
# Green-poison safety reflex (drift-immune)
# ===========================================================================
# Independent of the mapped poison layer (which A* + DWA already avoid): a true
# last-resort guard that fires ONLY when projected green floor is IMMINENT --
# within GREEN_REFLEX_DIST metres directly ahead in the body frame.  It must NOT
# fire for distant poison (caused a blue->yellow deadlock on Maze5: the robot
# faced far poison and froze even though a valid path routed around it).
GREEN_REFLEX_ENABLED = True
GREEN_REFLEX_DIST = 0.45        # m ahead; closer projected green -> block forward
GREEN_REFLEX_HALF_W = 0.20      # m, half-width of the imminent-poison corridor
GREEN_REFLEX_MIN_PTS = 6        # min projected green points in that box to fire

# ===========================================================================
# IR bumper (NEW for Maze4 -- close-range backstop for low floating panels)
# ===========================================================================
# The 4 chassis IR sensors sit ~5-9 cm above the floor on the Rosbot proto,
# directly covering WallShort(15)'s span (0.015-0.065 m) -- the one panel
# that is invisible to BOTH the lidar and (at very close range) possibly the
# depth camera.  Independent, sensor-direct hard-stop: never trust mapping for
# the last few centimetres.
# DISABLED: The ir_lookup.py linear interpolation between the first/last
# lookup-table entries is fundamentally wrong for the Rosbot's non-linear IR
# response curve.  This produced false-positive "obstacle at 5 cm" readings
# that latched ir_block=Y after just 22 cm of travel, freezing the robot.
# The depth-camera aux layer already covers the floating walls at > 0.6 m
# range; WallShort(15) is now handled by the lowered AUX_Z_MIN.
IR_BUMPER_ENABLED = False
IR_BUMPER_STOP_DIST = 0.08      # m; any IR below this while moving forward -> v=0

# ===========================================================================
# Recovery (stuck handling)
# ===========================================================================
STUCK_PROGRESS_MIN_M = 0.07     # m progress expected per window
STUCK_TIMEOUT_S = 5.0           # s without progress -> recovery (raised from 3.0:
                                # old value false-triggered during legitimate frontier
                                # re-evaluation spins and NO_FRONTIER_SPIN_S rescans)
FROZEN_TIMEOUT_S = 8.0          # s with NO position AND NO heading change in a
                                # driving state -> recovery (backstop against any
                                # v=0,w=0 deadlock, e.g. a latched safety reflex;
                                # must be > STUCK_TIMEOUT_S)
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
# Pillars now have explicit height 0.3 m at z=0.15, matching Maze1/Maze5.
# (Previously buried at z=-0.6 with Webots default 2.0 m height.)
PILLAR_HEIGHT = 0.30            # m
PILLAR_MIN_PIXELS = 6           # min coloured blob area (lowered: detect at longer range)
PILLAR_MAX_DETECT_RANGE = 5.0   # m
PILLAR_HEIGHT_MIN_FRAC = 0.15   # accept more height variance at long range
PILLAR_HEIGHT_MAX_FRAC = 2.00
PILLAR_ASPECT_MAX = 2.2         # reject wide blobs (walls share the colour rarely)
PILLAR_OBS_AVG_N = 3            # running mean window for world position (lowered: confirm faster)
PILLAR_OUTLIER_REJECT_M = 1.2   # m, discard detections this far from the mean
GREEN_PROJECT_STRIDE = 3        # subsample factor when projecting green floor
GREEN_PROJECT_MAX_RANGE = 3.0   # m, drop far (noisy) green projections (keeps map crisp)

# ===========================================================================
# Mission / logging
# ===========================================================================
PILLAR_REACH_DIST = 0.35        # m, mission "reached the pillar" threshold (centre dist)
GO_FAIL_COOLDOWN_S = 2.5        # s, after a failed GO, explore this long before retry
NO_FRONTIER_SPIN_S = 4.0        # s, spin-and-rescan when no frontier is available
PERCEPTION_EVERY_TICKS = 2      # run colour perception every N ticks
DEPTH_AUX_EVERY_TICKS = 2       # run depth-aux processing every N ticks
SNAPSHOT_PERIOD_S = 3.0         # s, write a map PNG this often
LOG_EVERY_TICKS = 24
OUTPUT_DIRNAME = "maps"
DONE_LINGER_S = 1.5             # s to keep window after DONE
