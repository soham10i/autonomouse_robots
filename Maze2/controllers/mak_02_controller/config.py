"""Single source of truth for the Maze2 ROSbot controller.

No Webots import here so every algorithmic module that depends on it stays
importable / testable outside the simulator.  Hardware-facing code lives in
:mod:`robot_io`.

This is the proven Maze4 frontier-exploration SLAM stack (a Python port of the
move_base / gmapping pattern of github.com/HanwenCao/Frontier_Exploration: a
correlative scan-matcher de-drifts wheel+IMU odometry, a log-odds lidar grid
with a depth "aux" layer for lidar-blind walls, frontier+A* picks the global
goal, and a DWA local planner threads the live-lidar corridor centreline)
re-tuned for the Maze2 arena.  Only the geometry/resolution constants below
differ from Maze4 -- the algorithms are unchanged.  The controller is
START-RELATIVE: the pose starts at the origin with an IMU-seeded heading and
the pillars are DISCOVERED by vision, so NO world coordinates are hardcoded in
the runtime path; the facts below are documentation that drove the tuning.

Maze2 facts (read directly from ``Maze2/worlds/Maze2.wbt``; controller frame =
world translated by the robot start, axes aligned, heading IMU-seeded, so
ctrl = world - start with start = (-0.9552, 0.3878)):
  * Robot start  world (-0.9552, 0.3878), yaw ~ +1.2416 rad (~71.1 deg).
  * Blue pillar  world ( 1.49, -1.77) -> ctrl ( 2.45, -2.16)  (reach FIRST;
    ~3.26 m from start, SE, behind the floating/tilted-wall + poison cluster).
  * Yellow pillar world (-0.93,  0.99) -> ctrl ( 0.03,  0.60)  (reach SECOND;
    ~0.60 m N of start; mission time stops here).
  * Cylinder pillars: explicit height 0.4 m, z=0.2, radius 0.1 -> span
    z in [0.0, 0.4], ground-based and fully lidar-visible.  PILLAR_HEIGHT = 0.40 m
    (this differs from Maze4's 0.30 m and feeds the pixel-height range estimate).
  * TWO green "poison" floor patches (boxes 0.2 x 0.5 x 0.1 at z=-0.049),
    world (2.37, -1.73) and (1.52, -0.92) -- both on the blue approach.  Same
    flat-decal handling as Maze4/Maze5 (colour-projected, not lidar).
  * ~61 standing walls: the normal full-height type are WallShort boxes
    0.5 x 0.05 x 0.5 centred at z=0.18 -> span z in [-0.07, 0.43], crossing the
    2-D lidar plane (z=0.10) normally.
  * Lidar-blind hazards clustered on the SE blue approach (world x[0.95,2.77],
    y[-1.28,-0.70]) -- the hard part, exactly the Maze4/Maze1 character:
        WallShort(18)  world (2.73,-0.70) z in [0.20,0.70]  FLOATING: lidar
            passes UNDER (plane 0.10 < 0.20) but the 0.22 m robot would clip it.
        WallShort(17/14) world (1.89,-1.10)/(2.21,-1.28)  TUMBLED panels tilted
            off-axis, sitting low in the body band -> the 2-D lidar sees only a
            sliver or misses them.
        a near-floor tilted panel  world (0.95,-1.28) z ~ -0.01.
    These need the depth-camera "aux" layer (depth resolves them at > 0.6 m).
  * Genuine PASS-UNDER bridges (must NOT be mapped): two walls world
    (1.73,-0.64) and (1.73,-0.50) at z=0.50 -> span z in [0.25,0.75], bottom
    0.25 > ROBOT_HEIGHT 0.22 -> the robot drives under.  The aux height band
    [AUX_Z_MIN, AUX_Z_MAX=0.22] excludes them automatically.
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
GRID_RESOLUTION = 0.025         # m / cell -- HIGH-RESOLUTION SLAM (was Maze4's 0.04):
                                # 0.025 m sharpens the walls and the floating/tilted-wall
                                # footprints while the lethal-inflation rounding still
                                # matches the 0.116 m footprint (round(0.12/0.025)=5 cells
                                # = 0.125 m) and a 0.39 m gap stays multi-cell plannable.
GRID_SIZE_M = 9.0               # square side; centred on the start it spans ctrl
                                # [-4.5, 4.5] in both axes -- covers the whole reachable
                                # Maze2 arena (walls reach ctrl x[-0.45,3.99], y[-4.71,4.13])
                                # within the EXPL_MAX_RADIUS_M exploration cap below.
GRID_CELLS = int(round(GRID_SIZE_M / GRID_RESOLUTION))   # 360
GRID_ORIGIN = (-4.5, -4.5)      # world coord of cell (0,0) lower-left corner (ctrl frame)

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
AUX_Z_MIN = 0.03                # m above floor -- Maze2's floating/tilted walls sit in the
                                # body band (WallShort(18) bottom 0.20, tumbled 17/14 ~0.10,
                                # a near-floor tilted panel ~ -0.01); 0.03 catches the
                                # body-band hits while staying above the z=-0.049 poison
                                # decals and floor noise (raised from Maze4's 0.01).
AUX_Z_MAX = ROBOT_HEIGHT         # m (== 0.22); higher pixels are passable (excludes the
                                # z>=0.25 pass-under bridges at world (1.73,-0.64/-0.50)).
AUX_MAX_RANGE = 1.3              # m; mark in-band depth hits out to here.  The depth camera
                                # is blind < 0.6 m, so a floating wall is visible only in the
                                # window [0.6, AUX_MAX_RANGE]; 1.3 m gives a ~0.7 m / ~2.5 s
                                # confirmation window before the dead zone (enough for the
                                # PERSISTENT mark below to carry it through) WITHOUT marking
                                # far obstacles so aggressively that the layer over-grows
                                # (1.8 m contributed to the aux smear that boxed the robot).
AUX_CLEAR_MARGIN = 0.05          # m; stop the see-through decay ray this short of a hit
AUX_COL_SUBSAMPLE = 160          # cap depth-image columns used for the see-through decay rays
AUX_LIDAR_BLIND_CELLS = 6        # a dense depth hit is kept ONLY where the 2-D lidar is
                                 # genuinely blind -- no lidar-occupied cell within this many
                                 # cells (0.15 m).  This is the anti-smear gate: the depth
                                 # camera also sees the chassis band of NORMAL full-height
                                 # walls the lidar already maps; without the gate every wall
                                 # is duplicated into the aux layer and the map smears shut.

# ---------------------------------------------------------------------------
# Persistent depth-obstacle confidence (the floating-wall fix)
# ---------------------------------------------------------------------------
# ROOT CAUSE this replaces: the old aux layer raytrace-CLEARED every tick, so a
# floating wall was ERASED the instant the robot closed inside the 0.6 m depth
# blind zone (the column then reads "clear" through the now-invisible wall) --
# the robot drove into walls the map had just forgotten (log: aux 181 -> 54 on
# approach, then a no-progress wedge).  Now each lidar-blind in-band hit
# ACCUMULATES confidence; a cell is lethal aux at >= AUX_MIN_HITS and saturates
# at AUX_HIT_CAP; a CLEAR sight-line only DECAYS confidence (by AUX_DECAY) and --
# crucially -- the decay ray STARTS at the depth min range, so a wall that has
# entered the < 0.6 m dead zone is NEVER decayed (absence of evidence there is
# not evidence of absence).  A well-confirmed wall (near the cap) survives many
# stray clear frames, so it stays mapped all the way to contact; a one-off noise
# hit peaks below the threshold and fades.  This is teleop_mapping's persistence
# with mak_02's anti-smear lidar gate -- see mapping.integrate_depth_obstacles.
AUX_HIT_INC = 2.0               # confidence added to a lidar-blind in-band hit cell per frame
AUX_DECAY = 0.5                # confidence removed per frame a cell is seen CLEAR (slow ->
                               # a confirmed wall persists through the dead-zone approach)
AUX_GLOBAL_DECAY = 0.04       # confidence removed from EVERY confident cell each frame -- the
                               # accumulation bound: a mark left behind (no longer seen, never
                               # driven over) fades in ~10-15 s, so the layer cannot grow
                               # without limit and smear the map shut (the boxed-in failure);
                               # small enough that a cap-confirmed wall stays lethal through
                               # the ~2-3 s dead-zone approach.
AUX_MIN_HITS = 3.0             # confidence at/above which a cell is lethal aux (2-frame confirm)
AUX_HIT_CAP = 12.0             # confidence ceiling; a confirmed wall here survives ~24 stray
                               # clear frames (>> the few frames the dead-zone approach lasts)

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
EXPL_MAX_RADIUS_M = 4.5         # m from start; do not chase frontiers past this.
                                # (blue pillar ~3.26 m, yellow ~0.60 m from start in
                                #  Maze2; 4.5 m gives margin to route around the SE
                                #  floating-wall cluster while keeping the robot inside
                                #  the 9 m grid and off the far N/S maze arms.)
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
# Multi-frame confirmation (de-latch).  Floor projection drifts on oblique views,
# so distant green can momentarily back-project into the < 0.45 m ahead-box and
# FALSE-trigger the freeze (observed in Maze2: the robot froze ~0.45 m short of
# blue while the real poison sat > 1 m away).  Require the imminent-green
# condition to hold for this many CONSECUTIVE perception frames before zeroing
# forward speed; a one-frame drift splash no longer freezes the robot.
GREEN_REFLEX_CONFIRM_FRAMES = 2

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
# Maze2 pillars are explicit height 0.4 m at z=0.2 (span [0.0, 0.4]) -- TALLER
# than Maze4's 0.30 m.  This is the known-height fallback the perception layer
# uses to estimate pillar range from its pixel height, so it must match the world.
PILLAR_HEIGHT = 0.40            # m
PILLAR_MIN_PIXELS = 6           # min coloured blob area (lowered: detect at longer range)
PILLAR_MAX_DETECT_RANGE = 5.0   # m
PILLAR_HEIGHT_MIN_FRAC = 0.15   # accept more height variance at long range
PILLAR_HEIGHT_MAX_FRAC = 2.00
PILLAR_ASPECT_MAX = 2.2         # reject wide blobs (walls share the colour rarely)
PILLAR_OBS_AVG_N = 3            # running mean window for world position (lowered: confirm faster)
PILLAR_OUTLIER_REJECT_M = 1.2   # m, discard detections this far from the mean
GREEN_PROJECT_STRIDE = 3        # subsample factor when projecting green floor
GREEN_PROJECT_MAX_RANGE = 1.2   # m, drop far (noisy) green projections.  LOWERED from
                                # 3.0: the flat-floor back-projection error grows fast
                                # with range (pixels near the horizon map to wildly wrong
                                # ground points), so far green was the main source of the
                                # poison smear that choked the blue approach.  Only close,
                                # geometrically reliable green is mapped now.

# ===========================================================================
# Poison confidence gating (self-correcting against projection drift)
# ===========================================================================
# The poison layer used to be a write-only sticky bitmap: every projected green
# point set a cell forever, with no clear/decay.  Combined with the projection
# drift above, that grew to ~700 phantom cells (== ~0.44 m^2 for two 0.2x0.5 m
# real patches) smeared across the SE blue approach, inflating the A* costmap
# until the only gaps to blue were blocked.  Now each cell ACCUMULATES confidence
# (POISON_HIT_INC per frame it is seen) while un-reconfirmed cells DECAY
# (POISON_DECAY per frame), so a cell must be seen on CONSECUTIVE close frames to
# become lethal and a one-off drift mark fades within a couple of frames.  A cell
# that reaches POISON_HIT_CAP is treated as confirmed and stops decaying (a truly
# observed patch stays sticky).  Timeline at INC=2/DECAY=1/MIN=3: a consistently
# seen cell is lethal after 2 frames; a single drift hit peaks at 2 (< 3) and is
# gone in 2 frames.
POISON_HIT_INC = 2.0            # confidence added to a cell each frame it is projected
POISON_DECAY = 1.0             # confidence removed each frame from un-reconfirmed cells
POISON_MIN_HITS = 3.0          # confidence at/above which a cell is lethal poison
POISON_HIT_CAP = 8.0           # confidence ceiling; a cell here is confirmed -> sticky

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
