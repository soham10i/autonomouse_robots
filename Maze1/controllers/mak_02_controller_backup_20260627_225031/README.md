# mak_02_controller — SLAM + Frontier Exploration (Webots ROSbot)

A from-scratch Python reimplementation of the gmapping-style **frontier
exploration** approach from
[HanwenCao/Frontier_Exploration](https://github.com/HanwenCao/Frontier_Exploration),
adapted to the Maze1 Webots ROSbot. It replaces the drifting, odometry-only
mapping of the previous controllers with a proper **scan-matching SLAM** front
end, so the 2D occupancy grid stays clean enough for A\* planning.

The reference repo is ROS + gmapping + the ROS navigation stack. Since this
project may not use the Webots Supervisor and runs a single Webots controller,
the same *logic* is reimplemented in pure Python:

| Reference (ROS)              | Here (`mak_02_controller`)                          |
| ---------------------------- | --------------------------------------------------- |
| gmapping 2D SLAM             | `slam/` — log-odds grid + correlative scan matcher  |
| frontier detection           | `exploration/frontier.py`                            |
| utility goal selection       | `exploration/goal_selection.py` (info gain / dist)  |
| move_base / nav stack        | `exploration/astar.py` + `control/pure_pursuit.py`  |
| RViz visualisation           | `viz.py` (PNG snapshots + final map)                 |

## How it works (the three phases requested)

**Phase A — SLAM (drift correction).** Wheel encoders + IMU yaw *predict* the
pose each step (`slam/odometry.py`). A **correlative scan matcher**
(`slam/scan_matcher.py`, coarse-to-fine) then slides/rotates the current lidar
scan over the map's **likelihood field** (`slam/likelihood_field.py`) and snaps
the pose to where the scan best lines up with mapped walls. Scans are integrated
*at the corrected pose* — this is what keeps walls one cell thick instead of the
fat, doubled bands the old controller produced. Matching only runs after the
robot has moved/turned past a threshold (gmapping-style update gating).

**Phase B — clean occupancy grid** (`slam/occupancy_grid.py`):
- Log-odds with free/occupied increments close in magnitude, so stray hits are
  erased quickly instead of becoming permanent.
- Free space carved by de-duplicated ray sampling (each cell updated once/scan).
- Depth-camera **floating-wall layer** is *hit-count gated* (needs `AUX_MIN_HITS`
  observations before it counts) and *decays* where the lidar later proves the
  cell free — this kills the giant "purple blob" over-painting.
- Per-pixel **green-poison** projection; poison is lethal and never erased.

**Phase C — frontier exploration** (`exploration/`): detect frontier clusters
(free cells touching unknown), score them by `info_gain/distance` utility, plan
with A\* on the inflated costmap, follow with pure pursuit. A mission FSM visits
**blue then yellow**:

```
INIT_SCAN -> EXPLORE_BLUE -> GO_BLUE -> EXPLORE_YELLOW -> GO_YELLOW -> DONE
                  \__________ RECOVERY __________/   (on stuck)
```

## Running it in Webots

1. Open `worlds/Maze1.wbt` in Webots.
2. Select the **Rosbot** node and set its `controller` field to
   **`mak_02_controller`**. (The world currently ships with `teleop_mission`.)
3. Run the simulation. The robot spins once to seed the map, then explores to
   find and visit the pillars.
4. Press **`Q`** to finalise early. Outputs are written to `maps/`:
   - `final_map.png` — SLAM map + trajectory + landmarks
   - `map.npz` — binary map (warm-start a later mission run)
   - `mak02_XXXX.png` — live snapshots (great for the submission video)
5. The console prints the timing table required for submission:
   `start->BLUE`, `BLUE->YELLOW`, and total.

Dependencies: `numpy`, `scipy` (distance transforms / clustering), `matplotlib`
(snapshots — optional, degrades to a no-op). OpenCV is **not** required;
perception uses a NumPy HSV path.

## Code layout

```
mak_02_controller/
├── mak_02_controller.py   # entry: mission FSM + per-tick loop
├── settings.py            # all tunables (no Webots import => testable)
├── geometry.py            # SE(2) pose math
├── slam/                  # Phase A + B: odometry, grid, likelihood field, matcher
├── mapping/               # lidar + depth sensor models (raw data -> points)
├── exploration/           # Phase C: frontier, goal selection, A*
├── control/               # diff-drive kinematics, pure pursuit
├── perception/            # HSV pillar + green-poison detection
├── hardware/robot_io.py   # the ONLY Webots-facing module
├── viz.py                 # snapshots / final map / npz export
├── tests/                 # unit + closed-loop SLAM tests (stdlib unittest)
└── run_tests.py           # test runner (no pytest needed)
```

## Testing

The Webots `controller` runtime only exists inside Webots, so the algorithm
modules are deliberately decoupled from hardware and unit-tested standalone:

```bash
python3 run_tests.py
```

43 tests cover geometry, kinematics, the occupancy grid (thin walls, aux gating,
aux decay, poison preservation), the likelihood field, the scan matcher
(recovers a known pose from a perturbed guess), frontier detection, goal
selection, A\*, pure pursuit, the sensor models, and a **closed-loop SLAM
integration test** that drives a simulated robot with drifting odometry and
asserts the matcher pulls the pose back to ground truth and keeps walls thin.

## Tuning notes / things to watch

- **Lidar/camera specs** are queried from the devices at runtime, so the code
  adapts if the ROSbot proto changes resolution/FOV.
- If scan matching ever loses lock (e.g. a long featureless corridor), the
  matcher rejects the low-confidence result and falls back to odometry for that
  step (`SM_MIN_SCORE_FRAC`).
- Floating walls **above** `ROBOT_HEIGHT` (0.22 m) are intentionally *not*
  marked, so high passages the robot can drive under stay open
  (`AUX_Z_MAX`).
- Search-window sizes (`SM_COARSE_*`, `SM_FINE_*`) trade accuracy vs CPU; widen
  them if odometry is very noisy, narrow them if the loop runs slow.
