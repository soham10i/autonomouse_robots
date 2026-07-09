# mak_02_controller — Maze5 frontier-exploration navigation (ROSbot / Webots)

Autonomous navigation for the Husarion ROSbot in `Maze5.wbt`: drive from the
**blue** pillar to the **yellow** pillar in the least simulation time, **without
touching walls** or the **green poison floor**, using only on-board sensors
(**no Webots Supervisor**, no edits to the robot or world).

## How to run

1. Open `Maze5/worlds/Maze5.wbt` in Webots R2025a. The `Rosbot` node's
   `controller` field is already set to **`mak_02_controller`**.
2. The controller needs a Python with **NumPy** (required). **OpenCV** (`cv2`)
   and/or **Pillow** are optional — used only for the live window / PNG saving;
   the controller degrades gracefully without them.
   - Use the same Python you use for the other mazes. In Webots:
     *Tools → Preferences → Python command* → point it at a Python that has
     NumPy, e.g. the project venv:
     `…/autonomous_system/env/bin/python3`.
3. Press **Play**. Watch the terminal for the device inventory, the FSM
   transitions and the **timing table** printed when the yellow pillar is
   reached. Press **`Q`** in the 3D view to finalise outputs early.

Disable the live OpenCV window with the env var `MAK02_LIVE=0`.

## What it does (one paragraph)

The robot localises with **wheel + IMU-yaw odometry**, refined by a light
**lidar scan matcher** (the IMU yaw is drift-free, so the matcher mostly fixes
translation). It builds a **2-D log-odds occupancy grid** from the lidar — in
Maze5 every wall crosses the lidar plane, so a clean lidar-only map is enough
(no fragile depth layer). The **camera** detects the blue/yellow pillars (HSV +
height/aspect gating) and projects **green poison** pixels onto the floor into a
sticky lethal layer. It **explores with visual frontiers** (free cells touching
unknown), ranking them by `info_gain − A*-path-length`, until a target pillar is
seen; then it plans with **A\*** on the inflated costmap and follows the path
with a **pure-pursuit carrot + DWA local planner** that drives off the **live
lidar**, so wall clearance is immune to map drift. A green **safety reflex** and
**stuck-recovery** are independent backstops.

Mission FSM:
`INIT_SCAN → EXPLORE_BLUE → GO_BLUE → EXPLORE_YELLOW → GO_YELLOW → DONE`
(`RECOVERY` is reachable from any driving state).

## Code structure & uniform interface

The controller class is **`NavigationController`** (identical name in every maze),
exposing a uniform method surface — `update_sensing`, `run_slam_step`,
`update_perception`, `refresh_costmap`, `plan_path_to`, `select_frontier_goal`,
`drive_along_path`, `step_mission`, `finalize`, `run`. The Webots device layer is
**`RobotInterface`** (`robot_io.py`), the only simulator-facing module. This
shared contract is what the fleet-wide wrapper in `../../../common/` drives; the
controller itself does not import that package, so it stays self-contained.

## Observability & fault handling (enterprise-style tracing)

- **Console logs** via the `logging` module (`navctl.maze5`) — leveled,
  timestamped versions of the operator messages the controller always printed.
- **Structured run log** at `maps/run_events.jsonl`, one JSON object per line:
  `run_start` (with the device inventory), mission `state_transition`s,
  `pillar_reached` splits, `recovery_enter`, `status` heartbeats, `fault`
  records (with tracebacks), the `timing_table`, and `run_end`.
- **Fault isolation**: every control-loop stage runs inside a `guarded_stage`
  barrier, so an exception in one stage is logged with tick/state context and the
  loop continues with the last safe command instead of aborting. `RobotInterface`
  reads are individually guarded, and a top-level safety net guarantees the
  wheels are stopped if the controller ever crashes. Provided by the vendored
  `observability.py` (a copy of `../../../common/observability.py`).

## Files

| file | role |
|------|------|
| `mak_02_controller.py` | `NavigationController` — mission FSM + main loop (orchestrator) |
| `config.py` | **all** tunable constants (single source of truth) |
| `robot_io.py` | `RobotInterface` — the only Webots-facing module (devices, I/O) |
| `observability.py` | logging + JSONL event log + per-stage fault barriers (vendored) |
| `geometry.py` | SE(2) pose maths + diff-drive kinematics |
| `odometry.py` | wheel + IMU-yaw odometry |
| `mapping.py` | occupancy grid, scan matcher, poison layer, costmap |
| `perception.py` | HSV pillar detection + green-floor projection + reflex |
| `frontier.py` | frontier detection + clustering |
| `astar.py` | A\* on the costmap + line-of-sight path simplification |
| `local_planner.py` | pure-pursuit carrot + DWA driver + recovery helpers |
| `viz.py` | live map / frontier visualisation + PNG snapshots |
| `selftest.py` | Webots-free pipeline self-test (no GUI needed) |

Outputs are written to `maps/`: `live_map.png` (updated during the run),
`final_map.png`, `map.npz` (`L` log-odds grid + `poison` mask), and
`run_events.jsonl` (structured runtime telemetry).

## Tuning

Everything tunable lives in `config.py`. The values most worth touching:
`V_MAX / V_CRUISE` (speed), `DWA_SAFE_RADIUS` (wall clearance vs. narrow-passage
threading), `POISON_HARD_DIST` (poison standoff), `HARD_OBS_DIST` (how tight a
gap A\* will plan through), and the `HSV_*` colour thresholds.

## Self-test (no Webots)

```
…/env/bin/python3 selftest.py
```
Runs the mapping → costmap → A\* → frontier → DWA → perception pipeline on
synthetic data and prints PASS/FAIL.
