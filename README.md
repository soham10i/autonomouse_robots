# Autonomous Robots — Modularbeit (OTH Amberg-Weiden)

ROSbot autonomous maze navigation in **Webots R2025a**. Robot must reach the
**blue** pillar first, then the **yellow** pillar, in minimum simulation time,
without ever crossing the **green** poison patch.

## Current controllers (one per maze)

Each maze world runs one tuned controller. After the fleet-wide cleanup they all
share the same class name (**`NavigationController`**), the same Webots device
layer (**`RobotInterface`** in `robot_io.py`), and the same enterprise-style
observability (leveled logging + a structured `maps/run_events.jsonl` event log +
per-stage fault barriers — see any controller's README). Each package stays fully
self-contained so it runs standalone in Webots.

| maze | controller (world binding) | notes |
|---|---|---|
| Maze1 | `Maze1/controllers/mak_02_controller` | full stack (lidar SLAM + depth-aux + IR bumper) |
| Maze2 | `Maze2/controllers/mak_02_controller` | full stack, high-res 0.025 m grid |
| Maze3 | `Maze3/controllers/mak_04_controller` | fusion: teleop-crisp map-consistent SLAM + map-based poison |
| Maze4 | `Maze4/controllers/mak_02_controller` | full stack (reference tuning) |
| Maze5 | `Maze5/controllers/mak_02_controller` | baseline (lidar-only; no depth-aux/IR) |

Each controller has its own `README.md` with the pipeline, FSM, observability and
validation details. Every package ships a Webots-free `selftest.py` (and, where
applicable, `dryrun.py`) — the regression gate used throughout the cleanup.

### Simulation Videos

**Maze 3 Test Run:**

<video src="https://github.com/soham10i/autonomouse_robots/raw/main/Maze3/Maze3_simulation_video.mp4" controls="controls" style="max-width: 100%;">
  Your browser does not support the video tag. You can <a href="https://github.com/soham10i/autonomouse_robots/raw/main/Maze3/Maze3_simulation_video.mp4">download the video here</a>.
</video>

**Maze 4 Test Run:**

<video src="https://github.com/soham10i/autonomouse_robots/raw/main/Maze4/Maze4_simulation_video.mp4" controls="controls" style="max-width: 100%;">
  Your browser does not support the video tag. You can <a href="https://github.com/soham10i/autonomouse_robots/raw/main/Maze4/Maze4_simulation_video.mp4">download the video here</a>.
</video>

**Maze 5 Test Run:**

<video src="https://github.com/soham10i/autonomouse_robots/raw/main/Maze5/Maze5_simulation_video.mp4" controls="controls" style="max-width: 100%;">
  Your browser does not support the video tag. You can <a href="https://github.com/soham10i/autonomouse_robots/raw/main/Maze5/Maze5_simulation_video.mp4">download the video here</a>.
</video>

### `common/` — shared wrapper & observability (outside every maze)

[`common/`](common/) sits **outside** all `Maze*/` folders and is imported by
**none** of the running controllers, so it cannot affect the code that runs in
Webots. It provides a uniform **`MazeControllerWrapper`** that resolves any maze
id to its `NavigationController` and exposes **only** the shared method contract,
plus the canonical `observability.py` that each controller vendors a copy of.
See [`common/README.md`](common/README.md). Validate discovery with
`python3 common/smoke_test.py`.

---

> **Note:** the "Phased development" material below is **legacy/historical** — it
> describes an earlier controller lineup (`teleop_mapping`, `auto_explorer`,
> `path_runner`, `map_runner`, `teleop_mission`, `maze_navigator`) that predates
> the current per-maze `NavigationController` layout. It is retained for design
> context; for how to run today, use the per-maze controllers listed above.

This README covers how to run and tune the controller.

## Setup

Target: Python 3.9+. A venv is already present under `./env`.

```bash
source env/bin/activate
pip install -r requirements.txt
```

## Phased development

The project is being built in three phases so each piece can be debugged
on its own.

| Phase | Controller        | What it does                                                                                                                                                                |
|-------|-------------------|-----------------------------------------------------------------------------------------------------------------------------------------------------------------------------|
| 1     | `teleop_mapping`  | Manual mapping only. Arrow keys drive; lidar + depth + odom record into a 2D grid + 3D cloud. `Q` finalises.                                                                |
| 2     | `auto_explorer`   | Frontier-based autonomous mapping (still rough — see notes). Same outputs as Phase 1; pillar positions are *recorded* but not pursued.                                       |
| 3     | `path_runner`     | Loads a saved `map.npz`, runs A* `start → BLUE → YELLOW` with poison veto. Use when you have a pre-recorded map.                                                            |
| 3     | `map_runner`      | Loads `scene.ply` for **crisp wall reconstruction** (height filter + morph close + open) plus `map.npz` for pillars/poison. Plans both legs offline, executes open-loop with motors + encoders + IMU only — LiDAR / camera / IR are disabled for sim speed.  |
| **1+3** | **`teleop_mission`** | **Recommended.** Drive manually until both pillars are seen, press **M**, the bot then runs A* autonomously to BLUE then YELLOW using the just-built map.               |
| 3+    | `maze_navigator`  | (Legacy hybrid) Maps on the fly *and* runs the mission in one process. Kept around for experiments.                                                                          |

Switch which controller a world uses by editing the `controller "..."`
field in `Maze1/worlds/Maze1.wbt` (≈ line 36) or the matching line in
`Maze2/worlds/Maze2.wbt`. Allowed values: `teleop_mapping`,
`auto_explorer`, `path_runner`, `map_runner`, `teleop_mission`,
`maze_navigator`.

## Run

Open either world in Webots:

* `Maze1/worlds/Maze1.wbt`
* `Maze2/worlds/Maze2.wbt`

The Maze2 controllers are thin re-exports of the Maze1 ones — you maintain
one implementation per phase. Each world's `maps/` output dir is redirected
to its own controller folder so dumps don't collide.

On first start the controller prints a full device inventory. If a device
shows `MISSING`, edit `Maze1/controllers/maze_navigator/config.py` to match
the real name in the per-index listing.

### Phase 1 — manual mapping (current focus)

1. Set the world's controller to `teleop_mapping`.
2. Click **Run** in Webots, then **click into the 3D view** so the
   simulator actually receives keyboard input.
3. Drive:

   ```
   ↑ / ↓     forward / reverse
   ← / →     turn (combine with ↑/↓ for arcs)
   Space     emergency stop
   Q         finalise + write outputs + quit
   ```

4. While driving, live PNGs land in `maps/teleop_NNNN.png` so you can
   watch coverage grow.
5. Press **Q** to stop. The controller writes:

   | File             | What it is                                                                                                            |
   |------------------|-----------------------------------------------------------------------------------------------------------------------|
   | `final_map.png`  | cleaned 2D occupancy map with the trajectory overlaid                                                                 |
   | `scene.ply`      | accumulated 3D point cloud (open in MeshLab / CloudCompare / Open3D)                                                  |
   | `map.npz`        | binary 2D map for Phase 3 — keys: `occupied`, `free`, `unknown`, `aux_obstacle`, `poison`, `resolution`, `origin`, `cells`, `pose_history` |

   Outputs land in `Maze1/controllers/teleop_mapping/maps/` (or the Maze2
   equivalent).

### Phase 2 — autonomous mapping

1. Set the world's controller to `auto_explorer`.
2. Click **Run**. The robot performs an initial in-place scan, then
   repeatedly:

   * extracts frontiers (boundary between known-free and unknown),
   * picks the best one (size / distance, biased forward),
   * plans an A* path on the soft costmap,
   * follows it with pure pursuit + collision veto,
   * recovers (reverse → spin → settle) when stuck.

3. Press **Q** at any time to stop. Otherwise the controller exits
   automatically when the unknown-cell count stops shrinking
   (`EXPL_NO_FRONTIER_QUIT_S` in `auto_explorer.py`, default 12 s).
4. Outputs are the same as Phase 1, written to
   `Maze1/controllers/auto_explorer/maps/`:

   * `final_map.png`, `scene.ply`, `map.npz`
   * `explore_NNNN.png` live snapshots (path + frontier overlaid).

   `map.npz` is binary-compatible with the Phase-1 file, so Phase 3 will
   accept either as input.

### Phase 1 + 3 combined — `teleop_mission` (recommended)

This is the simplest end-to-end demo. You drive the bot manually until
the camera has seen both pillars, then press **M** and the bot runs
A* shortest-path autonomously to BLUE → YELLOW.

1. Set the world's controller to `teleop_mission`.
2. Click **Run**. The startup banner prints the controls:

   ```
   Up / Down    : drive forward / reverse
   Left / Right : turn
   Space        : emergency stop
   M            : switch to autonomous BLUE→YELLOW
   Q            : finalise + quit
   ```
3. Drive around the maze. Watch the periodic log line:
   ```
   [t= 14.30s state=TELEOP    ] pose=(...) B=(+2.45,+0.84) Y=? poison=83
   ```
   You're looking for **both** `B=(...)` and `Y=(...)` to be set, and the
   `poison` count to be in the dozens (not thousands — see "common
   gotchas" below).
4. Once both pillars show coordinates and you've covered the corridors
   between them, press **M**:
   ```
   [mission] M pressed — switching to autonomous
   [fsm] TELEOP -> TRANSITION
   [fsm] TRANSITION -> GO_BLUE
   [mission] BLUE leg planned: standoff=(+2.00, +0.65) len=8 pts ≈ 2.91 m
   ```
5. The bot drives itself to BLUE, pauses ~1 s, then plans + drives to
   YELLOW. On reaching YELLOW it transitions to `DONE` and stops. Live
   mapping continues until you press **Q**.
6. **Q** finalises and writes:
   * `final_map.png` — full map + trajectory + pillar markers
   * `scene.ply` — 3D point cloud
   * `map.npz` — re-usable binary map
   * `mission_log.txt` — leg timings and path lengths
   * `mission_NNNN.png` — live snapshots from each second of the run

If the bot can't reach a pillar:

* `[mission] FAILED to plan to ... pillar at ...` — the saved map has no
  free path. Either the pillar position is wrong (drive a little closer
  during teleop to refine the running mean) or the corridor between
  start and pillar wasn't mapped (drive through it during teleop).
* `[mission] recovery chain exhausted — abandoning` — a real obstacle
  is blocking the path even after recovery. Press Q, fix mapping, retry.

**Common gotchas during teleop**

* **Don't drive over the green patch.** The per-pixel poison projection
  marks every green pixel under the camera as lethal — driving on top
  of it accumulates a huge poison region that blocks A*. Drive around
  it; the camera can still see the patch from the side.
* **See each pillar from at least two angles.** The running mean over
  the last `PILLAR_OBS_AVG_N` (= 8) detections is more accurate when
  fed from different viewpoints.
* **Cover all corridors.** A* needs a continuous chain of free cells
  between start and each pillar. If the planner says NO PATH, the
  bot's mapping has gaps.

### Phase 3 — `map_runner` (open-loop, scene.ply + map.npz)

This controller is **open-loop after planning**: it parses the offline
map products, plans both legs of the mission once at startup, then
executes them via a queue of pure ROTATE / TRANSLATE primitives using
only motors + wheel encoders + IMU. LiDAR, RGB camera, depth camera,
and IR rangefinders are **all disabled** during execution → noticeably
faster sim wall-clock.

Wall reconstruction
```
scene.ply → height filter (z ∈ [0.05, 1.50] m) → 2D bin → threshold
   → morphological CLOSE (fill gaps) → morphological OPEN (drop noise)
   → unioned with map.npz "occupied" (lidar log-odds threshold) and
     "aux_obstacle" (depth-cam projection during teleop)
   → final crisp wall mask
```

Pillars (BLUE / YELLOW) and the poison region come from `map.npz`'s
camera-derived layers — the PLY's RGB is a height-rainbow, not real
scene colour, so colour-based pillar detection from the cloud isn't
possible without enriching `clearance.depth_to_world_cloud` to also
capture per-pixel RGB.

Costmap + planner

* Walls dilated by `INFLATE_CELLS = 3` cells (= 12 cm robot-clearance halo).
* Poison cells get **weighted cost** (`POISON_COST = 50×`), not lethal —
  so the planner skirts them but can cross if no other route exists.
* A* (8-connected, weighted) on the cell-cost map.
* Ramer-Douglas-Peucker simplifies each cell-path down to a handful of
  corners (`RDP_EPSILON_M = 0.06 m`).

Open-loop primitives

* `ROTATE(Δθ)`: spins until the IMU yaw has accumulated Δθ within ±2°.
* `TRANSLATE(Δd)`: drives forward at 0.12 m/s until encoder odometry
  reports Δd within ±2 cm.

Run it:

1. Make sure `Maze1/controllers/teleop_mapping/maps/` has both `scene.ply`
   and `map.npz` from a prior teleop pass.
2. Set the world's controller to `map_runner`.
3. Click **Run**. The plan + instructions are printed before the bot
   moves at all:
   ```
   [plan] PLY-derived walls: 1411 cells
   [plan] occ=2108  aux=2099  poison=3097  ply_walls=1411
   [plan] BLUE   pillar @ (+2.46, +0.44)
   [plan] YELLOW pillar @ (+0.78, -0.85)
   [plan] A*  start → BLUE  (poison cost = 50.0×)
   [plan] ✓ 12 waypoints, ≈3.89 m
        START → BLUE
       1. ROTATE  +53.13°  (LEFT)
       2. DRIVE    0.20 m
       …
   ```
4. The bot executes both legs and writes `mission_log.txt` + an
   annotated `final_map.png` showing both A* paths.

Override the input file paths via env vars if needed:

```bash
MAP_RUNNER_PLY=/path/to/scene.ply MAP_RUNNER_NPZ=/path/to/map.npz ./run_webots.sh
```

### Phase 3 — pillar mission on a saved map

Pre-requisite: a `map.npz` from Phase 1 (`teleop_mapping`) or Phase 2
(`auto_explorer`). The map should have **both pillar positions** baked
in. Confirm via:

```bash
env/bin/python3 tools/inspect_map.py Maze1/controllers/teleop_mapping/maps/map.npz
```

Look for `[CONFIRMED]` (best) or `[seen-only]` next to each pillar.
"NOT SEEN" means the bot never saw that pillar during mapping — re-run
mapping with that area covered.

To run:

1. Set the world's controller to `path_runner`.
2. Click **Run**. The controller:
   * loads `map.npz` (resolution order: `$PATH_RUNNER_MAP` → controller-local
     `maps/map.npz` → teleop's → auto_explorer's),
   * cleans the saved poison + aux layers (morphological opening +
     connected-component filter to drop drift speckles),
   * spins briefly to seed the live grid,
   * plans **start → BLUE** with A* + standoff goal selection,
   * follows with the SmoothDriver (VFH gap-finder),
   * pauses at the BLUE standoff, then plans **BLUE → YELLOW**,
   * announces each leg's planned length and arrival time.
3. Press **Q** at any time to abort and finalise.
4. Outputs are written to `Maze1/controllers/path_runner/maps/`:
   * `mission_NNNN.png` snapshots (live grid + planned path).
   * `final_mission_map.png` — clean trajectory + pillar markers.
   * `mission_log.txt` — start/blue/yellow timestamps, leg lengths.

If poison over-marking blocks all routes (pre-Phase-2 maps with the
old per-pixel projection often had this issue):

```bash
PATH_RUNNER_DISABLE_POISON=1 ./run_webots.sh
```

This skips the saved poison layer entirely and relies on live
re-detection during the mission.

### Phase 3+ legacy — `maze_navigator`

Set the world's controller back to `maze_navigator` and run. This
controller maps + runs the mission in one process (no `map.npz` reload).
Runtime artifacts go to `Maze1/controllers/maze_navigator/maps/`:

| File                        | What it is                                       |
|-----------------------------|--------------------------------------------------|
| `grid_NNNN.png`             | live occupancy + poison + path + state overlay   |
| `cost_NNNN.png`             | A* costmap heatmap (purple = lethal)             |
| `cam_NNNN_masks.png`        | RGB + blue/yellow/green HSV masks                |
| `final_map.png`             | cleaned final 2D map + trajectory + pillars      |
| `scene.ply`                 | accumulated 3D point cloud (open in MeshLab)     |

## Architecture

```
sensors          robot_io.py           device discovery + thin wrappers
pose             odometry.py           wheel + IMU yaw fusion
mapping          grid_2d.py            log-odds lidar grid
                                        + depth aux obstacle layer
                                        + green poison layer (lethal)
                                        + costmap + inflated lethal
perception       perception.py         HSV blue/yellow/green + depth
clearance        clearance.py          depth-cam fwd cone min range
exploration      explorer.py           frontier extraction
planning         planning.py           A* + standoff goal selection
local control    local_control.py      pure-pursuit + collision veto
mission FSM      fsm.py                INIT_SCAN -> EXPLORE_BLUE -> GO_BLUE
                                        -> EXPLORE_YELLOW -> GO_YELLOW -> DONE
recovery         recovery.py           reverse + alternating spin + replan
debug            debug_viz.py          PNG dumps of grid, costmap, masks
glue             maze_navigator.py     main loop
```

The robot **never** drives over the poison: every green-mask centroid is
projected to a lethal disc in the grid, and the planner refuses any cell
whose poison distance is below the robot footprint.

## Logged events

```
[robot_io]  device inventory
[fsm]       INIT -> INIT_SCAN -> EXPLORE_BLUE -> GO_BLUE -> ...
[mission]   GREEN poison detected at (..., ...)
[mission]   BLUE detected at (..., ...) — planning approach
[mission]   BLUE PILLAR REACHED @ t=...
[mission]   YELLOW PILLAR REACHED @ t=... (blue->yellow leg = ...s)
[planner]   blue plan: standoff=(...) len=...
[planner]   FAILED to plan to blue pillar @ ...
[planner]   frontier unreachable, blacklisting (...)
[recovery]  BEGIN/END
```

## Tuning

All runtime parameters live in `config.py`. The table below lists the most
useful knobs:

| Parameter                       | What it does                                       | Tune up if…                              | Tune down if…                          |
|---------------------------------|----------------------------------------------------|------------------------------------------|----------------------------------------|
| `GRID_RESOLUTION` (0.04)        | cell size in metres                                | runtime is slow                          | walls look fragmented                  |
| `INFLATION_RADIUS` (0.18)       | obstacle inflation                                 | robot scrapes walls                      | planner says "no path" in tight gaps   |
| `POISON_INFLATION_RADIUS` (0.27)| poison safety margin                               | robot risks poison                       | planner can't squeeze past poison      |
| `PILLAR_STANDOFF` (0.45)        | radius around pillar where we stop                 | pillar still triggers an obstacle hit    | reach test never fires                 |
| `PILLAR_REACH_TOL` (0.55)       | arrival distance                                   | pillar visible but not "reached"         | declares arrival prematurely           |
| `LOOKAHEAD_*`                   | pure-pursuit lookahead distance                    | robot wobbles                            | robot cuts corners into walls          |
| `PF_FWD_BRAKE_DIST` (0.30)      | hard-brake distance from any obstacle              | robot collides                           | robot freezes in narrow gaps           |
| `STUCK_TIMEOUT_S` (4.0)         | seconds without progress = recovery                | recovery fires too readily               | robot wedges silently                  |
| `HSV_*`                         | colour thresholds                                  | colour rarely detected                   | colour false-positives                 |

## Known limitations / next improvements

* No scan-to-map ICP correction. Odometry+IMU is good enough for the
  ~5 m mazes, but on much larger or more visually homogeneous maps an
  EKF/ICP step would help.
* Frontier exploration scoring is heuristic; replacing it with an
  information-gain Bayesian objective could yield faster convergence on
  large unknown maps.
* The auxiliary depth obstacle layer is "set once, persistent" — it
  cannot be cleared if the camera mistakenly tags free space. Acceptable
  here because false-positive obstacles are safe; in worlds with moving
  obstacles this would need a decay term.
* `RECOVERY_MAX_CHAIN` caps consecutive recoveries at 3. If a true
  dead-end is encountered the robot blacklists the area and tries
  another frontier.
