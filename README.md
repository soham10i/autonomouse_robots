# Autonomous Robots — Modularbeit (OTH Amberg-Weiden)

ROSbot autonomous maze navigation in **Webots R2025a**. Robot must reach the
**blue** pillar first, then the **yellow** pillar, in minimum simulation time,
without ever crossing the **green** poison patch.

The detailed design rationale is in [APPROACH_REPORT.md](APPROACH_REPORT.md).
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

| Phase | Controller       | What it does                                                                                                                                            |
|-------|------------------|---------------------------------------------------------------------------------------------------------------------------------------------------------|
| 1     | `teleop_mapping` | You drive with the arrow keys; lidar + depth + odometry record into a 2D grid and a 3D point cloud. Press `Q` to write `final_map.png`, `scene.ply`, `map.npz`. |
| 2     | `maze_navigator` | Autonomous frontier exploration on top of the same mapping pipeline.                                                                                    |
| 3     | `maze_navigator` | A* planner + pure-pursuit follower goes for blue → yellow with poison veto.                                                                              |

Switch which controller a world uses by editing the `controller "..."`
field in `Maze1/worlds/Maze1.wbt` (≈ line 36) or the matching line in
`Maze2/worlds/Maze2.wbt`. Allowed values: `teleop_mapping`, `maze_navigator`.

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

### Phase 2/3 — autonomous

Set the world's controller back to `maze_navigator` and run. Runtime
artifacts go to `Maze1/controllers/maze_navigator/maps/`:

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
