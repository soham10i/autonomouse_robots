# mak_04_controller — Maze3

Webots ROSbot controller for the OTH "Autonomous Robots" Modularbeit: drive from
the start to the **BLUE** pillar, then to the **YELLOW** pillar, in the least
simulation time, **without touching a wall or the green poison floor**. No
Supervisor; no edits to the robot or world beyond the controller binding.

`mak_04` is a **fusion controller**: it keeps the proven navigation strategy
(visual-frontier exploration → A* → DWA on the live lidar, depth "aux" layer,
recovery, BLUE→YELLOW FSM) and pairs it with **teleop-crisp, map-consistent
mapping** tuned for long, pivot-heavy runs.

---

## Pipeline (per tick)

```
encoders+IMU ─▶ odometry ─▶ scan-match (lidar vs map) ─▶ pose
lidar ───────▶ log-odds occupancy grid ─┐
depth camera ▶ thin-scan ▶ AUX layer ────┼▶ costmap (A*)  +  live obstacles (DWA)
RGB camera ──▶ HSV pillar / poison ──────┘
                                   │
        VISUAL FRONTIER  ──────────┤  goal selection
        (explore unknown space     │  • frontier util = info_gain − path_len
         until the camera sees     │  • when the camera fixes the target pillar
         the target pillar, then   │    AND A* finds a path → switch to GO
         switch straight to it)    │
                                   ▼
                A* global path ─▶ pure-pursuit carrot ─▶ DWA  ─▶ (v, w)
```

The global A* path only chooses *where* to aim; the **DWA** decides *how* to get
there from the **live lidar** every tick, so wall clearance is drift-immune.

## Map-consistent SLAM (the Maze3-specific tuning)

Mapping is kept teleop-crisp (raw-odom base, high occupancy threshold) but made
**map-consistent** so it does not erase its own thin walls on long runs:

* **Moved-enough gate** — the scan is integrated only after the robot has
  actually translated/rotated (`C.SM_MIN_TRAVEL_M` / `C.SM_MIN_TURN_RAD`), so a
  stationary/pivoting robot no longer re-stamps free-rays through nearby walls
  and clears them.
* **Conservative scan-match** (`C.SM_ENABLED`) — each scan is aligned to the
  existing map before integration, so beams land back on the *same* wall cells
  (reinforcing them) and slow odometry drift is removed; only confident, *small*
  corrections are accepted, avoiding the large-jump jitter that smeared earlier
  maps.
* **Map-based poison** — the binary green reflex was removed; depth-validated
  green projection feeds an accurate poison layer that A* and the DWA costmap
  route around.

## Mission state machine

```
INIT_SCAN -> EXPLORE_BLUE -> GO_BLUE -> EXPLORE_YELLOW -> GO_YELLOW -> DONE
(RECOVERY is reachable from any driving state when progress stalls.)
```

## Code structure & uniform interface

The controller class is **`NavigationController`** (identical name in every maze),
exposing a uniform method surface — `update_sensing`, `run_slam_step`,
`update_perception`, `refresh_costmap`, `plan_path_to`, `select_frontier_goal`,
`drive_along_path`, `step_mission`, `finalize`, `run`. The Webots device layer is
**`RobotInterface`** (`robot_io.py`), the only simulator-facing module. This
shared contract is what the fleet-wide wrapper in `../../../common/` drives; the
controller itself does not import that package, so it stays self-contained.

## Observability & fault handling (enterprise-style tracing)

- **Console logs** via the `logging` module (`navctl.maze3`) — leveled,
  timestamped versions of the operator messages the controller always printed.
- **Structured run log** at `maps/run_events.jsonl`, one JSON object per line:
  `run_start` (with the device inventory), mission `state_transition`s,
  `pillar_reached` splits, `recovery_enter`, `status` heartbeats, `fault`
  records (with tracebacks), the `timing_table`, and `run_end`.
- **Fault isolation**: every control-loop stage runs inside a `guarded_stage`
  barrier, so an exception in one stage is logged with tick/state context and the
  loop continues with the last safe command instead of aborting. `RobotInterface`
  reads are individually guarded, and a top-level safety net guarantees the
  wheels are stopped if the controller ever crashes.

Provided by the vendored `observability.py` (a copy of the canonical module in
`../../../common/observability.py`).

## Validate without Webots

```bash
python3 selftest.py   # unit checks: mapping, scan-match, A*, frontier, DWA,
                      # perception, depth-aux mark/clear, IR lookup, stuck watchdog
python3 dryrun.py     # closed-loop: carrot+DWA through synthetic corridors;
                      # asserts the robot MOVES at cruising speed and reaches goal
```

## Run in Webots

The Rosbot node's `controller` field is set to `mak_04_controller` in
`Maze3.wbt`. **Reload the world** (not just *continue*) so the controller process
picks up edited files. Press **Q** to finalise outputs (map PNG + timing table)
early. Live map, snapshots and `run_events.jsonl` are written to `maps/`.

## Files

| file | role |
|---|---|
| `mak_04_controller.py` | `NavigationController` — FSM, main loop, sensing/planning/driving orchestration |
| `local_planner.py` | arc-length pure-pursuit carrot + DWA (precise-speed policy) |
| `mapping.py` | log-odds grid, costmap, poison + aux floating-wall layers, scan-match |
| `depth_model.py` | depth image → thin-scan (hit/clear per column) |
| `frontier.py` / `astar.py` | frontier detection/clustering, A* + path simplify |
| `perception.py` | HSV pillar detection + green-poison projection |
| `robot_io.py` | `RobotInterface` — the only Webots-facing module (motors, lidar, cameras, IR) |
| `observability.py` | logging + JSONL event log + per-stage fault barriers (vendored) |
| `config.py` | single source of truth for all Maze3 tuning constants |
| `geometry.py` / `odometry.py` | pose maths, wheel+IMU odometry |
| `selftest.py` / `dryrun.py` | Webots-free unit + closed-loop validation |
| `viz.py` | live map / costmap / path rendering |
