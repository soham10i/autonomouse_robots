# mak_02_controller — Maze1

Webots ROSbot controller for the OTH "Autonomous Robots" Modularbeit: drive from
the start to the **BLUE** pillar, then to the **YELLOW** pillar, in the least
simulation time, **without touching a wall or the green poison floor**. No
Supervisor; no edits to the robot or world beyond the controller binding.

This controller shares the proven navigation stack used across the fleet (lidar
SLAM → visual frontier exploration → A* → DWA, with the depth-camera "aux" layer
and chassis-IR backstop for low/floating wall panels). Only `config.py` — the
single source of tuning constants — is specialised for Maze1's geometry; all
other modules are identical to the fleet reference.

> **World binding:** `Maze1.wbt` sets the Rosbot's `controller` field to
> `mak_02_controller` (this controller). **Reload the world** (not just
> *continue*) after editing so the process picks up changes.

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

- **Console logs** via the `logging` module (`navctl.maze1`) — leveled,
  timestamped versions of the operator messages the controller always printed.
- **Structured run log** at `maps/run_events.jsonl`, one JSON object per line:
  `run_start` (with the device inventory), mission `state_transition`s,
  `pillar_reached` splits, `recovery_enter`, `status` heartbeats, `fault`
  records (with tracebacks), the `timing_table`, and `run_end`.
- **Fault isolation**: every control-loop stage runs inside a `guarded_stage`
  barrier, so an exception in one stage is logged with tick/state context and the
  loop continues with the last safe command instead of aborting. `RobotInterface`
  reads are individually guarded (a faulted device returns its documented
  `None`/`NaN` fallback), and a top-level safety net guarantees the wheels are
  stopped if the controller ever crashes.

Provided by the vendored `observability.py` (a copy of the canonical module in
`../../../common/observability.py`).

## Validate without Webots

```bash
python3 selftest.py   # unit checks: mapping, scan-match, A*, frontier, DWA,
                      # perception, depth-aux mark/clear, IR lookup, stuck watchdog
python3 dryrun.py     # closed-loop: carrot+DWA through synthetic corridors;
                      # asserts the robot MOVES at cruising speed and reaches goal
```

## Files

| file | role |
|---|---|
| `mak_02_controller.py` | `NavigationController` — FSM, main loop, sensing/planning/driving orchestration |
| `local_planner.py` | arc-length pure-pursuit carrot + DWA (precise-speed policy) |
| `mapping.py` | log-odds grid, costmap, poison + aux floating-wall layers, scan-match |
| `depth_model.py` | depth image → thin-scan (hit/clear per column) |
| `frontier.py` / `astar.py` | frontier detection/clustering, A* + path simplify |
| `perception.py` | HSV pillar detection + green-poison projection/reflex |
| `robot_io.py` | `RobotInterface` — the only Webots-facing module (motors, lidar, cameras, IR) |
| `observability.py` | logging + JSONL event log + per-stage fault barriers (vendored) |
| `config.py` | single source of truth for all Maze1 tuning constants |
| `geometry.py` / `odometry.py` | pose maths, wheel+IMU odometry |
| `selftest.py` / `dryrun.py` | Webots-free unit + closed-loop validation |
| `viz.py` | live map / costmap / path rendering |
