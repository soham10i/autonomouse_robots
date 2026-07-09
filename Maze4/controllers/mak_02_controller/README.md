# mak_02_controller — Maze4

Webots ROSbot controller for the OTH "Autonomous Robots" Modularbeit: drive from
the start to the **BLUE** pillar, then to the **YELLOW** pillar, in the least
simulation time, **without touching a wall or the green poison floor**. No
Supervisor; no edits to the robot or world.

It reuses the proven Maze5 stack (lidar SLAM → frontier exploration → A* →
DWA) and adds the **floating-wall avoidance** designed in
`Maze1/controllers/mak_02_controller/`, because Maze4 has 4 low wall panels the
single-plane 2-D lidar partially or fully misses.

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
         until the camera sees     │  • the moment the camera fixes the target
         the target pillar, then   │    pillar AND A* finds a path → switch to GO
         switch straight to it)    │
                                   ▼
                A* global path ─▶ pure-pursuit carrot ─▶ DWA  ─▶ (v, w)
                                              "visual frontier + DWA blend"
```

* **Visual frontier + DWA blend.** Exploration is frontier-driven toward unknown
  space; goal acquisition is *visual* — as soon as the RGB camera localises the
  target pillar and A* can reach it, the FSM commits to it (`_maybe_go`). The
  global A* path only chooses *where* to aim; the **DWA** decides *how* to get
  there from the **live lidar** every tick, so wall clearance is drift-immune.

* **Floating-wall avoidance (from Maze1).** The depth camera back-projects each
  image column to a body-frame point and keeps the nearest one inside the
  collision height band `[AUX_Z_MIN, AUX_Z_MAX=ROBOT_HEIGHT]`
  (`depth_model.thin_scan`). Hits stamp a boolean **aux** grid; a clear
  sight-line **raytrace-clears** the cells along it *every tick*
  (`mapping.integrate_aux`). This is the principled mark/clear model Maze1's
  postmortem recommended — **no** sticky-hit counters, decay, or
  "reconcile-with-lidar" hacks, the very things that caused Maze1's "purple
  thickening" / boxed-in robot. Aux cells feed A* (lethal, like a wall) and the
  DWA obstacle set. Panels higher than `ROBOT_HEIGHT` are correctly treated as
  drivable-under.

---

## Precise speed — the two bugs that were fixed

The robot previously "moved only a few centimetres". Two compounding root causes:

1. **Multiplicative DWA slowdown caps (the crawl).** The forward ceiling was
   scaled by *three* factors — heading-misalignment × front-cone × side-wall
   proximity. In a corridor whose side walls are always ~0.12 m away, all three
   fired at once and multiplied down to ≈0. Maze1's postmortem had already
   deleted exactly this cap. **Fix:** speed is now governed *only* by per-rollout
   clearance rejection (canonical DWA) plus one clean, non-compounding
   **pivot-in-place** gate (`DWA_PIVOT_BEARING`). The robot runs at full `V_MAX`
   when the path ahead is clear and slows only when a fast rollout would actually
   graze something. See `local_planner.DWAPlanner.compute`.

2. **Backward-aiming carrot on simplified straight paths.** A* + line-of-sight
   simplification collapses an open corridor to just `[start, goal]`. The old
   carrot picked "the first path *vertex* ≥ lookahead away, from the closest
   vertex" — once the robot passed the start vertex that vertex stayed closest
   yet sat *behind* the robot, so the carrot snapped backward and the pivot gate
   spun the robot around. **Fix:** arc-length pure pursuit — project onto the
   nearest path *segment*, then walk `lookahead` metres *forward along the path*.
   See `local_planner.choose_carrot`.

`V_MAX` is kept at a precise, controlled **0.35 m/s**; raise it once a clean
Webots run confirms behaviour.

---

## Centreline alignment — the tight-gap fix

Ground-truth analysis of `Maze4.wbt` (exact box/cylinder geometry) vs. the exact
ROSbot footprint from the proto:

| robot | value | source |
|---|---|---|
| passage half-width | **0.116 m** (0.232 m wide) | wheels: anchor y=±0.110, tyre cylinder height 0.035 → outer edge 0.116 |
| circumscribed radius | **0.128 m** | rear-wheel corner |

| route | tightest gap | slack **per side** |
|---|---|---|
| start → BLUE (winds through the y<0 region) | **0.388 m** | +0.078 m |
| start → YELLOW (0.6 m from start) | **0.418 m** | +0.093 m |

Both pillars are reachable, but only with ~8 cm of slack each side — so the robot
**must ride the passage centreline**. Three changes make it do so:

1. **Footprint-accurate inflation.** `HARD_OBS_DIST` was `0.105` — *below* the
   0.116 m half-width, so A* literally planned the wheels into the wall. It now
   equals the true footprint. The grid was refined `0.05 → 0.04 m` so that
   lethal-inflation rounding matches the footprint *and* a 0.388 m gap still
   leaves a multi-cell free band down its centre.

2. **Center-seeking costmap.** The old fixed soft-halo went *flat* in the middle
   of a narrow gap, giving A* no reason to pick dead-centre. The cost now
   decreases smoothly with clearance out to `CENTER_PREF_RANGE`, so A* rides the
   **medial axis** of every corridor. `astar.simplify` is **clearance-aware**: it
   only line-of-sight-shortcuts through genuinely open cells, so it never
   straightens the centred path back against a wall in a tight gap.

3. **Strong DWA centring + precise tight-gap speed.** `DWA_SAFE_RADIUS` matches
   the footprint, the clearance/centring weight is raised, and a single floored
   tight-gap speed factor eases the robot to ~0.16–0.26 m/s only where a side
   wall is genuinely close — precise, never a crawl.

Verified end-to-end: A* rides a 0.40 m corridor at **0.000 m** centre deviation;
the closed-loop DWA threads the real 0.388 m gap dead-centre and recovers from a
6 cm offset without clipping (`selftest.py`, `dryrun.py`).

> **Drive format.** The ROSbot is 4-wheel skid-steer but is already commanded as
> a **2-wheel differential drive** — `geometry.cmd_to_wheels` maps `(v, w)` to a
> left pair and a right pair (`robot_io` drives fl+rl together and fr+rr
> together). No robot edit is needed or allowed; centreline precision comes from
> the planner, not a drivetrain change.

---

## Validate without Webots (the orchestration loop)

```bash
python3 selftest.py   # 31 unit checks: mapping, scan-match, A*, frontier,
                      # DWA, perception, depth-aux mark/clear, IR lookup,
                      # + a closed-loop "no-crawl" regression
python3 dryrun.py     # closed-loop: real carrot+DWA driven through synthetic
                      # straight / narrow / L-corner / pivot corridors;
                      # asserts the robot MOVES at cruising speed, stays clear
                      # of walls, and reaches the goal
```

`dryrun.py` is the design/validation loop: it integrates the planner's `(v, w)`
over hundreds of ticks and turns "the bot doesn't move" into a hard assertion on
**average speed** and **distance travelled** — so the crawl bug cannot silently
return. Latest: straight/narrow/corner/pivot all reach goal at ≈0.30–0.34 m/s.

---

## Code structure & uniform interface

The controller class is **`NavigationController`** (identical name in every maze),
exposing a uniform method surface — `update_sensing`, `run_slam_step`,
`update_perception`, `refresh_costmap`, `plan_path_to`, `select_frontier_goal`,
`drive_along_path`, `step_mission`, `finalize`, `run`. The Webots device layer is
**`RobotInterface`** (`robot_io.py`), the only simulator-facing module. This
shared contract is what the fleet-wide wrapper in `../../../common/` drives; the
controller itself does **not** import that package, so it stays self-contained.

## Observability & fault handling (enterprise-style tracing)

- **Console logs** via the `logging` module (`navctl.maze4`) — leveled,
  timestamped versions of the operator messages the controller always printed.
- **Structured run log** at `maps/run_events.jsonl`, one JSON object per line:
  `run_start` (with the device inventory), mission `state_transition`s,
  `pillar_reached` splits, `recovery_enter`, `status` heartbeats, `fault`
  records (with tracebacks), the `timing_table`, and `run_end`. This is the
  machine-readable trail for reconstructing a failed or slow run after the fact.
- **Fault isolation**: every control-loop stage (sensing → SLAM → perception →
  depth-aux → IR → costmap → mission → visualise) runs inside a `guarded_stage`
  barrier — an exception in one stage is logged with tick/state context and the
  loop continues on the next tick with the last safe command, so a single bad
  sensor frame cannot abort the mission. `RobotInterface` reads are individually
  guarded (a faulted device returns its documented `None`/`NaN` fallback), and a
  top-level safety net guarantees the wheels are stopped if the controller ever
  crashes.

These are provided by the vendored `observability.py` (a copy of the canonical
module in `../../../common/observability.py`).

## Run in Webots

The Rosbot node's `controller` field is already set to `mak_02_controller` in
`Maze4.wbt`. **Reload the world** (not just *continue*) so the controller process
picks up edited files. Press **Q** to finalise outputs (map PNG + timing table)
early. Live map, snapshots and `run_events.jsonl` are written to `maps/`.

## Files

| file | role |
|---|---|
| `mak_02_controller.py` | `NavigationController` — FSM, main loop, sensing/planning/driving orchestration |
| `local_planner.py` | arc-length pure-pursuit carrot + DWA (precise-speed policy) |
| `mapping.py` | log-odds grid, costmap, poison + **aux floating-wall** layers, scan-match |
| `depth_model.py` | depth image → thin-scan (hit/clear per column) — Maze1 maths |
| `frontier.py` / `astar.py` | frontier detection/clustering, A* + path simplify |
| `perception.py` | HSV pillar detection + green-poison projection/reflex |
| `robot_io.py` | `RobotInterface` — the only Webots-facing module (motors, lidar, cameras, IR) |
| `observability.py` | logging + JSONL event log + per-stage fault barriers (vendored) |
| `config.py` | single source of truth for all tuning constants |
| `geometry.py` / `odometry.py` | pose maths, wheel+IMU odometry |
| `selftest.py` / `dryrun.py` | Webots-free unit + closed-loop validation |
| `viz.py` | live map / costmap / path rendering |
