# mak_02_controller — Maze2

Webots ROSbot controller for the OTH "Autonomous Robots" Modularbeit: drive from
the start to the **BLUE** pillar, then to the **YELLOW** pillar, in the least
simulation time, **without touching a wall or the green poison floor**. No
Supervisor; no edits to the robot or world.

It is the proven Maze4 stack (lidar SLAM → frontier exploration → A* → DWA, a
Python port of the move_base/gmapping pattern of
[Frontier_Exploration](https://github.com/HanwenCao/Frontier_Exploration)),
re-tuned for Maze2. Only the geometry/resolution constants in `config.py` differ
— the algorithms are unchanged. Two adaptations matter for Maze2:

* **High-resolution SLAM.** `GRID_RESOLUTION` is `0.025 m` (vs Maze4's `0.04 m`)
  on a `9.0 m` square grid (360 × 360 cells) centred on the start — sharper walls
  and crisper floating-/tilted-wall footprints for cleaner A* planning.
* **Floating-wall avoidance** (designed in `Maze1/controllers/mak_02_controller/`),
  because Maze2's blue approach is guarded by a low/floating/tilted wall cluster
  the single-plane 2-D lidar partially or fully misses.

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

* **Floating-wall avoidance (persistent depth-obstacle layer).** A 2-D lidar at
  z=0.10 m is blind to walls off its plane and marks their footprint *free*; the
  depth camera sees them but only beyond its 0.6 m min range. So there is a
  **dead shell < 0.6 m** where a floating wall is invisible to every sensor — the
  only defence is to have *remembered* it. The depth camera densely back-projects
  every pixel in the collision band `[AUX_Z_MIN, AUX_Z_MAX=ROBOT_HEIGHT]`
  (`depth_model.dense_obstacles`); each hit **accumulates per-cell confidence**
  where the lidar is blind (`mapping.integrate_depth_obstacles`). A cell is lethal
  **aux** at `AUX_MIN_HITS`; a clear sight-line only **decays** confidence, and the
  decay ray starts at the depth min range so a wall inside the dead shell is
  **never forgotten on approach**. The lidar-blind gate stops normal full-height
  walls being duplicated (the anti-smear rule); confidence caps at `AUX_HIT_CAP`
  so a confirmed wall rides out the few stray clear frames of the dead-zone
  approach. Aux cells feed A* (lethal, like a wall) and the DWA obstacle set.
  Panels taller than `ROBOT_HEIGHT` (Maze2's z≥0.25 pass-under bridges) stay
  drivable-under and are never marked.

---

## Maze2 facts that drove the tuning

Read directly from `worlds/Maze2.wbt`. Controller frame = world translated by the
robot start (axes aligned, heading IMU-seeded), so `ctrl = world − start` with
`start = (-0.9552, 0.3878)`. **No** world coordinate is hardcoded in the runtime
path — the pillars are discovered by vision; these are documentation.

| object | world | ctrl frame | note |
|---|---|---|---|
| robot start | (-0.9552, 0.3878), yaw +1.2416 rad | (0, 0) | IMU-seeded heading |
| BLUE pillar | (1.49, -1.77) | (2.45, -2.16) | reach **first**, ~3.26 m, SE |
| YELLOW pillar | (-0.93, 0.99) | (0.03, 0.60) | reach **second**, ~0.60 m N; time stops |
| poison ×2 | (2.37,-1.73), (1.52,-0.92) | — | both on the blue approach |

Pillars are explicit **height 0.4 m**, z=0.2, radius 0.1 → ground-based and
lidar-visible (`PILLAR_HEIGHT = 0.40` feeds the vision range estimate; this
differs from Maze4's 0.30).

**Lidar-blind hazards on the SE blue approach** (the hard part — same character
as Maze4/Maze1):

| wall | world | z-span | hazard |
|---|---|---|---|
| WallShort(18) | (2.73, -0.70) | [0.20, 0.70] | **floating**: lidar passes under (0.10 < 0.20), robot (0.22) would clip |
| WallShort(17/14) | (1.89,-1.10)/(2.21,-1.28) | body band | **tumbled** off-axis panels — lidar sees a sliver or misses |
| tilted panel | (0.95, -1.28) | ~ -0.01 | near-floor tilted panel |
| bridges ×2 | (1.73,-0.64)/(1.73,-0.50) | [0.25, 0.75] | **pass-under** — excluded by the `[AUX_Z_MIN, 0.22]` band |

`AUX_Z_MIN` is raised to `0.03` so the depth-aux layer catches the body-band
hits while staying above the z=-0.049 poison decals and floor noise.

---

## Centreline alignment (tight gaps)

The exact ROSbot footprint (from the proto) is what every clearance is matched to:

| robot | value | source |
|---|---|---|
| passage half-width | **0.116 m** (0.232 m wide) | wheels: anchor y=±0.110, tyre cylinder height 0.035 → outer edge 0.116 |
| circumscribed radius | **0.128 m** | rear-wheel corner |

The maze's wall gaps leave only a few cm of slack per side, so the robot **must
ride the passage centreline**. Three things make it do so:

1. **Footprint-accurate inflation.** `HARD_OBS_DIST` equals the true 0.116 m
   half-width (rounds to 5 cells = 0.125 m at the 0.025 grid), so A* never plans
   the wheels into a wall, while a ~0.39 m gap keeps a multi-cell free band down
   its centre.
2. **Center-seeking costmap.** Cost decreases smoothly with clearance out to
   `CENTER_PREF_RANGE`, so A* rides the **medial axis** of every corridor;
   `astar.simplify` is clearance-aware (it only shortcuts through genuinely open
   cells, never straightening a centred path back against a tight wall).
3. **Strong DWA centring + precise tight-gap speed.** `DWA_SAFE_RADIUS` matches
   the footprint, the clearance/centring weight is raised, and a single floored
   tight-gap speed factor eases the robot to ~0.16–0.26 m/s only where a side wall
   is genuinely close — precise, never a crawl.

> **Drive format.** The ROSbot is 4-wheel skid-steer but is commanded as a
> **2-wheel differential drive** — `geometry.cmd_to_wheels` maps `(v, w)` to a
> left pair and a right pair. No robot edit is needed or allowed; centreline
> precision comes from the planner, not a drivetrain change.

---

## Validate without Webots

```bash
python3 selftest.py   # 38 unit checks: mapping, scan-match, A*, frontier, DWA,
                      # perception, depth-aux mark/clear, IR lookup, + a
                      # closed-loop "no-crawl" regression
python3 dryrun.py     # 13 assertions / 5 closed-loop scenarios: real carrot+DWA
                      # driven through synthetic straight / 0.388 m-gap / offset /
                      # L-corner / pivot corridors; asserts the robot MOVES at
                      # cruising speed, stays clear of walls, reaches the goal
```

Both pass at the Maze2 config (use the project venv: `../../../env/bin/python3`).
Latest dry-run: all 5 scenarios reach goal at ≈0.22–0.29 m/s, min clearance
> 0.116 m, centred (max deviation ≤ 0.08 m).

---

## Run in Webots

The Rosbot node's `controller` field is set to `mak_02_controller` in
`worlds/Maze2.wbt`. **Reload the world** (not just *continue*) so the controller
process picks up the files. Press **Q** to finalise outputs (map PNG + timing
table) early. The live map and snapshots are written to `maps/`.

## Files

| file | role |
|---|---|
| `mak_02_controller.py` | FSM, main loop, sensing/planning/driving orchestration |
| `local_planner.py` | arc-length pure-pursuit carrot + DWA (precise-speed policy) |
| `mapping.py` | log-odds grid, costmap, poison + **aux floating-wall** layers, scan-match |
| `depth_model.py` | depth image → thin-scan (hit/clear per column) — Maze1 maths |
| `frontier.py` / `astar.py` | frontier detection/clustering, A* + path simplify |
| `perception.py` | HSV pillar detection + green-poison projection/reflex |
| `robot_io.py` | the only Webots-facing module (motors, lidar, cameras, IR) |
| `config.py` | single source of truth for all tuning constants (Maze2 geometry) |
| `geometry.py` / `odometry.py` | pose maths, wheel+IMU odometry |
| `selftest.py` / `dryrun.py` | Webots-free unit + closed-loop validation |
| `viz.py` | live map / costmap / path rendering |
