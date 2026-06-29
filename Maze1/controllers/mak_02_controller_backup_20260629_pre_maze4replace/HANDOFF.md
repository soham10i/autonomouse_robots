# mak_02_controller — Session Handoff

> Purpose: transfer full context to a new chat so it can continue debugging the
> SLAM + frontier-exploration controller without re-deriving everything. Read
> this top-to-bottom before changing code. Date of handoff: 2026-06-27.

---

## 1. Project (the assignment)

- Course: OTH Amberg-Weiden, **"Autonomous Robots"**, Prof. Nierhoff. Deadline **2026-07-10**.
- Robot: **Webots ROSbot** (Husarion, `Rosbot` R2025a proto) in world `Maze1.wbt`.
- Task: drive **from the blue pillar to the yellow pillar** in the shortest time.
  - **Never** cross **green poison** floor (mission failure).
  - **No** Webots Supervisor API. **Do not** modify the robot or the world.
- The maze has narrow passages and **floating walls** with 3 cases:
  1. wall top **≤ lidar plane** → robot collides (lidar may pass over it).
  2. wall **above robot height** → robot drives **under** it (must NOT be an obstacle).
  3. wall **in-between** → obstacle.
- Submission needs: code + README + **per-environment video** (visible sim time) +
  **PDF with a timing table** (start→blue, blue→yellow). **Not started yet.**
- Language: **Python** (entire codebase).
- Reference repo we ported from: `https://github.com/HanwenCao/Frontier_Exploration`
  (that repo is **gmapping** = RBPF particle filter, lidar-only, + ROS `move_base`).

### Runtime sensor specs (from real runs)
- Lidar `laser`: n=400, fov=2π (360°), range 0.2–12 m.
- Depth `camera depth`: 640×480, fov 1.04 rad, **min range 0.6 m** (blind closer than that), max 8 m.
- RGB `camera rgb`: 640×480, fov 1.04 rad.
- IMU `imu inertial_unit`. Timestep 32 ms.
- Mounts: lidar 0.10 m, camera 0.165 m, robot height 0.22 m, robot radius 0.125 m.

### How to run / test
- **Run in Webots:** set the Rosbot node's `controller` field to `mak_02_controller`.
  Press `Q` in the sim to finalise and write outputs early.
- **Webots venv (has cv2 4.13, numpy 2.5, scipy 1.18):**
  `/Users/sohampatel/OTH/Study Material/semester 4/autonomous_system/env/bin/python3`
- **Tests (stdlib unittest, no pytest):**
  `cd .../controllers/mak_02_controller && <venv python> run_tests.py`
  → currently **55 tests pass**.
- Outputs (snapshots, final_map.png, map.npz) → `mak_02_controller/maps/`.
- Live window: OpenCV (or matplotlib fallback). `MAK02_LIVE=0` disables.

### Paths
- Controller root: `/Users/sohampatel/OTH/Study Material/semester 4/autonomous_system/Maze1/controllers/mak_02_controller/`
- ⚠️ Note: the git repo `cwd` for this session was actually the **`brats-uad`** project
  (a *different* OTH project); the Maze1 controller lives at the absolute path above.
  Don't confuse the two. Persistent memory file:
  `~/.claude/.../memory/maze1-mak02-slam-controller.md`.

---

## 2. Architecture (what each module does)

Pure-algorithm modules have **no Webots import** (so they're unit-testable);
only `hardware/robot_io.py` imports the Webots `controller` module.
`settings.py` is the single source of truth (also Webots-free).

```
mak_02_controller.py      Main FSM + main loop (the orchestrator)
settings.py               ALL tunable constants
geometry.py               pose compose/relative/transform/wrap_angle
hardware/robot_io.py      ONLY Webots-facing module (devices, step, cmd)
slam/
  odometry.py             wheel deltas for translation, IMU yaw for heading
  occupancy_grid.py       log-odds grid + aux (depth) layer + poison + costmap
  likelihood_field.py     distance-transform Gaussian field for scan matching
  scan_matcher.py         correlative coarse->fine matcher over the field
mapping/
  lidar_model.py          ranges -> body-frame points (Webots angle convention)
  depth_model.py          depth image -> THIN virtual scan of floating-wall pts
exploration/
  frontier.py             frontier detection/clustering
  goal_selection.py       utility (info_gain/dist) goal pick
  astar.py                A* on costmap + Planner wrapper
perception/
  color_detector.py       numpy HSV (no cv2) pillar detect + green poison proj
control/
  diff_drive.py           kinematics
  pure_pursuit.py         path tracker (chooses the carrot)
  local_planner.py        DWAPlanner (move_base local-planner equivalent)
viz.py / live_view.py     snapshot/final map / live OpenCV window
tests/                    55 stdlib unittest tests; run via run_tests.py
```

### Pipeline per tick (in `Mak02Controller.run`)
1. `_odom_and_predict` — wheel+IMU odometry → predicted pose (IMU yaw trusted).
2. `_read_scan` — lidar → `self.scan_body` (robot frame).
3. `_slam_step` — if moved enough: scan-match predicted pose vs likelihood field
   → corrected pose → `integrate_scan` (lidar) → `_integrate_depth` (aux) →
   **`reconcile_aux_with_walls`** → **`decay_aux_where_free`** → rebuild field.
4. `_perception_step` — RGB pillar detect (running mean) + green poison projection.
5. `_fsm` — mission state machine → (v, w). First line each tick:
   `mark_free_disc(robot pose, ROBOT_RADIUS*0.85)` to keep own cell plannable.
6. `set_cmd`, `_virtual_bumper`, snapshots, live view, logging.

### FSM
`INIT_SCAN → EXPLORE_BLUE → GO_BLUE → EXPLORE_YELLOW → GO_YELLOW → DONE`
with `RECOVERY` reachable from any driving state on "stuck".
- In an EXPLORE state the goal is a **FRONTIER** (chosen by **A\* path length**,
  so unreachable frontiers are filtered out), **NOT** the pillar.
- `_maybe_go(name, go_state)` switches EXPLORE→GO **only if** the pillar is known
  AND an A\* path to it currently exists (else sets a `GO_FAIL_COOLDOWN_S` and keeps
  exploring — this killed an earlier EXPLORE↔GO flip-flop).
- Navigation: pure-pursuit picks a carrot on the global A\* path; **DWA** drives to
  it using live lidar + mapped aux/poison (obstacle rejection + corridor centring).

---

## 3. Debugging history (chronological — what was tried and why)

1. **Old controllers were faulty** (pure-odometry drift → thick walls, purple
   blobs). User asked to rebuild from scratch → `mak_02_controller`.
2. **Wall clipping** (bare pure-pursuit, no avoidance) → added **DWA local planner**
   + live OpenCV window.
3. **Floating-wall wedging** (lidar can't see them, depth blind <0.6 m) → **sticky
   aux** (`AUX_STICKY_HITS`), **virtual bumper** (stamp obstacle ahead on no-progress),
   DWA also avoids mapped aux/poison.
4. **EXPLORE↔GO flip-flop** (pillar known but unreachable) → `_maybe_go` requires a
   real A\* path + `GO_FAIL_COOLDOWN_S`; aligned clearances (`LP_SAFE_RADIUS` 0.155→0.10).
5. **Permanent freeze in a corner** → aux only marks lidar-free cells; `_stuck_check`
   added to the no-frontier branch; DWA proximity slowdown; `W_MAX` 2.0→1.6.
6. **Round "purple thickening / poor map"** → `DepthModel.aux_points_world` rewritten
   from "dump every in-band pixel" to a **`depthimage_to_laserscan`-style THIN scan**
   (nearest in-band obstacle per image column). `SM_SIGMA_M` 0.08→0.06. Added
   `sm=`/`aux=` diagnostics to the log.
7. **Round 2 (corner bulge / "purple once placed is never replaced" — user nailed it)**:
   the aux layer **marks but never clears** (ROS `costmap_2d` needs both; clearing =
   raytracing). Added **`reconcile_aux_with_walls`** (clear aux near lidar walls, even
   sticky) + **rear-aware turning** (`_rear_blocked`, `_spin_to_freer_side`, DWA boxed
   spin toward freer side).
8. **Round 3** — aux exploded to 518 + boxed-in recovery loop. Fixed: `mark_free_disc`
   clears sticky aux in footprint; `AUX_WALL_CLEAR_RADIUS` 0.08→0.15; `clear_aux_disc`
   escape hatch on repeated recovery. RESULT: aux now bounded (~150–300), blue detected.
9. **Round 4** — read the world file; found blue behind floating `WallMedium(3)`; wired IR
   sensors (confirmed names `fl_range/fr_range/rl_range/rr_range`) + first dead-end handling.
10. **Round 5** — fixed the GO↔RECOVERY loop (stuck-GO returns to EXPLORE), added the permanent
    barrier layer (perpendicular seal), and fed live depth into the DWA.
11. **Round 6** — live-depth-into-DWA unfiltered choked narrow passages; went lidar-only for the
    map; that REMOVED floating-wall info from A* → robot drove under floating walls + dead-end
    stamped huge barriers (bar→107) → walled itself off. Bad round.
12. **Round 7** — MAJOR REFACTOR: proper ROS depth-obstacle layer (mark + raytrace clear),
    ripped out the aux/barrier/dead-end hacks.
13. **Round 8** — depth layer worked (BLUE REACHED) but over-marked normal walls (dobs~1000) and
    erased low slabs in the blind zone on approach. Fixed both.
14. **Round 9** — removed the omnidirectional proximity-slowdown crawl + relaxed rear-block.
15. **Round 10** — analyzed `map.npz`: map GOOD, pillars REACHABLE, robot reached blue but
    livelocked (arrival only checked in GO). Added EXPLORE-state reach check.
16. **Round 11 (THIS handoff) — see §4i.** Best run: full speed + BLUE reached + into GO_YELLOW.
    Explained the depth 0.6 m blind-zone limit; rewrote recovery to reverse-then-rotate-to-gap.

---

## 4. Latest run analysis + fixes (the current frontier of work)

### What the last run (≈270 s) showed
- `sm=0.92–1.00` the **entire** run ⇒ **SLAM/scan-matching is PERFECT**; drift is NOT
  the problem and the lidar (black) walls are clean.
- `aux=` exploded: `15 → 55 → 112 → 229 → 444 → 518` cells. **The purple layer is the
  problem** — it paints over structure and walls off passages.
- t≈220→270 s: **permanent recovery loop** at ~(1.8, 1.9), `v=+0.00`, just rotating.
  Robot **boxed in by false aux**. Never detected blue (`blue=False` all run) because it
  got boxed before reaching the eastern passages where the blue pillar is.

### Root causes found
1. **The purple is the chassis-height band of NORMAL full-height walls.** Every wall
   has material in [0.04, 0.22] m, so the depth cam paints it; points land in the FREE
   cells just in front of the lidar line (depth noise + oblique viewing), accumulate
   past the sticky threshold, become permanent. Reconcile radius (0.08 m) was too small
   to catch that shadow.
2. **The robot couldn't erase its own box:** `mark_free_disc` (every tick, at the robot's
   own pose) was *preserving* sticky aux → boxed robot can never erode the box → infinite
   spin.

### Fixes applied this round (all committed to files, 55 tests pass)
1. **`mark_free_disc` now clears ALL aux incl. sticky** in the robot footprint (the robot
   is physically there ⇒ provably free). Bumper stamps are placed *ahead*
   (`BUMPER_MARK_AHEAD` > footprint radius) so genuine hits survive. Poison still preserved.
2. **`AUX_WALL_CLEAR_RADIUS` 0.08 → 0.15 m** so `reconcile_aux_with_walls` actually wipes
   the wall-near-face shadow (the explosion source). Genuine floating walls are >0.15 m
   from any lidar wall and survive.
3. **Escape hatch:** new `OccupancyGrid.clear_aux_disc(wx, wy, r)`; after
   `RECOVERY_MAX_CHAIN` recoveries, `_do_recovery` wipes aux in `RECOVERY_ESCAPE_CLEAR_R=0.25` m
   around the (freely-spinning ⇒ navigable) robot and logs `[recovery] escape: cleared N aux cells`.
   `recovery_chain` now also resets on real progress in `_stuck_check`.

New tests: `test_reconcile_clears_sticky_aux_shadowing_a_wall`,
`test_reconcile_keeps_isolated_floating_wall`,
`test_free_disc_clears_sticky_aux_in_footprint`, `test_clear_aux_disc_removes_sticky`.

### Clarifications given to the user
- **The A\* "destination switching" is expected**, not a bug: in EXPLORE_BLUE the goal is
  a frontier (A\*-path-length ranked), not the pillar. The yellow marker on the map is just
  the detected yellow pillar drawn for reference.
- Mission order is correct: must reach **blue first**, then yellow (yellow was detected early
  at t≈29 s but is intentionally ignored until blue is reached).

---

## 4b. Round 4 — world ground truth + IR + dead-end (latest)

Read `worlds/Maze1.wbt` (the source of truth — do this before guessing geometry).
**Controller frame ≈ world shifted by start: `world ≈ controller_xy + (-0.68, 1.24)`**
(robot start translation `-0.679799 1.23818`, heading IMU-seeded ≈ 1.512 rad).

Key world facts:
- **Blue pillar** `BlueCylinder` at world **(1.28, 0.83)**; **Yellow** at **(-0.03, 0.31)**;
  **Poison** box at world **(-0.05, 0.77)**, size 0.4×0.5.
- **Blue is behind `WallMedium(3)`** — a **floating wall**: `translation 1.12 1.16 0.45`,
  size `1×0.05×0.5` ⇒ spans **z 0.20→0.70**. Bottom edge 0.20 m, robot is 0.22 m tall ⇒
  **can't pass under**; depth cam sees blue through the 0–0.20 under-gap; lidar passes under
  and marks cells free ⇒ A* routes at blue ⇒ GO fails ⇒ flip-flop/wedge. THIS was the blocker.
- Floating/low walls to know: `WallMedium(5)` spans 0.25→0.75 (bottom 0.25 > 0.22 ⇒ correct
  drive-under); `WallMedium(24)` rotated about X ⇒ very low (~0.05–0.105 m, near lidar plane,
  collision case); `WallShort(5)` & `WallMedium(9)` are tilted (z-size 0.3).
- Narrow-passage corner the user wants smooth entry into: `WallMedium(8)` @ (1.40, 2.52) horiz
  + `WallMedium(12)` @ (2.27, 2.67) vert (NE region). Close-clip spots: `WallMedium(4)`
  @ (0.09, 1.92), `WallMedium(6)` @ (-0.64, 2.32).

Fixes applied:
1. **IR/ToF wired** (`robot_io.read_ranges`, `settings.RANGE_SENSOR_*`): 4 VL53L0X to fill the
   lidar's <0.2 m blind zone (close-corner clipping). Front IR → `_ir_points_body` →
   `_local_obstacles_body` (DWA); rear IR → `_rear_blocked`. **A full device dump is printed
   at init** (`[robot_io] ALL devices: [...]`) — read it from the next run to CONFIRM the IR
   names; `RANGE_SENSOR_POSES` are best-guess ROSbot mounts, tune if behaviour disagrees.
2. **Dead-end handling** (`_register_go_failure`): per GO episode, measure net approach
   (`_go_entry_dist`); after `GO_DEADEND_FAILS=3` attempts with `<GO_DEADEND_PROGRESS_M=0.25 m`
   progress, stamp a sticky aux barrier toward the pillar (A* reroutes), blacklist the
   approach, and suppress GO for `GO_DEADEND_COOLDOWN_S=25 s` so it explores for a route in
   from another side instead of ramming the floating wall.

## 4c. Round 5 — break the GO↔RECOVERY loop + proactive depth (latest)

Run showed: IR working (device dump confirms `fl_range/fr_range/rl_range/rr_range`), robot
navigates corners/narrow passages well, reaches the blue-detection point — then **looped
GO_BLUE↔RECOVERY for ~90 s** at floating `WallMedium(3)`, never reaching blue, never exploring
elsewhere. Three bugs, all fixed:
1. **Loop:** stuck-in-GO set `return_state=GO`, so recovery went GO→RECOVERY→GO and never hit
   EXPLORE, where the 25 s dead-end GO-suppression is checked. Now `_do_go` calls
   `_stuck_check(explore)` → a stuck GO returns to EXPLORE → suppression applies → it explores.
2. **Barrier erased:** the escape `clear_aux_disc(0.25 m)` wiped the dead-end barrier (stamped
   0.225 m ahead, was aux-based).
3. **Too small:** one aux disc can't seal a 1 m floating wall.

Fix: new **permanent `OccupancyGrid.barrier`** bool layer (in `lethal_mask`, `costmap` obs, DWA
`_local_obstacles_body`, viz orange-red, saved in npz) that **no aux clear touches**. The
dead-end now stamps a **perpendicular barrier LINE** (`mark_barrier_disc` at
`GO_DEADEND_BARRIER_OFFSETS = (-0.2,-0.1,0,0.1,0.2)`) sealing ~0.6 m.

**Proactive depth avoidance** (the user's explicit ask): a floating wall's depth marks are
continuously erased by `decay_aux_where_free` (the lidar passes under → cell reads free → decay
fires before the mark goes sticky). So the **live depth thin-scan** (`_last_aux_world`, cached
each `_integrate_depth`) is now fed straight into the DWA `_local_obstacles_body`, so the robot
refuses to drive under a floating wall it can currently see — independent of the persistent map.

## 4d. Round 6 — lidar-only map + fix narrow-passage regression (latest)

Run showed: robot navigates but **couldn't enter a narrow passage, collided corners often,
map full of scattered purple, still no blue after 350 s**. Causes + fixes:
1. **Regression:** Round 5's live-depth-into-DWA was UNFILTERED — it added depth points for
   NORMAL walls (chassis band, drift-offset), choking narrow passages. Fixed:
   `_live_floating_obstacles_body()` keeps only **lidar-blind** depth points (no occupied cell
   within `LIVE_FLOATING_BLIND_CELLS=2`) → normal walls aren't double-counted.
2. **Map clutter / bad A*:** persistent depth aux was the purple. Now **`MAP_DEPTH_AUX_TO_GRID
   = False` → the map is LIDAR-ONLY** (clean thin walls, proper costmap). Depth is used only for
   live floating-wall avoidance + learned barriers. `aux=` should be ~0 now.
3. **False far barrier:** dead-end stamped a barrier far from blue. Now gated by
   `GO_DEADEND_MAX_DIST = 1.3 m` (only stamp when stalled close to the pillar).
4. **Bumper now stamps a permanent `barrier`** (not aux, which no longer persists).
5. Log gained `bar=` (barrier cell count); the brown/orange patch on the map = a learned barrier.

Open question for the next chat: the robot still hasn't reached blue. Need a **final_map.png**
(or `map.npz`) to see the full explored area and blue's real ground opening. Blue is at world
(1.28,0.83), enclosed by `WallShort(5,6,7,10,11,13)` + floating `WallMedium(3)` on the north;
the ground opening is most likely on the **south or east** side — may require analyzing the
wall geometry from `worlds/Maze1.wbt` and/or biasing exploration toward it.

## 4e. Round 7 — proper depth-obstacle layer (MAJOR refactor, latest)

The recurring root cause, stated once: **the lidar reports a floating wall's cell FREE (it
passes underneath) while the depth cam reports it OCCUPIED, and the controller never held that
contradiction correctly** — so each round added a hack (sticky aux → reconcile → decay →
barriers → dead-end) that fought the others. Round 6's lidar-only map made A* blind to floating
walls → it planned *under* them → stuck → dead-end stamped ~70-cell barriers (`bar`→107) →
walled the robot off.

**Fix = the standard ROS `costmap_2d` obstacle-layer / `depthimage_to_laserscan` pattern:**
- New **`OccupancyGrid.depth_L`** — a SEPARATE log-odds grid written ONLY by the depth camera,
  via **`integrate_depth_rays(pose, hits, free_ends)`**: mark in-band obstacle endpoints AND
  **raytrace-clear** the robot-height tunnel along bearings that are clear. The lidar NEVER
  writes it (so floating walls it can't see persist); depth's own clearing erases stale/false
  marks (self-correcting). `depth_obs_mask()` is unioned into `lethal_mask`/`costmap`/DWA.
- New **`DepthModel.depth_rays_world(depth, pose, clear_range)`** → `(hits, free_ends)`, the
  depthimage_to_laserscan split (one range per column, obstacle vs clear).
- **Removed from the pipeline:** `mark_aux_points`, `reconcile_aux_with_walls`,
  `decay_aux_where_free`, the dead-end perpendicular barrier stamp (dead-end is now just
  cooldown + blacklist, NO permanent wall), and the recovery `clear_aux_disc` escape. The
  `aux_*` methods still exist + are unit-tested but are no longer called. The bumper still
  stamps a small permanent `barrier` for truly-blind (<0.6 m) / tilted walls.
- Settings: `DEPTH_L_OCC=0.70`, `DEPTH_L_FREE=-0.50`, `DEPTH_OCC_THRESH=0.50`,
  `DEPTH_CLEAR_RANGE=1.6`. Log: `aux=` → `dobs=` (depth-obstacle cells). viz purple = depth_obs.
- 61 tests pass (added: depth-layer mark / raytrace-clear / lidar-independence;
  depth_rays in-band-hit / high-wall-clear).

Expected next run: clean thin map (purple only on genuine floating walls), `dobs` small &
stable, `bar` ≈ 0, A* routes AROUND floating walls. If blue still isn't reached, the blocker is
exploration finding blue's ground opening — get `final_map.png`. User-named stuck spots to watch:
`WallMedium(22)` (bottom corner), `WallMedium(4)` (narrow corner to blue), `WallMedium(8)` (NE
narrow passage), tilted floating `WallShort(5)` + `WallMedium(9)` (z=0.3 — robot rides up them).

## 4f. Round 8 — two depth-layer bug fixes (latest)

Round 7's depth layer **worked — the robot reached BLUE at t=152.7 s** — but the run exposed two
genuine bugs in `OccupancyGrid.integrate_depth_rays`:

1. **Over-marking** (`dobs` 65 while stationary → ~1000): the depth layer copied every NORMAL
   wall's chassis band (the lidar already maps those). **Fix:** mark a depth hit only where the
   lidar is NOT already a wall (`DEPTH_MARK_NEAR_WALL_CELLS = 2` dilation of `occupied_mask`), so
   `depth_obs` holds ONLY the lidar-blind floating/low walls. → small `dobs`, clean map.
2. **Blind-zone erasure** (the `WallMedium(9)` failure): that wall's diagonal rotation makes its
   0.05 m thickness vertical → it's a **low horizontal slab at z≈0.155–0.205 m**. The depth marks
   it when far, but as the robot closes it enters the **0.6 m depth blind zone**, the camera sees
   *past* it (clear), and the raytrace clearing erased the mark right before collision. **Fix:**
   `_free_cells_along_rays(min_dist=DEPTH_OBS_MIN_RANGE=0.6)` — never clear inside the blind zone,
   so close low slabs stay marked. Bumper + IR remain the last-resort for the truly-blind contact.

63 tests pass (added: depth double-mark-skip; blind-zone mark not cleared).

Expected next: `dobs` small (tens), tilted/low slabs (`WallMedium(9)`, `WallShort(5)`) get
purpled and avoided (no more riding up "upside down"), blue reached faster, then
EXPLORE_YELLOW → GO_YELLOW → DONE should complete (yellow already known ~t=29). After that the
remaining work is corner centering polish and the **submission video + PDF timing table** (not
started; deadline 2026-07-10).

## 4g. Round 9 — the real recurring bug was the LOCAL PLANNER (latest)

Round 8's depth fixes worked: `dobs` is now 38–115, the map is clean, the depth sensor is no
longer the problem. The log then made the TRUE recurring failure unmistakable: the robot **pinned
at one spot (≈1.0, −0.77) for 40 s** (t=53–94), tiny `v` + flip-flop `w`, never translating —
a local-planner / recovery trap, not mapping. Confirmed against move_base's known failure modes
(stuck in costmap → repeats trajectory → loops recovery).

Two root causes in the control path:
1. **Omnidirectional proximity slowdown → permanent crawl.** `DWAPlanner.compute` capped forward
   speed by the *nearest obstacle in any direction*; in a corridor the side walls (~0.12 m) made
   `v_cap ≈ V_MIN`, so it crawled down every passage and the stuck-watchdog
   (`STUCK_PROGRESS_MIN_M=0.08` / `3 s`) fired on the lack of progress. **Fix:** removed the
   global speed cap entirely — speed is now set by the standard DWA **per-trajectory clearance
   rejection** (a fast trajectory that approaches an obstacle is rejected, so it slows only for
   things in its ACTUAL path and runs full speed when clear).
2. **Never reversed → couldn't escape traps.** `RECOVERY_REAR_MIN_CLEAR=0.25` meant the rear was
   "blocked" in any clutter, so recovery only spun in place and returned to the same trap.
   **Fix:** `RECOVERY_REAR_MIN_CLEAR` 0.25→0.15 (just past the robot radius), `REAR_HALF_WIDTH`
   0.18→0.16, so the robot can actually back out.

64 tests pass (added `test_tight_corridor_does_not_crawl`). If flip-flop `w` at junctions
persists, bump `LP_W_STRAIGHT`. Still want `final_map.png` to confirm blue's ground opening.

## 4h. Round 10 — the livelock-on-pillar bug, found in map.npz (latest)

Diagnosed from the SAVED grid (`maps/map.npz`), not speculation — see `/tmp/diag*.py` (load the
npz, rebuild the costmap, run A*). Findings:
- **Map is good; both pillars reachable.** Rebuilding `OccupancyGrid.costmap()` from the saved
  masks: `A* start→blue` FOUND (len 148), `A* start→yellow` FOUND (len 109); start/blue/yellow all
  snap into the same free connected component. Mapping + planning are NOT the failure.
- **Robot reached blue** (min 0.08 m at ~t=142 s) but then **livelocked for 34 s in a 0.4 m band
  right next to blue** (heading frozen ≈ −89°, oscillating ±y), never moving toward yellow.

**Root cause:** pillar arrival (`_on_pillar_reached`) was only checked inside `GO_BLUE`/`GO_YELLOW`
(`_do_go`). But the robot arrives at blue while in **`EXPLORE_BLUE`**, chasing a frontier *beyond*
blue — so it drives onto the pillar, the frontier pulls it forward, it backs off, repeats: a limit
cycle on top of the pillar it already reached. That's why every run "couldn't reach" a pillar — it
physically did, but the FSM never registered it.

**Fix (one targeted change):** `_fsm` now calls `_pillar_in_reach(name)` in the EXPLORE states too
(blue in `EXPLORE_BLUE`, yellow in `EXPLORE_YELLOW`, mission order preserved) → `_on_pillar_reached`
fires the instant the robot is within `PILLAR_REACH_DIST` (0.45 m) of the target, in any seeking
state. 64 tests pass.

Expected next: blue registers as reached when within 0.45 m (no livelock) → `EXPLORE_YELLOW` →
yellow reached → DONE + timing table. Secondary/lower priority: ~23 % stalled frames and slow
exploration (136 s to first reach blue) — local-execution polish. Then the submission deliverables.

## 4i. Round 11 — depth blind-zone truth + recovery rewrite (latest)

Best run so far: `v=0.32` in corridors (round 9), `[mission] BLUE reached t=136 s` (round 10),
then `GO_YELLOW` — the mission now executes. Two remaining issues:

- **"Floating walls not detected by depth."** They ARE (`dobs` 200–350; A* avoids the mapped
  ones). But the depth camera's **min range is 0.6 m** (hardware spec; can't change — no robot
  mods allowed), and the lidar + the 4 IR ToF sensors all sit at ~0.10 m, *below* a floating
  wall. So a floating wall in the 0.10–0.22 m band first encountered **within 0.6 m** is invisible
  to every sensor until contact → the `[bumper]` is the only fallback there. Depth maps them fine
  from >0.6 m. This is a sensor-geometry limit, not a missing code path.
- **Recovery wobble.** Rewrote `_do_recovery` to: **reverse straight out** (skip only if the rear
  is HARD-blocked < `RECOVERY_REAR_HARD=0.12 m`, via `_rear_hard_blocked`), **then rotate toward
  the widest lidar gap** (new pure fn `control.local_planner.widest_gap_sign` → +1 left / −1 right
  toward the more-open side). The spin direction is committed once per recovery (no flip-flop).
  Removed the old `_rear_blocked` / `_spin_to_freer_side`.

67 tests pass (added `widest_gap_sign` open-left/right/empty).

## 5. What to do next (for the continuing chat)

**The immediate next step is to get a NEW Webots run + log + live screenshot from the user**
and read these signals:

- **`aux=` in the periodic log is THE number to watch.** Last run it ran away to 518. It
  should now stay roughly **bounded (tens, not hundreds)**. If it still climbs ⇒ the
  reconcile/clearing isn't enough.
- `sm=` should stay ~0.9–1.0 with no `!` (matcher healthy). It already does.
- Look for `[recovery] escape: cleared N aux cells` — confirms the box-breaker fired.
- Goal: robot explores the **east** side, **detects blue** (`blue=True`), switches to
  **GO_BLUE**, reaches it, then EXPLORE_YELLOW → GO_YELLOW → DONE, printing the timing table.

**If `aux=` still grows / passages still block ⇒ implement the proper fix that was deferred:**
- **Depth raytrace-clearing** (the full ROS `costmap_2d` clearing model). For each depth
  image column (bearing): if there's NO in-band obstacle, raytrace-CLEAR aux along that
  bearing out to `AUX_MAX_RANGE`; if the nearest in-band hit is at range r, clear aux from
  `depth_min` to `r - margin`. This makes the aux layer self-correcting and removes the need
  for the magic `AUX_WALL_CLEAR_RADIUS`. Implement as: `DepthModel` returns per-bearing
  (free / hit-range) info; `OccupancyGrid.clear_aux_along_rays(pose, bearings, ranges)`.

**Other known follow-ups (lower priority):**
- Speed profile: user wants fast straight / medium in corners (currently proximity-slowdown
  crawls). If too slow, raise the slowdown floor (`LP_SLOWDOWN_DIST` / the `frac` floor in
  `DWAPlanner.compute`). Deferred until navigation is reliable.
- Eventually: produce the **submission video + PDF timing table** (not started).

**Working style the user insists on:** find the ROOT cause and fix it properly — **no
temporary patches with bugs**. Reference the GitHub repo / standard ROS practice. The user
runs each build in Webots and returns logs + live screenshots + map images for the next iteration.

---

## 6. Key current settings (see `settings.py` for the rest)

```
# velocity
V_MAX=0.32  V_MIN=0.04  W_MAX=1.6
# grid
GRID_RESOLUTION=0.04  GRID_CELLS=200  GRID_ORIGIN=(-4,-4)
# log-odds
L_OCC=0.65  L_FREE=-0.45  L_OCC_THRESH=0.50  L_FREE_THRESH=-0.40
# aux (depth floating-wall layer)
AUX_Z_MIN=0.04  AUX_Z_MAX=ROBOT_HEIGHT(0.22)  AUX_MAX_RANGE=1.5
AUX_MIN_HITS=3  AUX_STICKY_HITS=6  AUX_CAP=12
AUX_WALL_CLEAR_RADIUS=0.15            # <- bumped this round
# scan matching
SM_SIGMA_M=0.06  SM_MIN_SCORE_FRAC=0.30  SM_UPDATE_LIN_M=0.05  SM_UPDATE_ANG_RAD=5deg
# costmap / planning
HARD_OBS_MARGIN=0.025 (=> A* hard clearance 0.10)  SOFT_OBS_HALO=0.12
# DWA
LP_SAFE_RADIUS=0.10  LP_W_CLEAR=0.9  LP_W_STRAIGHT=0.15  LP_SLOWDOWN_DIST=0.45
LP_MAPPED_OBS_RADIUS=1.5
# recovery
STUCK_TIMEOUT_S=3.0  RECOVERY_REVERSE_V=0.20  RECOVERY_REVERSE_T=1.5
RECOVERY_MAX_CHAIN=3  RECOVERY_ESCAPE_CLEAR_R=0.25   # <- new this round
RECOVERY_REAR_MIN_CLEAR=0.25  RECOVERY_REAR_HALF_WIDTH=0.18   # <- rear-aware turning
# bumper
BUMPER_STALL_TIME_S=0.6  BUMPER_MARK_AHEAD=ROBOT_RADIUS+0.04  BUMPER_MARK_RADIUS=0.06
```

---

## 7. Mental model / gotchas

- **gmapping vs us:** the reference repo is a particle filter, lidar-only, no depth. We use
  a **single-hypothesis** scan matcher + IMU heading + a **thin depth scan** for the floating
  walls the maze needs (which gmapping wouldn't even detect). We are NOT implementing a
  particle filter.
- **The aux layer is the recurring villain.** It exists for floating walls but keeps
  over-painting normal walls. Every fix so far has been about *bounding* it (mark only
  lidar-free cells, thin scan, reconcile near walls, clear footprint/escape). The clean
  end-state is depth raytrace-clearing (see §5).
- The lidar SLAM itself is solid — don't "fix" the scan matcher; `sm` proves it works.
- Index convention in the grid: `L[ix, iy]` — axis 0 = x (column), axis 1 = y (row).
- Floating-wall height classification is correct: depth points with height in
  [AUX_Z_MIN, AUX_Z_MAX(=ROBOT_HEIGHT)] → obstacle; above ROBOT_HEIGHT → drive under.
