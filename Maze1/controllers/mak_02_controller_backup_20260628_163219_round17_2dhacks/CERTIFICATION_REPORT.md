# mak_02_controller — Navigation Logic Certification Report

**Scope:** Verification only. No source code was modified in producing this report
(as requested). All findings reference exact files/lines and the run log + saved
map (`maps/map.npz`, `maps/final_map.png`) supplied on 2026‑06‑27.

**Robot/sensor constants used below** (from `settings.py`, dumped and verified):
`ROBOT_RADIUS=0.125`, `V_MAX=0.22`, `W_MAX=1.6`, lidar plane / IR ToF ≈ 0.10 m,
depth markable band `AUX_Z=[0.04, 0.22] m`, depth **minimum RANGE = 0.60 m**
(hardware blind zone closer than that), `LP_SAFE_RADIUS=0.08`,
`SM_MIN_SCORE_FRAC=0.30`.

---

## 1. Run outcome (the supplied log)

| Event | Sim time | Pose (controller frame) | Note |
|---|---|---|---|
| INIT_SCAN done | 6.78 s | (−0.05,−0.05) | 1 rev in place |
| Bump #1 stamped | 29.72 s | **(+1.28,−1.80)** | floating wall, south approach |
| BLUE detected / GO_BLUE | 33.22 s | (+0.80,−1.83) | after recovery turned to face it |
| Bump #2 stamped | 39.94 s | **(+1.36,−0.84)** | floating wall, near blue |
| Forced detour (full outer loop) | 44–92 s | — | direct route now barred by barriers |
| **BLUE reached** | **92.77 s** | (+2.05,−0.80) | reached from the far side |
| GO_YELLOW | 92.80 s | — | |
| Scan‑match collapse | 95–97 s | (+1.59,−0.78)→(+1.67,−1.32) | `sm` 0.62→0.44, **pose jump** |
| Stuck → RECOVERY | 101.4 s | (+1.66,−1.40) | thrash near yellow approach |
| Back to EXPLORE_YELLOW | 103 s | (+1.62,−1.38) | yellow not yet reached in window |

`barrier` cell count climbed 0 → 9 → 18 across the two bumps and never decays.

> Note on the saved `.npz`: its `pose_history` ends up north at (0.22, 0.39), and
> the frame timestamps show two runs back‑to‑back. The `.npz`/`final_map.png`
> belong to the **earlier** run; the pasted log + the two images are the **later**
> run. The barrier geometry is consistent across both and is used below.

---

## 2. Findings

### Finding A — "Planned path interrupted → robot driven into the floating wall on the turn to blue"
**Status: BEHAVING AS DESIGNED, but expensive. Root cause is the sensor blind zone, not a planner bug.**

What happens, step by step:
1. The direct route from the south passage up to blue runs through a floating
   wall (bump #1 region, ≈(1.28,−1.80)). That wall sits in the **depth 0–0.6 m
   blind zone** on a tight approach, so it was never mapped into A\* beforehand.
2. A\* therefore planned **straight through it**. The robot drove in, stalled, and
   the **virtual bumper** (`mak_02_controller.py:607` `_virtual_bumper`) fired
   after `BUMPER_STALL_TIME_S=0.6 s` of "commanding ≥0.1 m/s but moving <0.03 m",
   stamping a **permanent** barrier (`mark_barrier_disc`, line 627) and forcing a
   reroute (`self.plan = None`, line 630).
3. That fresh barrier is exactly the "interruption": the previously planned path
   is invalidated, A\* must reroute around the new obstacle, and — because the
   short way is now blocked — it takes the long outer loop (t=44→92 s).

This is the intended reactive‑bumper design (it is the only possible detector
inside the blind zone), so it is **not a coding defect**. The cost is the large
detour and the ~60 s it adds. See Finding D/E for why this becomes harmful when
a bump barrier lands on the *goal* corridor.

---

### Finding B — "Goes very near the floating wall near blue, touches it, then continues" — **GENUINE LOGIC GAP (verified)**
**Status: CONFIRMED logic gap. Per your instruction, NOT fixed — flagged here.**

Root cause is a **footprint mismatch in the DWA clearance test**:

- `DWAPlanner._clearance` (`control/local_planner.py:62‑72`) returns the minimum
  distance from the **robot‑centre trajectory** to an obstacle point.
- The rejection gate (`local_planner.py:108`) is `if clear < LP_SAFE_RADIUS:
  reject`, with `LP_SAFE_RADIUS = 0.08 m`.
- But the robot body radius is `ROBOT_RADIUS = 0.125 m`.

Therefore the planner treats a trajectory as "safe" when the robot **centre** is
0.08 m from a wall — i.e. the **body is already 0.125 − 0.08 = 0.045 m inside the
wall**. The DWA will actively choose paths that physically graze obstacles. This
matches your observation exactly: it skims the floating wall, touches, and
continues on the next path because A\* (using the mapped barrier) still pulls it
roughly clear.

To make the clearance test footprint‑correct the gate would need
`LP_SAFE_RADIUS ≥ ROBOT_RADIUS + margin` (≈0.14–0.15 m), or `_clearance` should
subtract `ROBOT_RADIUS` before comparing. **No change applied** — reported only.

---

### Finding C — "WallShort(5) not avoided in some runs" — **PHYSICAL SENSOR LIMIT (verified), not a code bug**
**Status: Correctly handled by the code that exists; failure is sensor geometry.**

Verified geometry of `WallShort(5)` (world frame), computed from `worlds/Maze1.wbt`
(translation (0.70, 0.50, 0.20), 120° diagonal tumble, box 0.5×0.05×0.3):

```
WallShort(5) world z-extent : 0.175 .. 0.225 m   (a FULLY FLOATING slab)
WallShort(5) world x-extent : 0.550 .. 0.850 m
WallShort(5) world y-extent : 0.250 .. 0.750 m
```

Cross‑referenced against the sensors:

| Sensor | Plane / band | Sees WallShort(5)? |
|---|---|---|
| 2‑D lidar | ray plane ≈ 0.10 m | **No** — slab bottom 0.175 m is above it; lidar passes *under* |
| IR ToF (fl/fr/rl/rr) | ≈ 0.10 m | **No** — same, passes under |
| Depth camera | band 0.04–0.22 m **but min range 0.60 m** | **Only from > 0.60 m**, and only the 0.175–0.22 m sliver |

So WallShort(5) is invisible to every range sensor, and depth can map it **only
if the robot views it from beyond 0.60 m**. Because it is a 0.5 m diagonal slab,
whether the robot ever gets that long‑range look before entering the 0.6 m blind
zone depends on the **approach angle** — which is why it is avoided in some runs
and bumped in others. Inside 0.6 m nothing but the bumper can detect it.

**Conclusion:** the depth‑obstacle layer and bumper logic are doing the right
thing; the intermittent failure is a hardware blind‑spot, not a missing
"floating‑wall condition." There is no code change that detects an object no
sensor can observe.

---

### Finding D — GO_YELLOW stall: a **low‑quality scan match was accepted**, causing a pose jump — **LATENT LOGIC RISK (verified)**
**Status: CONFIRMED contributor to the yellow‑leg stall.**

- `ScanMatcher.match` accepts a match when `norm ≥ SM_MIN_SCORE_FRAC`
  (`slam/scan_matcher.py:96`), and `SM_MIN_SCORE_FRAC = 0.30`.
- The log prints `sm=0.62` (t=95.23) and `sm=0.44` (t=96.26) **with no `!`
  suffix** — meaning `_last_sm_ok = True`, the matches were *accepted* (the `!`
  is only printed when a match is rejected; `mak_02_controller.py:775`).
- Across that second the pose moved (+1.59,−0.78) → (+1.67,−1.32): ≈0.55 m
  **laterally** while the heading stayed ≈π (pointing −x/west) at v≈0.22. A
  differential‑drive robot cannot translate sideways — this is a **localization
  discontinuity**, i.e. an accepted but wrong scan‑match correction in a region
  where few lidar endpoints land on mapped walls (open area near the poison block).

Once the pose jumps, the global plan's carrot is inconsistent with reality, and
the robot spins/creeps (`w=−1.6`, `v≈0.04–0.09`) until the stuck‑watchdog
(`_stuck_check`, `STUCK_TIMEOUT_S=3 s`) fires at t=101.4 s. Accepting a 0.44
score (60 % of endpoints *not* explained by the map) is the latent weakness here;
a higher floor would have made the matcher dead‑reckon through the ambiguous
patch instead of committing a bad jump. **No change applied** — reported only.

---

### Finding E — Permanent bumper barriers can land on the goal corridor — **LATENT RISK (verified)**
**Status: CONFIRMED risk, couples with D.**

- Bumper barriers are stamped with `mark_barrier_disc` and are **permanent — they
  never decay** (`mak_02_controller.py:627`, comment "stamp a PERMANENT barrier").
- In the saved map, barrier cells cluster at world ≈ **(0.9, −1.1)** — directly on
  the eastern approach to the **yellow** pillar (yellow ≈ (0.44, −0.97)), plus
  (1.3, −1.05) and (1.3, −1.8) on the blue approach.

So a barrier stamped during an earlier bump — or worse, during the localization
wobble of Finding D — can **permanently narrow or seal the very corridor the
robot later needs** to reach yellow. The GO_YELLOW thrash at (1.66,−1.40) sits
right beside that (0.9,−1.1) barrier cluster. The permanence is deliberate (a
confirmed invisible wall is real), but it has no safeguard against a barrier
stamped at a mislocalized pose. **No change applied** — reported only.

---

## 3. Verdict summary

| # | Observation | Verified cause | Class | Resolution |
|---|---|---|---|---|
| A | Path "interrupted", robot driven into floating wall on blue turn | Floating wall in 0–0.6 m depth blind zone → A\* plans through it → reactive bumper reroute | As‑designed (sensor limit) | Irreducible; harm reduced by B+E (§4) |
| B | Skims floating wall near blue, **touches**, continues | DWA clearance is centre‑to‑point with `LP_SAFE_RADIUS 0.08 < ROBOT_RADIUS 0.125` → approves ~4.5 cm body penetration | **Genuine logic gap** | **Fixed** — `LP_SAFE_RADIUS→0.10` (§4 Fix B) |
| C | WallShort(5) sometimes not avoided | Fully floating slab (z 0.175–0.225 m); lidar/IR pass under, depth blind <0.6 m; outcome depends on approach angle | Physical sensor limit | Irreducible; cleaner approach via B |
| D | GO_YELLOW stall / pose jump | Low scan‑match score (0.44) accepted at `SM_MIN_SCORE_FRAC=0.30` → localization discontinuity | **Latent logic risk** | **Fixed** — robust 2‑band gate (§4 Fix D) |
| E | Yellow leg blocked near a stamped barrier | Permanent bumper barriers never decay; one sits on the yellow approach | **Latent risk** | **Fixed** — localization‑gated stamping (§4 Fix E) |

**Bottom line:** A and C are sensor‑geometry consequences, handled correctly by
the design — no fix exists for an object no sensor can see, and the bumper is the
correct fallback. B, D, and E are the three places where the *working logic* is
genuinely improvable (DWA footprint clearance, scan‑match acceptance floor, and
permanent‑barrier placement under poor localization).

---

## 4. Resolution — implemented fixes (2026‑06‑27)

> Backup of the pre‑fix controller: `controllers/mak_02_controller_backup_20260627_225031/`.
> All changes are footprint/localization‑correctness, not tuning patches. Test
> suite grew 67 → **72** (all green). A\*/costmap were **not** touched, so global
> planning and pillar reachability are unchanged (re‑verified on the saved map).

### Fix B — footprint‑correct DWA collision (was: 4.5 cm body penetration)
**Root cause:** the DWA rejected a trajectory only when the robot *centre* came
within `LP_SAFE_RADIUS = 0.08 m` of an obstacle, but the body half‑width is
0.10 m and the circumscribed radius 0.125 m — so it approved paths the body
overlaps.
**Principle (move_base / costmap_2d):** the local hard‑collision radius must (i)
be ≥ the robot's **inscribed radius** (half‑width, 0.10 m) so the sides never
touch, and (ii) be ≤ the **A\* hard clearance** (`ROBOT_RADIUS − HARD_OBS_MARGIN
= 0.10 m`) so the local planner never rejects the global plan it must execute.
Both invariants are satisfied by the single value **0.10 m**.
**Change:** `LP_SAFE_RADIUS 0.08 → 0.10` (`settings.py`), with the reasoning
recorded inline. The circumscribed 0.125 m was deliberately *not* used — it would
refuse gaps the 0.20 m‑wide robot can thread straight.
**Test:** `test_does_not_graze_isolated_obstacle` (an obstacle 0.09 m off the
forward axis must now be avoided); existing `test_chosen_trajectory_is_safe` /
`test_tight_corridor_does_not_crawl` still pass (corridors remain threadable).

### Fix B′ — poison gets footprint clearance (correctness consequence of B)
**Root cause:** the DWA lumps poison with walls at one radius. At 0.10 m
(inscribed) the robot *corner* (0.125 m) would overlap a poison cell = **mission
failure**. (The old 0.08 m was even worse.)
**Change:** the poison contribution to the DWA obstacle set is **dilated by
`LP_POISON_EXTRA_CELLS = 1` cell** (`mak_02_controller._local_obstacles_body`),
so its effective clearance ≈ 0.14 m ≥ the footprint, matching the A\* poison hard
clearance (`ROBOT_RADIUS − HARD_POISON_MARGIN = 0.145 m`). Walls keep the tighter
0.10 m so passages stay threadable; poison alone gets the wide berth.

### Fix D — robust scan‑match acceptance (was: a 0.44 score teleported the pose)
**Root cause:** any match with `norm ≥ SM_MIN_SCORE_FRAC = 0.30` was committed,
so an ambiguous 0.44 match in the open area near the poison block was accepted
and the pose jumped ~0.55 m laterally.
**Principle (cartographer / gmapping):** down‑weight low‑confidence matches
instead of committing a jump. A **two‑band gate** on the score, combined with a
**correction‑magnitude check**:
* `norm ≥ SM_TRUST_SCORE_FRAC (0.60)` → strong, unambiguous → accept any correction;
* `0.40 ≤ norm < 0.60` (marginal) → accept **only if** the correction
  `≤ SM_MARGINAL_MAX_CORR_M (0.10 m)`; a marginal score that *also* wants to
  teleport the pose is the lost/ambiguous failure → **dead‑reckon** on the good
  wheel+IMU odometry instead;
* `norm < 0.40` → lost → dead‑reckon.
**Change:** `slam/scan_matcher.match` now computes the correction magnitude and
applies the gate; `info` carries `correction` and a `reason`. Constants in
`settings.py` (`SM_MIN_SCORE_FRAC 0.30→0.40`, new `SM_TRUST_SCORE_FRAC`,
`SM_MARGINAL_MAX_CORR_M`). Normal operation (the run sat at 0.90–0.99) is
unaffected — only ambiguous‑and‑jumpy updates are now rejected.
**Test:** `TestAcceptanceGate` — marginal+large rejected, marginal+small
accepted, strong+large accepted, lost rejected.

### Fix E — localization‑gated barrier stamping (was: phantom wall on the goal corridor)
**Root cause:** the virtual bumper stamped a **permanent** barrier wherever the
robot stalled — even if the *pose* was wrong — so a stall during poor
localization printed a phantom wall (the (0.9, −1.1) barrier on the yellow
approach).
**Principle:** never write a permanent obstacle into the map from an untrusted
pose. Combined with Fix D (which keeps the pose from drifting in the first
place), this removes both ways a phantom is born.
**Change:** `_virtual_bumper` stamps only when `_last_sm_ok` **and**
`_last_sm_norm ≥ BUMPER_MIN_SM_NORM = 0.45`; otherwise it skips the stamp and
lets recovery unwedge (the wall is re‑detected once relocalized). Legitimate
bumps in the run happened at `sm` 0.95–0.97, so they are unaffected.

### A and C — unchanged by design, harm reduced
The 0–0.6 m depth blind zone and the fully‑floating `WallShort(5)` remain
physically unsensable up close; the bumper is still the only possible detector
there. But Fix B means that when A\* routes *past* a mapped floating wall the
robot no longer grazes it, and Fix E means fewer spurious detours from phantom
barriers — so the *practical* cost of A/C is reduced even though the sensor limit
is irreducible.

### Architectural invariant established
Local hard radius (0.10 m) **==** A\* wall hard clearance (0.10 m); DWA poison
berth (~0.14 m) **≈** A\* poison hard clearance (0.145 m). The local and global
layers now agree on the footprint, which is what makes execution faithfully track
the plan instead of grazing inside it.

---

## 5. Finding F — "reached the pillar" without being at the pillar (next run)

**Observed:** the next run logged `BLUE reached at t=96.13 s`, but the robot was
**not** at the blue pillar — it had detected blue *through/under* the floating
wall WallMedium(3) and stopped on the wrong side.

**Verified cause (quantitative).** Converting the world ground truth into this
run's controller frame (pure translation by the start pose `(-0.6798, 1.2382)`;
`sm` held 0.9–1.0 so frame drift is cm-level):

| | true pillar (ctrl frame) | tracked estimate | estimate error |
|---|---|---|---|
| blue | (1.96, −0.41) | (1.744, −0.062) | **0.41 m** |
| yellow | (0.65, −0.93) | (0.699, −1.128) | 0.20 m |

At the "BLUE reached" instant the robot was at ~(1.48, 0.31) — i.e. **0.87 m
from the real blue pillar**, but within `PILLAR_REACH_DIST = 0.45 m` of the
*biased* estimate. The arrival test (`_pillar_in_reach`, `_do_go`) compared
odometry to a **vision estimate** that is biased ~0.4 m when the pillar is only
seen obliquely through a floating wall (occluded top → shifted blue-blob centroid
→ biased bearing/position, accumulated in the running mean). There was **no
requirement that the robot ever take a close, clean depth look at the pillar**, so
a glimpse-from-afar was enough to declare success. (Yellow had the same defect,
0.2 m milder.)

**Fix — depth-verified close arrival (+ estimate snap).** The depth camera is
blind < 0.6 m, so confirmation is two-stage:
1. **Confirm while approaching:** when the pillar is seen within the camera's
   reliable close band (`PILLAR_CONFIRM_MIN_RANGE 0.5` …
   `PILLAR_DEPTH_CONFIRM_RANGE 0.95` m) and the reading agrees with the tracked
   estimate (`PILLAR_CONFIRM_SNAP_TOL 0.6` m), after `PILLAR_CONFIRM_HITS = 3`
   such readings the pillar is **confirmed** and its estimate is **snapped** to
   that accurate close measurement — discarding the through-wall bias.
2. **Arrive** only when `is_confirmed AND pose_distance < PILLAR_REACH_DIST`, now
   measured against the *snapped* (accurate) estimate.

So a glimpse through a floating wall can no longer trigger arrival; the robot must
drive up to within ~0.9 m of the **real** pillar with a clean view. If it confirms
through the wall, the snap relocates the target to the true pillar and the
proximity requirement forces it to route around to a reachable side. The
`PillarTracker` was extracted to `perception/pillar_tracker.py` (no Webots import)
so the logic is unit-tested — `tests/test_pillar_tracker.py` includes the exact
through-wall bias scenario (far biased estimate → close readings snap it onto the
true pillar and confirm). 72 → **77 tests**, all green.

---

## 6. Finding G — livelock: can't turn around in a channel too narrow for the footprint

**Observed:** the robot drove into the floating-wall zone by the blue pillar and
got stuck in a ~0.4 m box (≈(1.45, 0.55)), oscillating EXPLORE→RECOVERY→EXPLORE
for ~60 s, bumping a wall, reversing, turning, hitting it again — never escaping
toward yellow. The user noted the turn itself drives it back into the wall.

**Verified cause (quantitative, from the saved map).**
- The pocket is bounded by a **real lidar wall 0.170 m** from the robot centre
  (`depth_obs` is 0.48 m away — dropping it changes nothing; this is **not**
  over-inflation). The channel is ~**0.34 m** wide of real walls.
- The robot is 0.25 m wide but ~**0.32 m on the diagonal**, so the in-place
  rotation margin in a 0.34 m channel is only ~**0.045 m**. The 4-wheel
  **skid-steer scrub** drifts the body ~3–5 cm during a spin, which eats that
  margin → a corner hits the wall (exactly the user's observation). It is a
  **geometric** limit, not a tuning artefact.
- The trapping wall (0.170 m) is **inside the 2-D lidar's 0.2 m minimum range**,
  so the lidar cannot even see what the robot is wedged against — only the IR and
  the **map** know.
- Clearance to the nearest lethal cell (0.089 m) is below the DWA reject radius
  (0.10 m), so every forward trajectory is rejected → the robot can only spin →
  recovery reverses it back into the same channel → livelock.

**On the proposed 2WD/4WD switch.** It would not fix this. The ROSbot is a fixed
4-wheel skid-steer; rotational scrub (and the resulting drift) is *inherent* to
that geometry. Driving only two wheels does not remove the scrub — the undriven
wheels drag (in Webots velocity mode a 0-velocity wheel brakes), which is as bad
or worse, and it reduces control authority. The real constraint is that the
channel is narrower than the footprint's turning circle; no drive mode changes
that.

**Why not inflation / speed.** The channel is *real* and *traversable straight*
(0.34 m > 0.25 m width). Raising the costmap inflation enough to make A\* avoid
channels too narrow to *turn* in (~0.32 m) would also block the legitimate narrow
passages the maze requires — wrong trade. Speed only affects overshoot, not the
geometric margin.

**Fix — back out before turning (the human move).** Recovery now reverses
**straight** along its entry (where the robot is known to fit) until the **map**
reports the nearest wall is ≥ `RECOVERY_ROTATE_CLEAR = 0.20 m` away — a spot wide
enough that the skid-steer drift is harmless — and only **then** rotates toward
the widest gap. The reverse is guarded so it never backs into poison or a wall
(`_rear_lethal_close`, `_rear_hard_blocked`) and is capped at
`RECOVERY_REVERSE_T = 2.2 s` (~0.44 m). Rotation clearance is read from the map,
not the lidar, because the trapping wall is inside the lidar blind zone. New grid
helpers `nearest_lethal_dist` / `is_lethal_world` (unit-tested). Verified against
the actual pocket: at (1.46, 0.55) clearance is 0.16 m (can't turn) but 0.20 m
back it is 0.29 m (can turn) — so the robot backs out and turns cleanly instead
of scrubbing in. 77 → **78 tests**, all green.

**Residual.** If the robot keeps being *lured* back into the same floating-wall
pocket (frontier/pillar pull), the deeper lever is goal/frontier selection that
declines to send it into channels narrower than its turning circle — a larger
change with trade-offs, deferred until the back-out recovery is observed in sim.

---

## 7. Finding H — pillar confirmed THROUGH a floating wall; floating-wall berth too small

**Observed (next run).** "BLUE reached" fired with the robot **0.93 m from the
true pillar**, separated from it by the floating wall WallMedium(3); the robot
also bumped WallShort(9). Recovery (Finding G) now *did* escape the pocket and the
mission ran to yellow, but the blue arrival was false.

**Verified cause.** The depth-confirm (Finding F) checks *closeness* but not
whether the robot can actually **drive** to the pillar. The depth camera at
0.165 m sees the pillar **under** WallMedium(3) (bottom 0.20 m) and returns a
range in the confirm band, so the robot confirmed and "arrived" while a floating
wall it cannot pass sat between them. From the saved map, the segment robot→true
blue at the arrival pose is blocked by `depth_obs` at (1.83, 0.18) then occupied
walls — a clear through-wall sighting. (WallShort(9) is a separate issue: a normal
**ground** wall, bottom at the floor, that simply wasn't mapped at its centre cell
— the robot drove into the unmapped thin wall and the bumper+recovery caught it.)

**Fix 1 — confirmation requires a CLEAR GROUND PATH.** `OccupancyGrid.segment_clear`
tests the straight robot→pillar segment for any lethal cell (wall / floating wall
/ poison), ignoring the last `PILLAR_LOS_STOP_SHORT = 0.25 m` so the pillar's own
occupied cell isn't the blocker. The controller computes this each detection and
passes `los_clear` to `PillarTracker.observe`, which now adds a confirm hit only
when **close AND consistent AND los_clear**. Verified on the real map: from the
false-arrival pose LOS is blocked (no confirm); from a clear SE approach it is
open (confirms). The robot must now route to a side it can actually reach.

**Fix 2 — wider berth for floating walls in the DWA.** The depth camera is blind
< 0.6 m, so close in only the *mapped* `depth_obs` can stop the robot driving
under/into a floating wall. `_local_obstacles_body` now dilates `depth_obs` by
`LP_DEPTH_OBS_EXTRA_CELLS = 1` cell (≈0.14 m effective clearance at the 0.10 m DWA
radius), the same technique used for poison. Kept at 1 cell because the
floating-wall passages are ~0.34 m wide — 2 cells would start to choke them.

77 → **80 tests** (added `segment_clear`, through-wall-no-confirm, and the
existing close/clear confirm still holds), all green.

**Note / residual.** WallShort(9) is a thin ground wall missed by the lidar map
(likely approached inside the 0.2 m lidar blind zone). It is handled reactively
(bumper + recovery). A durable fix would integrate the IR/ToF hits into the
occupancy grid (not just the DWA) so thin near walls get mapped for A\* — deferred.

---

## 8. Finding I — parked at an OCCLUDED estimate next to a misplaced floating wall

**Observed (next run).** Blue is now approached from the right (clear-side) — the
LOS gate works — but the robot then parks at (1.62, −0.20), **0.40 m from blue**,
and thrashes with **scan-match failures (`sm` 0.39–0.44, rejected)**, eventually
`[deadend] blue not reachable`. Three distinct problems, pinned from the map:

**(1) MAPPING — WallShort(5) depth points are displaced.** WallShort(5) is a
*tumbled* floating wall (z 0.175–0.225 m, 120° tilt). Its mapped `depth_obs` sits
at controller x[1.18, 1.38], y[−0.34, −0.90] but its *true* footprint is
x[1.23, 1.53], y[−0.49, −0.99] — **shifted ~0.15 m north-west**, because the depth
camera only sees a thin sliver of the tilted wall in the 0.04–0.22 m band and that
sliver is the wall's offset edge, not its centre. The displaced mark reaches up to
WallMedium(1)'s south end and **closes the gap** between the two walls. *However*,
A\* shows blue is still reachable (0.50 m) with or without that `depth_obs`, so the
displacement is a real inaccuracy but **not** the hard blocker here.

**(2) PATH PLANNING — GO targets an OCCLUDED estimate (the real blocker).** Blue is
walled in; the only clear-LOS, in-confirm-range viewpoints are to its **south /
west / east — none to the north**. But the biased estimate (2.05, −0.25, ~0.4 m
north of true blue) sits in the north pocket, so GO drove the robot there, where
walls block the view → it can never confirm and just thrashes — even though a
clear viewpoint exists **0.25 m away**. **Fix:** until a pillar is confirmed, GO
now navigates to the nearest **viewpoint** it can actually see the pillar from
(`select_viewpoint`: nearest non-lethal, clear-LOS point in the confirm band
around the pillar), not the raw estimate. Verified on the map: from the stall the
chosen viewpoint is (1.40, −0.08) — 0.25 m away, 0.65 m from blue, **clear LOS**.

**(3) LOCALIZATION — scan-match collapses in the feature-poor pocket.** Right next
to WallShort(5) (floating → invisible to the lidar) the scan has few matchable
features, so the score fell to ~0.39 and — with `SM_MIN_SCORE_FRAC` at 0.40 — was
*rejected*, leaving the robot dead-reckoning while spinning. **Fix:** lowered the
floor back to **0.30**; the displacement gate (Finding D) still prevents a bad
jump (a low-score match may only *nudge* the pose, ≤ 0.10 m, never teleport it), so
the robot stays localized in feature-poor spots without risking the old GO_YELLOW
jump.

80 → **82 tests** (added `select_viewpoint` enclosed-pillar + open-pillar tests),
all green. The mapping displacement (1) is inherent to projecting a tilted
floating wall from one viewpoint; the viewpoint goal (2) routes the robot to a
spot it can confirm from regardless, and (3) keeps it localized to get there.

---

## 9. Finding J — confirm through a gap (sliver), and the displaced-wall gap-wedge

**Observed (next run).** Blue marked reached again at **~0.76 m** from the true
pillar; then the robot **permanently wedged** at (1.38, 0.00) in GO_YELLOW.

**(A) False confirm — biased through-gap vision.** The blue estimate is biased
**0.4 m north into open space** (1.83, −0.02 vs true 1.96, −0.41): the robot
confirms blue while seeing it as a **narrow sliver through a gap** at the camera's
edge, where the depth at the blob centroid reads the **occluding edge**, not the
pillar. The LOS gate (Finding H) then passes — the path to that open-space point
*is* clear. (The occupied-proximity idea fails: the thin pillar is barely mapped,
so true blue is *farther* from occupied (0.23 m) than the biased point (0.13 m).)
**Fix:** confirmation now requires a **clean, whole-pillar view** — the blob may
not touch the left/right frame edge (`side_clipped`) and must be ≥
`PILLAR_CONFIRM_MIN_WIDTH_PX = 70` px wide (a real pillar at ≤ 0.95 m is ≥ ~120 px;
a through-gap sliver is far narrower). Such a view still updates the estimate but
cannot confirm — so the robot must get a clean head-on view from the unoccluded
side, which also de-biases the estimate.

**(B) Gap-wedge — a displaced floating-wall mark sealed the robot in.** At
(1.38, 0.00) the real WallMedium(1) is 0.057 m behind and the **displaced**
WallShort(5) `depth_obs` 0.040 m ahead — the tumbled wall is mapped ~0.15 m off
(near-face sliver) and that mark, being *dynamic*, was painted right where the
robot stood, sealing it in. **Fix:** the footprint clear (`mark_free_disc`, run
every tick at the robot's pose) now also wipes **`depth_obs`** in the footprint —
a cell the robot physically occupies cannot hold a floating wall. The depth is
blind < 0.6 m so it never re-marks the footprint, and the true wall (never driven
through) keeps its mark. Verified on the wedge map: clearance 0.045 m → 0.100 m,
freeing the robot to drive out; the displaced mark also self-corrects along the
path the robot actually drives. (Chosen over a 3D-mapping rewrite / resolution
increase — the targeted 2D fix unblocks the route without the larger risk.)

82 → **86 tests** (added edge-clip detector tests + footprint depth-clear test),
all green.
