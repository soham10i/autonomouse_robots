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

| # | Observation | Verified cause | Class | Code change? |
|---|---|---|---|---|
| A | Path "interrupted", robot driven into floating wall on blue turn | Floating wall in 0–0.6 m depth blind zone → A\* plans through it → reactive bumper reroute | As‑designed (sensor limit) | None needed |
| B | Skims floating wall near blue, **touches**, continues | DWA clearance is centre‑to‑point with `LP_SAFE_RADIUS 0.08 < ROBOT_RADIUS 0.125` → approves ~4.5 cm body penetration | **Genuine logic gap** | Flagged, not changed |
| C | WallShort(5) sometimes not avoided | Fully floating slab (z 0.175–0.225 m); lidar/IR pass under, depth blind <0.6 m; outcome depends on approach angle | Physical sensor limit | None possible |
| D | GO_YELLOW stall / pose jump | Low scan‑match score (0.44) accepted at `SM_MIN_SCORE_FRAC=0.30` → localization discontinuity | **Latent logic risk** | Flagged, not changed |
| E | Yellow leg blocked near a stamped barrier | Permanent bumper barriers never decay; one sits on the yellow approach | **Latent risk** | Flagged, not changed |

**Bottom line:** A and C are sensor‑geometry consequences, handled correctly by
the design — no fix exists for an object no sensor can see, and the bumper is the
correct fallback. B, D, and E are the three places where the *working logic* is
genuinely improvable (DWA footprint clearance, scan‑match acceptance floor, and
permanent‑barrier placement under poor localization). Per the request, none of
these were modified — they are documented here for the certification write‑up.
