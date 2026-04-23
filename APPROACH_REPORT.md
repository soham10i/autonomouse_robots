# Autonomous Navigation of the ROSbot in Webots — Concept Report

**Course:** Autonomous Robots (Prof. Dr.-Ing. T. Nierhoff) — Modularbeit
**Environment:** Webots R2025a, Husarion ROSbot, Maze1/Maze2 worlds
**Goal:** Drive from start → **blue pillar** → **yellow pillar** in minimum simulation time, without using the Webots supervisor, without touching walls, and without crossing the green ("poison") patch.

---

## 1. Problem analysis (read directly from the world)

From [Maze1.wbt](Maze1/worlds/Maze1.wbt) and the task PDF:

| Entity | Pose / size | Notes |
|---|---|---|
| Robot start | `(-0.828, 0.813, 0.01)`, yaw ≈ 0 | Husarion ROSbot (diff-drive) |
| Blue cylinder (sub-goal) | `(1.28, 0.83)`, r = 0.1 m | Must be reached *first* |
| Yellow cylinder (final) | `(-0.03, 0.31)`, r = 0.1 m | Timer stops here |
| Green "Poison" | box `0.4 × 0.5 × 0.1` at `(-0.05, 0.77, -0.04)` | Flush with ground — **no-go** |
| Walls | boxes `0.5 × 0.05 × 0.5` (short) and `1 × 0.05 × 0.5` (medium) | Wooden (`RoughPine`) |
| Floating walls | e.g. `WallMedium(3)` at z = 0.45, `WallMedium(5)` at z = 0.5, tilted `WallShort(5)` / `WallMedium(9)` | Lidar ring at one height can miss these |
| Short walls at low z | `WallMedium(24)` at z = 0.08 | Below lidar plane — only depth cam / bump logic catches them |

**ROSbot sensing (from the Cyberbotics [Rosbot proto](https://www.cyberbotics.com/doc/guide/rosbot)):**
- RPLidar A2 — 2D planar lidar, 360°, ~8 m, 400 Hz.
- Orbbec Astra — RGB + depth (monocular + range image), ~0.6–8 m usable.
- 4× IR `DistanceSensor` (front-L/R, rear-L/R), ~0.03–0.9 m.
- 2× encoder on each driven wheel, IMU (accelerometer + gyro + compass).
- Differential drive: wheel radius `r ≈ 0.0425 m`, track width `L ≈ 0.2 m`.

**What the problem difficulty implies:**
1. The map is unknown at run-time (no supervisor) ⇒ we need **online mapping and localisation**, not a hard-coded path.
2. Floating walls ⇒ 2D lidar alone is insufficient; we must fuse the **depth camera** in, and we must preserve the vertical geometry — this motivates a **dual 2D + 3D map** rather than flattening everything into one grid.
3. Green patch is purely visual (no geometry the lidar sees) ⇒ the **monocular camera** must classify ground colour into the cost map.
4. "Too narrow" passages exist ⇒ the planner must know the robot footprint and inflate obstacles correctly.
5. Two ordered goals ⇒ a simple finite-state machine on top of the navigator.

---

## 2. Software architecture

A layered "sense → model → plan → act" pipeline, executed every Webots time step (`basicTimeStep`, typically 32 ms):

```
                ┌──────────────────────────────────────┐
Sensors ──►     │ 1. Perception                        │
 lidar,         │    - lidar ranges → points           │
 depth,  ──►    │    - depth image → points            │
 RGB,           │    - RGB → colour mask (green,       │
 IMU,           │         blue, yellow) + pillar bearing│
 encoders       │    - encoders+IMU → odometry         │
                └──────────────────────┬───────────────┘
                                       ▼
                ┌──────────────────────────────────────┐
                │ 2. State estimation (EKF)            │
                │    fuse wheel odom + IMU yaw         │
                │    + MANDATORY scan-to-map ICP       │
                │      correction of (x, y)            │
                └──────────────────────┬───────────────┘
                                       ▼
                ┌──────────────────────────────────────┐
                │ 3. Mapping  (dual 2D + 3D)           │
                │   2D layer (navigation):             │
                │     occupancy grid (log-odds) from   │
                │     lidar + projected 3D obstacles,  │
                │     poison layer, inflation          │
                │   3D layer (representation):         │
                │     voxel grid / OctoMap from depth, │
                │     preserves floating & low walls   │
                │   projection: 3D → 2D costmap for    │
                │                 planner/DWA          │
                └──────────────────────┬───────────────┘
                                       ▼
                ┌──────────────────────────────────────┐
                │ 4. Mission FSM                       │
                │    INIT→SCAN→EXPLORE→NAV_BLUE→       │
                │    EXPLORE→NAV_YELLOW→DONE           │
                │    + RECOVERY (stuck detector)       │
                │    (explore is default; pillar        │
                │     detection is passive via camera)  │
                └──────────────────────┬───────────────┘
                                       ▼
                ┌──────────────────────────────────────┐
                │ 5. Global planner (A* / D* Lite)     │
                │    on the inflated grid              │
                └──────────────────────┬───────────────┘
                                       ▼
                ┌──────────────────────────────────────┐
                │ 6. Local planner (DWA)               │
                │    pick (v,ω) that (a) follows path, │
                │    (b) respects dynamic obstacles,   │
                │    (c) never enters poison           │
                └──────────────────────┬───────────────┘
                                       ▼
                                Wheel velocity commands
```

---

## 3. Mathematical foundations

### 3.1 Differential-drive kinematics [Siegwart et al., 2011]

Let the pose be `q = [x, y, θ]ᵀ`. With left/right wheel speeds `ω_L, ω_R` and wheel radius `r`, track `L`:

- Linear vel.  `v  = r·(ω_R + ω_L)/2`
- Angular vel. `ω  = r·(ω_R − ω_L)/L`

Continuous model:

    ẋ = v·cos θ
    ẏ = v·sin θ
    θ̇ = ω

Discrete update over `Δt` (midpoint integration):

    θ_{k+1} = θ_k + ω·Δt
    x_{k+1} = x_k + v·Δt·cos(θ_k + ω·Δt/2)
    y_{k+1} = y_k + v·Δt·sin(θ_k + ω·Δt/2)

Inverse (given a commanded `v*, ω*`):

    ω_R = (2v* + ω*·L) / (2r),   ω_L = (2v* − ω*·L) / (2r)

### 3.2 State estimation — EKF (odometry ⊕ IMU ⊕ scan-matching) [Thrun et al., 2005; Moore & Stouch, 2014; Censi, 2008]

State `x = [x, y, θ]ᵀ`, control `u = [v, ω]ᵀ` from encoders.

**Predict.**

    x̂_{k|k−1} = f(x̂_{k−1}, u_k)
    P_{k|k−1} = F_k P_{k−1} F_kᵀ + V_k Q V_kᵀ

with Jacobians

    F = ∂f/∂x = [[1, 0, −v Δt sin θ],
                 [0, 1,  v Δt cos θ],
                 [0, 0,  1          ]]
    V = ∂f/∂u

**Update 1 — IMU yaw** `z_imu = θ`:

    H_imu = [0, 0, 1],   y = z_imu − H_imu·x̂
    K = P H_imuᵀ (H_imu P H_imuᵀ + R_imu)⁻¹
    x̂ ← x̂ + K y,   P ← (I − K H_imu) P

**Update 2 — MANDATORY scan-to-map matching** `z_icp = [x, y, θ]_icp`:

The Webots `CarpetFibers` floor combined with the ROSbot's four-wheel drive produces substantial
**wheel slip** during turns and accelerations. IMU fusion corrects the heading `θ`, but leaves the
translational states `(x, y)` to pure dead-reckoning. Over a 4 m run this drift easily exceeds
one grid cell (5 cm), which smears walls across the occupancy grid and collapses the
`1.2·r_rob` inflation tolerance that the planner relies on. Once the grid is smeared the
"too narrow passage" detection breaks down.

The fix is a **scan-to-map correction step** applied every `N_icp` ticks (e.g. every 5 ticks):
align the current lidar scan `S_k` against the existing occupancy grid `M` by minimising

    (Δx, Δy, Δθ)* = argmin  Σᵢ  ρ( d(T(Δx,Δy,Δθ)·pᵢ, M) )
                    Δx,Δy,Δθ   pᵢ∈S_k

where `d(·, M)` is the distance from a transformed scan point to the nearest occupied cell
(a likelihood-field lookup is the cheap form), and `ρ` is a Huber robust kernel. Point-to-line
ICP [Censi, 2008] or a simple hill-climb over the blurred grid (Hector-SLAM style) both work
for this scale of maze. The correction is then fused into the EKF as a full-pose observation
with `H_icp = I₃` and covariance `R_icp` derived from the Hessian of the alignment cost.

Without this update 2, the filter is *observable in θ but not in (x, y)* — so it **cannot**
be skipped or marked optional. Encoder noise ≫ IMU noise on carpet; the lidar is the only
sensor that re-grounds translation.

### 3.3 Dual mapping — 2D occupancy grid ⊕ 3D voxel map [Elfes, 1989; Thrun ch. 9; Hornung et al., 2013]

The pipeline maintains **two co-registered maps, updated every tick from the same EKF
pose**. Flattening depth-camera data into a single 2D grid, as an earlier draft of this
report did, throws away the vertical geometry needed to distinguish a floating wall from a
low wall from an open doorway — information the course asks the robot to *map*, not just
avoid. We therefore separate representation (3D) from the minimal view used for planning
(2D).

#### 3.3.1 2D navigation layer — log-odds occupancy grid

Grid cells `m_i` with prior `p(m_i)=0.5`. Store
`l_i = log( p(m_i) / (1 − p(m_i)) )`.

For each range reading `z_t` along the beam with inverse-sensor-model

    l_i ← l_i + ℓ(m_i | z_t, x_t) − l_0

- Cells along the beam up to the hit ⇒ `ℓ_free` (e.g. −0.4).
- Cell at the hit ⇒ `ℓ_occ` (e.g. +0.85).
- Cells beyond the hit ⇒ untouched.

Recover probability with `p = 1 − 1/(1 + exp(l))`. Threshold at e.g. `p > 0.65` for
occupied, `< 0.2` for free.

The 2D layer is fed by:
- **direct:** every 2D lidar beam (the lidar is the primary navigation sensor);
- **indirect:** the 3D → 2D projection described in §3.3.3, which reintroduces floating
  walls, low walls and other features the lidar ring misses.

This is the *only* map the planner and DWA consume. Keeping it thin keeps planning real-time.

#### 3.3.2 3D representation layer — voxel / OctoMap

In parallel we maintain a sparse 3D voxel grid (an **OctoMap** [Hornung et al., 2013] is
the natural choice — it stores log-odds per voxel with the same Bayesian update as above,
and its octree backing makes it memory-efficient for a sparsely-occupied indoor scene).

Back-project every valid depth pixel `(u, v, d)` through the camera intrinsics

    X_c = (u − c_x)·d / f_x
    Y_c = (v − c_y)·d / f_y
    Z_c = d

transform by the camera-to-world pose `{R_wc, t_wc}` derived from the EKF pose

    p_w = R_wc · [X_c, Y_c, Z_c]ᵀ + t_wc

and write the log-odds update into the voxel containing `p_w` (hit ⇒ `ℓ_occ`, cells along
the ray ⇒ `ℓ_free`, via a 3D Bresenham / DDA ray cast). Voxel edge length `0.05 m` matches
the 2D grid resolution so the two are trivially aligned.

What the 3D layer gives us that the 2D layer cannot:
- Floating wall at `z = 0.45 m` is stored *at that height* — distinguishable from a
  full-height wall at the same `(x, y)`.
- Low walls (e.g. `WallMedium(24)` at `z = 0.08 m`, below the 2D lidar plane) are
  still mapped.
- The final deliverable video and report can render a coloured 3D map of the maze, which
  is the standard visual expected of a SLAM pipeline.

#### 3.3.3 Projection 3D → 2D for planning

Only voxels that can *actually collide with the robot's chassis* must leak into the 2D
navigation layer. Let the chassis vertical extent be `z ∈ [0, H_rob]` with `H_rob ≈ 0.22 m`.
Define the projection

    C(x, y) = {  v ∈ voxels : v.x = x, v.y = y, v.z ∈ [0, H_rob], v occupied  }

    π(x, y) = 1   if |C(x, y)| ≥ 1
            = 0   otherwise

and OR `π` into the 2D log-odds layer (i.e. add `ℓ_occ` to cells where `π = 1` and a voxel
above the chassis *did not* already account for it). Voxels strictly above `H_rob` — a
suspended wall the robot can drive *under*, were there one — are kept in 3D but do **not**
pollute the 2D costmap. This is the same slice-and-flatten trick used by ROS 2 nav2's
`voxel_layer`.

The 3D map is therefore the *source of truth*; the 2D grid is a derived, chassis-relevant
slice of it, supplemented by the lidar. Both are updated with the same EKF pose and the
same log-odds math, so they stay consistent.

#### 3.3.4 Poison layer — flat-floor homography (not depth)
 The Orbbec depth image is noisy and
frequently returns `NaN`/zero on low-texture, co-planar surfaces such as a painted floor
patch. Relying on per-pixel depth to place the green patch in the grid therefore leaves
ragged holes in the poison layer exactly where we most need it to be solid. We instead
use a **flat-floor assumption with a ground-plane homography**, treating each green pixel
as a ray and intersecting that ray with the known `Z = 0` plane.

Let `K = [[f_x,0,c_x],[0,f_y,c_y],[0,0,1]]` be the RGB intrinsics and `{R_c, t_c}` the
camera pose in the robot frame (constant, taken from the proto). For a green pixel
`(u, v)`, the normalised ray in the camera frame is

    d_c = K⁻¹ · [u, v, 1]ᵀ                    (direction, un-normalised)
    d_w = R_wr · R_c · d_c,   o_w = p_r + R_wr·t_c

where `{R_wr, p_r}` is the robot-to-world pose from the EKF. Intersect the ray
`x(λ) = o_w + λ·d_w` with `Z = 0`:

    λ* = −o_w.z / d_w.z          (skip pixels with d_w.z ≥ 0 — they look at the horizon)
    (X, Y) = (o_w.x + λ*·d_w.x,  o_w.y + λ*·d_w.y)

Mark cell `(X, Y)` in the poison layer. The homography is a single precomputed
`3×3` matrix `H_floor = K·[r_c1, r_c2, t_c]` (columns 1, 2, and translation of the
camera-to-floor extrinsics) that we invert once; per-pixel it is a matrix-vector
multiply, which is cheap. This produces dense, hole-free ground labelling independent
of depth noise.

The poison layer is stored separately from the occupancy log-odds, persists monotonically
(a cell once marked green is never cleared — the patch cannot move), and is consumed as a
**hard lethal constraint** by the planner and local controller (see §3.4 and §3.6), not as
a soft penalty.

### 3.4 Obstacle inflation (with dynamic goal masking)

Let `r_rob ≈ 0.11 m`. Every occupied cell radiates a cost

    c(d) = c_max · exp(−α·(d − r_rob))   for d > r_rob
    c(d) = ∞                               for d ≤ r_rob

This turns *geometric* collision checking into a *lookup* during planning.
**Poison cells are treated identically to `d ≤ r_rob`** — they carry `c = ∞` rather than a
finite penalty, so both the global planner and the DWA admissibility test reject any
trajectory touching a green cell.

**Dynamic inflation masking around the goal pillar.**
The pillars have radius `r_pil = 0.1 m` and therefore inflate out to
`r_pil + r_rob ≈ 0.21 m` of lethal cost around their centre. The arrival tolerance for the
FSM is `0.25 m` to the pillar centre (§3.8) — only a `4 cm` sliver of the inflated cost
field is "reachable". In practice the DWA then *actively steers the robot away from the
pillar* as the robot closes in, because every candidate that enters the inflation halo
scores as inadmissible. The robot oscillates at a distance ≈ `0.22 m` and never satisfies
the arrival predicate.

The fix is a **dynamic inflation mask** that is applied only while navigating to a known
pillar:

    c'(cell) = 0                       if cell ∈ disk(p_goal, r_pil + r_rob + ε)
             = c(cell)                 otherwise

i.e. the lethal disk around the *current* target pillar is erased from the cost map that
is given to the A* / D* Lite planner **and** to the DWA admissibility check. The mask is
applied when the FSM transitions into `NAVIGATE_BLUE` / `NAVIGATE_YELLOW` and lifted on
any other transition, so all *other* pillars and obstacles remain lethal at all times. The
mask radius `r_pil + r_rob + ε` is chosen so that the robot is allowed to breach the
inflation halo exactly enough to satisfy the `0.25 m` arrival condition, and no further.

### 3.5 Global planning — A* / D* Lite [Hart et al., 1968; Koenig & Likhachev, 2002]

On the inflated grid with 8-connected neighbours, minimise

    f(n) = g(n) + h(n),   h(n) = ‖n − goal‖₂  (admissible)

Step cost `g` = Euclidean distance × (1 + k·c_cell) so the path is pushed away from walls and *strongly* away from green.

**Why D\* Lite rather than vanilla A\*:** the map changes every step as new scans arrive. D\* Lite re-uses the previous search tree and only fixes the affected sub-tree when a cell's cost changes — `O(` changed edges `)` per replan instead of a full replanning.

### 3.6 Local control — Dynamic Window Approach [Fox, Burgard, Thrun 1997]

At each tick sample candidate pairs `(v, ω)` from the **dynamic window**

    V_d = { (v, ω) :
            v ∈ [v_c − a_v·Δt, v_c + a_v·Δt],
            ω ∈ [ω_c − a_ω·Δt, ω_c + a_ω·Δt],
            v ≤ v_max,  |ω| ≤ ω_max,
            (v, ω) admissible } ,

where **admissibility is a hard constraint set**, not a penalty term:

    admissible(v, ω) ⇔
        (i)   stop-distance(v) < obstacle-distance(v, ω)    — wall clearance
        (ii)  ∀ t ∈ [0, T_sim] : cell(traj(v,ω,t)) ∉ POISON  — hard poison constraint
        (iii) ∀ t ∈ [0, T_sim] : cell(traj(v,ω,t)) ∉ OCCUPIED (inflated, with goal mask)

Any candidate that rolls onto a green cell within the forward-simulation horizon is
**immediately rejected**, not merely penalised. Earlier drafts of this report used a soft
`−δ·poison(v, ω)` term; that formulation allows the robot to shave the edge of the patch
if the heading reward is large enough, which violates the task rules. A hard constraint
forbids any incursion at all.

Score each remaining admissible candidate

    G(v, ω) = α·heading(v, ω, path)
            + β·clearance(v, ω)
            + γ·velocity(v, ω)

Pick the arg-max and send to the wheels. If `V_d ∩ admissible = ∅` (every candidate is
blocked), the DWA returns a zero-velocity command and the FSM's stuck detector takes
over (see §3.8 — `RECOVERY`).

The DWA guarantees dynamic feasibility (the robot can actually accelerate to the chosen
command in one tick) and gives reactive obstacle avoidance without a second planner.

### 3.7 Perception

**Pillar detection (monocular).**
- Convert RGB → HSV.
- Blue mask: `H∈[100,130], S>120, V>60`. Yellow: `H∈[20,35], S>120, V>120`. Green: `H∈[40,80], S>100, V>60`.
- Largest blob per colour; centroid `(u,v)`, area `A`.
- Bearing to the pillar: `β = (u − c_x)/f_x` ⇒ angle `atan(β)` relative to camera optical axis.
- Range from the depth image at that pixel, or triangulated from bearing + known pillar radius:

      Z ≈ f_x · (2·r_pillar) / width_in_pixels

A confirmed blue pillar centroid that the depth image puts within, say, 0.3 m is the *terminal condition* for the first leg.

**Lidar pre-processing.**
- Drop points with `r < r_min` or `r > r_max` and points matching the robot's own chassis.
- Median filter per scan to remove specular spikes.
- Transform to world with the EKF pose before adding to the grid.

### 3.8 Mission FSM

The robot's **default behaviour is exploration** — it never idles. Pillar detection is a **passive check** that runs via the camera every Nth timestep, regardless of the current FSM state. The robot only transitions to navigation when a pillar has been detected AND a verified A* path exists.

```
INIT
 └── sensors ready → INITIAL_SCAN
INITIAL_SCAN
 └── 360° rotation complete → EXPLORE
EXPLORE  (default state — robot is always moving)
 ├── blue pillar detected + A* path exists  → NAVIGATE_BLUE
 ├── yellow detected + blue reached + path  → NAVIGATE_YELLOW
 ├── stuck (v̂_cmd ≈ 0 for 2–3 s)            → RECOVERY
 └── else → frontier-sweep exploration (see below)
NAVIGATE_BLUE   [inflation mask around blue pillar active]
 ├── blue reached (dist < 0.25 m)           → EXPLORE (now seeking yellow)
 ├── stuck (v̂_cmd ≈ 0 for 2–3 s)            → RECOVERY
 └── path blocked / new obstacle            → EXPLORE (replan)
NAVIGATE_YELLOW [inflation mask around yellow pillar active]
 ├── yellow reached (dist < 0.25 m)         → DONE
 ├── stuck (v̂_cmd ≈ 0 for 2–3 s)            → RECOVERY
 └── path blocked / new obstacle            → EXPLORE (replan)
RECOVERY
 ├── back up 0.5 m OR spin ±90° in place
 ├── scan-match to re-localise
 └── timer done → EXPLORE
DONE
 └── stop motors, print elapsed time
```

**Key design rules:**
1. The robot is ALWAYS either exploring, navigating, or recovering — never idle or "searching" passively.
2. Exploration = frontier-sweep movement + continuous mapping + passive camera pillar checks.
3. Navigation is triggered ONLY when: pillar detected AND A* path verified on inflated grid (with the goal-pillar mask applied, see §3.4).
4. If a navigation path gets blocked (new obstacle discovered), fall back to EXPLORE.
5. Camera runs every 8th timestep in ALL states — pillar detection is a background task.

**Frontier-sweep exploration (FOV-aware).** Naïve frontier pursuit [Yamauchi, 1997] drives
the robot straight at the nearest unknown cell. That fails here because the 2D lidar is
`360°` but the depth camera is **forward-facing only**, and floating walls are only
observable by the depth camera. If the robot charges sideways at a frontier, the depth
camera never sees the airspace the chassis is about to occupy, and a floating wall can be
struck before it enters the grid. The modified rule is:

1. Extract frontier cells `F` from the occupancy grid (boundary of known-free ↔ unknown).
2. Cluster `F` into frontier groups `F_j` and compute each centroid `c_j`.
3. For each candidate `c_j`, penalise it by the angular offset between the robot's
   heading `θ` and the bearing to `c_j`:

       J(c_j) = d_A*(p_rob, c_j) + λ_θ · |wrap(atan2(c_j.y − p_rob.y, c_j.x − p_rob.x) − θ)|

   with `λ_θ` large enough that the robot *prefers* frontiers already inside the camera
   FOV (say `±30°` around the optical axis).
4. Before moving *toward* the chosen frontier, **rotate in place first** until the
   frontier bearing is inside the camera FOV and at least one depth frame has been
   integrated over that sector. Only then permit forward motion.

This "sweep before step" rule guarantees that airspace the chassis is about to enter has
first been observed by the depth camera, which is the only sensor that catches floating
walls and low walls (e.g. `WallMedium(24)` at `z = 0.08`).

**Stuck detection and `RECOVERY`.** The DWA horizon is short (typically `1–2 s`), so it is
prone to local minima — U-shaped dead-ends, tight corners between a wall and the poison
patch, and inflation-halo funnels around narrow passages. Once trapped, every candidate in
`V_d` is inadmissible and DWA emits `v_cmd = 0`. The FSM monitors

    stuck ⇔  |v_cmd| + |ω_cmd|·r_rob  <  ε_stuck   for  T_stuck ∈ [2, 3] s

On `stuck` the FSM overrides DWA and enters `RECOVERY`, which executes one of:
- **Reverse** at `−0.1 m/s` for `0.5 m` (measured by EKF), to exit the local minimum.
- **Spin** `±90°` (alternating on successive recoveries) to present new sectors to the
  depth camera and refresh the local costmap.

During recovery the scan-matching update is run at a higher rate to re-anchor the pose,
because open-loop reversing on carpet drifts badly. After the manoeuvre completes the FSM
returns to `EXPLORE`, which forces a fresh frontier decision with the updated map instead
of letting DWA fall back into the same trap.

"Reached" := robot centre within `0.25 m` of the pillar centre, confirmed by both depth
and RGB bearing; only evaluated while the corresponding inflation mask is active, so
the robot is physically permitted to get that close.

---

## 4. Sequence for implementation (recommended order)

1. **Webots controller skeleton** (Python). Enable devices with their own time step. Confirm with dummy open-loop commands that lidar, depth and camera produce data.
2. **Odometry + EKF + scan-matching.** Drive in a square open-loop; verify the estimated pose closes on the start within a few cm. Because the floor is `CarpetFibers`, wheel-odom-only will drift visibly in `(x, y)` — implement the mandatory scan-to-map ICP update (§3.2) here, not later, and confirm the closing error drops to < 1 grid cell. Skipping this step invalidates everything built on top of the grid.
3. **Occupancy grid from lidar only.** Teleop around and render the grid live (a small matplotlib window or a PNG dumped each second).
4. **Add the 3D voxel / OctoMap layer** (§3.3.2) fed by the depth camera. Drive under a floating wall and a low wall and confirm both are stored at the correct `z`.
5. **Wire the 3D → 2D projection** (§3.3.3): chassis-height voxels are OR-ed into the 2D navigation grid; voxels above `H_rob` stay in 3D only. Verify the planner now avoids floating/low walls that the lidar alone missed.
6. **Add the RGB green-mask layer** via flat-floor homography (§3.3.4) and visualise poison cells on the 2D grid.
7. **Global planner** (A* first — simpler to debug). Drive to a hard-coded goal on the 2D layer.
8. **Upgrade to D\* Lite** once A* works — purely a performance improvement.
9. **DWA local controller** replaces raw "follow-the-path" steering.
10. **Pillar detector** in HSV; publish `(bearing, distance, confidence)`.
11. **Mission FSM** wiring it all together. Include `RECOVERY` from the first wire-up — do not treat it as optional polish; tight corners near the poison patch expose local-minima immediately.
12. **Dynamic goal inflation mask** around the active pillar (§3.4) — verify the robot can actually close the last 4 cm and satisfy the `0.25 m` tolerance.
13. **3D map export / visualisation**. Dump the OctoMap periodically (e.g. `.ot` file or an ASCII point-cloud) for the deliverable video — this is the "mapping" artefact expected alongside the navigation run.
14. **Tuning pass**: log-odds thresholds, inflation radius, voxel size, DWA weights `(α,β,γ)`, stuck timer `T_stuck`, `v_max`. The grade rewards *time*, so push `v_max` as high as the clearance allows.
15. **Video + report**. Log the simulation clock at pillar arrivals. Render the final 3D voxel map alongside the 2D grid.

### What I would *not* do

- No ROS bridge, no Cartographer, no external nav2 stack — for a ~4 m maze they are overkill and risk breaking the "no supervisor / single repo" spirit.
- No learned policy. Training cost ≫ payoff for one maze family; graders value transparent, referenced maths.
- No global SLAM loop closure. The maze is small; dead-reckoning corrected by local scan-to-map matching is enough. Loop closure is only worth the complexity if odometry drift exceeds ~5 cm over the run.

---

## 5. Risks and mitigations

| Risk | Cause | Mitigation |
|---|---|---|
| Invisible floating wall hits chassis during a turn | Depth camera is forward-only; a sideways move puts chassis into unseen airspace | Frontier-sweep rule (§3.8): rotate-then-translate so the depth camera observes every sector before the chassis enters it |
| Drive over green | Depth-based poison projection has `NaN` holes | Replace depth back-projection with **flat-floor homography** (§3.3); treat poison as a **hard** admissibility constraint in DWA (§3.6), not a soft penalty |
| Stuck in "too narrow" passage or U-shape | DWA local minima | `RECOVERY` state (§3.8) triggered by `stuck ⇔ \|v_cmd\| ≈ 0` for 2–3 s; reverse 0.5 m or spin ±90° and replan |
| Cannot close the final 4 cm to the pillar | Pillar's own inflation halo (`0.21 m`) is wider than the `0.25 m` goal tolerance | **Dynamic inflation mask** (§3.4) erases the lethal disk around the *current* target pillar while `NAVIGATE_*` is active |
| Translation drift smears the occupancy grid | Wheel slip on `CarpetFibers` — IMU only observes heading | **Mandatory** scan-to-map ICP update in the EKF (§3.2); without it, smeared walls break the `1.2·r_rob` inflation tolerance and "too narrow" detection |
| Yaw drift | Encoder slip on carpet | Fuse IMU gyro in the EKF; weight compass low (it can be noisy in Webots) |
| Plan lag at high `v_max` | Grid too fine | Grid cell 5 cm is a good compromise |

---

## 6. References

**Mapping and SLAM**
- Elfes, A. (1989). *Using Occupancy Grids for Mobile Robot Perception and Navigation.* IEEE Computer 22(6). <https://ieeexplore.ieee.org/document/30720>
- Thrun, S., Burgard, W., Fox, D. (2005). *Probabilistic Robotics.* MIT Press. <https://mitpress.mit.edu/9780262201629/probabilistic-robotics/>
- Kohlbrecher, S. et al. (2011). *A Flexible and Scalable SLAM System with Full 3D Motion Estimation* (Hector SLAM). IEEE SSRR. <http://mlab-upenn.github.io/f110/readings/downloads/HectorSLAM11.pdf>
- Grisetti, G., Stachniss, C., Burgard, W. (2007). *Improved Techniques for Grid Mapping with Rao-Blackwellized Particle Filters* (GMapping). IEEE T-RO 23(1). <https://www.ipb.uni-bonn.de/wp-content/papercite-data/pdf/grisetti07tro.pdf>
- Hess, W. et al. (2016). *Real-Time Loop Closure in 2D LIDAR SLAM* (Cartographer). IEEE ICRA. <https://research.google.com/pubs/archive/45466.pdf>
- Hornung, A., Wurm, K. M., Bennewitz, M., Stachniss, C., Burgard, W. (2013). *OctoMap: An Efficient Probabilistic 3D Mapping Framework Based on Octrees.* Autonomous Robots 34(3). <https://octomap.github.io/>

**Exploration and planning**
- Yamauchi, B. (1997). *A Frontier-Based Approach for Autonomous Exploration.* IEEE CIRA. <https://www.cs.cmu.edu/~motionplanning/papers/sbp_papers/integrated1/yamauchi_frontiers.pdf>
- Hart, P., Nilsson, N., Raphael, B. (1968). *A Formal Basis for the Heuristic Determination of Minimum Cost Paths* (A*). IEEE SSC. <https://ieeexplore.ieee.org/document/4082128>
- Koenig, S., Likhachev, M. (2002). *D\* Lite.* AAAI. <https://aaai.org/Papers/AAAI/2002/AAAI02-072.pdf>
- Dolgov, D. et al. (2008). *Practical Search Techniques in Path Planning for Autonomous Driving* (Hybrid A*). Stanford AI Lab. <https://ai.stanford.edu/~ddolgov/papers/dolgov_gpp_stair08.pdf>

**Reactive / local control**
- Fox, D., Burgard, W., Thrun, S. (1997). *The Dynamic Window Approach to Collision Avoidance.* IEEE RAM 4(1). <https://www.ri.cmu.edu/pub_files/pub1/fox_dieter_1997_1/fox_dieter_1997_1.pdf>
- Rösmann, C., Hoffmann, F., Bertram, T. (2017). *Integrated Online Trajectory Planning and Optimization in Distinctive Topologies* (Timed Elastic Band). RAS 88. <https://github.com/rst-tu-dortmund/teb_local_planner>

**Kinematics and state estimation**
- Siegwart, R., Nourbakhsh, I., Scaramuzza, D. (2011). *Introduction to Autonomous Mobile Robots* (2nd ed.). MIT Press. <https://mitpress.mit.edu/9780262015356/introduction-to-autonomous-mobile-robots/>
- Moore, T., Stouch, D. (2014). *A Generalized Extended Kalman Filter Implementation for ROS* (`robot_localization`). IAS-13. <http://docs.ros.org/en/noetic/api/robot_localization/html/>
- Censi, A. (2008). *An ICP variant using a point-to-line metric (PLICP).* IEEE ICRA. <https://censi.science/pub/research/2008-icra-plicp.pdf>

**Vision**
- OpenCV docs — *Thresholding Operations using inRange.* <https://docs.opencv.org/4.x/da/d97/tutorial_threshold_inRange.html>
- Mallick, S. *Blob Detection Using OpenCV.* LearnOpenCV. <https://learnopencv.com/blob-detection-using-opencv-python-c/>

**Webots / ROSbot**
- Cyberbotics — *Husarion's ROSbot.* <https://www.cyberbotics.com/doc/guide/rosbot>
- Cyberbotics — *Lidar node reference.* <https://www.cyberbotics.com/doc/reference/lidar>
- Cyberbotics — *RangeFinder node reference.* <https://www.cyberbotics.com/doc/reference/rangefinder>
- Cyberbotics — *Camera node reference.* <https://www.cyberbotics.com/doc/reference/camera>
