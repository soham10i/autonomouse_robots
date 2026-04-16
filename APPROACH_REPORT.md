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
2. Floating walls ⇒ 2D lidar alone is insufficient; we must fuse **depth camera** into the grid.
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
                │    optional: scan-matching correction│
                └──────────────────────┬───────────────┘
                                       ▼
                ┌──────────────────────────────────────┐
                │ 3. Mapping                           │
                │    2D occupancy grid (log-odds)      │
                │    + "poison" layer from camera      │
                │    + inflation by robot radius       │
                └──────────────────────┬───────────────┘
                                       ▼
                ┌──────────────────────────────────────┐
                │ 4. Mission FSM                       │
                │    INIT→SCAN→EXPLORE→NAV_BLUE→       │
                │    EXPLORE→NAV_YELLOW→DONE           │
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

### 3.2 State estimation — EKF (odometry ⊕ IMU) [Thrun et al., 2005; Moore & Stouch, 2014]

State `x = [x, y, θ]ᵀ`, control `u = [v, ω]ᵀ` from encoders.

**Predict.**

    x̂_{k|k−1} = f(x̂_{k−1}, u_k)
    P_{k|k−1} = F_k P_{k−1} F_kᵀ + V_k Q V_kᵀ

with Jacobians

    F = ∂f/∂x = [[1, 0, −v Δt sin θ],
                 [0, 1,  v Δt cos θ],
                 [0, 0,  1          ]]
    V = ∂f/∂u

**Update** with IMU yaw `z = θ` (or compass):

    y = z − Hx̂,   H = [0, 0, 1]
    K = P Hᵀ (H P Hᵀ + R)⁻¹
    x̂ = x̂ + K y,   P = (I − KH) P

Encoder noise dominates position drift; IMU noise dominates heading drift. Both are zero-mean Gaussian in the standard formulation, which is good enough for the modest scale of this maze (< 4 m).

### 3.3 Occupancy grid mapping in log-odds [Elfes, 1989; Thrun ch. 9]

Grid cells `m_i` with prior `p(m_i)=0.5`. Store
`l_i = log( p(m_i) / (1 − p(m_i)) )`.

For each range reading `z_t` along the beam with inverse-sensor-model

    l_i ← l_i + ℓ(m_i | z_t, x_t) − l_0

- Cells along the beam up to the hit ⇒ `ℓ_free` (e.g. −0.4).
- Cell at the hit ⇒ `ℓ_occ` (e.g. +0.85).
- Cells beyond the hit ⇒ untouched.

Recover probability with `p = 1 − 1/(1 + exp(l))`. Threshold at e.g. `p > 0.65` for occupied, `< 0.2` for free.

**Sensor fusion for floating walls:** *Both* the 2D lidar beam and depth-image points are cast into the same grid. For the depth camera, back-project every pixel `(u,v)` with depth `d` using the pinhole model

    X = (u − c_x)·d / f_x,
    Y = (v − c_y)·d / f_y,
    Z = d

transform by the camera-to-world pose, project to the 2D grid (drop Y), and apply the same log-odds update. This is how hovering walls get into the map.

**Poison layer.** The RGB camera's green mask (HSV range) is back-projected to the ground plane (Y=0) via the depth image (or by assuming a flat floor and ray-casting the pixel onto Z=0). Green cells get a large positive cost — not "occupied" (we could drive across if forced) but heavily penalised.

### 3.4 Obstacle inflation

Let `r_rob ≈ 0.11 m`. Every occupied cell radiates a cost

    c(d) = c_max · exp(−α·(d − r_rob))   for d > r_rob
    c(d) = ∞                               for d ≤ r_rob

This turns *geometric* collision checking into a *lookup* during planning.

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
            (v, ω) admissible ⇒ stop-distance < obstacle-distance }

Score each admissible candidate

    G(v, ω) = α·heading(v, ω, path)
            + β·clearance(v, ω)
            + γ·velocity(v, ω)
            − δ·poison(v, ω)

Pick the arg-max and send to the wheels. The DWA guarantees dynamic feasibility (the robot can actually accelerate there in one tick) and smoothly yields reactive obstacle avoidance without an extra planner.

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
 └── else → pick best frontier, drive toward it (Yamauchi 1997)
NAVIGATE_BLUE
 ├── blue reached (dist < 0.25 m)           → EXPLORE (now seeking yellow)
 └── path blocked / new obstacle            → EXPLORE (replan)
NAVIGATE_YELLOW
 ├── yellow reached (dist < 0.25 m)         → DONE
 └── path blocked / new obstacle            → EXPLORE (replan)
DONE
 └── stop motors, print elapsed time
```

**Key design rules:**
1. The robot is ALWAYS either exploring or navigating — never idle or "searching" passively.
2. Exploration = frontier-based movement + continuous mapping + passive camera pillar checks.
3. Navigation is triggered ONLY when: pillar detected AND A* path verified on inflated grid.
4. If a navigation path gets blocked (new obstacle discovered), fall back to EXPLORE.
5. Camera runs every 8th timestep in ALL states — pillar detection is a background task.

"Reached" := centre-of-pillar inside a small tolerance (e.g. robot centre within `0.25 m` of pillar centre, confirmed by both depth and bearing).

Frontier exploration [Yamauchi, 1997]: take the boundary between *known-free* and *unknown* cells in the grid; pick the closest frontier (by A* cost) as a temporary goal. This keeps the robot moving into new territory until a pillar is visible.

---

## 4. Sequence for implementation (recommended order)

1. **Webots controller skeleton** (Python). Enable devices with their own time step. Confirm with dummy open-loop commands that lidar, depth and camera produce data.
2. **Odometry + EKF.** Drive in a square open-loop; verify the estimated pose closes on the start within a few cm.
3. **Occupancy grid from lidar only.** Teleop around and render the grid live (a small matplotlib window or a PNG dumped each second).
4. **Add the depth camera** to the same grid; drive under a floating wall to confirm it now appears.
5. **Add the RGB green-mask layer** and visualise poison cells on the grid.
6. **Global planner** (A* first — simpler to debug). Drive to a hard-coded goal.
7. **Upgrade to D\* Lite** once A* works — purely a performance improvement.
8. **DWA local controller** replaces raw "follow-the-path" steering.
9. **Pillar detector** in HSV; publish `(bearing, distance, confidence)`.
10. **Mission FSM** wiring it all together.
11. **Tuning pass**: log-odds thresholds, inflation radius, DWA weights `(α,β,γ,δ)`, `v_max`. The grade rewards *time*, so push `v_max` as high as the clearance allows.
12. **Video + report**. Log the simulation clock at pillar arrivals.

### What I would *not* do

- No ROS bridge, no Cartographer, no external nav2 stack — for a ~4 m maze they are overkill and risk breaking the "no supervisor / single repo" spirit.
- No learned policy. Training cost ≫ payoff for one maze family; graders value transparent, referenced maths.
- No global SLAM loop closure. The maze is small; dead-reckoning corrected by local scan-to-map matching is enough. Loop closure is only worth the complexity if odometry drift exceeds ~5 cm over the run.

---

## 5. Risks and mitigations

| Risk | Cause | Mitigation |
|---|---|---|
| Invisible floating wall collides with robot | 2D lidar plane passes under it | Depth-camera points into the grid; also keep a hard "any point within 0.3 m in depth image" emergency stop |
| Drive over green | Grid poison layer stale if robot turned away from patch | Carry the poison layer persistently once a cell is marked; never decay it |
| Stuck in "too narrow" passage | Inflation radius too small | Start with inflation = `1.2·r_rob`, only reduce if clearly failing |
| Oscillation near the blue pillar | DWA keeps switching frontiers | Add hysteresis: once committed to a pillar goal, keep it until reached or 5 s have passed without progress |
| Yaw drift | Encoder slip on carpet (`CarpetFibers` appearance) | Fuse IMU/compass in the EKF; weight compass low (it can be noisy in Webots) |
| Plan lag at high `v_max` | Grid too fine | Grid cell 5 cm is a good compromise |

---

## 6. References

**Mapping and SLAM**
- Elfes, A. (1989). *Using Occupancy Grids for Mobile Robot Perception and Navigation.* IEEE Computer 22(6). <https://ieeexplore.ieee.org/document/30720>
- Thrun, S., Burgard, W., Fox, D. (2005). *Probabilistic Robotics.* MIT Press. <https://mitpress.mit.edu/9780262201629/probabilistic-robotics/>
- Kohlbrecher, S. et al. (2011). *A Flexible and Scalable SLAM System with Full 3D Motion Estimation* (Hector SLAM). IEEE SSRR. <http://mlab-upenn.github.io/f110/readings/downloads/HectorSLAM11.pdf>
- Grisetti, G., Stachniss, C., Burgard, W. (2007). *Improved Techniques for Grid Mapping with Rao-Blackwellized Particle Filters* (GMapping). IEEE T-RO 23(1). <https://www.ipb.uni-bonn.de/wp-content/papercite-data/pdf/grisetti07tro.pdf>
- Hess, W. et al. (2016). *Real-Time Loop Closure in 2D LIDAR SLAM* (Cartographer). IEEE ICRA. <https://research.google.com/pubs/archive/45466.pdf>

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

**Vision**
- OpenCV docs — *Thresholding Operations using inRange.* <https://docs.opencv.org/4.x/da/d97/tutorial_threshold_inRange.html>
- Mallick, S. *Blob Detection Using OpenCV.* LearnOpenCV. <https://learnopencv.com/blob-detection-using-opencv-python-c/>

**Webots / ROSbot**
- Cyberbotics — *Husarion's ROSbot.* <https://www.cyberbotics.com/doc/guide/rosbot>
- Cyberbotics — *Lidar node reference.* <https://www.cyberbotics.com/doc/reference/lidar>
- Cyberbotics — *RangeFinder node reference.* <https://www.cyberbotics.com/doc/reference/rangefinder>
- Cyberbotics — *Camera node reference.* <https://www.cyberbotics.com/doc/reference/camera>
