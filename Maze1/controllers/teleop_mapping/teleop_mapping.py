"""Phase 1 — manual teleop mapping controller.

Drive the ROSbot manually with the arrow keys; lidar + depth + odometry are
recorded into a unified 2D occupancy grid AND a 3D point cloud. Press ``Q``
to finalise and write the outputs to ``maps/``:

    final_map.png   cleaned 2D occupancy map with trajectory overlay
    scene.ply       3D point cloud (height-coloured, voxel-downsampled)
    map.npz         binary 2D map ready for A* (Phase 3)
                    keys: occupied, free, unknown, aux_obstacle, poison,
                          resolution, origin, cells, pose_history

Live PNG snapshots of the in-progress 2D map are dropped into ``maps/`` every
few seconds so you can watch coverage grow.

Controls
--------
    Up / Down arrows   drive forward / reverse
    Left / Right       turn (combine with Up/Down for arcs)
    Space              emergency stop (cmd = 0)
    Q                  finalise + write outputs + quit

Module reuse: this controller imports the same modules as ``maze_navigator``
so the geometry and device discovery stay in one place. ``config.py`` is the
single source of truth.
"""
import os
import sys
import math

import numpy as np

# Make the maze_navigator modules importable regardless of which world this
# is launched from. We try the parallel sibling first (Maze1 layout), then
# the cross-tree path (used when this very file is reused from Maze2).
_HERE = os.path.dirname(os.path.abspath(__file__))
for cand in (
    os.path.normpath(os.path.join(_HERE, "..", "maze_navigator")),
    os.path.normpath(os.path.join(_HERE, "..", "..", "..", "Maze1",
                                  "controllers", "maze_navigator")),
):
    if os.path.isdir(cand) and cand not in sys.path:
        sys.path.insert(0, cand)

from controller import Keyboard  # type: ignore  # provided by Webots

import config as C
from robot_io import RobotIO
from odometry import Odometry
from lidar_scan import ScanProcessor
from clearance import ClearanceChecker
from grid_2d import Grid2D
from perception import PillarDetector
from mapping_export import export_ply, export_final_png


# ---------------------------------------------------------------------------
# Teleop tuning. Independent of the autonomous controller — keep it here so
# tweaking driving feel doesn't bleed into Phase 2/3.
# ---------------------------------------------------------------------------
TELE_V_FWD = 0.30          # m/s while UP is held
TELE_V_REV = 0.18          # m/s while DOWN is held
TELE_W = 1.5               # rad/s while LEFT/RIGHT is held
TELE_PNG_PERIOD_S = 2.0    # live grid snapshot cadence
TELE_CLOUD_PERIOD_S = 0.4  # 3D point-cloud accumulation cadence


class TeleopMapping:
    def __init__(self):
        self.io = RobotIO()
        self.dt = self.io.timestep * 1e-3

        # Webots Keyboard device — has to be enabled with the same timestep.
        self.kb = self.io.robot.getKeyboard()
        self.kb.enable(self.io.timestep)

        # Sensors / mapping (re-used from the autonomous controller).
        self.odom = Odometry()
        self.scan = ScanProcessor(self.io.lidar) if self.io.lidar is not None else None
        self.clear = ClearanceChecker(self.io.depth, self.io.camera)
        self.grid = Grid2D()
        self.perc = (
            PillarDetector(self.io.camera, self.io.depth)
            if self.io.camera is not None else None
        )

        # Pillar position memory (running mean over the last N detections).
        self._pillar_obs = {"blue": [], "yellow": []}
        self.pillar_world_pos = {"blue": None, "yellow": None}
        self._announced = {"blue": False, "yellow": False}

        self.sim_time = 0.0
        self.tick = 0

        # maps/ output dir. Maze2's launcher overrides this via env var so
        # outputs from the two worlds don't collide.
        default_dir = os.path.join(_HERE, "maps")
        self.maps_dir = os.environ.get("MAZE_MAPPING_OUTPUT_DIR", default_dir)
        os.makedirs(self.maps_dir, exist_ok=True)

        self.cloud_chunks = []
        self.pose_history = []
        self._last_cloud_t = -math.inf
        self._last_png_t = -math.inf

        self._print_banner()

    # ----------------------------- helpers ------------------------------

    def _print_banner(self):
        print("=" * 60, flush=True)
        print("PHASE 1: teleop mapping mode", flush=True)
        print("Drive manually; sensors are recording. Press Q to stop.", flush=True)
        print("  Up / Down  : drive forward / reverse", flush=True)
        print("  Left/Right : turn", flush=True)
        print("  Space      : emergency stop", flush=True)
        print("  Q          : finalise + write outputs + quit", flush=True)
        print(f"Outputs will be written to: {self.maps_dir}", flush=True)
        print("=" * 60, flush=True)

    def _read_keys(self):
        """Return the set of currently-pressed key codes (modifier-stripped).

        Webots' ``getKey()`` drains one key per call and returns -1 when the
        queue is empty. Multi-key combos (e.g. UP+LEFT) only show up if we
        keep pulling until the queue empties.
        """
        keys = set()
        while True:
            k = self.kb.getKey()
            if k == -1:
                break
            keys.add(k & 0xFFFF)
        return keys

    def _cmd_from_keys(self, keys):
        v = 0.0
        w = 0.0
        if Keyboard.UP in keys:
            v += TELE_V_FWD
        if Keyboard.DOWN in keys:
            v -= TELE_V_REV
        if Keyboard.LEFT in keys:
            w += TELE_W
        if Keyboard.RIGHT in keys:
            w -= TELE_W
        if ord(' ') in keys:
            return 0.0, 0.0
        return v, w

    # ----------------------------- perception ----------------------------

    def _smooth_pillar(self, name, pdet, pose):
        """Fold the latest detection into a running-mean world position.

        Filters applied in order:
          1. Range cap  (PILLAR_MAX_DETECT_RANGE)
          2. Height check — estimated real-world height must be
             [PILLAR_HEIGHT * MIN_FRAC .. PILLAR_HEIGHT * MAX_FRAC]
          3. Aspect ratio — blob must not be wider than PILLAR_ASPECT_MAX
          4. Outlier rejection vs current running mean
          5. Minimum observation count before publishing
        """
        rng = pdet.get("range", float("nan"))
        bearing = pdet["bearing"]
        if not np.isfinite(rng) or rng <= 0.0 or rng > C.PILLAR_MAX_DETECT_RANGE:
            return

        # ── Height validation ──
        est_h = pdet.get("est_height", float("nan"))
        if np.isfinite(est_h):
            h_min = C.PILLAR_HEIGHT * C.PILLAR_HEIGHT_MIN_FRAC
            h_max = C.PILLAR_HEIGHT * C.PILLAR_HEIGHT_MAX_FRAC
            if est_h < h_min or est_h > h_max:
                if pdet["area"] > 500: # Only print if it's a large blob to avoid spam
                    print(f"[debug] Rejected {name} pillar: height {est_h:.2f}m not in [{h_min:.2f}, {h_max:.2f}]")
                return  # too short (noise) or too tall (wall segment)

        # ── Aspect ratio validation ──
        aspect = pdet.get("aspect", float("nan"))
        if np.isfinite(aspect) and aspect > C.PILLAR_ASPECT_MAX:
            if pdet["area"] > 500:
                print(f"[debug] Rejected {name} pillar: aspect {aspect:.2f} > {C.PILLAR_ASPECT_MAX}")
            return  # too wide — likely a wall, not a pillar

        rx, ry, rth = pose
        gx = rx + rng * math.cos(rth + bearing)
        gy = ry + rng * math.sin(rth + bearing)

        cur = self.pillar_world_pos[name]
        if cur is not None:
            if math.hypot(gx - cur[0], gy - cur[1]) > C.PILLAR_OUTLIER_REJECT_M:
                return  # discard outlier — too far from current estimate

        buf = self._pillar_obs[name]
        buf.append((gx, gy))
        if len(buf) > C.PILLAR_OBS_AVG_N:
            buf.pop(0)
        if len(buf) >= C.PILLAR_MIN_OBS_FOR_CONFIRM:
            xs = np.array([p[0] for p in buf])
            ys = np.array([p[1] for p in buf])
            self.pillar_world_pos[name] = (float(xs.mean()), float(ys.mean()))
            if not self._announced[name]:
                wx, wy = self.pillar_world_pos[name]
                print(f"[teleop] {name.upper()} pillar @ ({wx:+.2f}, {wy:+.2f}) "
                      f"h={est_h:.3f}m aspect={aspect:.2f} after {len(buf)} obs",
                      flush=True)
                self._announced[name] = True

    def _run_perception(self, pose):
        """Run pillar HSV detection and per-pixel green-floor projection.

        For green: we project EVERY green pixel individually onto the
        floor plane (pinhole model with z=0) and mark each landed cell as
        poison. This produces a footprint that matches the actual green
        patch outline, rather than a fat 30cm disc around the centroid
        that gets repainted from every viewpoint and accumulates into a
        giant blob.
        """
        if self.perc is None:
            return
        dets = self.perc.detect()
        if dets.get("blue") is not None:
            self._smooth_pillar("blue", dets["blue"], pose)
        if dets.get("yellow") is not None:
            self._smooth_pillar("yellow", dets["yellow"], pose)
        # Per-pixel projection — the centroid+disc method was producing an
        # oversized poison blob covering the whole map centre.
        added = self.perc.project_green_floor_to_grid(self.grid, pose)
        if added > 0 and not getattr(self, "_green_announced", False):
            print(f"[teleop] GREEN poison: first projection added "
                  f"{added} cells", flush=True)
            self._green_announced = True

    # ----------------------------- per-tick ------------------------------

    def _update_pose(self):
        wl, wr = self.io.read_encoders()
        if wl is not None:
            yaw = self.io.read_yaw()
            self.odom.update(wl, wr, yaw, self.dt)

    def _update_grid(self, pose):
        # 1. lidar log-odds
        if self.scan is not None:
            pts, _ = self.scan.scan_world(pose)
            self.grid.integrate_scan(pose, pts)
        # 2. depth-cam aux obstacle layer (catches floating walls)
        if self.clear.ok:
            d = self.clear._depth_image()
            if d is not None:
                x_r, y_r, z_r = self.clear._robot_frame(d)
                # z_r > AUX_OBSTACLE_Z_MIN (0.04m) catches LOW walls below
                # the lidar plane (lidar is at 0.10m). Using LIDAR_MOUNT_Z
                # here was a bug — it made the teleop map miss exactly the
                # obstacles the depth-cam aux layer is meant to fill in.
                valid = (np.isfinite(d) & (d > self.clear.depth_min)
                         & (d < C.AUX_OBSTACLE_MAX_RANGE)
                         & (z_r > C.AUX_OBSTACLE_Z_MIN)
                         & (z_r < C.AUX_OBSTACLE_Z_MAX))
                if valid.any():
                    rx, ry, rth = pose
                    c, s = math.cos(rth), math.sin(rth)
                    xr = x_r[valid]
                    yr = y_r[valid]
                    wx = rx + c * xr - s * yr
                    wy = ry + s * xr + c * yr
                    self.grid.mark_aux_obstacle_points(np.stack([wx, wy], axis=1))
        # 3. keep the robot's own footprint always plannable (poison
        #    cells are preserved by mark_free_disc).
        rx, ry, _ = pose
        self.grid.mark_free_disc(rx, ry, C.ROBOT_CHASSIS_HALF_LENGTH * 0.85)

    def _maybe_accumulate_cloud(self, pose):
        if not self.clear.ok:
            return
        if self.sim_time - self._last_cloud_t < TELE_CLOUD_PERIOD_S:
            return
        pts = self.clear.depth_to_world_cloud(pose)
        if pts.shape[0] > 0:
            self.cloud_chunks.append(pts)
        self._last_cloud_t = self.sim_time

    def _maybe_save_png(self, pose):
        if (self.sim_time - self._last_png_t) < TELE_PNG_PERIOD_S:
            return
        self._last_png_t = self.sim_time
        stem = f"teleop_{int(self.sim_time):04d}.png"
        text = (f"PHASE 1 teleop  t={self.sim_time:6.2f}s\n"
                f"pose=({pose[0]:+.2f}, {pose[1]:+.2f}, {pose[2]:+.2f})\n"
                f"cloud_chunks={len(self.cloud_chunks)}")
        try:
            self.grid.save_png(
                os.path.join(self.maps_dir, stem),
                pose=pose, state_text=text,
            )
        except Exception as e:
            print(f"[teleop] live PNG failed: {e}", flush=True)

    # ----------------------------- finalise ------------------------------

    def _finalize(self):
        """Write all three outputs. Each step is wrapped so a failure in
        one (e.g. matplotlib unavailable) doesn't lose the others."""
        print("[teleop] finalising — writing outputs...", flush=True)

        # 1. cleaned 2D PNG with trajectory + colour overlays
        try:
            export_final_png(
                os.path.join(self.maps_dir, "final_map.png"),
                self.grid,
                pose_history=self.pose_history,
                pillar_positions={
                    "blue": self.pillar_world_pos.get("blue"),
                    "yellow": self.pillar_world_pos.get("yellow"),
                },
                show_aux=True,
                show_poison=True,
            )
        except Exception as e:
            print(f"[teleop] final_map.png failed: {e}", flush=True)

        # 2. 3D point cloud as PLY (open in MeshLab / CloudCompare / Open3D)
        try:
            if self.cloud_chunks:
                all_pts = np.concatenate(self.cloud_chunks, axis=0)
                export_ply(
                    os.path.join(self.maps_dir, "scene.ply"),
                    all_pts, voxel=0.04,
                )
            else:
                print("[teleop] no 3D points captured", flush=True)
        except Exception as e:
            print(f"[teleop] scene.ply failed: {e}", flush=True)

        # 3. binary 2D map for Phase 3 (A*) — includes resolution + origin
        #    so cell ↔ world conversion is unambiguous.
        try:
            occ = self.grid.occupied_mask().astype(bool)
            free = self.grid.free_mask().astype(bool)
            unk = self.grid.unknown_mask().astype(bool)
            poison = self.grid.poison.astype(bool)
            aux = self.grid.aux_obstacle.astype(bool)

            # Pillar positions are stored as length-2 float arrays. NaN means
            # "not detected during the recording". Phase 3 will sanity-check
            # this against the live camera before locking in a goal.
            def _pos_or_nan(p):
                return (np.array([p[0], p[1]], dtype=np.float32)
                        if p is not None
                        else np.array([np.nan, np.nan], dtype=np.float32))

            # First recorded (x, y, θ) — anchor for the IMU-based localization
            # fallback in map_runner. NaN if no pose was sampled.
            start_pose = (np.array(self.pose_history[0], dtype=np.float32)
                          if self.pose_history
                          else np.array([np.nan, np.nan, np.nan],
                                        dtype=np.float32))

            np.savez_compressed(
                os.path.join(self.maps_dir, "map.npz"),
                occupied=occ,
                free=free,
                unknown=unk,
                aux_obstacle=aux,
                poison=poison,
                resolution=np.float32(self.grid.res),
                origin=np.array(self.grid.origin, dtype=np.float32),
                cells=np.int32(self.grid.cells),
                pose_history=np.array(self.pose_history, dtype=np.float32),
                start_pose=start_pose,
                pillar_blue=_pos_or_nan(self.pillar_world_pos.get("blue")),
                pillar_yellow=_pos_or_nan(self.pillar_world_pos.get("yellow")),
            )
            blue = self.pillar_world_pos.get("blue")
            yellow = self.pillar_world_pos.get("yellow")
            print(
                f"[teleop] wrote map.npz "
                f"(occ={int(occ.sum())} free={int(free.sum())} "
                f"unk={int(unk.sum())} aux={int(aux.sum())} "
                f"poison={int(poison.sum())}) "
                f"blue={blue} yellow={yellow}",
                flush=True,
            )
        except Exception as e:
            print(f"[teleop] map.npz failed: {e}", flush=True)

    # ----------------------------- main loop -----------------------------

    def run(self):
        try:
            while self.io.step() != -1:
                self.sim_time += self.dt
                self.tick += 1

                self._update_pose()
                pose = self.odom.pose()
                if self.tick % 5 == 0:
                    self.pose_history.append(pose)

                self._update_grid(pose)
                self._run_perception(pose)
                self._maybe_accumulate_cloud(pose)

                keys = self._read_keys()
                # Q quits; check before issuing a drive command.
                if ord('Q') in keys or ord('q') in keys:
                    print("[teleop] Q pressed — stopping recording.", flush=True)
                    self.io.stop()
                    break

                v, w = self._cmd_from_keys(keys)
                self.io.set_cmd(v, w)

                self._maybe_save_png(pose)

                if self.tick % 32 == 0:
                    print(
                        f"[t={self.sim_time:6.2f}s] "
                        f"pose=({pose[0]:+.2f}, {pose[1]:+.2f}, {pose[2]:+.2f}) "
                        f"v={v:+.2f} w={w:+.2f} "
                        f"chunks={len(self.cloud_chunks)}",
                        flush=True,
                    )
        except KeyboardInterrupt:
            print("[teleop] interrupted", flush=True)
        finally:
            self._finalize()


def main():
    nav = TeleopMapping()
    nav.run()


if __name__ == "__main__":
    main()