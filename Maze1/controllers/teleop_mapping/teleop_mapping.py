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

        Skipped when the depth at the centroid is unreliable (NaN, ≤0, or
        absurdly far) — we don't want a single bad sample to drag the mean.
        """
        rng = pdet.get("range", float("nan"))
        bearing = pdet["bearing"]
        if not np.isfinite(rng) or rng <= 0.0 or rng > 6.0:
            return
        rx, ry, rth = pose
        gx = rx + rng * math.cos(rth + bearing)
        gy = ry + rng * math.sin(rth + bearing)
        buf = self._pillar_obs[name]
        buf.append((gx, gy))
        if len(buf) > C.PILLAR_OBS_AVG_N:
            buf.pop(0)
        if len(buf) >= 2:
            xs = np.array([p[0] for p in buf])
            ys = np.array([p[1] for p in buf])
            self.pillar_world_pos[name] = (float(xs.mean()), float(ys.mean()))
            if not self._announced[name]:
                wx, wy = self.pillar_world_pos[name]
                print(f"[teleop] {name.upper()} pillar @ ({wx:+.2f}, {wy:+.2f})",
                      flush=True)
                self._announced[name] = True

    def _project_green_to_grid(self, gdet, pose):
        """Project the green floor centroid to a world point and mark a
        poison disc. Same projection as the autonomous controller — falls
        back to a flat-floor pin-hole estimate when depth is unreliable."""
        if gdet is None or self.perc is None:
            return None
        rng = gdet.get("range", float("nan"))
        v_pix = gdet.get("v", self.perc.cy)
        bearing = gdet.get("bearing", 0.0)
        v_below = v_pix - self.perc.cy
        if np.isfinite(rng) and rng > 0.15:
            d_floor = float(rng)
        elif v_below > 5:
            d_floor = C.CAMERA_MOUNT_Z * self.perc.fx / v_below
            if not (0.1 < d_floor < 4.5):
                return None
        else:
            return None
        x_r = d_floor
        y_r = -math.tan(bearing) * d_floor
        rx, ry, rth = pose
        c, s = math.cos(rth), math.sin(rth)
        wx = rx + c * x_r - s * y_r
        wy = ry + s * x_r + c * y_r
        if not self.grid.in_bounds(*self.grid.w2i(wx, wy)):
            return None
        was_marked = self.grid.is_poison_world(wx, wy)
        self.grid.mark_poison(wx, wy, radius=C.POISON_DETECT_RADIUS)
        if not was_marked:
            print(f"[teleop] GREEN poison detected at ({wx:+.2f}, {wy:+.2f})",
                  flush=True)
        return (wx, wy)

    def _run_perception(self, pose):
        if self.perc is None:
            return
        dets = self.perc.detect()
        if dets.get("blue") is not None:
            self._smooth_pillar("blue", dets["blue"], pose)
        if dets.get("yellow") is not None:
            self._smooth_pillar("yellow", dets["yellow"], pose)
        self._project_green_to_grid(dets.get("green"), pose)

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
                valid = (np.isfinite(d) & (d > self.clear.depth_min)
                         & (d < 3.0)
                         & (z_r > C.LIDAR_MOUNT_Z)
                         & (z_r < C.ROBOT_HEIGHT + 0.05))
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
