"""Webots device discovery and I/O for the ROSbot.

This is the only module that imports the Webots ``controller`` runtime, so it is
exercised inside the simulator rather than by the unit tests.  It keeps the rest
of the codebase hardware-agnostic: everything else consumes plain arrays and
pose tuples.
"""
from __future__ import annotations

import numpy as np

from controller import Robot, Keyboard  # type: ignore

import settings as S
from control.diff_drive import cmd_to_wheels


def _first_device(robot, names):
    for n in names:
        d = robot.getDevice(n)
        if d is not None:
            return d, n
    return None, None


class RobotIO:
    def __init__(self):
        self.robot = Robot()
        self.timestep = int(self.robot.getBasicTimeStep())
        self.dt = self.timestep * 1e-3

        # --- wheels (velocity control) ---
        self.motors = [self.robot.getDevice(n) for n in S.MOTOR_NAMES]
        for m in self.motors:
            if m is None:
                raise RuntimeError("Missing wheel motor — check settings.MOTOR_NAMES")
            m.setPosition(float("inf"))
            m.setVelocity(0.0)

        # --- encoders ---
        self.encoders = [self.robot.getDevice(n) for n in S.ENCODER_NAMES]
        for e in self.encoders:
            if e is not None:
                e.enable(self.timestep)

        # --- lidar ---
        self.lidar = self.robot.getDevice(S.LIDAR_NAME)
        if self.lidar is not None:
            self.lidar.enable(self.timestep)
            self.lidar.enablePointCloud()

        # --- rgb camera ---
        self.camera = self.robot.getDevice(S.RGB_CAMERA_NAME)
        if self.camera is not None:
            self.camera.enable(self.timestep)

        # --- depth / range-finder ---
        self.depth, self.depth_name = _first_device(self.robot, S.DEPTH_NAME_CANDIDATES)
        if self.depth is not None:
            self.depth.enable(self.timestep)

        # --- imu (inertial unit) ---
        self.imu, self.imu_name = _first_device(self.robot, S.IMU_NAME_CANDIDATES)
        if self.imu is not None:
            self.imu.enable(self.timestep)

        # --- IR / ToF range sensors (fill the lidar's <0.2 m blind zone) ---
        self.range_sensors = []          # aligned to S.RANGE_SENSOR_NAMES
        for n in S.RANGE_SENSOR_NAMES:
            d = self.robot.getDevice(n)
            if d is not None:
                d.enable(self.timestep)
            self.range_sensors.append(d)

        # --- keyboard (optional quit / manual nudge) ---
        self.keyboard = self.robot.getKeyboard()
        self.keyboard.enable(self.timestep)

        self._log_inventory()

    # ------------------------------------------------------------- logging
    def _log_inventory(self):
        ok = lambda d: "OK" if d is not None else "MISSING"
        print("[robot_io] timestep =", self.timestep, "ms")
        print("[robot_io] motors   :", [ok(m) for m in self.motors])
        print("[robot_io] encoders :", [ok(e) for e in self.encoders])
        print("[robot_io] lidar    :", ok(self.lidar))
        print("[robot_io] rgb cam  :", ok(self.camera))
        print("[robot_io] depth    :", ok(self.depth), "(", self.depth_name, ")")
        print("[robot_io] imu      :", ok(self.imu), "(", self.imu_name, ")")
        print("[robot_io] ranges   :",
              [n for n, d in zip(S.RANGE_SENSOR_NAMES, self.range_sensors) if d is not None]
              or "NONE FOUND")
        # One-time full device dump so exact names (esp. IR sensors) can be
        # confirmed from the run log without guessing.
        try:
            names = [self.robot.getDeviceByIndex(i).getName()
                     for i in range(self.robot.getNumberOfDevices())]
            print("[robot_io] ALL devices:", names)
        except Exception as e:
            print("[robot_io] device dump failed:", e)

    # ---------------------------------------------------------------- step
    def step(self):
        return self.robot.step(self.timestep)

    # -------------------------------------------------------------- motors
    def set_cmd(self, v, w):
        wl, wr = cmd_to_wheels(v, w)
        wl = max(-S.WHEEL_ANG_MAX, min(S.WHEEL_ANG_MAX, wl))
        wr = max(-S.WHEEL_ANG_MAX, min(S.WHEEL_ANG_MAX, wr))
        # ROSbot order: fl, fr, rl, rr  ->  left wheels {fl, rl}, right {fr, rr}
        self.motors[0].setVelocity(wl)
        self.motors[2].setVelocity(wl)
        self.motors[1].setVelocity(wr)
        self.motors[3].setVelocity(wr)

    def stop(self):
        for m in self.motors:
            m.setVelocity(0.0)

    # ------------------------------------------------------------ sensors
    def read_encoders(self):
        """Return (left_rad, right_rad) averaged front/rear, or (None, None)."""
        if any(e is None for e in self.encoders):
            return None, None
        fl, fr, rl, rr = [e.getValue() for e in self.encoders]
        return 0.5 * (fl + rl), 0.5 * (fr + rr)

    def read_yaw(self):
        if self.imu is not None:
            return self.imu.getRollPitchYaw()[2]
        return None

    def read_ranges(self):
        """Return a list of IR/ToF distances (m), aligned to RANGE_SENSOR_NAMES.

        Missing sensors and out-of-range readings are ``None``.  Webots
        DistanceSensor in "metres" mode returns the distance directly; if a
        sensor reports raw lookup values this still works as long as the proto
        maps them to metres (the ROSbot VL53L0X protos do).
        """
        out = []
        for d in self.range_sensors:
            if d is None:
                out.append(None)
                continue
            v = float(d.getValue())
            if v < S.RANGE_SENSOR_MIN or v > S.RANGE_SENSOR_MAX or not np.isfinite(v):
                out.append(None)
            else:
                out.append(v)
        return out

    def read_lidar_ranges(self):
        if self.lidar is None:
            return None
        return np.asarray(self.lidar.getRangeImage(), dtype=np.float32)

    def lidar_specs(self):
        """Return ``(n_beams, fov, r_min, r_max)`` or ``None``."""
        if self.lidar is None:
            return None
        return (
            self.lidar.getHorizontalResolution(),
            self.lidar.getFov(),
            self.lidar.getMinRange(),
            self.lidar.getMaxRange(),
        )

    def read_rgb_bgr(self):
        """Return an (H, W, 3) BGR uint8 array, or ``None``."""
        if self.camera is None:
            return None
        raw = self.camera.getImage()
        if raw is None:
            return None
        h = self.camera.getHeight()
        w = self.camera.getWidth()
        arr = np.frombuffer(raw, dtype=np.uint8).reshape(h, w, 4)
        return arr[:, :, :3]

    def read_depth(self):
        """Return an (H, W) float depth array (metres), or ``None``."""
        if self.depth is None:
            return None
        raw = self.depth.getRangeImage()
        if raw is None:
            return None
        h = self.depth.getHeight()
        w = self.depth.getWidth()
        return np.asarray(raw, dtype=np.float32).reshape(h, w)

    def camera_specs(self):
        """Return ``(width, height, fov)`` for the RGB camera, or ``None``."""
        if self.camera is None:
            return None
        return (self.camera.getWidth(), self.camera.getHeight(), self.camera.getFov())

    def depth_specs(self):
        if self.depth is None:
            return None
        dmin = dmax = None
        try:
            dmin = float(self.depth.getMinRange())
            dmax = float(self.depth.getMaxRange())
        except Exception:
            dmin, dmax = 0.05, 10.0
        return (self.depth.getWidth(), self.depth.getHeight(),
                self.depth.getFov(), dmin, dmax)

    def poll_key(self):
        """Drain the keyboard queue; return the set of pressed key codes."""
        keys = set()
        while True:
            k = self.keyboard.getKey()
            if k == -1:
                break
            keys.add(k & 0xFFFF)
        return keys
