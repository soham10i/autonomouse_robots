"""Webots device discovery and I/O for the ROSbot.

The ONLY module that imports the Webots ``controller`` runtime, so it is the
only one that needs the simulator.  Everything else consumes plain NumPy arrays
and pose tuples and stays simulator-agnostic / testable.
"""
from __future__ import annotations

import numpy as np

from controller import Robot, Keyboard  # type: ignore

import config as C
from geometry import cmd_to_wheels


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

        # wheels (velocity control)
        self.motors = [self.robot.getDevice(n) for n in C.MOTOR_NAMES]
        for m in self.motors:
            if m is None:
                raise RuntimeError("Missing a wheel motor — check config.MOTOR_NAMES")
            m.setPosition(float("inf"))
            m.setVelocity(0.0)

        # encoders
        self.encoders = [self.robot.getDevice(n) for n in C.ENCODER_NAMES]
        for e in self.encoders:
            if e is not None:
                e.enable(self.timestep)

        # lidar
        self.lidar = self.robot.getDevice(C.LIDAR_NAME)
        if self.lidar is not None:
            self.lidar.enable(self.timestep)
            self.lidar.enablePointCloud()

        # rgb camera
        self.camera = self.robot.getDevice(C.RGB_CAMERA_NAME)
        if self.camera is not None:
            self.camera.enable(self.timestep)

        # depth / range-finder (used only for pillar range; not for mapping)
        self.depth, self.depth_name = _first_device(self.robot, C.DEPTH_NAME_CANDIDATES)
        if self.depth is not None:
            self.depth.enable(self.timestep)

        # imu (inertial unit) — absolute, drift-free yaw
        self.imu, self.imu_name = _first_device(self.robot, C.IMU_NAME_CANDIDATES)
        if self.imu is not None:
            self.imu.enable(self.timestep)

        self.keyboard = self.robot.getKeyboard()
        self.keyboard.enable(self.timestep)

        self._cmd_v = 0.0
        self._cmd_w = 0.0
        self._log_inventory()

    # ------------------------------------------------------------- logging
    def _log_inventory(self):
        ok = lambda d: "OK" if d is not None else "MISSING"
        print("[robot_io] timestep =", self.timestep, "ms")
        print("[robot_io] motors   :", [ok(m) for m in self.motors])
        print("[robot_io] encoders :", [ok(e) for e in self.encoders])
        print("[robot_io] lidar    :", ok(self.lidar), self.lidar_specs())
        print("[robot_io] rgb cam  :", ok(self.camera), self.camera_specs())
        print("[robot_io] depth    :", ok(self.depth), "(", self.depth_name, ")")
        print("[robot_io] imu      :", ok(self.imu), "(", self.imu_name, ")")
        print("[robot_io] --- all devices ---")
        for i in range(self.robot.getNumberOfDevices()):
            d = self.robot.getDeviceByIndex(i)
            print("   ", i, repr(d.getName()), "type=", d.getNodeType())

    # ---------------------------------------------------------------- step
    def step(self):
        return self.robot.step(self.timestep)

    def time(self):
        return self.robot.getTime()

    # -------------------------------------------------------------- motors
    def set_cmd(self, v, w):
        self._cmd_v, self._cmd_w = v, w
        wl, wr = cmd_to_wheels(v, w)
        m = C.WHEEL_ANG_MAX
        wl = max(-m, min(m, wl))
        wr = max(-m, min(m, wr))
        # ROSbot order: fl, fr, rl, rr  -> left {fl, rl}, right {fr, rr}
        self.motors[0].setVelocity(wl)
        self.motors[2].setVelocity(wl)
        self.motors[1].setVelocity(wr)
        self.motors[3].setVelocity(wr)

    def stop(self):
        self.set_cmd(0.0, 0.0)

    # ------------------------------------------------------------- sensors
    def read_encoders(self):
        """(left_rad, right_rad) averaged front/rear, or (None, None)."""
        if any(e is None for e in self.encoders):
            return None, None
        fl, fr, rl, rr = [e.getValue() for e in self.encoders]
        return 0.5 * (fl + rl), 0.5 * (fr + rr)

    def read_yaw(self):
        if self.imu is not None:
            return self.imu.getRollPitchYaw()[2]
        return None

    def read_lidar_ranges(self):
        if self.lidar is None:
            return None
        return np.asarray(self.lidar.getRangeImage(), dtype=np.float32)

    def lidar_specs(self):
        if self.lidar is None:
            return None
        return (self.lidar.getHorizontalResolution(), self.lidar.getFov(),
                self.lidar.getMinRange(), self.lidar.getMaxRange())

    def read_rgb_bgr(self):
        """(H, W, 3) BGR uint8 array, or None."""
        if self.camera is None:
            return None
        raw = self.camera.getImage()
        if raw is None:
            return None
        h, w = self.camera.getHeight(), self.camera.getWidth()
        arr = np.frombuffer(raw, dtype=np.uint8).reshape(h, w, 4)
        return arr[:, :, :3]

    def read_depth(self):
        if self.depth is None:
            return None
        raw = self.depth.getRangeImage()
        if raw is None:
            return None
        h, w = self.depth.getHeight(), self.depth.getWidth()
        return np.asarray(raw, dtype=np.float32).reshape(h, w)

    def camera_specs(self):
        if self.camera is None:
            return None
        return (self.camera.getWidth(), self.camera.getHeight(), self.camera.getFov())

    def poll_key(self):
        keys = set()
        while True:
            k = self.keyboard.getKey()
            if k == -1:
                break
            keys.add(k & 0xFFFF)
        return keys
