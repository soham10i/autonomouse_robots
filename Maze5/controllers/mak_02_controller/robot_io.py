"""Webots hardware abstraction and device I/O boundary layer.

Serves as the exclusive bridge between the simulated Webots `controller` API and
the decoupled navigation logic. Discovers all necessary sensors and actuators,
and exposes them via resilient polling methods returning plain NumPy arrays and
standard Python types. Defensive exception barriers prevent intermittent hardware
faults from crashing the core controller.
"""
from __future__ import annotations

import logging
from typing import Any, Optional

import numpy as np

from controller import Robot, Keyboard  # type: ignore

import config as C
from geometry import cmd_to_wheels
from observability import get_logger


def _first_device(robot: Any, names: list[str]) -> tuple[Any, Optional[str]]:
    """Searches sequentially for the first successfully retrieved device.

    Args:
        robot (Robot): The primary Webots Robot API instance.
        names (list[str]): A list of string device names to probe for.

    Returns:
        tuple[Any, Optional[str]]: A tuple containing the `(device_handle, name)`
            if found; otherwise `(None, None)`.
    """
    for n in names:
        d = robot.getDevice(n)
        if d is not None:
            return d, n
    return None, None


class RobotInterface:
    """The central hardware interaction layer for the ROSbot.

    Initializes motors, encoders, IMU, Lidar, and depth/RGB cameras. Wraps
    the `robot.step()` clock function and provides safe getters and setters
    for actuating the platform.

    Attributes:
        log (logging.Logger): The dedicated IO diagnostic logger.
        robot (Robot): The underlying Webots Robot instance.
        timestep (int): Fundamental simulation timestep in milliseconds.
        dt (float): Timestep converted to seconds.
        motors (list[Any]): A list of Webots Motor instances (fl, fr, rl, rr).
        encoders (list[Any]): A list of Webots PositionSensor instances.
        lidar (Any): The primary Webots Lidar instance.
        camera (Any): The Webots Camera instance for RGB imaging.
        depth (Any): The Webots RangeFinder or alternate camera for depth sensing.
        depth_name (Optional[str]): The string identifier of the discovered depth device.
        imu (Any): The Webots InertialUnit providing global yaw.
        imu_name (Optional[str]): The string identifier of the discovered IMU.
        keyboard (Any): The Webots Keyboard interface for manual intervention.
    """

    def __init__(self, logger: Optional[logging.Logger] = None) -> None:
        """Discovers, configures, and enables all required simulation devices.

        A missing drive motor raises an immediate RuntimeError, as locomotion
        is strictly mandatory. Other peripheral failures are logged as warnings.

        Args:
            logger (Optional[logging.Logger], optional): An externally provided logger.
                If None, a module-specific logger is initialized.
        """
        self.log = logger or get_logger("navctl.robot_io")
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
    def device_inventory(self) -> dict[str, Any]:
        """Provides a serialized summary of successfully initialized hardware.

        Returns:
            dict[str, Any]: A dictionary detailing the operational status of components.
        """
        present = lambda d: d is not None
        return {
            "timestep_ms": self.timestep,
            "motors_ok": [present(m) for m in self.motors],
            "encoders_ok": [present(e) for e in self.encoders],
            "lidar": present(self.lidar),
            "camera": present(self.camera),
            "depth": present(self.depth),
            "imu": present(self.imu),
        }

    def _log_inventory(self) -> None:
        """Internal routine to emit the device discovery state to the system log."""
        ok = lambda d: "OK" if d is not None else "MISSING"
        self.log.info("timestep = %d ms", self.timestep)
        self.log.info("motors=%s encoders=%s",
                      [ok(m) for m in self.motors], [ok(e) for e in self.encoders])
        self.log.info("lidar=%s rgb=%s depth=%s(%s) imu=%s(%s)",
                      ok(self.lidar), ok(self.camera), ok(self.depth), self.depth_name,
                      ok(self.imu), self.imu_name)

    # ---------------------------------------------------------------- step
    def step(self) -> int:
        """Advances the Webots simulation clock by one atomic timestep.

        Returns:
            int: The return code from `robot.step()`. A value of -1 indicates
                the simulation is terminating.
        """
        return self.robot.step(self.timestep)

    def time(self) -> float:
        """Queries the current simulation clock time.

        Returns:
            float: The elapsed simulation time in seconds.
        """
        return self.robot.getTime()

    # -------------------------------------------------------------- motors
    def set_cmd(self, v: float, w: float) -> None:
        """Actuates the drivetrain motors to achieve the target body velocities.

        Translates target linear and angular velocity into independent left and
        right wheel speeds, clamps them to mechanical limits, and dispatches the
        commands to the Webots Motor devices. Failures are caught and logged.

        Args:
            v (float): Target forward linear velocity in meters/second.
            w (float): Target rotational velocity in radians/second.
        """
        self._cmd_v, self._cmd_w = v, w
        wl, wr = cmd_to_wheels(v, w)
        m = C.WHEEL_ANG_MAX
        wl = max(-m, min(m, wl))
        wr = max(-m, min(m, wr))
        # ROSbot order: fl, fr, rl, rr  -> left {fl, rl}, right {fr, rr}
        try:
            self.motors[0].setVelocity(wl)
            self.motors[2].setVelocity(wl)
            self.motors[1].setVelocity(wr)
            self.motors[3].setVelocity(wr)
        except Exception as exc:  # noqa: BLE001 - actuation write barrier
            self.log.error("motor command failed (v=%.2f w=%.2f): %s", v, w, exc)

    def stop(self) -> None:
        """Immediately commands all motors to halt (zero velocity)."""
        self.set_cmd(0.0, 0.0)

    # ------------------------------------------------------------- sensors
    def read_encoders(self) -> tuple[Optional[float], Optional[float]]:
        """Reads and aggregates the current wheel encoder values.

        Averages the front and rear encoder pairs into a single effective
        left and right reading. Intermittent read failures yield `None`.

        Returns:
            tuple[Optional[float], Optional[float]]: A tuple containing the
                `left_rad, right_rad` total accumulated angles, or `(None, None)`
                if any hardware is faulty.
        """
        if any(e is None for e in self.encoders):
            return None, None
        try:
            fl, fr, rl, rr = [e.getValue() for e in self.encoders]
        except Exception as exc:  # noqa: BLE001 - device read barrier
            self.log.warning("encoder read failed: %s", exc)
            return None, None
        return 0.5 * (fl + rl), 0.5 * (fr + rr)

    def read_yaw(self) -> Optional[float]:
        """Polls the onboard IMU for the absolute heading (yaw) angle.

        Returns:
            Optional[float]: The yaw angle in radians, or None if reading fails.
        """
        if self.imu is None:
            return None
        try:
            return self.imu.getRollPitchYaw()[2]
        except Exception as exc:  # noqa: BLE001 - device read barrier
            self.log.warning("imu read failed: %s", exc)
            return None

    def read_lidar_ranges(self) -> Optional[np.ndarray]:
        """Fetches the latest planar Lidar range array.

        Returns:
            Optional[np.ndarray]: A 1D float32 array of range distances in meters,
                ordered sequentially. Returns None if the device fails.
        """
        if self.lidar is None:
            return None
        try:
            return np.asarray(self.lidar.getRangeImage(), dtype=np.float32)
        except Exception as exc:  # noqa: BLE001 - device read barrier
            self.log.warning("lidar read failed: %s", exc)
            return None

    def lidar_specs(self) -> Optional[tuple[int, float, float, float]]:
        """Queries the immutable configuration parameters of the Lidar.

        Returns:
            Optional[tuple[int, float, float, float]]: A tuple detailing
                `(n_beams, fov_rad, min_range_m, max_range_m)`, or None.
        """
        if self.lidar is None:
            return None
        return (self.lidar.getHorizontalResolution(), self.lidar.getFov(),
                self.lidar.getMinRange(), self.lidar.getMaxRange())

    def read_rgb_bgr(self) -> Optional[np.ndarray]:
        """Retrieves and processes the latest camera image frame.

        Webots returns BGRA memory arrays; this strips the alpha channel to emit
        a pure BGR matrix natively consumable by the perception logic.

        Returns:
            Optional[np.ndarray]: An ``(H, W, 3)`` uint8 array representing the
                camera frame in BGR order. Returns None on fault.
        """
        if self.camera is None:
            return None
        try:
            raw = self.camera.getImage()
            if raw is None:
                return None
            h, w = self.camera.getHeight(), self.camera.getWidth()
            arr = np.frombuffer(raw, dtype=np.uint8).reshape(h, w, 4)
            return arr[:, :, :3]
        except Exception as exc:  # noqa: BLE001 - device read barrier
            self.log.warning("rgb camera read failed: %s", exc)
            return None

    def read_depth(self) -> Optional[np.ndarray]:
        """Retrieves the latest output from the depth or range-finder sensor.

        Returns:
            Optional[np.ndarray]: A 2D float32 array of size ``(H, W)`` encoding
                z-depth distances in meters. Returns None on fault.
        """
        if self.depth is None:
            return None
        try:
            raw = self.depth.getRangeImage()
            if raw is None:
                return None
            h, w = self.depth.getHeight(), self.depth.getWidth()
            return np.asarray(raw, dtype=np.float32).reshape(h, w)
        except Exception as exc:  # noqa: BLE001 - device read barrier
            self.log.warning("depth camera read failed: %s", exc)
            return None

    def camera_specs(self) -> Optional[tuple[int, int, float]]:
        """Queries the intrinsic dimension parameters of the RGB camera.

        Returns:
            Optional[tuple[int, int, float]]: A tuple describing `(width, height, fov_rad)`.
        """
        if self.camera is None:
            return None
        return (self.camera.getWidth(), self.camera.getHeight(), self.camera.getFov())

    def poll_key(self) -> set[int]:
        """Drains the keyboard input buffer since the last polling tick.

        Returns:
            set[int]: A set of integer keycodes that were actively pressed.
        """
        keys = set()
        while True:
            k = self.keyboard.getKey()
            if k == -1:
                break
            keys.add(k & 0xFFFF)
        return keys
