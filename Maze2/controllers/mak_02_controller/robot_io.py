"""Webots device discovery and I/O for the ROSbot.

The ONLY module that imports the Webots ``controller`` runtime, so it is the
only one that needs the simulator.  Everything else consumes plain NumPy arrays
and pose tuples and stays simulator-agnostic / testable.

Maze4-specific addition vs. the Maze5 baseline: the 4 chassis IR proximity
sensors (``fl_range``/``fr_range``/``rl_range``/``rr_range``) are enabled and
read here.  They sit ~5-9 cm above the floor on the Rosbot proto and act as a
close-range backstop for the low floating wall panels the 2-D lidar misses
(see config.py's Maze4 docstring).
"""
from __future__ import annotations

import logging
from typing import TYPE_CHECKING, Any, Dict, List, Optional, Set, Tuple

import numpy as np

from controller import Robot, Keyboard  # type: ignore

if TYPE_CHECKING:
    from controller import Device  # type: ignore

import config as C
import ir_lookup
from geometry import cmd_to_wheels
from observability import get_logger


def _first_device(robot: Robot, names: Tuple[str, ...]) -> Tuple[Optional[Device], Optional[str]]:
    """Finds the first available device from a list of candidate names.

    Args:
        robot (Robot): The Webots Robot instance.
        names (Tuple[str, ...]): A tuple of candidate device names.

    Returns:
        Tuple[Optional[Device], Optional[str]]: The device instance and its name, or (None, None) if not found.
    """
    for n in names:
        d = robot.getDevice(n)
        if d is not None:
            return d, n
    return None, None


class RobotInterface:
    """The single Webots-facing device layer for the ROSbot.

    This is the only module that imports the Webots ``controller`` runtime; it
    discovers the motors, encoders, lidar, cameras and chassis IR sensors and
    exposes them as plain NumPy arrays and scalars. Every read is defensive: a
    device that raises or returns nothing yields the documented safe fallback
    (``None``/``NaN``) plus a logged warning, so a flaky sensor degrades the run
    instead of crashing the control loop.
    """

    def __init__(self, logger: Optional[logging.Logger] = None) -> None:
        """Discover and enable all devices; a missing wheel motor is fatal.

        Args:
            logger (Optional[logging.Logger], optional): Shared logger; a dedicated one is created if omitted.

        Raises:
            RuntimeError: If any of the essential wheel motors are missing.
        """
        self.log: logging.Logger = logger or get_logger("navctl.robot_io")
        self.robot: Robot = Robot()
        self.timestep: int = int(self.robot.getBasicTimeStep())
        self.dt: float = self.timestep * 1e-3

        # wheels (velocity control)
        self.motors: List[Optional[Device]] = [self.robot.getDevice(n) for n in C.MOTOR_NAMES]
        for m in self.motors:
            if m is None:
                raise RuntimeError("Missing a wheel motor — check config.MOTOR_NAMES")
            m.setPosition(float("inf"))
            m.setVelocity(0.0)

        # encoders
        self.encoders: List[Optional[Device]] = [self.robot.getDevice(n) for n in C.ENCODER_NAMES]
        for e in self.encoders:
            if e is not None:
                e.enable(self.timestep)

        # lidar
        self.lidar: Optional[Device] = self.robot.getDevice(C.LIDAR_NAME)
        if self.lidar is not None:
            self.lidar.enable(self.timestep)
            self.lidar.enablePointCloud()

        # rgb camera
        self.camera: Optional[Device] = self.robot.getDevice(C.RGB_CAMERA_NAME)
        if self.camera is not None:
            self.camera.enable(self.timestep)

        # depth / range-finder
        self.depth: Optional[Device]
        self.depth_name: Optional[str]
        self.depth, self.depth_name = _first_device(self.robot, C.DEPTH_NAME_CANDIDATES)
        if self.depth is not None:
            self.depth.enable(self.timestep)

        # imu
        self.imu: Optional[Device]
        self.imu_name: Optional[str]
        self.imu, self.imu_name = _first_device(self.robot, C.IMU_NAME_CANDIDATES)
        if self.imu is not None:
            self.imu.enable(self.timestep)

        # chassis IR proximity sensors
        self.ir_sensors: List[Tuple[str, Device]] = []
        self._ir_lookup: Dict[str, Tuple[float, float, float, float, float]] = {}
        for n in C.IR_RANGE_NAMES:
            d = self.robot.getDevice(n)
            if d is not None:
                d.enable(self.timestep)
                self.ir_sensors.append((n, d))
                self._ir_lookup[n] = self._build_ir_lookup(d)

        self.keyboard: Keyboard = self.robot.getKeyboard()
        self.keyboard.enable(self.timestep)

        self._cmd_v: float = 0.0
        self._cmd_w: float = 0.0
        self._log_inventory()

    def device_inventory(self) -> Dict[str, Any]:
        """Return a serialisable summary of which devices were discovered.

        Returns:
            Dict[str, Any]: A dictionary detailing the status of all requested devices.
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
            "ir_sensors": [n for n, _ in self.ir_sensors],
        }

    def _log_inventory(self) -> None:
        """Log the discovered-device inventory at startup for fault tracing."""
        ok = lambda d: "OK" if d is not None else "MISSING"
        self.log.info("timestep = %d ms", self.timestep)
        self.log.info("motors=%s encoders=%s",
                      [ok(m) for m in self.motors], [ok(e) for e in self.encoders])
        self.log.info("lidar=%s rgb=%s depth=%s(%s) imu=%s(%s) ir=%s",
                      ok(self.lidar), ok(self.camera), ok(self.depth), self.depth_name,
                      ok(self.imu), self.imu_name, [n for n, _ in self.ir_sensors])

    def step(self) -> int:
        """Advance the simulation one basic timestep.

        Returns:
            int: -1 when Webots exits, otherwise 0.
        """
        return self.robot.step(self.timestep)

    def time(self) -> float:
        """Get the current simulation time.

        Returns:
            float: Simulation time in seconds.
        """
        return self.robot.getTime()

    def set_cmd(self, v: float, w: float) -> None:
        """Command body velocity ``(v, w)`` by driving the wheel motors.

        Maps ``(v, w)`` to left/right wheel angular velocities, clamps them to
        the drive limit, and applies them. A motor-write failure is logged but
        never propagated.

        Args:
            v (float): Linear velocity in m/s.
            w (float): Angular velocity in rad/s.
        """
        self._cmd_v, self._cmd_w = v, w
        wl, wr = cmd_to_wheels(v, w)
        m = C.WHEEL_ANG_MAX
        wl = max(-m, min(m, wl))
        wr = max(-m, min(m, wr))
        try:
            self.motors[0].setVelocity(wl)
            self.motors[2].setVelocity(wl)
            self.motors[1].setVelocity(wr)
            self.motors[3].setVelocity(wr)
        except Exception as exc:  # noqa: BLE001
            self.log.error("motor command failed (v=%.2f w=%.2f): %s", v, w, exc)

    def stop(self) -> None:
        """Command zero velocity (halt the robot)."""
        self.set_cmd(0.0, 0.0)

    def read_encoders(self) -> Tuple[Optional[float], Optional[float]]:
        """Return averaged front/rear encoder positions.

        Returns:
            Tuple[Optional[float], Optional[float]]: `(left_rad, right_rad)` or `(None, None)` on failure.
        """
        if any(e is None for e in self.encoders):
            return None, None
        try:
            fl, fr, rl, rr = [e.getValue() for e in self.encoders]
        except Exception as exc:  # noqa: BLE001
            self.log.warning("encoder read failed: %s", exc)
            return None, None
        return 0.5 * (fl + rl), 0.5 * (fr + rr)

    def read_yaw(self) -> Optional[float]:
        """Return the IMU yaw in radians.

        Returns:
            Optional[float]: Yaw in radians, or `None` on failure.
        """
        if self.imu is None:
            return None
        try:
            return self.imu.getRollPitchYaw()[2]
        except Exception as exc:  # noqa: BLE001
            self.log.warning("imu read failed: %s", exc)
            return None

    def read_lidar_ranges(self) -> Optional[np.ndarray]:
        """Return the raw lidar range image.

        Returns:
            Optional[np.ndarray]: Float32 array of lidar ranges, or `None` on failure.
        """
        if self.lidar is None:
            return None
        try:
            return np.asarray(self.lidar.getRangeImage(), dtype=np.float32)
        except Exception as exc:  # noqa: BLE001
            self.log.warning("lidar read failed: %s", exc)
            return None

    def lidar_specs(self) -> Optional[Tuple[int, float, float, float]]:
        """Return lidar specifications.

        Returns:
            Optional[Tuple[int, float, float, float]]: `(n_beams, fov, min_range, max_range)` or `None`.
        """
        if self.lidar is None:
            return None
        return (self.lidar.getHorizontalResolution(), self.lidar.getFov(),
                self.lidar.getMinRange(), self.lidar.getMaxRange())

    def read_rgb_bgr(self) -> Optional[np.ndarray]:
        """Return the RGB camera frame as a BGR image.

        Returns:
            Optional[np.ndarray]: Uint8 array of shape `(H, W, 3)`, or `None` on failure.
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
        except Exception as exc:  # noqa: BLE001
            self.log.warning("rgb camera read failed: %s", exc)
            return None

    def read_depth(self) -> Optional[np.ndarray]:
        """Return the depth range image.

        Returns:
            Optional[np.ndarray]: Float32 array of shape `(H, W)`, or `None` on failure.
        """
        if self.depth is None:
            return None
        try:
            raw = self.depth.getRangeImage()
            if raw is None:
                return None
            h, w = self.depth.getHeight(), self.depth.getWidth()
            return np.asarray(raw, dtype=np.float32).reshape(h, w)
        except Exception as exc:  # noqa: BLE001
            self.log.warning("depth camera read failed: %s", exc)
            return None

    def camera_specs(self) -> Optional[Tuple[int, int, float]]:
        """Return RGB camera specifications.

        Returns:
            Optional[Tuple[int, int, float]]: `(width, height, fov)` or `None`.
        """
        if self.camera is None:
            return None
        return (self.camera.getWidth(), self.camera.getHeight(), self.camera.getFov())

    def depth_specs(self) -> Optional[Tuple[int, int, float, float, float]]:
        """Return depth camera specifications.

        Returns:
            Optional[Tuple[int, int, float, float, float]]: `(width, height, fov, min_range, max_range)` or `None`.
        """
        if self.depth is None:
            return None
        try:
            dmin, dmax = float(self.depth.getMinRange()), float(self.depth.getMaxRange())
        except Exception:
            dmin, dmax = 0.05, 10.0
        return (self.depth.getWidth(), self.depth.getHeight(), self.depth.getFov(), dmin, dmax)

    def _build_ir_lookup(self, sensor: Device) -> Tuple[float, float, float, float, float]:
        """Pre-compute a value->metres mapping from the proto's lookupTable.

        Args:
            sensor (Device): The Webots DistanceSensor device.

        Returns:
            Tuple[float, float, float, float, float]: Precomputed cache parameters.
        """
        try:
            tbl = sensor.getLookupTable()
        except Exception:
            tbl = []
        try:
            fallback = float(sensor.getMaxValue())
        except Exception:
            fallback = 4.0
        return ir_lookup.build_lookup(tbl, fallback)

    def read_ir_m(self) -> Dict[str, float]:
        """Return distances measured by chassis IRs.

        Returns:
            Dict[str, float]: Mapping from sensor name to distance in metres.
        """
        out: Dict[str, float] = {}
        for n, dev in self.ir_sensors:
            info = self._ir_lookup.get(n)
            try:
                raw = dev.getValue()
            except Exception:
                out[n] = float("nan")
                continue
            out[n] = ir_lookup.value_to_meters(info, raw) if info else float("nan")
        return out

    def poll_key(self) -> Set[int]:
        """Drain and return the set of keyboard key codes pressed this tick.

        Returns:
            Set[int]: A set of Webots key codes.
        """
        keys: Set[int] = set()
        while True:
            k = self.keyboard.getKey()
            if k == -1:
                break
            keys.add(k & 0xFFFF)
        return keys
