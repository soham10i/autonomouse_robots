"""Device discovery and thin wrappers around the Webots device handles.

On first run we cannot be 100% sure of every device name in the Rosbot proto
(the MPU-9250 sub-devices are the usual offender). `RobotIO.__init__`
therefore enumerates every device and resolves the names by pattern, logging
what it found.  Tune `config.py` if something is missing.
"""
from controller import Robot  # type: ignore
import config as C


def _first_existing(robot, names):
    for n in names:
        d = robot.getDevice(n)
        if d is not None:
            return d, n
    return None, None


class RobotIO:
    def __init__(self):
        self.robot = Robot()
        self.timestep = int(self.robot.getBasicTimeStep())

        self.motors = [self.robot.getDevice(n) for n in C.MOTOR_NAMES]
        for m in self.motors:
            if m is None:
                raise RuntimeError("Missing a wheel motor — check config.MOTOR_NAMES")
            m.setPosition(float("inf"))
            m.setVelocity(0.0)

        self.encoders = [self.robot.getDevice(n) for n in C.ENCODER_NAMES]
        for e in self.encoders:
            if e is not None:
                e.enable(self.timestep)

        self.lidar = self.robot.getDevice(C.LIDAR_NAME)
        if self.lidar is not None:
            self.lidar.enable(self.timestep)
            self.lidar.enablePointCloud()

        self.camera = self.robot.getDevice(C.CAMERA_NAME)
        if self.camera is not None:
            self.camera.enable(self.timestep)

        self.depth, self.depth_name = _first_existing(self.robot, C.DEPTH_NAME_CANDIDATES)
        if self.depth is not None:
            self.depth.enable(self.timestep)

        self.imu, self.imu_name = _first_existing(self.robot, C.IMU_NAME_CANDIDATES)
        if self.imu is not None:
            self.imu.enable(self.timestep)

        self.gyro, _ = _first_existing(self.robot, C.GYRO_NAME_CANDIDATES)
        if self.gyro is not None:
            self.gyro.enable(self.timestep)

        self.accel, _ = _first_existing(self.robot, C.ACCEL_NAME_CANDIDATES)
        if self.accel is not None:
            self.accel.enable(self.timestep)

        self.compass, _ = _first_existing(self.robot, C.COMPASS_NAME_CANDIDATES)
        if self.compass is not None:
            self.compass.enable(self.timestep)

        self.ir_sensors = []
        for n in C.IR_RANGE_NAMES:
            d = self.robot.getDevice(n)
            if d is not None:
                d.enable(self.timestep)
                self.ir_sensors.append((n, d))

        self._log_inventory()

    def _log_inventory(self):
        found = lambda x: "OK" if x is not None else "MISSING"
        print("[robot_io] timestep =", self.timestep, "ms")
        print("[robot_io] motors      :", [found(m) for m in self.motors])
        print("[robot_io] encoders    :", [found(e) for e in self.encoders])
        print("[robot_io] lidar       :", found(self.lidar))
        print("[robot_io] camera(RGB) :", found(self.camera))
        print("[robot_io] depth       :", found(self.depth), "(", self.depth_name, ")")
        print("[robot_io] imu unit    :", found(self.imu), "(", self.imu_name, ")")
        print("[robot_io] gyro        :", found(self.gyro))
        print("[robot_io] compass     :", found(self.compass))
        print("[robot_io] ir          :", [n for n, _ in self.ir_sensors])

        print("[robot_io] --- every device on this robot ---")
        for i in range(self.robot.getNumberOfDevices()):
            d = self.robot.getDeviceByIndex(i)
            print("   ", i, d.getName(), "type=", d.getNodeType())

    def step(self):
        return self.robot.step(self.timestep)

    def set_wheel_velocities(self, w_left, w_right):
        w_left = max(-C.WHEEL_ANG_MAX, min(C.WHEEL_ANG_MAX, w_left))
        w_right = max(-C.WHEEL_ANG_MAX, min(C.WHEEL_ANG_MAX, w_right))
        self.motors[0].setVelocity(w_left)
        self.motors[2].setVelocity(w_left)
        self.motors[1].setVelocity(w_right)
        self.motors[3].setVelocity(w_right)

    def set_cmd(self, v, w):
        w_r = (2.0 * v + w * C.ROBOT_WHEEL_SEPARATION) / (2.0 * C.ROBOT_WHEEL_RADIUS)
        w_l = (2.0 * v - w * C.ROBOT_WHEEL_SEPARATION) / (2.0 * C.ROBOT_WHEEL_RADIUS)
        self.set_wheel_velocities(w_l, w_r)

    def stop(self):
        self.set_wheel_velocities(0.0, 0.0)

    def read_encoders(self):
        """Return (left_rad, right_rad) averaged front/rear, or (None, None)."""
        if any(e is None for e in self.encoders):
            return None, None
        fl, fr, rl, rr = [e.getValue() for e in self.encoders]
        return 0.5 * (fl + rl), 0.5 * (fr + rr)

    def read_yaw(self):
        if self.imu is not None:
            rpy = self.imu.getRollPitchYaw()
            return rpy[2]
        return None

    def read_gyro_z(self):
        if self.gyro is not None:
            return self.gyro.getValues()[2]
        return None
