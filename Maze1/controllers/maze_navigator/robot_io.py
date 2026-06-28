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
        # Per-sensor lookup-table cache: maps device name → (val_min, val_max,
        # dist_at_val_min, dist_at_val_max). Webots ``DistanceSensor.getValue()``
        # returns a raw value mapped via the proto's ``lookupTable`` — for
        # ROSbot's IRs this is typically inverse-linear (close → high value,
        # far → low value). We store the endpoints so ``read_ir_m()`` can
        # invert the mapping cheaply every tick.
        self._ir_lookup = {}
        for n in C.IR_RANGE_NAMES:
            d = self.robot.getDevice(n)
            if d is not None:
                d.enable(self.timestep)
                self.ir_sensors.append((n, d))
                self._ir_lookup[n] = self._build_ir_lookup(d)

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

    # ------------------------------------------------------------------
    # IR proximity sensors
    # ------------------------------------------------------------------

    def _build_ir_lookup(self, sensor):
        """Pre-compute a value→metres mapping from the proto's lookupTable.

        Webots ``DistanceSensor.getLookupTable()`` returns a flat list of
        triples ``[d0, val0, std0, d1, val1, std1, ...]`` sorted by
        distance. The ROSbot IR proto is inverse-linear (close ⇒ high
        value, far ⇒ low value); a few protos use the opposite direction
        or a non-linear curve. We support both linear cases by caching
        the table's two extreme points and linearly inverting between
        them. Sensors with no usable table fall back to "raw value is
        already metres" — callers can spot mis-conversion via the
        startup sanity print.

        Returns ``(d_min, d_max, val_at_dmin, val_at_dmax, max_range)``.
        """
        try:
            tbl = sensor.getLookupTable()
        except Exception:
            tbl = []
        if not tbl or len(tbl) < 6:
            # No usable lookup → assume getValue() is already metres,
            # clamped to whatever getMaxValue() is.
            try:
                mx = float(sensor.getMaxValue())
            except Exception:
                mx = 4.0
            return (0.0, mx, 0.0, mx, mx)

        # Walk in groups of 3 to find min/max distance points.
        rows = [(float(tbl[i]), float(tbl[i + 1])) for i in range(0, len(tbl), 3)]
        rows.sort(key=lambda r: r[0])
        d_min, val_min = rows[0]
        d_max, val_max = rows[-1]
        return (d_min, d_max, val_min, val_max, d_max)

    def _ir_value_to_meters(self, name, value):
        """Convert one IR raw value to metres using the cached mapping."""
        info = self._ir_lookup.get(name)
        if info is None:
            return float("nan")
        d_min, d_max, v_at_dmin, v_at_dmax, _ = info
        # Guard against degenerate tables.
        if abs(v_at_dmax - v_at_dmin) < 1e-6:
            return d_max
        # Linear inversion: distance = d_min + (val - v_at_dmin) * slope
        # where slope = (d_max - d_min) / (v_at_dmax - v_at_dmin). For an
        # inverse-linear table (val_at_dmin > val_at_dmax) the slope is
        # negative, which is exactly what we want.
        slope = (d_max - d_min) / (v_at_dmax - v_at_dmin)
        d = d_min + (value - v_at_dmin) * slope
        # Clamp to the table's domain — readings outside it are noise.
        if d < d_min:
            d = d_min
        elif d > d_max:
            d = d_max
        return d

    def read_ir_m(self):
        """Return ``{sensor_name: distance_in_metres}`` for all IRs.

        ``NaN`` for any sensor whose raw read or lookup conversion fails.
        Empty dict if no IRs were discovered at startup. Cheap enough
        to call every tick.
        """
        out = {}
        for n, dev in self.ir_sensors:
            try:
                raw = dev.getValue()
            except Exception:
                out[n] = float("nan")
                continue
            out[n] = self._ir_value_to_meters(n, raw)
        return out

    def ir_max_range(self, name):
        """Return the configured max distance for sensor ``name`` (m)."""
        info = self._ir_lookup.get(name)
        return info[4] if info else float("inf")

    def print_ir_status(self, label="open-space"):
        """Diagnostic: print one line of IR distances + raw values.

        Call AFTER a few simulation steps so the sensor buffers are
        populated. Use it to sanity-check the lookup-table conversion
        before relying on the IRs in the control loop.
        """
        if not self.ir_sensors:
            print("[ir] no IR sensors discovered", flush=True)
            return
        parts = []
        for n, dev in self.ir_sensors:
            try:
                raw = dev.getValue()
                m = self._ir_value_to_meters(n, raw)
                parts.append(f"{n}: {m:.2f}m (raw={raw:.0f})")
            except Exception as e:
                parts.append(f"{n}: ERR ({e})")
        print(f"[ir] {label} | " + " | ".join(parts), flush=True)
        # Per-sensor lookup-table summary so we can verify mapping orientation.
        for n, dev in self.ir_sensors:
            info = self._ir_lookup.get(n)
            if info is None:
                continue
            d_min, d_max, v_min, v_max, _ = info
            kind = "inverse" if v_min > v_max else "direct"
            print(f"[ir] {n}: lookup {kind}-linear  "
                  f"d∈[{d_min:.2f}, {d_max:.2f}]m  "
                  f"v[{v_min:.0f}…{v_max:.0f}]", flush=True)
