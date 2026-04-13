"""Sensor initialization and reading wrappers for the ROSbot."""

from controller import Robot
from constants import TIME_STEP


class SensorManager:
    """Manages all ROSbot sensors and motors."""

    def __init__(self, robot: Robot):
        self.robot = robot
        self.timestep = TIME_STEP
        self._init_motors()
        self._init_position_sensors()
        self._init_camera()
        self._init_lidar()
        self._init_imu()
        self._init_distance_sensors()

    # --- Motors ---
    def _init_motors(self):
        self.motors = {
            "fl": self.robot.getDevice("fl_wheel_joint"),
            "fr": self.robot.getDevice("fr_wheel_joint"),
            "rl": self.robot.getDevice("rl_wheel_joint"),
            "rr": self.robot.getDevice("rr_wheel_joint"),
        }
        for motor in self.motors.values():
            motor.setPosition(float("inf"))
            motor.setVelocity(0.0)

    def set_speeds(self, left: float, right: float):
        """Set left/right wheel speeds (rad/s). Clamps to MAX_VELOCITY."""
        from constants import MAX_VELOCITY
        left = max(-MAX_VELOCITY, min(MAX_VELOCITY, left))
        right = max(-MAX_VELOCITY, min(MAX_VELOCITY, right))
        self.motors["fl"].setVelocity(left)
        self.motors["rl"].setVelocity(left)
        self.motors["fr"].setVelocity(right)
        self.motors["rr"].setVelocity(right)

    def stop(self):
        self.set_speeds(0.0, 0.0)

    # --- Position sensors (wheel encoders) ---
    def _init_position_sensors(self):
        self.encoders = {
            "fl": self.robot.getDevice("front left wheel motor sensor"),
            "fr": self.robot.getDevice("front right wheel motor sensor"),
            "rl": self.robot.getDevice("rear left wheel motor sensor"),
            "rr": self.robot.getDevice("rear right wheel motor sensor"),
        }
        for enc in self.encoders.values():
            enc.enable(self.timestep)

    def get_encoder_values(self):
        """Returns (left_avg, right_avg) encoder radians."""
        fl = self.encoders["fl"].getValue()
        rl = self.encoders["rl"].getValue()
        fr = self.encoders["fr"].getValue()
        rr = self.encoders["rr"].getValue()
        left_avg = (fl + rl) / 2.0
        right_avg = (fr + rr) / 2.0
        return left_avg, right_avg

    # --- RGB Camera ---
    def _init_camera(self):
        self.camera_rgb = self.robot.getDevice("camera rgb")
        self.camera_rgb.enable(self.timestep)
        self.camera_depth = self.robot.getDevice("camera depth")
        self.camera_depth.enable(self.timestep)

    def get_camera_image(self):
        """Returns (image_bytes, width, height) for the RGB camera."""
        w = self.camera_rgb.getWidth()
        h = self.camera_rgb.getHeight()
        img = self.camera_rgb.getImage()
        return img, w, h

    def get_camera_fov(self):
        return self.camera_rgb.getFov()

    def get_depth_image(self):
        """Returns (depth_data, width, height) for the depth camera (RangeFinder)."""
        w = self.camera_depth.getWidth()
        h = self.camera_depth.getHeight()
        depth = self.camera_depth.getRangeImage()
        return depth, w, h

    # --- LiDAR ---
    def _init_lidar(self):
        self.lidar = self.robot.getDevice("laser")
        self.lidar.enable(self.timestep)
        self.lidar.enablePointCloud()

    def get_lidar_ranges(self):
        """Returns list of range values (m) from the LiDAR. Index 0 = front."""
        return self.lidar.getRangeImage()

    def get_lidar_num_points(self):
        return self.lidar.getHorizontalResolution()

    def get_lidar_fov(self):
        return self.lidar.getFov()

    # --- IMU ---
    def _init_imu(self):
        self.accelerometer = self.robot.getDevice("imu accelerometer")
        self.gyro = self.robot.getDevice("imu gyro")
        self.compass = self.robot.getDevice("imu compass")
        self.accelerometer.enable(self.timestep)
        self.gyro.enable(self.timestep)
        self.compass.enable(self.timestep)

    def get_compass_heading(self):
        """Returns heading in radians. 0 = north (+Y), pi/2 = east (+X)."""
        import math
        values = self.compass.getValues()
        # Webots compass: values[0] = x component, values[2] = y component (in XZ plane)
        # But for a 2D ground robot, we use x and y components
        # Compass returns the direction of magnetic north in the robot's frame
        heading = math.atan2(values[0], values[1])
        return heading

    def get_accelerometer(self):
        return self.accelerometer.getValues()

    def get_gyro(self):
        return self.gyro.getValues()

    # --- Distance sensors ---
    def _init_distance_sensors(self):
        self.dist_sensors = {
            "fl": self.robot.getDevice("fl_range"),
            "fr": self.robot.getDevice("fr_range"),
            "rl": self.robot.getDevice("rl_range"),
            "rr": self.robot.getDevice("rr_range"),
        }
        for ds in self.dist_sensors.values():
            ds.enable(self.timestep)

    def get_distance_sensors(self):
        """Returns dict of distance sensor values (m)."""
        return {
            name: ds.getValue()
            for name, ds in self.dist_sensors.items()
        }
