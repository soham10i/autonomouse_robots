"""Hardware abstraction layer — enables and reads all ROSbot sensors.

Wraps the Webots controller API so that other modules never call
robot.getDevice() directly. This makes the code testable and
keeps device name strings in one place.

Reference: Webots ROSbot proto
  https://www.cyberbotics.com/doc/guide/rosbot
"""

import math
from constants import TIME_STEP


class SensorManager:
    """Initialise and read all ROSbot devices."""

    def __init__(self, robot):
        self.robot = robot

        # --- Motors (differential drive, 4 wheels) ---
        self.motors = {
            'fl': robot.getDevice('front left wheel motor'),
            'fr': robot.getDevice('front right wheel motor'),
            'rl': robot.getDevice('rear left wheel motor'),
            'rr': robot.getDevice('rear right wheel motor'),
        }
        for m in self.motors.values():
            m.setPosition(float('inf'))  # velocity control mode
            m.setVelocity(0.0)

        # --- Wheel position sensors (encoders) ---
        self.encoders = {
            'fl': robot.getDevice('front left wheel motor sensor'),
            'fr': robot.getDevice('front right wheel motor sensor'),
            'rl': robot.getDevice('rear left wheel motor sensor'),
            'rr': robot.getDevice('rear right wheel motor sensor'),
        }
        for enc in self.encoders.values():
            enc.enable(TIME_STEP)

        # --- IMU: compass (heading), accelerometer, gyro ---
        self.compass = robot.getDevice('imu compass')
        self.compass.enable(TIME_STEP)

        self.accelerometer = robot.getDevice('imu accelerometer')
        self.accelerometer.enable(TIME_STEP)

        self.gyro = robot.getDevice('imu gyro')
        self.gyro.enable(TIME_STEP)

        # --- LiDAR ---
        self.lidar = robot.getDevice('laser')
        self.lidar.enable(TIME_STEP)
        self.lidar.enablePointCloud()

        # --- RGB Camera ---
        self.camera = robot.getDevice('camera rgb')
        self.camera.enable(TIME_STEP)

        # --- Depth Camera (RangeFinder) ---
        self.depth = robot.getDevice('camera depth')
        self.depth.enable(TIME_STEP)

        # --- IR Distance Sensors (4 corners) ---
        self.range_sensors = {
            'fl': robot.getDevice('fl_range'),
            'fr': robot.getDevice('fr_range'),
            'rl': robot.getDevice('rl_range'),
            'rr': robot.getDevice('rr_range'),
        }
        for rs in self.range_sensors.values():
            rs.enable(TIME_STEP)

    # ----- Motor Commands -----

    def set_velocity(self, left_speed, right_speed):
        """Set wheel velocities (rad/s). Both sides driven together."""
        self.motors['fl'].setVelocity(left_speed)
        self.motors['rl'].setVelocity(left_speed)
        self.motors['fr'].setVelocity(right_speed)
        self.motors['rr'].setVelocity(right_speed)

    def stop(self):
        """Immediately stop all motors."""
        self.set_velocity(0.0, 0.0)

    # ----- Encoder Readings -----

    def get_encoder_values(self):
        """Return (left, right) average encoder values in radians."""
        left = (self.encoders['fl'].getValue() +
                self.encoders['rl'].getValue()) / 2.0
        right = (self.encoders['fr'].getValue() +
                 self.encoders['rr'].getValue()) / 2.0
        return left, right

    # ----- IMU / Compass -----

    def get_heading(self):
        """Return compass heading in radians [0, 2π).

        Webots compass convention: values[0] = X, values[2] = Z.
        Heading = atan2(X, Z) mapped to [0, 2π).
        """
        values = self.compass.getValues()
        heading = math.atan2(values[0], values[2])
        if heading < 0:
            heading += 2 * math.pi
        return heading

    # ----- LiDAR -----

    def get_lidar_ranges(self):
        """Return list of range values from the 2D LiDAR scan."""
        return self.lidar.getRangeImage()

    def get_lidar_num_points(self):
        """Return number of points in one LiDAR scan."""
        return self.lidar.getHorizontalResolution()

    def get_lidar_fov(self):
        """Return LiDAR field of view in radians."""
        return self.lidar.getFov()

    # ----- Camera -----

    def get_camera_image(self):
        """Return raw camera image data (BGRA bytes)."""
        return self.camera.getImage()

    def get_camera_width(self):
        return self.camera.getWidth()

    def get_camera_height(self):
        return self.camera.getHeight()

    # ----- Depth Camera -----

    def get_depth_image(self):
        """Return depth image as a flat list of float distances (meters)."""
        return self.depth.getRangeImage()

    def get_depth_width(self):
        return self.depth.getWidth()

    def get_depth_height(self):
        return self.depth.getHeight()

    # ----- IR Range Sensors -----

    def get_range_values(self):
        """Return dict of IR distance sensor readings (meters)."""
        return {name: rs.getValue()
                for name, rs in self.range_sensors.items()}

    def get_min_range(self):
        """Return the minimum reading across all IR sensors."""
        values = self.get_range_values()
        return min(values.values()) if values else float('inf')
