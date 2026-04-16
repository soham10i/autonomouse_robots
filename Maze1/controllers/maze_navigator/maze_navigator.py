"""Maze navigator — main controller entry point.

STEP 1-3 TEST VERSION:
  - Initialises sensors
  - Reads odometry
  - Prints pose + sensor data every 0.5s
  - Robot slowly rotates in place so you can verify compass + encoders

Run in Webots to verify sensors work before building the full navigator.
"""

from controller import Robot
from constants import TIME_STEP, TURN_SPEED
from sensors import SensorManager
from odometry import Odometry


def main():
    robot = Robot()
    sensors = SensorManager(robot)
    odom = Odometry()

    step_count = 0
    print_interval = int(0.5 / (TIME_STEP / 1000.0))  # print every ~0.5s

    print("[NAV] Controller started — sensor test mode")
    print("[NAV] Robot will rotate slowly. Check pose + sensor values.")

    # Slow in-place rotation to verify compass + encoders
    sensors.set_velocity(-TURN_SPEED, TURN_SPEED)

    while robot.step(TIME_STEP) != -1:
        step_count += 1

        # Update odometry
        left_enc, right_enc = sensors.get_encoder_values()
        heading = sensors.get_heading()
        odom.update(left_enc, right_enc, heading)

        # Print diagnostics periodically
        if step_count % print_interval == 0:
            x, y, theta = odom.get_pose()
            sim_time = robot.getTime()

            # LiDAR info
            ranges = sensors.get_lidar_ranges()
            num_points = len(ranges) if ranges else 0
            min_range = min(ranges) if ranges else -1

            # IR range sensors
            ir_vals = sensors.get_range_values()
            min_ir = min(ir_vals.values()) if ir_vals else -1

            print(f"[NAV] t={sim_time:.1f}s | "
                  f"pose=({x:.2f}, {y:.2f}, θ={theta:.2f}) | "
                  f"LiDAR: {num_points} pts, min={min_range:.2f}m | "
                  f"IR min={min_ir:.3f}m")

        # Stop after 2 full rotations (about 12 seconds)
        if robot.getTime() > 12.0:
            sensors.stop()
            print("[NAV] Sensor test complete. All systems OK.")
            print("[NAV] Ready for Step 4 (occupancy grid).")
            break


if __name__ == "__main__":
    main()
