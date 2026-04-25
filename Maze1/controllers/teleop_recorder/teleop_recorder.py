from controller import Robot, Keyboard
from pathlib import Path
import json, math, csv

robot = Robot()
TIME_STEP = int(robot.getBasicTimeStep())

keyboard = Keyboard()
keyboard.enable(TIME_STEP)

lidar = robot.getDevice("laser")
if lidar: lidar.enable(TIME_STEP)

imu = robot.getDevice("imu inertial_unit")
if imu: imu.enable(TIME_STEP)

motors = {
    "fl": robot.getDevice("fl_wheel_joint"),
    "fr": robot.getDevice("fr_wheel_joint"),
    "rl": robot.getDevice("rl_wheel_joint"),
    "rr": robot.getDevice("rr_wheel_joint"),
}
encoders = {
    "fl": robot.getDevice("front left wheel motor sensor"),
    "fr": robot.getDevice("front right wheel motor sensor"),
    "rl": robot.getDevice("rear left wheel motor sensor"),
    "rr": robot.getDevice("rear right wheel motor sensor"),
}

for m in motors.values():
    if m:
        m.setPosition(float("inf"))
        m.setVelocity(0.0)
for e in encoders.values():
    if e: e.enable(TIME_STEP)

OUT = Path(__file__).resolve().parent / "recordings_pose"
LIDAR_DIR = OUT / "lidar"
LIDAR_DIR.mkdir(parents=True, exist_ok=True)

pose_csv = open(OUT / "pose_log.csv", "w", newline="")
writer = csv.writer(pose_csv)
writer.writerow([
    "step","sim_time_s","cmd","x_m","y_m","yaw_fused","yaw_imu",
    "left_enc","right_enc","ds","dtheta_enc"
])

MAX_SPEED = 6.0
TURN_SPEED = 4.0
SCAN_SAVE_EVERY = 5

WHEEL_RADIUS_M = 0.043
AXLE_TRACK_M = 0.265
IMU_BLEND = 0.05

LIDAR_X_OFFSET = 0.0
LIDAR_Y_OFFSET = 0.0
LIDAR_YAW_OFFSET = 0.0

def wrap(a):
    return math.atan2(math.sin(a), math.cos(a))

step_count = 0
scan_id = 0
x_m, y_m, yaw_fused = 0.0, 0.0, 0.0
yaw0 = None
prev_left = None
prev_right = None

while robot.step(TIME_STEP) != -1:
    step_count += 1
    t = robot.getTime()

    # keys
    keys = []
    while True:
        k = keyboard.getKey()
        if k == -1:
            break
        keys.append(k)
    key = keys[-1] if keys else -1

    lv = rv = 0.0
    cmd = "STOP"
    if key == Keyboard.UP:
        lv = rv = MAX_SPEED; cmd = "FORWARD"
    elif key == Keyboard.DOWN:
        lv = rv = -MAX_SPEED; cmd = "BACKWARD"
    elif key == Keyboard.LEFT:
        lv = -TURN_SPEED; rv = TURN_SPEED; cmd = "LEFT"
    elif key == Keyboard.RIGHT:
        lv = TURN_SPEED; rv = -TURN_SPEED; cmd = "RIGHT"

    motors["fl"].setVelocity(lv)
    motors["rl"].setVelocity(lv)
    motors["fr"].setVelocity(rv)
    motors["rr"].setVelocity(rv)

    roll, pitch, yaw_abs = imu.getRollPitchYaw()
    if yaw0 is None:
        yaw0 = yaw_abs
    yaw_imu = wrap(yaw_abs - yaw0)

    left_enc = (encoders["fl"].getValue() + encoders["rl"].getValue()) / 2.0
    right_enc = (encoders["fr"].getValue() + encoders["rr"].getValue()) / 2.0

    if prev_left is None:
        prev_left, prev_right = left_enc, right_enc

    dleft = left_enc - prev_left
    dright = right_enc - prev_right
    prev_left, prev_right = left_enc, right_enc

    left_dist = dleft * WHEEL_RADIUS_M
    right_dist = dright * WHEEL_RADIUS_M
    ds = (left_dist + right_dist) / 2.0
    dtheta_enc = (right_dist - left_dist) / AXLE_TRACK_M

    yaw_pred = wrap(yaw_fused + dtheta_enc)
    yaw_err = wrap(yaw_imu - yaw_pred)
    yaw_fused = wrap(yaw_pred + IMU_BLEND * yaw_err)

    x_m += ds * math.cos(yaw_fused)
    y_m += ds * math.sin(yaw_fused)

    writer.writerow([step_count, t, cmd, x_m, y_m, yaw_fused, yaw_imu, left_enc, right_enc, ds, dtheta_enc])

    if step_count % SCAN_SAVE_EVERY == 0:
        ranges = list(lidar.getRangeImage())
        fov = lidar.getFov()
        res = lidar.getHorizontalResolution()
        angle_min = -0.5 * fov
        angle_increment = fov / max(res - 1, 1)

        payload = {
            "step": step_count,
            "timestamp": t,
            "robot_pose_odom": {
                "x": x_m,
                "y": y_m,
                "yaw_fused": yaw_fused,
                "yaw_imu": yaw_imu
            },
            "lidar_pose_relative_to_robot": {
                "x_offset": LIDAR_X_OFFSET,
                "y_offset": LIDAR_Y_OFFSET,
                "yaw_offset": LIDAR_YAW_OFFSET
            },
            "lidar_meta": {
                "fov": fov,
                "horizontal_resolution": res,
                "angle_min": angle_min,
                "angle_increment": angle_increment
            },
            "ranges": [None if not math.isfinite(r) else float(r) for r in ranges]
        }

        with open(LIDAR_DIR / f"scan_{scan_id:06d}.json", "w") as f:
            json.dump(payload, f, indent=2)

        scan_id += 1

pose_csv.close()