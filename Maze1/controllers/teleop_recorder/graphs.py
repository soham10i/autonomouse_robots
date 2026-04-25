from pathlib import Path
import json, math
import numpy as np
import matplotlib.pyplot as plt

BASE = Path("Maze1/controllers/teleop_recorder/recordings_pose")
LIDAR_DIR = BASE / "lidar"
OUT = BASE / "maps_fixed"
OUT.mkdir(parents=True, exist_ok=True)

MAX_RANGE = 5.0
scan_files = sorted(LIDAR_DIR.glob("scan_*.json"))

pts = []
traj = []

for sf in scan_files:
    with open(sf, "r") as f:
        d = json.load(f)

    try:
        pose = d["robot_pose_odom"]
        lidar_rel = d["lidar_pose_relative_to_robot"]
        meta = d.get("lidar_meta", d)

        rx = pose["x"]
        ry = pose["y"]
        ryaw = pose.get("yaw_fused", pose.get("yaw", 0.0))
    except KeyError:
        continue

    lx = lidar_rel["x_offset"]
    ly = lidar_rel["y_offset"]
    lyaw = lidar_rel["yaw_offset"]

    lidar_x = rx + lx * math.cos(ryaw) - ly * math.sin(ryaw)
    lidar_y = ry + lx * math.sin(ryaw) + ly * math.cos(ryaw)
    lidar_yaw = ryaw + lyaw

    traj.append((rx, ry))

    angle_min = meta["angle_min"]
    angle_increment = meta["angle_increment"]
    ranges = d["ranges"]

    for i, r in enumerate(ranges):
        if r is None:
            continue
        if not np.isfinite(r) or r <= 0 or r > MAX_RANGE:
            continue

        beam = angle_min + i * angle_increment
        wx = lidar_x + r * math.cos(lidar_yaw + beam)
        wy = lidar_y + r * math.sin(lidar_yaw + beam)
        pts.append((wx, wy))

pts = np.array(pts)
traj = np.array(traj)

fig, ax = plt.subplots(figsize=(9, 9))
if len(pts):
    ax.scatter(pts[:, 0], pts[:, 1], s=2, alpha=0.2, c="darkorange", label="LiDAR hits")
if len(traj):
    ax.plot(traj[:, 0], traj[:, 1], c="royalblue", lw=2, label="Trajectory")
    ax.scatter(traj[0, 0], traj[0, 1], c="green", s=80, label="Start")
    ax.scatter(traj[-1, 0], traj[-1, 1], c="red", marker="x", s=80, label="End")

ax.set_title("Pose-aware corrected LiDAR map")
ax.set_xlabel("x (m)")
ax.set_ylabel("y (m)")
ax.set_aspect("equal")
ax.grid(True, alpha=0.3)
ax.legend()
fig.tight_layout()
fig.savefig(OUT / "corrected_lidar_map.png", dpi=180, bbox_inches="tight")
plt.close(fig)