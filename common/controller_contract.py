"""Uniform behavioural contract shared by every maze navigation controller.

Each maze ships its own tuned ``NavigationController`` (the internals differ per
maze: mapping, perception and config diverge), but after the fleet-wide naming
pass they all expose the *same* method surface. This module declares that
surface as a :class:`typing.Protocol` so it can be referenced by the wrapper and
by type checkers without forcing the controllers to import or inherit anything —
keeping the running controllers completely decoupled from this package.

``CONTRACT_METHODS`` is the authoritative allow-list the wrapper uses to expose
"only the methods needed to implement logic" for any maze.
"""
from __future__ import annotations

from typing import List, Protocol, runtime_checkable

#: The methods any maze controller is guaranteed to provide. The wrapper exposes
#: exactly these and nothing else, so external logic cannot reach into a
#: controller's private state.
CONTRACT_METHODS: List[str] = [
    "update_sensing",
    "run_slam_step",
    "update_perception",
    "refresh_costmap",
    "plan_path_to",
    "select_frontier_goal",
    "drive_along_path",
    "step_mission",
    "finalize",
    "run",
]


@runtime_checkable
class MazeControllerContract(Protocol):
    """Structural interface implemented by every maze ``NavigationController``.

    A controller satisfies this contract simply by defining these methods; no
    inheritance is required. See :data:`CONTRACT_METHODS` for the canonical list
    the wrapper enforces at runtime.
    """

    def update_sensing(self) -> None:
        """Read odometry/IMU/lidar and refresh the current pose belief."""

    def run_slam_step(self) -> None:
        """Scan-match against the map and integrate the latest scan."""

    def update_perception(self) -> None:
        """Update pillar detections and the poison-floor layer from the camera."""

    def refresh_costmap(self, sim_time: float, force: bool = False) -> None:
        """Rebuild the planning costmap when stale (or when ``force``)."""

    def plan_path_to(self, goal_world: object) -> object:
        """Return an A* world-space path to ``goal_world`` (or ``None``)."""

    def select_frontier_goal(self) -> object:
        """Choose the best reachable exploration frontier."""

    def drive_along_path(self, path: object, v_cap: object = None) -> object:
        """Produce a ``(v, w)`` command that follows ``path`` via the planner."""

    def step_mission(self, sim_time: float) -> object:
        """Advance the mission state machine one tick; return ``(v, w)``."""

    def finalize(self) -> None:
        """Stop the robot and write final map/timing/telemetry artifacts."""

    def run(self) -> None:
        """Execute the full sense-plan-act loop until the mission completes."""
