"""Maze2 Phase 3 — re-uses Maze1's map_runner."""
import os
import sys

_HERE = os.path.dirname(os.path.abspath(__file__))
_MAZE1 = os.path.normpath(
    os.path.join(_HERE, "..", "..", "..", "Maze1", "controllers", "map_runner")
)
if _MAZE1 not in sys.path:
    sys.path.insert(0, _MAZE1)

# Default to Maze2's own teleop outputs unless overridden.
_MAZE2_TELEOP = os.path.normpath(
    os.path.join(_HERE, "..", "teleop_mapping", "maps")
)
os.environ.setdefault("MAP_RUNNER_NPZ",
                      os.path.join(_MAZE2_TELEOP, "map.npz"))
os.environ.setdefault("MAP_RUNNER_PLY",
                      os.path.join(_MAZE2_TELEOP, "scene.ply"))

from map_runner import main  # noqa: E402

if __name__ == "__main__":
    main()
