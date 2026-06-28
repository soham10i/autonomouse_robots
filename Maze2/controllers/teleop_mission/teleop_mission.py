"""Maze2 Phase 1+3 — re-uses Maze1's teleop_mission."""
import os
import sys

_HERE = os.path.dirname(os.path.abspath(__file__))
_MAZE1 = os.path.normpath(
    os.path.join(_HERE, "..", "..", "..", "Maze1",
                 "controllers", "teleop_mission")
)
if _MAZE1 not in sys.path:
    sys.path.insert(0, _MAZE1)

os.environ.setdefault("TELEOP_MISSION_MAPS_DIR",
                      os.path.join(_HERE, "maps"))

from teleop_mission import main  # noqa: E402

if __name__ == "__main__":
    main()
