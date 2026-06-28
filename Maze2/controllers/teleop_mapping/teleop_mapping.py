"""Maze2 teleop controller — re-uses Maze1's teleop_mapping.

Same approach as the autonomous re-export: add Maze1's controller dir to
sys.path and call its main(). The maps/ output dir is overridden so the
two worlds don't trash each other's recordings.
"""
import os
import sys

_HERE = os.path.dirname(os.path.abspath(__file__))
_MAZE1_TELE = os.path.normpath(
    os.path.join(_HERE, "..", "..", "..", "Maze1", "controllers", "teleop_mapping")
)
if _MAZE1_TELE not in sys.path:
    sys.path.insert(0, _MAZE1_TELE)

os.environ["MAZE_MAPPING_OUTPUT_DIR"] = os.path.join(_HERE, "maps")

from teleop_mapping import main  # noqa: E402

if __name__ == "__main__":
    main()
