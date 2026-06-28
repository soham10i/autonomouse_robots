"""Maze2 controller — single source of truth lives in Maze1.

We add Maze1's controller dir to ``sys.path`` and re-import its main(). This
keeps the same algorithm running for both worlds without duplicating code.
"""
import os
import sys

_HERE = os.path.dirname(os.path.abspath(__file__))
_MAZE1_CTRL = os.path.normpath(
    os.path.join(_HERE, "..", "..", "..", "Maze1", "controllers", "maze_navigator")
)
if _MAZE1_CTRL not in sys.path:
    sys.path.insert(0, _MAZE1_CTRL)

# Redirect the maps/ output dir to a Maze2-local folder so dumps don't
# overwrite Maze1's.
os.environ["MAZE_NAVIGATOR_MAPS_DIR"] = os.path.join(_HERE, "maps")

from maze_navigator import main  # noqa: E402

if __name__ == "__main__":
    main()
