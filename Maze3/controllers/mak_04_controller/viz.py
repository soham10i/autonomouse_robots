"""Live map / frontier visualisation.

Renders the occupancy grid, sticky poison, detected frontiers, the global A*
path, the carrot, both pillars and the robot pose into an RGB image.  Saves PNG
snapshots and (optionally) shows a live OpenCV window.  Degrades gracefully if
OpenCV / PIL are unavailable.
"""
from __future__ import annotations

import math
import os
from typing import Any, Dict, List, Optional, Tuple

import numpy as np
import numpy.typing as npt

import config as C

try:
    import cv2  # type: ignore
    _HAVE_CV2 = True
except Exception:
    cv2 = None
    _HAVE_CV2 = False

_LIVE = os.environ.get("MAK02_LIVE", "1") != "0"

# colours are RGB
_C_UNKNOWN = (128, 128, 128)
_C_FREE = (245, 245, 245)
_C_OCC = (25, 25, 25)
_C_POISON = (0, 190, 0)
_C_FRONTIER = (0, 220, 220)
_C_PATH = (255, 140, 0)
_C_ROBOT = (220, 30, 30)
_C_BLUE = (40, 80, 255)
_C_YELLOW = (245, 220, 0)
_C_CARROT = (230, 0, 230)


class Visualizer:
    """Renders the live occupancy map, costmap, planned path, and overlay items.

    Converts the internal metric grids into pixel representations, draws the
    robot pose, path, frontiers, and pillars on top, and displays it locally
    using OpenCV (if available) or saves it to disk via PIL.

    Attributes:
        grid: The main :class:`mapping.OccupancyGrid` instance.
        scale: Integer scale multiplier for rendering (e.g. 3x resolution).
        window_ready: ``True`` if the live OpenCV GUI window is active.
    """

    def __init__(self, grid: Any, scale: int = 3) -> None:
        """Initialise the visualizer.

        Args:
            grid: The main :class:`mapping.OccupancyGrid`.
            scale: Integer scale factor to enlarge the grid cells for display.
        """
        self.grid = grid
        self.scale = int(scale)
        self.window_ready = False

    def _to_px(self, wx: float, wy: float) -> Tuple[int, int]:
        """Convert a world coordinate to an image pixel coordinate.

        Args:
            wx: World X coordinate.
            wy: World Y coordinate.

        Returns:
            ``(col, row)`` image coordinates (scaled appropriately).
        """
        g = self.grid
        ix = (wx - g.ox) / g.res
        iy = (wy - g.oy) / g.res
        col = int(ix * self.scale)
        row = int((g.n - 1 - iy) * self.scale)
        return col, row

    def render(self, pose: Tuple[float, float, float],
               frontier_mask: Optional[npt.NDArray[np.bool_]],
               path_world: Optional[List[Tuple[float, float]]],
               carrot: Optional[Tuple[float, float]],
               pillars: Dict[str, Tuple[float, float]],
               state: str, t: float) -> npt.NDArray[np.uint8]:
        """Render the current system state into an RGB image array.

        Args:
            pose: Robot odometry pose ``(x, y, theta)``.
            frontier_mask: Optional ``(N, N)`` boolean mask of active frontiers.
            path_world: Optional list of ``(x, y)`` vertices defining the global path.
            carrot: Optional ``(x, y)`` look-ahead point for local planning.
            pillars: Dictionary of known pillar coordinates ``{"name": (x, y)}``.
            state: String label of the current FSM state (for HUD).
            t: Current simulation time (for HUD).

        Returns:
            ``(H, W, 3)`` RGB image array of ``uint8`` pixels.
        """
        g = self.grid
        n = g.n
        # base layers (note: array is [ix, iy]; flip iy so +y points up)
        occ = g.occupied_mask()
        free = g.free_mask()
        base = np.full((n, n, 3), _C_UNKNOWN, dtype=np.uint8)
        base[free] = _C_FREE
        base[occ] = _C_OCC
        base[g.poison] = _C_POISON
        if frontier_mask is not None:
            base[frontier_mask] = _C_FRONTIER
        # transpose ix,iy -> row=iy then flip vertically
        img = np.flipud(np.transpose(base, (1, 0, 2))).copy()

        if self.scale != 1:
            img = np.repeat(np.repeat(img, self.scale, axis=0), self.scale, axis=1)

        if _HAVE_CV2:
            bgr = cv2.cvtColor(img, cv2.COLOR_RGB2BGR)
            self._draw_cv(bgr, pose, path_world, carrot, pillars, state, t)
            img = cv2.cvtColor(bgr, cv2.COLOR_BGR2RGB)
        return img

    def _draw_cv(self, bgr: npt.NDArray[np.uint8],
                 pose: Tuple[float, float, float],
                 path_world: Optional[List[Tuple[float, float]]],
                 carrot: Optional[Tuple[float, float]],
                 pillars: Dict[str, Tuple[float, float]],
                 state: str, t: float) -> None:
        """Draw vector overlays (path, robot, HUD) directly onto a BGR canvas using OpenCV."""
        rgb2bgr = lambda c: (c[2], c[1], c[0])
        # path
        if path_world and len(path_world) >= 2:
            pts = [self._to_px(px, py) for px, py in path_world]
            for a, b in zip(pts[:-1], pts[1:]):
                cv2.line(bgr, a, b, rgb2bgr(_C_PATH), 2)
        # carrot
        if carrot is not None:
            cv2.circle(bgr, self._to_px(*carrot), 4, rgb2bgr(_C_CARROT), -1)
        # pillars
        for name, col in (("blue", _C_BLUE), ("yellow", _C_YELLOW)):
            p = pillars.get(name)
            if p is not None:
                cv2.circle(bgr, self._to_px(*p), 6, rgb2bgr(col), -1)
                cv2.circle(bgr, self._to_px(*p), 6, (0, 0, 0), 1)
        # robot
        rc = self._to_px(pose[0], pose[1])
        cv2.circle(bgr, rc, 5, rgb2bgr(_C_ROBOT), -1)
        hx = pose[0] + 0.28 * math.cos(pose[2])
        hy = pose[1] + 0.28 * math.sin(pose[2])
        cv2.line(bgr, rc, self._to_px(hx, hy), rgb2bgr(_C_ROBOT), 2)
        # HUD
        cv2.putText(bgr, f"{state}  t={t:5.1f}s", (8, 20),
                    cv2.FONT_HERSHEY_SIMPLEX, 0.55, (0, 0, 0), 2, cv2.LINE_AA)
        cv2.putText(bgr, f"{state}  t={t:5.1f}s", (8, 20),
                    cv2.FONT_HERSHEY_SIMPLEX, 0.55, (255, 255, 255), 1, cv2.LINE_AA)

    def show(self, img: npt.NDArray[np.uint8]) -> None:
        """Display an RGB image frame in a live OpenCV window.

        No-op if OpenCV is unavailable or if the ``MAK02_LIVE`` env var is disabled.

        Args:
            img: ``(H, W, 3)`` RGB image array.
        """
        if not (_LIVE and _HAVE_CV2):
            return
        try:
            bgr = cv2.cvtColor(img, cv2.COLOR_RGB2BGR)
            cv2.imshow("mak_04 — Maze3 navigation", bgr)
            cv2.waitKey(1)
            self.window_ready = True
        except Exception:
            pass

    def save(self, img: npt.NDArray[np.uint8], path: str) -> bool:
        """Save an RGB image frame to disk as a PNG.

        Args:
            img: ``(H, W, 3)`` RGB image array.
            path: Target file path (e.g. ``"live_map.png"``).

        Returns:
            ``True`` if saved successfully, ``False`` otherwise.
        """
        try:
            if _HAVE_CV2:
                cv2.imwrite(path, cv2.cvtColor(img, cv2.COLOR_RGB2BGR))
                return True
        except Exception:
            pass
        try:
            from PIL import Image  # type: ignore
            Image.fromarray(img).save(path)
            return True
        except Exception:
            return False

    def close(self) -> None:
        """Close the live OpenCV window, if open."""
        if _LIVE and _HAVE_CV2:
            try:
                cv2.destroyAllWindows()
            except Exception:
                pass
