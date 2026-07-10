"""Live map and frontier visualization tooling.

Renders the occupancy grid, sticky poison layer, detected frontiers, global A* path,
pure pursuit carrot, mapped pillars, and the robot pose into a cohesive RGB image.
Supports saving PNG snapshots and (optionally) displaying a live OpenCV window.
The module degrades gracefully if OpenCV or PIL dependencies are unavailable.
"""
from __future__ import annotations

import math
import os
from typing import Any, Optional

import numpy as np

import config as C

try:
    import cv2  # type: ignore
    _HAVE_CV2 = True
except Exception:
    cv2 = None
    _HAVE_CV2 = False

_LIVE = os.environ.get("MAK02_LIVE", "1") != "0"

# Colors are specified in RGB format.
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
    """Renders the live occupancy map, costmap, planned path, and state overlay.

    Attributes:
        grid: The OccupancyGrid instance providing map geometry and layers.
        scale (int): Display scaling factor mapping one grid cell to `scale` pixels.
        window_ready (bool): Indicates if the live OpenCV window has been successfully opened.
    """

    def __init__(self, grid: Any, scale: int = 3) -> None:
        """Initializes the visualizer instance.

        Args:
            grid (OccupancyGrid): The underlying log-odds occupancy map.
            scale (int, optional): Spatial multiplier for the output image. Defaults to 3.
        """
        self.grid = grid
        self.scale = scale
        self.window_ready = False

    def _to_px(self, wx: float, wy: float) -> tuple[int, int]:
        """Converts world coordinates to image pixel indices.

        Args:
            wx (float): World x-coordinate in meters.
            wy (float): World y-coordinate in meters.

        Returns:
            tuple[int, int]: The corresponding `(column, row)` pixel indices.
        """
        g = self.grid
        ix = (wx - g.ox) / g.res
        iy = (wy - g.oy) / g.res
        col = int(ix * self.scale)
        row = int((g.n - 1 - iy) * self.scale)
        return col, row

    def render(
        self,
        pose: tuple[float, float, float],
        frontier_mask: Optional[np.ndarray],
        path_world: list[tuple[float, float]],
        carrot: Optional[tuple[float, float]],
        pillars: dict[str, tuple[float, float]],
        state: str,
        t: float
    ) -> np.ndarray:
        """Generates an RGB image representing the current system state.

        Composites the static occupancy grid layers (free, occupied, poison, unknown),
        overlays dynamic markers (frontiers, pillars), and draws the robot trajectory
        and state information using OpenCV (if available).

        Args:
            pose (tuple[float, float, float]): The robot's current ``(x, y, theta)`` world pose.
            frontier_mask (Optional[np.ndarray]): A 2D boolean array highlighting frontier cells.
            path_world (list[tuple[float, float]]): The A* path sequence in world coordinates.
            carrot (Optional[tuple[float, float]]): The current local-planner target waypoint.
            pillars (dict[str, tuple[float, float]]): World coordinates of known pillars keyed by name.
            state (str): The current string identifier of the mission finite state machine.
            t (float): Current simulation time in seconds.

        Returns:
            np.ndarray: An ``(H, W, 3)`` uint8 numpy array containing the composited RGB image.
        """
        g = self.grid
        n = g.n
        img = np.empty((n, n, 3), dtype=np.uint8)
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

    def _draw_cv(
        self,
        bgr: np.ndarray,
        pose: tuple[float, float, float],
        path_world: list[tuple[float, float]],
        carrot: Optional[tuple[float, float]],
        pillars: dict[str, tuple[float, float]],
        state: str,
        t: float
    ) -> None:
        """Internal helper to overlay dynamic vector graphics via OpenCV.

        Args:
            bgr (np.ndarray): The base image in BGR color space to be modified in-place.
            pose (tuple[float, float, float]): The robot world pose.
            path_world (list[tuple[float, float]]): The global path to render.
            carrot (Optional[tuple[float, float]]): The local planner lookahead target.
            pillars (dict[str, tuple[float, float]]): Coordinates of identified pillars.
            state (str): Name of the current mission state.
            t (float): Simulation timestamp.
        """
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

    def show(self, img: np.ndarray) -> None:
        """Displays the rendered image in a live GUI window.

        Args:
            img (np.ndarray): The RGB image generated by `render()`.
        """
        if not (_LIVE and _HAVE_CV2):
            return
        try:
            bgr = cv2.cvtColor(img, cv2.COLOR_RGB2BGR)
            cv2.imshow("mak_02 — Maze5 frontier exploration", bgr)
            cv2.waitKey(1)
            self.window_ready = True
        except Exception:
            pass

    def save(self, img: np.ndarray, path: str) -> bool:
        """Saves the rendered RGB image to disk as a PNG file.

        Prioritizes OpenCV for writing; falls back to PIL if OpenCV is absent.

        Args:
            img (np.ndarray): The RGB image to save.
            path (str): The destination file path.

        Returns:
            bool: True if the file was saved successfully, False otherwise.
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
        """Destroys any live GUI windows instantiated by the visualizer."""
        if _LIVE and _HAVE_CV2:
            try:
                cv2.destroyAllWindows()
            except Exception:
                pass
