"""Live map / frontier visualisation.

Renders the occupancy grid, sticky poison, detected frontiers, the global A*
path, the carrot, both pillars and the robot pose into an RGB image. Saves PNG
snapshots and (optionally) shows a live OpenCV window. Degrades gracefully if
OpenCV / PIL are unavailable.
"""
from __future__ import annotations

import math
import os
from typing import Optional, Dict, List, Tuple, Any

import numpy as np
import numpy.typing as npt

import config as C
from mapping import OccupancyGrid

try:
    import cv2  # type: ignore
    _HAVE_CV2 = True
except Exception:
    cv2 = None
    _HAVE_CV2 = False

_LIVE = os.environ.get("MAK02_LIVE", "1") != "0"

# colours are RGB
_C_UNKNOWN: Tuple[int, int, int] = (128, 128, 128)
_C_FREE: Tuple[int, int, int] = (245, 245, 245)
_C_OCC: Tuple[int, int, int] = (25, 25, 25)
_C_POISON: Tuple[int, int, int] = (0, 190, 0)
_C_FRONTIER: Tuple[int, int, int] = (0, 220, 220)
_C_PATH: Tuple[int, int, int] = (255, 140, 0)
_C_ROBOT: Tuple[int, int, int] = (220, 30, 30)
_C_BLUE: Tuple[int, int, int] = (40, 80, 255)
_C_YELLOW: Tuple[int, int, int] = (245, 220, 0)
_C_CARROT: Tuple[int, int, int] = (230, 0, 230)


class Visualizer:
    """Renders the live occupancy map, costmap, planned path, frontier and pillar overlay for monitoring a run.
    
    Attributes:
        grid (OccupancyGrid): The occupancy grid mapping instance.
        scale (int): The scaling factor for rendering.
        window_ready (bool): Whether the display window is initialized.
    """
    def __init__(self, grid: OccupancyGrid, scale: int = 3) -> None:
        """Initializes the visualizer with the given grid and scale.
        
        Args:
            grid (OccupancyGrid): The occupancy grid instance.
            scale (int, optional): The scale multiplier for rendering. Defaults to 3.
        """
        self.grid = grid
        self.scale = scale
        self.window_ready = False

    def _to_px(self, wx: float, wy: float) -> Tuple[int, int]:
        """Converts world coordinates to pixel coordinates.
        
        Args:
            wx (float): World X coordinate.
            wy (float): World Y coordinate.
            
        Returns:
            Tuple[int, int]: The corresponding (x, y) pixel coordinates.
        """
        g = self.grid
        ix = (wx - g.ox) / g.res
        iy = (wy - g.oy) / g.res
        col = int(ix * self.scale)
        row = int((g.n - 1 - iy) * self.scale)
        return col, row

    def render(self, pose: Tuple[float, float, float], frontier_mask: Optional[npt.NDArray[np.bool_]], path_world: Optional[List[Tuple[float, float]]], carrot: Optional[Tuple[float, float]], pillars: Dict[str, Tuple[float, float]], state: str, t: float) -> npt.NDArray[np.uint8]:
        """Renders the current state into an RGB image array.
        
        Args:
            pose (Tuple[float, float, float]): Robot's (x, y, theta) pose.
            frontier_mask (Optional[npt.NDArray[np.bool_]]): Boolean mask of frontier cells.
            path_world (Optional[List[Tuple[float, float]]]): The planned A* path in world coordinates.
            carrot (Optional[Tuple[float, float]]): The look-ahead carrot point.
            pillars (Dict[str, Tuple[float, float]]): Dictionary of detected pillars.
            state (str): Current mission state string.
            t (float): Current simulation time in seconds.
            
        Returns:
            npt.NDArray[np.uint8]: The rendered RGB image array.
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

    def _draw_cv(self, bgr: npt.NDArray[np.uint8], pose: Tuple[float, float, float], path_world: Optional[List[Tuple[float, float]]], carrot: Optional[Tuple[float, float]], pillars: Dict[str, Tuple[float, float]], state: str, t: float) -> None:
        """Draws overlays on the BGR image using OpenCV.
        
        Args:
            bgr (npt.NDArray[np.uint8]): The BGR image to draw on.
            pose (Tuple[float, float, float]): Robot's (x, y, theta) pose.
            path_world (Optional[List[Tuple[float, float]]]): The planned A* path in world coordinates.
            carrot (Optional[Tuple[float, float]]): The look-ahead carrot point.
            pillars (Dict[str, Tuple[float, float]]): Dictionary of detected pillars.
            state (str): Current mission state string.
            t (float): Current simulation time in seconds.
        """
        def rgb2bgr(c: Tuple[int, int, int]) -> Tuple[int, int, int]:
            return (c[2], c[1], c[0])
            
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
        """Displays the image in an OpenCV window if live mode is enabled.
        
        Args:
            img (npt.NDArray[np.uint8]): The RGB image to display.
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

    def save(self, img: npt.NDArray[np.uint8], path: str) -> bool:
        """Saves the image to the specified path.
        
        Args:
            img (npt.NDArray[np.uint8]): The RGB image to save.
            path (str): The file path where the image will be saved.
            
        Returns:
            bool: True if saving was successful, False otherwise.
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
        """Closes the OpenCV window if open."""
        if _LIVE and _HAVE_CV2:
            try:
                cv2.destroyAllWindows()
            except Exception:
                pass
