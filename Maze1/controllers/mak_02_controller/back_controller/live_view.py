"""Live exploration window (OpenCV if available, else matplotlib).

Renders the SLAM grid + frontiers + planned path + the trajectory actually
followed + pillars, refreshed in real time — the equivalent of the RViz view in
the frontier-exploration video.  Backend is auto-detected so it works whether or
not your Webots Python ships OpenCV:

* ``cv2``        -> a real ``cv2.imshow`` window (what you asked for).
* ``matplotlib`` -> an interactive figure (``plt.ion``).
* neither / no display -> disabled (the controller still writes PNG snapshots).

Set ``MAK02_LIVE=0`` in the environment to turn it off entirely (e.g. for
headless batch runs).
"""
from __future__ import annotations

import os

import numpy as np

from viz import _grid_to_rgb


class LiveView:
    def __init__(self, grid, upscale=3, window="mak_02 — frontier exploration"):
        self.ok = False
        self.backend = None
        self.U = upscale
        self.window = window
        self.cells = grid.cells
        self.res = grid.res
        self.origin = np.asarray(grid.origin, dtype=float)
        self._cv2 = None
        self._plt = None
        self._im = None

        if os.environ.get("MAK02_LIVE", "1") == "0":
            print("[live] disabled via MAK02_LIVE=0")
            return
        # Prefer OpenCV.
        try:
            import cv2
            self._cv2 = cv2
            cv2.namedWindow(self.window, cv2.WINDOW_NORMAL)
            cv2.resizeWindow(self.window, self.cells * self.U, self.cells * self.U)
            self.backend = "cv2"
            self.ok = True
            print("[live] OpenCV window active")
            return
        except Exception as e:
            print("[live] OpenCV unavailable:", e)
        # Fall back to matplotlib interactive.
        try:
            import matplotlib
            import matplotlib.pyplot as plt
            plt.ion()
            self._plt = plt
            self._fig, self._ax = plt.subplots(figsize=(6, 6))
            self._ax.set_title(self.window)
            self.backend = "mpl"
            self.ok = True
            print("[live] matplotlib interactive window active")
        except Exception as e:
            print("[live] no live backend available:", e)

    # --------------------------------------------------------- geometry
    def _w2px(self, wx, wy):
        ix = (wx - self.origin[0]) / self.res
        iy = (wy - self.origin[1]) / self.res
        col = int(ix * self.U + self.U / 2)
        row = int((self.cells - 1 - iy) * self.U + self.U / 2)
        return col, row

    def _base_rgb(self, grid):
        rgb = _grid_to_rgb(grid)                      # [ix, iy, 3] floats
        img = np.transpose(rgb, (1, 0, 2))[::-1]      # [row=iy(flipped), col=ix]
        return (img * 255.0).astype(np.uint8)         # RGB uint8

    # ------------------------------------------------------------- draw
    def update(self, grid, pose=None, plan=None, frontiers=None, goal=None,
               pillars=None, traj=None, text=None):
        if not self.ok:
            return
        if self.backend == "cv2":
            self._update_cv2(grid, pose, plan, frontiers, goal, pillars, traj, text)
        else:
            self._update_mpl(grid, pose, plan, frontiers, goal, pillars, traj, text)

    def _update_cv2(self, grid, pose, plan, frontiers, goal, pillars, traj, text):
        cv2 = self._cv2
        rgb = self._base_rgb(grid)
        img = cv2.cvtColor(rgb, cv2.COLOR_RGB2BGR)
        img = cv2.resize(img, (self.cells * self.U, self.cells * self.U),
                         interpolation=cv2.INTER_NEAREST)

        if traj and len(traj) > 1:
            pts = np.array([self._w2px(p[0], p[1]) for p in traj], dtype=np.int32)
            cv2.polylines(img, [pts], False, (0, 200, 0), 1)          # followed: green
        if plan and len(plan) > 1:
            pts = np.array([self._w2px(p[0], p[1]) for p in plan], dtype=np.int32)
            cv2.polylines(img, [pts], False, (255, 120, 0), 2)        # plan: blue
        if frontiers:
            for f in frontiers:
                c, r = self._w2px(*f.centroid_world)
                cv2.drawMarker(img, (c, r), (0, 140, 230), cv2.MARKER_TILTED_CROSS, 8, 1)
        if goal is not None:
            c, r = self._w2px(goal[0], goal[1])
            cv2.circle(img, (c, r), 6, (0, 215, 255), -1)             # goal: yellow
        if pillars:
            colmap = {"blue": (255, 60, 0), "yellow": (0, 215, 255)}
            for name, p in pillars.items():
                if p is None:
                    continue
                c, r = self._w2px(p[0], p[1])
                cv2.circle(img, (c, r), 7, colmap.get(name, (0, 0, 255)), -1)
                cv2.circle(img, (c, r), 7, (0, 0, 0), 1)
        if pose is not None:
            import math
            c, r = self._w2px(pose[0], pose[1])
            c2, r2 = self._w2px(pose[0] + 0.25 * math.cos(pose[2]),
                                pose[1] + 0.25 * math.sin(pose[2]))
            cv2.circle(img, (c, r), 4, (0, 0, 255), -1)               # pose: red
            cv2.line(img, (c, r), (c2, r2), (0, 0, 255), 2)
        if text:
            y = 16
            for line in text.split("\n"):
                cv2.putText(img, line, (8, y), cv2.FONT_HERSHEY_SIMPLEX, 0.4,
                            (20, 20, 20), 1, cv2.LINE_AA)
                y += 16
        cv2.imshow(self.window, img)
        cv2.waitKey(1)

    def _update_mpl(self, grid, pose, plan, frontiers, goal, pillars, traj, text):
        plt = self._plt
        ax = self._ax
        ax.clear()
        rgb = _grid_to_rgb(grid)
        ext = [self.origin[0], self.origin[0] + self.cells * self.res,
               self.origin[1], self.origin[1] + self.cells * self.res]
        ax.imshow(rgb.transpose(1, 0, 2), origin="lower", extent=ext,
                  interpolation="nearest")
        if traj and len(traj) > 1:
            ax.plot([p[0] for p in traj], [p[1] for p in traj], "-",
                    color="#2ca02c", lw=1)
        if plan and len(plan) > 1:
            ax.plot([p[0] for p in plan], [p[1] for p in plan], "-",
                    color="#1f77b4", lw=2)
        if frontiers:
            ax.plot([f.centroid_world[0] for f in frontiers],
                    [f.centroid_world[1] for f in frontiers], "x", color="#e67e22")
        if goal is not None:
            ax.plot([goal[0]], [goal[1]], "*", color="#f1c40f", markersize=14,
                    markeredgecolor="k")
        if pillars:
            colmap = {"blue": "#1f77b4", "yellow": "#f1c40f"}
            for name, p in pillars.items():
                if p is not None:
                    ax.plot([p[0]], [p[1]], "o", color=colmap.get(name, "r"),
                            markersize=9, markeredgecolor="k")
        if pose is not None:
            import math
            ax.plot([pose[0]], [pose[1]], "r.", markersize=8)
            ax.plot([pose[0], pose[0] + 0.25 * math.cos(pose[2])],
                    [pose[1], pose[1] + 0.25 * math.sin(pose[2])], "r-")
        if text:
            ax.set_title(text.replace("\n", " | "), fontsize=8)
        ax.set_aspect("equal")
        plt.pause(0.001)

    def close(self):
        if self.backend == "cv2" and self._cv2 is not None:
            try:
                self._cv2.destroyWindow(self.window)
            except Exception:
                pass
        elif self.backend == "mpl" and self._plt is not None:
            try:
                self._plt.close(self._fig)
            except Exception:
                pass
