import math
import unittest

import tests._bootstrap  # noqa: F401
import numpy as np

from mapping.depth_model import DepthModel


class TestDepthModel(unittest.TestCase):
    def _model(self):
        return DepthModel(width=20, height=20, fov=1.0, mount_z=0.165,
                          depth_min=0.05, depth_max=10.0)

    def test_chassis_height_row_marked(self):
        dm = self._model()
        depth = np.full((20, 20), np.inf, dtype=np.float32)
        depth[10, :] = 1.0                  # centre row => z ~ mount_z (in band)
        pts = dm.aux_points_world(depth, (0.0, 0.0, 0.0))
        self.assertGreater(pts.shape[0], 0)
        # thin virtual scan: at most one point per column, all at ~1 m ahead
        self.assertLessEqual(pts.shape[0], 20)
        self.assertTrue(np.all(np.abs(pts[:, 0] - 1.0) < 1e-3))

    def test_thin_scan_keeps_nearest_per_column(self):
        # A column with a near AND a far in-band obstacle must keep the NEAR one
        # (this is what makes walls thin instead of a smeared band).
        dm = self._model()
        depth = np.full((20, 20), np.inf, dtype=np.float32)
        depth[9, :] = 1.5                   # far, in band
        depth[11, :] = 0.8                  # near, in band
        pts = dm.aux_points_world(depth, (0.0, 0.0, 0.0))
        self.assertGreater(pts.shape[0], 0)
        self.assertTrue(np.all(np.abs(pts[:, 0] - 0.8) < 1e-3), "kept the far hit")

    def test_high_row_excluded(self):
        dm = self._model()
        depth = np.full((20, 20), np.inf, dtype=np.float32)
        depth[0, :] = 1.0                   # top row => z well above ROBOT_HEIGHT
        pts = dm.aux_points_world(depth, (0.0, 0.0, 0.0))
        self.assertEqual(pts.shape[0], 0)

    def test_all_invalid_returns_empty(self):
        dm = self._model()
        depth = np.full((20, 20), np.inf, dtype=np.float32)
        pts = dm.aux_points_world(depth, (0.0, 0.0, 0.0))
        self.assertEqual(pts.shape[0], 0)

    def test_depth_rays_in_band_wall_makes_hit(self):
        # An in-band (collision-height) wall straight ahead -> obstacle hits ~1 m.
        dm = self._model()
        depth = np.full((20, 20), np.inf, dtype=np.float32)
        depth[10, :] = 1.0                  # centre row => z ~ mount_z (in band)
        hits, frees = dm.depth_rays_world(depth, (0.0, 0.0, 0.0), clear_range=2.0)
        self.assertGreater(hits.shape[0], 0)
        self.assertTrue(np.all(np.abs(hits[:, 0] - 1.0) < 1e-3))

    def test_depth_rays_high_wall_is_clear_not_hit(self):
        # A wall only ABOVE robot height -> drive-under -> no hit, tunnel cleared.
        dm = self._model()
        depth = np.full((20, 20), np.inf, dtype=np.float32)
        depth[0, :] = 1.0                   # top row => z well above ROBOT_HEIGHT
        hits, frees = dm.depth_rays_world(depth, (0.0, 0.0, 0.0), clear_range=2.0)
        self.assertEqual(hits.shape[0], 0, "drive-under wall wrongly marked obstacle")
        self.assertGreater(frees.shape[0], 0, "tunnel not cleared")


if __name__ == "__main__":
    unittest.main()
