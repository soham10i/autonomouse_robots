import unittest

import tests._bootstrap  # noqa: F401
import numpy as np

import settings as S
from mapping.cloud_map import CloudMap
from slam.occupancy_grid import OccupancyGrid


class TestCloudMap(unittest.TestCase):
    def test_pack_unpack_roundtrip(self):
        v = np.array([[0, 0, 0], [5, -3, 12], [-130, 134, 40]], dtype=np.int64)
        keys = CloudMap._pack(v)
        out = CloudMap._unpack(keys)
        self.assertTrue(np.array_equal(v, out))

    def test_accumulate_counts_per_voxel(self):
        cm = CloudMap(voxel=0.1, z_min=-1.0, z_max=2.0)
        # three points in the same voxel, one in another
        pts = np.array([[0.01, 0.01, 0.5], [0.02, 0.03, 0.5],
                        [0.04, 0.01, 0.5], [0.55, 0.0, 0.5]])
        cm.add_points_world(pts)
        self.assertEqual(len(cm), 2)
        _, cnt = cm.points()
        self.assertEqual(sorted(cnt.tolist()), [1, 3])

    def test_height_band_filter_on_add(self):
        cm = CloudMap(voxel=0.1, z_min=0.0, z_max=1.0)
        pts = np.array([[0.0, 0.0, -0.5],    # below band -> dropped
                        [0.0, 0.0, 1.5],     # above band -> dropped
                        [0.0, 0.0, 0.5]])    # in band -> kept
        n = cm.add_points_world(pts)
        self.assertEqual(n, 1)
        self.assertEqual(len(cm), 1)

    def test_project_to_2d_marks_footprint_cell(self):
        grid = OccupancyGrid()
        cm = CloudMap(voxel=0.05)
        # a vertical stack of in-band points at world (1.0, 0.5): a floating wall
        z = np.linspace(S.CLOUD_OBS_Z_MIN + 0.01, S.CLOUD_OBS_Z_MAX - 0.01, 6)
        pts = np.stack([np.full_like(z, 1.0), np.full_like(z, 0.5), z], axis=1)
        # repeat to exceed CLOUD_OBS_MIN_HITS per voxel
        cm.add_points_world(np.vstack([pts] * (S.CLOUD_OBS_MIN_HITS + 1)))
        mask = cm.project_to_2d(grid)
        # the footprint lands at the voxel-centre cell, within one voxel of (1,0.5)
        ix, iy = grid.world_to_grid(1.0, 0.5)
        self.assertTrue(mask[ix - 2:ix + 3, iy - 2:iy + 3].any())

    def test_project_excludes_overhead_passage(self):
        grid = OccupancyGrid()
        cm = CloudMap(voxel=0.05)
        # points ALL above the collision band (a gantry/passage to drive under)
        z = np.full(8, S.CLOUD_OBS_Z_MAX + 0.20)
        pts = np.stack([np.full_like(z, 1.0), np.full_like(z, 0.5), z], axis=1)
        cm.add_points_world(np.vstack([pts] * (S.CLOUD_OBS_MIN_HITS + 1)))
        mask = cm.project_to_2d(grid)
        self.assertFalse(mask.any())

    def test_min_hits_gate(self):
        grid = OccupancyGrid()
        cm = CloudMap(voxel=0.05)
        # a single hit in band -> below CLOUD_OBS_MIN_HITS -> not projected
        cm.add_points_world(np.array([[1.0, 0.5, 0.10]]))
        mask = cm.project_to_2d(grid)
        self.assertFalse(mask.any())


class TestGridCloudObs(unittest.TestCase):
    def test_cloud_obs_lethal_only_when_enabled(self):
        # By default the accumulating cloud is OUT of the planner (it smears the
        # map shut); it is lethal only when use_cloud_obs is explicitly enabled.
        grid = OccupancyGrid()
        mask = np.zeros((grid.cells, grid.cells), dtype=bool)
        ix, iy = grid.world_to_grid(1.0, 0.5)
        mask[ix, iy] = True
        grid.set_cloud_obs(mask)
        self.assertFalse(grid.lethal_mask()[ix, iy])   # decoupled by default
        grid.use_cloud_obs = True
        self.assertTrue(grid.lethal_mask()[ix, iy])    # opt-in -> lethal

    def test_footprint_clear_wipes_cloud_obs(self):
        grid = OccupancyGrid()
        mask = np.zeros((grid.cells, grid.cells), dtype=bool)
        ix, iy = grid.world_to_grid(0.0, 0.0)
        mask[ix, iy] = True
        grid.set_cloud_obs(mask)
        # the robot is standing here -> footprint clear must wipe the cloud mark
        grid.mark_free_disc(0.0, 0.0, S.ROBOT_RADIUS)
        self.assertFalse(grid.cloud_obs[ix, iy])


if __name__ == "__main__":
    unittest.main()
