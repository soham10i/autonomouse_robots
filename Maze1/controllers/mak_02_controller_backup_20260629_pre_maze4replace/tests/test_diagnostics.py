import os
import tempfile
import unittest

import tests._bootstrap  # noqa: F401
import numpy as np

import settings as S
from diagnostics import FaultMonitor
from mapping.depth_model import DepthModel


def _base_snap(**kw):
    snap = {
        "t": 1.0, "tick": 2, "state": "EXPLORE_BLUE", "recovery_phase": None,
        "lidar_valid": 380, "lidar_total": 400, "depth_ok": True,
        "depth_valid_frac": 0.7, "depth_fwd": {}, "sm_norm": 0.98, "sm_ok": True,
        "pose": [0.0, 0.0, 0.0], "occ": 100, "dobs": 10, "cobs": 0, "vox": 0,
        "bar": 0, "fw": {"dist": float("inf"), "src": None, "mapped": False,
                         "blind": False},
        "cmd_v": 0.2, "cmd_w": 0.0, "dwa_boxed": False, "clearance_m": 0.3,
        "plan_len": 10, "plan_under_floating": False, "goal_str": None,
        "progress_m": 0.2, "stuck_t": 0.0, "recov_chain": 0,
        "blue_known": False, "blue_conf": False,
        "yellow_known": False, "yellow_conf": False,
    }
    snap.update(kw)
    return snap


class TestFaultMonitor(unittest.TestCase):
    def setUp(self):
        self.tmp = tempfile.mkdtemp()
        self.mon = FaultMonitor(self.tmp, enabled=True)

    def test_floating_wall_ahead_raises_alert(self):
        snap = _base_snap(fw={"dist": 0.25, "src": "mapped", "mapped": True,
                              "blind": False}, cmd_v=0.2)
        self.mon.record(snap)
        self.assertIn("FLOATWALL_AHEAD", self.mon.active)
        self.assertEqual(self.mon.active["FLOATWALL_AHEAD"][0], "FAIL")

    def test_alert_clears_when_safe_again(self):
        self.mon.record(_base_snap(fw={"dist": 0.25, "src": "mapped",
                                       "mapped": True, "blind": False}))
        self.assertIn("FLOATWALL_AHEAD", self.mon.active)
        self.mon.record(_base_snap())            # clear, no wall ahead
        self.assertNotIn("FLOATWALL_AHEAD", self.mon.active)

    def test_depth_unmapped_pipeline_gap(self):
        # depth SEES an in-band obstacle ahead beyond the blind zone, but it's
        # not on the map -> marking-pipeline-gap alert (the key floating-wall fault)
        snap = _base_snap(
            depth_fwd={"min_inband_range": 0.9, "min_inband_height": 0.18,
                       "n_inband_fwd": 50},
            fw={"dist": 0.9, "src": "live_depth", "mapped": False, "blind": False})
        self.mon.record(snap)
        self.assertIn("DEPTH_UNMAPPED", self.mon.active)

    def test_collision_suspect(self):
        snap = _base_snap(cmd_v=0.2, progress_m=0.005,
                          fw={"dist": 0.3, "src": "mapped", "mapped": True,
                              "blind": False})
        self.mon.record(snap)
        self.assertIn("COLLISION_SUSPECT", self.mon.active)

    def test_slam_lost_needs_a_streak(self):
        for _ in range(S.DIAG_SLAM_LOST_STREAK - 1):
            self.mon.record(_base_snap(sm_ok=False, sm_norm=0.2))
        self.assertNotIn("SLAM_LOST", self.mon.active)
        self.mon.record(_base_snap(sm_ok=False, sm_norm=0.2))
        self.assertIn("SLAM_LOST", self.mon.active)

    def test_telemetry_file_written(self):
        self.mon.record(_base_snap(tick=2))
        self.mon.close()
        with open(os.path.join(self.tmp, "telemetry.jsonl")) as f:
            self.assertGreaterEqual(len(f.read().strip().splitlines()), 1)


class TestForwardObstacle(unittest.TestCase):
    def _model(self):
        return DepthModel(width=40, height=40, fov=1.04, mount_z=0.165,
                          depth_min=0.05, depth_max=8.0)

    def test_sees_inband_wall_ahead(self):
        dm = self._model()
        depth = np.full((40, 40), np.inf, dtype=np.float32)
        depth[20, :] = 1.0                       # centre row => z ~ mount_z (in band)
        out = dm.forward_obstacle(depth)
        self.assertGreater(out["n_inband_fwd"], 0)
        self.assertAlmostEqual(out["min_inband_range"], 1.0, places=3)

    def test_high_wall_not_inband(self):
        dm = self._model()
        depth = np.full((40, 40), np.inf, dtype=np.float32)
        depth[0, :] = 1.0                        # top row => z well above ROBOT_HEIGHT
        out = dm.forward_obstacle(depth)
        self.assertEqual(out["n_inband_fwd"], 0)


if __name__ == "__main__":
    unittest.main()
