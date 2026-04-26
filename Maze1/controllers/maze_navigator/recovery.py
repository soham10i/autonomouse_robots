"""Deterministic recovery routines.

Three sub-phases run in sequence:

    REVERSE  : back up a fixed distance/time
    SPIN     : rotate by a fixed angle, alternating direction on repeat
    SETTLE   : stop briefly so sensors stabilise before replanning

Stuck detection (commanded vs. actual progress) lives in ``StuckMonitor``.
"""
import math

import config as C


class StuckMonitor:
    def __init__(self):
        self._ref = None
        self._t_ref = 0.0
        self._w_hist = []
        self._w_window = 12

    def reset(self, pose=None, sim_time=0.0):
        if pose is not None:
            self._ref = (pose[0], pose[1])
            self._t_ref = sim_time
        else:
            self._ref = None
            self._t_ref = 0.0
        self._w_hist.clear()

    def update(self, pose, w_cmd, sim_time):
        x, y, _ = pose
        if self._ref is None:
            self._ref = (x, y)
            self._t_ref = sim_time
        d = math.hypot(x - self._ref[0], y - self._ref[1])
        if d > C.STUCK_PROGRESS_MIN_M:
            self._ref = (x, y)
            self._t_ref = sim_time

        self._w_hist.append(w_cmd)
        if len(self._w_hist) > self._w_window:
            self._w_hist.pop(0)

    def stuck(self, sim_time, timeout_s=None):
        timeout_s = timeout_s or C.STUCK_TIMEOUT_S
        return (sim_time - self._t_ref) > timeout_s

    def oscillating(self):
        if len(self._w_hist) < self._w_window:
            return False
        s = sum(1 for w in self._w_hist if abs(w) > C.STUCK_OSC_W_THRESHOLD)
        if s < self._w_window // 2:
            return False
        # signs flipping at least 4 times = oscillation
        flips = 0
        for a, b in zip(self._w_hist[:-1], self._w_hist[1:]):
            if a * b < 0:
                flips += 1
        return flips >= 4


class Recovery:
    def __init__(self):
        self.phase = "REVERSE"
        self.t0 = 0.0
        self.spin_dir = 1.0
        self._toggle = False

    def begin(self, sim_time, prefer_spin_dir=None):
        self.phase = "REVERSE"
        self.t0 = sim_time
        if prefer_spin_dir is not None:
            self.spin_dir = math.copysign(1.0, prefer_spin_dir)
        else:
            self._toggle = not self._toggle
            self.spin_dir = 1.0 if self._toggle else -1.0

    def step(self, sim_time):
        """Returns (v, w, done). Caller transitions out when done==True."""
        t = sim_time - self.t0
        if self.phase == "REVERSE":
            if t < C.RECOVERY_REVERSE_T:
                return -0.12, 0.0, False
            self.phase = "SPIN"
            self.t0 = sim_time
            return 0.0, 0.0, False
        if self.phase == "SPIN":
            if t < C.RECOVERY_SPIN_T:
                return 0.0, self.spin_dir * C.RECOVERY_SPIN_W, False
            self.phase = "SETTLE"
            self.t0 = sim_time
            return 0.0, 0.0, False
        if self.phase == "SETTLE":
            if t < 0.25:
                return 0.0, 0.0, False
            return 0.0, 0.0, True
        return 0.0, 0.0, True
