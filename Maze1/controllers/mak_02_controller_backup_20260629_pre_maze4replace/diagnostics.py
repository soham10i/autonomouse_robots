"""Fault detection + real-time monitoring for the mak_02 controller.

A single ``FaultMonitor`` is fed one ``snapshot`` dict per tick (assembled by the
controller from every subsystem).  It does three things:

1. **Telemetry** — appends a compact JSON row per tick to ``diag/telemetry.jsonl``
   so a whole run can be replayed/plotted offline.
2. **Fault rules** — evaluates a set of cross-subsystem checks and raises an
   ALERT when one trips (and CLEARS it when it recovers), written to
   ``diag/alerts.log`` and printed once on each state change (no spam).
3. **Live dashboard** — every ``DIAG_DASHBOARD_PERIOD_S`` prints a multi-section
   status block (SENSORS / SLAM / MAP / DEPTH-FWD / FLOATWALL / PLAN / DWA / FSM)
   so you can watch which subsystem is active and where it's failing in real time.

The module is Webots-independent (json/os/time/numpy-free) so it can be unit
tested.  All numeric inputs are plain floats/ints the controller computes.
"""
from __future__ import annotations

import json
import math
import os
import time

import settings as S


# severity ordering for display
_SEV = {"INFO": 0, "WARN": 1, "FAIL": 2}


def _f(x, nd=2, inf="inf"):
    """Format a float compactly, tolerating None/inf/nan."""
    if x is None:
        return "-"
    try:
        if math.isinf(x):
            return inf
        if math.isnan(x):
            return "nan"
    except TypeError:
        return str(x)
    return f"{x:.{nd}f}"


class FaultMonitor:
    def __init__(self, out_dir, enabled=True):
        self.enabled = bool(enabled)
        self.active = {}          # key -> (severity, msg) currently-tripped alerts
        self._streak = {}         # rule -> consecutive-trip counter
        self._last_dash = -1e9
        self._tel = None
        self._alog = None
        if not self.enabled:
            return
        try:
            os.makedirs(out_dir, exist_ok=True)
            self._tel = open(os.path.join(out_dir, "telemetry.jsonl"), "w")
            self._alog = open(os.path.join(out_dir, "alerts.log"), "w")
            self._log_alert("INFO", "MONITOR", "fault monitor started", 0.0, 0)
        except Exception as e:
            print("[diag] could not open log files:", e, flush=True)
            self._tel = self._alog = None

    # ------------------------------------------------------------- public
    def event(self, tag, msg, t=0.0, tick=0):
        """Log a discrete decision/transition event (FSM, recovery, bumper...)."""
        if not self.enabled:
            return
        line = f"[{_f(t)}s #{tick}] EVENT {tag}: {msg}"
        print("[diag] " + line, flush=True)
        self._write_alog(line)

    def record(self, snap):
        """Ingest one per-tick snapshot: telemetry + fault eval + dashboard."""
        if not self.enabled:
            return
        t = snap.get("t", 0.0)
        tick = int(snap.get("tick", 0))
        if self._tel is not None and tick % max(1, S.DIAG_TELEMETRY_EVERY_TICKS) == 0:
            try:
                self._tel.write(json.dumps(snap, default=float) + "\n")
            except Exception:
                pass
        self._eval_faults(snap, t, tick)
        if t - self._last_dash >= S.DIAG_DASHBOARD_PERIOD_S:
            self._last_dash = t
            self._print_dashboard(snap)

    def close(self):
        if not self.enabled:
            return
        try:
            if self._tel:
                self._tel.flush(); self._tel.close()
            if self._alog:
                self._alog.flush(); self._alog.close()
        except Exception:
            pass

    # ------------------------------------------------------------- internals
    def _bump(self, rule, cond):
        """Track a consecutive-trip streak; return the streak length (0 if reset)."""
        n = self._streak.get(rule, 0)
        n = n + 1 if cond else 0
        self._streak[rule] = n
        return n

    def _raise(self, alerts, key, sev, msg):
        alerts[key] = (sev, msg)

    def _eval_faults(self, snap, t, tick):
        a = {}
        v = abs(snap.get("cmd_v", 0.0) or 0.0)
        moving = v > 0.03

        # ---- sensors ----
        if not snap.get("depth_ok", True) and moving:
            self._raise(a, "DEPTH_DEAD", "FAIL", "depth frame missing while moving")
        elif moving and snap.get("depth_valid_frac", 1.0) < 0.03:
            self._raise(a, "DEPTH_BLANK", "WARN",
                        f"depth almost all-invalid ({_f(snap.get('depth_valid_frac'))})")
        if snap.get("lidar_valid", 999) < 40:
            self._raise(a, "LIDAR_SPARSE", "WARN",
                        f"lidar only {snap.get('lidar_valid')} valid beams")

        # ---- SLAM ----
        if self._bump("slam", not snap.get("sm_ok", True)) >= S.DIAG_SLAM_LOST_STREAK:
            self._raise(a, "SLAM_LOST", "WARN",
                        f"scan-match rejected x{self._streak['slam']} "
                        f"(norm={_f(snap.get('sm_norm'))})")

        # ---- FLOATING WALL ahead (the headline fault) ----
        fw = snap.get("fw", {}) or {}
        d = fw.get("dist", float("inf"))
        if d < S.DIAG_FW_WARN_DIST:
            blind = fw.get("blind", False)
            mapped = fw.get("mapped", False)
            sev = "FAIL" if (d < S.DIAG_FW_CRIT_DIST and moving) else "WARN"
            self._raise(a, "FLOATWALL_AHEAD", sev,
                        f"floating wall {_f(d)}m ahead "
                        f"src={fw.get('src')} mapped={mapped} blind={blind} v={_f(v)}")

        # ---- depth sees it but it isn't on the map (marking-pipeline gap) ----
        fp = snap.get("depth_fwd", {}) or {}
        ib = fp.get("min_inband_range", float("inf"))
        if (ib < S.DEPTH_CLEAR_RANGE and ib > S.DEPTH_OBS_MIN_RANGE
                and not fw.get("mapped", False)):
            self._raise(a, "DEPTH_UNMAPPED", "WARN",
                        f"depth SEES in-band obstacle ahead r={_f(ib)}m "
                        f"z={_f(fp.get('min_inband_height'))} but it's NOT mapped "
                        "(marking pipeline / near-wall filter?)")

        # ---- floating wall in the blind zone & unmapped (sensor-limit) ----
        if d < S.DEPTH_OBS_MIN_RANGE and not fw.get("mapped", False) and fp.get(
                "n_inband_fwd", 0) == 0:
            self._raise(a, "FW_BLINDZONE", "WARN",
                        f"floating wall {_f(d)}m ahead is INSIDE depth blind zone "
                        "(<0.6m) and unmapped — only the bumper can catch it")

        # ---- collision suspect: pushing forward, no progress, wall ahead ----
        if (snap.get("cmd_v", 0.0) > S.BUMPER_MIN_CMD_V
                and snap.get("progress_m", 1.0) < S.DIAG_PROGRESS_MIN_M
                and d < 0.45):
            self._raise(a, "COLLISION_SUSPECT", "FAIL",
                        f"forward cmd but no progress ({_f(snap.get('progress_m'),3)}m) "
                        f"with obstacle {_f(d)}m ahead — likely contacting it")

        # ---- DWA boxed ----
        if self._bump("boxed", snap.get("dwa_boxed", False)) >= S.DIAG_BOXED_STREAK:
            self._raise(a, "DWA_BOXED", "WARN",
                        f"DWA boxed x{self._streak['boxed']} "
                        "(all forward trajectories rejected)")

        # ---- recovery in a too-tight pocket ----
        if (snap.get("state") == "RECOVERY"
                and snap.get("clearance_m", 9.0) < S.DIAG_CLEAR_TIGHT_M):
            self._raise(a, "RECOVERY_TIGHT", "WARN",
                        f"recovery clearance {_f(snap.get('clearance_m'))}m < footprint "
                        "— can't rotate without sweeping a wall")

        # ---- planning ----
        if snap.get("plan_under_floating", False):
            self._raise(a, "PLAN_UNDER_FW", "WARN",
                        "A* path passes through/near a floating-wall cell")

        self._diff_alerts(a, t, tick)

    def _diff_alerts(self, new, t, tick):
        """Print + log only the alerts that newly tripped or cleared."""
        for key, (sev, msg) in new.items():
            if key not in self.active or self.active[key] != (sev, msg):
                self._log_alert(sev, key, msg, t, tick)
        for key in list(self.active):
            if key not in new:
                self._log_alert("INFO", key, "cleared", t, tick)
        self.active = new

    def _log_alert(self, sev, key, msg, t, tick):
        line = f"[{_f(t)}s #{tick}] {sev:4s} {key}: {msg}"
        self._write_alog(line)
        tag = {"FAIL": "[FAIL]", "WARN": "[warn]", "INFO": "[ ok ]"}.get(sev, "[????]")
        print(f"[diag] {tag} {key}: {msg}", flush=True)

    def _write_alog(self, line):
        if self._alog is not None:
            try:
                self._alog.write(line + "\n")
                self._alog.flush()
            except Exception:
                pass

    # ------------------------------------------------------------- dashboard
    def _print_dashboard(self, snap):
        fw = snap.get("fw", {}) or {}
        fp = snap.get("depth_fwd", {}) or {}
        po = snap.get("pose", (0.0, 0.0, 0.0))
        sev_tags = sorted(
            (f"{k}" for k in self.active),
            key=lambda k: -_SEV.get(self.active[k][0], 0))
        lines = [
            "+-- DIAG t=%ss #%s -- state=%s phase=%s --" % (
                _f(snap.get("t")), snap.get("tick"), snap.get("state"),
                snap.get("recovery_phase")),
            "| SENSORS  lidar=%s/%s depth=%s(%s%%) ir=%s imu=%s" % (
                snap.get("lidar_valid"), snap.get("lidar_total"),
                "OK" if snap.get("depth_ok") else "DEAD",
                _f(100.0 * (snap.get("depth_valid_frac") or 0.0), 0),
                snap.get("ir"), _f(po[2])),
            "| SLAM     sm=%s %s pose=(%s,%s,%s)" % (
                _f(snap.get("sm_norm")), "acc" if snap.get("sm_ok") else "REJ",
                _f(po[0]), _f(po[1]), _f(po[2])),
            "| MAP      occ=%s dobs=%s cobs=%s vox=%s bar=%s" % (
                snap.get("occ"), snap.get("dobs"), snap.get("cobs"),
                snap.get("vox"), snap.get("bar")),
            "| DEPTHfwd valid=%s inband=%s nearest=%sm inband_r=%sm z=%s" % (
                fp.get("n_valid_fwd"), fp.get("n_inband_fwd"),
                _f(fp.get("min_range")), _f(fp.get("min_inband_range")),
                _f(fp.get("min_inband_height"))),
            "| FLOATWAL ahead=%sm src=%s mapped=%s blind=%s" % (
                _f(fw.get("dist")), fw.get("src"), fw.get("mapped"),
                fw.get("blind")),
            "| PLAN     len=%s under_fw=%s goal=%s" % (
                snap.get("plan_len"), snap.get("plan_under_floating"),
                snap.get("goal_str")),
            "| DWA      v=%s w=%s boxed=%s clear=%sm" % (
                _f(snap.get("cmd_v")), _f(snap.get("cmd_w")),
                snap.get("dwa_boxed"), _f(snap.get("clearance_m"))),
            "| FSM      stuck=%s/%ss chain=%s blue=%s%s yellow=%s%s" % (
                _f(snap.get("stuck_t")), _f(S.STUCK_TIMEOUT_S),
                snap.get("recov_chain"),
                snap.get("blue_known"), "*" if snap.get("blue_conf") else "",
                snap.get("yellow_known"), "*" if snap.get("yellow_conf") else ""),
            "| ALERTS   %s" % (", ".join(sev_tags) if sev_tags else "none"),
            "+--",
        ]
        print("\n".join(lines), flush=True)
