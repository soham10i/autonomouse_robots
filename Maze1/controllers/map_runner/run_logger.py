"""Comprehensive tick-by-tick JSONL logger for the map_runner controller.

Writes one JSON object per logged tick to ``run_log.jsonl``.  On
finalization, writes a human-readable ``mission_summary.txt``.

Usage
-----
    logger = RunLogger("/path/to/maps")
    # inside main loop:
    logger.log_tick(t=sim_time, tick=tick, state="GO_BLUE",
                    pose=(x, y, th), odom_raw=(ox, oy, oth),
                    icp_applied=True, icp_error=0.04,
                    icp_correction=(0.01, -0.005, 0.002),
                    v_cmd=0.3, w_cmd=0.1,
                    target_pillar="blue",
                    dist_to_target=1.2, path_length=2.5,
                    stuck=False, recovery_count=0)
    # on shutdown:
    logger.write_summary(final_state="DONE", ...)
    logger.close()

Post-run analysis (example)::

    import pandas as pd
    df = pd.read_json("maps/run_log.jsonl", lines=True)
    df.plot(x="t", y=["dist_to_target", "icp_error"])
"""
import json
import os
import time


class RunLogger:
    """Append-only JSONL logger for map_runner diagnostic data."""

    def __init__(self, maps_dir, filename="run_log.jsonl"):
        os.makedirs(maps_dir, exist_ok=True)
        self._path = os.path.join(maps_dir, filename)
        self._maps_dir = maps_dir
        self._fh = open(self._path, "w")
        self._tick_count = 0
        self._t_start = None
        self._icp_count = 0
        self._icp_error_sum = 0.0
        self._max_icp_error = 0.0
        self._recovery_total = 0
        print(f"[logger] logging to {self._path}", flush=True)

    # ------------------------------------------------------------------ #
    # Per-tick logging                                                     #
    # ------------------------------------------------------------------ #

    def log_tick(self, *, t, tick, state, pose,
                 odom_raw=None, icp_applied=False, icp_error=0.0,
                 icp_correction=None, v_cmd=0.0, w_cmd=0.0,
                 target_pillar=None, dist_to_target=None,
                 path_length=None, stuck=False, recovery_count=0,
                 extra=None):
        """Append a single JSON line for this simulation tick."""
        if self._t_start is None:
            self._t_start = t

        record = {
            "t": round(t, 4),
            "tick": tick,
            "state": state,
            "pose": [round(v, 5) for v in pose],
            "v_cmd": round(v_cmd, 4),
            "w_cmd": round(w_cmd, 4),
        }
        if odom_raw is not None:
            record["odom_raw"] = [round(v, 5) for v in odom_raw]
        if icp_applied:
            record["icp_applied"] = True
            record["icp_error"] = round(icp_error, 5)
            if icp_correction is not None:
                record["icp_correction"] = [round(v, 6) for v in icp_correction]
            self._icp_count += 1
            self._icp_error_sum += icp_error
            self._max_icp_error = max(self._max_icp_error, icp_error)
        if target_pillar is not None:
            record["target"] = target_pillar
        if dist_to_target is not None:
            record["dist_target"] = round(dist_to_target, 4)
        if path_length is not None:
            record["path_len"] = round(path_length, 4)
        if stuck:
            record["stuck"] = True
        if recovery_count > 0:
            record["recoveries"] = recovery_count
            self._recovery_total = max(self._recovery_total, recovery_count)
        if extra:
            record.update(extra)

        self._fh.write(json.dumps(record) + "\n")
        self._tick_count += 1

    # ------------------------------------------------------------------ #
    # Summary report                                                       #
    # ------------------------------------------------------------------ #

    def write_summary(self, *, final_state, t_end,
                      t_blue_reached=None, t_yellow_reached=None,
                      t_mission_start=None,
                      path_length_blue=0.0, path_length_yellow=0.0,
                      pillar_blue=None, pillar_yellow=None,
                      icp_corrections_total=0, poison_cells=0):
        """Write a human-readable mission summary."""
        path = os.path.join(self._maps_dir, "mission_summary.txt")
        lines = [
            "=" * 60,
            "MAP RUNNER — MISSION SUMMARY",
            "=" * 60,
            f"Final state            : {final_state}",
            f"Total sim time         : {t_end:.2f}s",
            f"Logged ticks           : {self._tick_count}",
            "",
            "── Timing ──",
        ]
        if t_mission_start is not None:
            lines.append(f"Mission started (sim)  : {t_mission_start:.2f}s")
        if t_blue_reached is not None and t_mission_start is not None:
            leg = t_blue_reached - t_mission_start
            lines.append(f"BLUE reached (sim)     : {t_blue_reached:.2f}s  (leg = {leg:.2f}s)")
        else:
            lines.append("BLUE reached           : NEVER")
        if t_yellow_reached is not None:
            ref = t_blue_reached or t_mission_start or 0.0
            leg = t_yellow_reached - ref
            total = t_yellow_reached - (t_mission_start or 0.0)
            lines.append(f"YELLOW reached (sim)   : {t_yellow_reached:.2f}s  "
                         f"(leg = {leg:.2f}s, total = {total:.2f}s)")
        else:
            lines.append("YELLOW reached         : NEVER")

        lines += [
            "",
            "── Path lengths (planned) ──",
            f"Start → BLUE           : {path_length_blue:.2f} m",
            f"BLUE  → YELLOW         : {path_length_yellow:.2f} m",
            "",
            "── ICP Scan Matching ──",
            f"Total corrections      : {icp_corrections_total}",
            f"Mean error             : {self._icp_error_sum / max(self._icp_count, 1):.4f} m",
            f"Max error              : {self._max_icp_error:.4f} m",
            "",
            "── Recovery ──",
            f"Total maneuvers        : {self._recovery_total}",
            "",
            "── Landmarks ──",
            f"BLUE pillar            : {pillar_blue}",
            f"YELLOW pillar          : {pillar_yellow}",
            f"Poison cells (final)   : {poison_cells}",
        ]

        text = "\n".join(lines) + "\n"
        with open(path, "w") as f:
            f.write(text)
        print(f"[logger] wrote {path}", flush=True)
        # Also print to console for immediate visibility
        print(text, flush=True)

    # ------------------------------------------------------------------ #
    # Cleanup                                                              #
    # ------------------------------------------------------------------ #

    def flush(self):
        """Flush the JSONL file buffer."""
        if self._fh and not self._fh.closed:
            self._fh.flush()

    def close(self):
        """Close the log file handle."""
        if self._fh and not self._fh.closed:
            self._fh.close()
            print(f"[logger] closed {self._path} "
                  f"({self._tick_count} ticks logged)", flush=True)
