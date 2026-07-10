"""Runtime observability for the maze navigation controllers.

This module is the single source of truth for how every controller reports what
it is doing and — more importantly for field debugging — what went wrong. It is
deliberately dependency-free (standard library only) so it can be vendored
verbatim into each Webots controller folder, which must stay self-contained and
importable without any package on ``sys.path`` beyond its own directory.

It provides three primitives used uniformly across the fleet:

* :func:`get_logger` — a configured, leveled :class:`logging.Logger` that writes
  human-readable lines to the console. Existing operator-facing messages are
  preserved through this so nothing a user used to see on stdout is lost.
* :class:`RunEventLog` — a structured, append-only JSON-Lines event log written
  alongside each run's map output. It captures the machine-readable timeline of
  a mission (state transitions, sensor/hardware faults, exceptions with
  tracebacks, timing) so a failure can be reconstructed after the fact, the way
  an enterprise service ships structured telemetry.
* :func:`guarded_stage` — a context manager that isolates one pipeline stage of
  the control loop: on an unexpected exception it records the fault (console +
  event log, with a traceback and the current tick/state context) and lets the
  loop continue, so a single bad sensor frame can never abort the whole mission.

Nothing here ever raises on the hot path: telemetry failures are swallowed and
downgraded to a warning, because losing a log line must never crash the robot.
"""
from __future__ import annotations

import contextlib
import json
import logging
import os
import sys
import time
import traceback
from typing import Any, Dict, Iterator, Optional

__all__ = ["get_logger", "RunEventLog", "guarded_stage"]

_LOG_FORMAT = "%(asctime)s %(levelname)-7s [%(name)s] %(message)s"
_DATE_FORMAT = "%H:%M:%S"


def get_logger(name: str, level: int = logging.INFO) -> logging.Logger:
    """Returns a console logger configured once per name.

    The logger is idempotent: repeated calls with the same `name` reuse the
    existing handler instead of stacking duplicates. Propagation is disabled so
    controller output does not also bubble up to the Webots root logger.

    Args:
        name (str): Dotted logger name, e.g. "navctl.maze5".
        level (int, optional): Minimum logging level to emit. Defaults to logging.INFO.

    Returns:
        logging.Logger: The configured logger instance.
    """
    logger = logging.getLogger(name)
    logger.setLevel(level)
    logger.propagate = False
    if not any(getattr(h, "_navctl_console", False) for h in logger.handlers):
        handler = logging.StreamHandler(stream=sys.stdout)
        handler.setFormatter(logging.Formatter(_LOG_FORMAT, _DATE_FORMAT))
        handler._navctl_console = True  # type: ignore[attr-defined]
        logger.addHandler(handler)
    return logger


class RunEventLog:
    """Append-only, structured JSON-Lines telemetry for a single run.

    Each call to :meth:`event` writes one self-describing JSON object per line to
    ``<output_dir>/<filename>``. Lines are flushed immediately so a mission that
    is killed mid-run still leaves a complete, parseable trail up to the failure.

    The log is best-effort: if the file cannot be opened or written (read-only
    volume, disk full) the error is reported once via ``logger`` and every
    subsequent event becomes a silent no-op, so telemetry problems never
    propagate into the control loop.
    """

    def __init__(self, output_dir: str, filename: str = "run_events.jsonl",
                 logger: Optional[logging.Logger] = None) -> None:
        """Opens the event log under the specified output directory.

        Args:
            output_dir (str): Directory to write the JSONL file into (created if missing).
            filename (str, optional): Event-log file name. Defaults to "run_events.jsonl".
            logger (Optional[logging.Logger], optional): Logger used to report telemetry faults. Defaults to None.
        """
        self._logger = logger
        self._path = os.path.join(output_dir, filename)
        self._fh = None
        try:
            os.makedirs(output_dir, exist_ok=True)
            self._fh = open(self._path, "a", encoding="utf-8")
        except OSError as exc:  # pragma: no cover - environment dependent
            if self._logger is not None:
                self._logger.warning("event log disabled (%s): %s", self._path, exc)

    @property
    def path(self) -> str:
        """str: Absolute or relative path of the backing JSONL file."""
        return self._path

    def event(self, name: str, level: str = "INFO", *,
              sim_time: Optional[float] = None, tick: Optional[int] = None,
              state: Optional[str] = None, **data: Any) -> None:
        """Records one structured event to the log.

        Args:
            name (str): Event identifier, e.g. "state_transition" or "fault".
            level (str, optional): Severity label ("INFO", "WARNING", "ERROR"). Defaults to "INFO".
            sim_time (Optional[float], optional): Simulation clock time, if known.
            tick (Optional[int], optional): Control-loop iteration counter, if known.
            state (Optional[str], optional): Current mission state label, if known.
            **data (Any): Arbitrary JSON-serialisable payload for the event.
        """
        if self._fh is None:
            return
        record: Dict[str, Any] = {
            "ts": round(time.time(), 3),
            "event": name,
            "level": level,
        }
        if sim_time is not None:
            record["sim_time"] = round(float(sim_time), 3)
        if tick is not None:
            record["tick"] = int(tick)
        if state is not None:
            record["state"] = state
        if data:
            record["data"] = data
        try:
            self._fh.write(json.dumps(record, default=str) + "\n")
            self._fh.flush()
        except (OSError, ValueError) as exc:  # pragma: no cover
            if self._logger is not None:
                self._logger.warning("event write failed: %s", exc)

    def close(self) -> None:
        """Flushes and closes the backing file; safe to call more than once."""
        if self._fh is not None:
            with contextlib.suppress(OSError):
                self._fh.close()
            self._fh = None


@contextlib.contextmanager
def guarded_stage(stage: str, logger: logging.Logger,
                  event_log: Optional[RunEventLog] = None, *,
                  sim_time: Optional[float] = None, tick: Optional[int] = None,
                  state: Optional[str] = None,
                  reraise: bool = False) -> Iterator[None]:
    """Runs one control-loop stage with fault isolation.

    Wraps a single pipeline stage (sensing, SLAM, perception, planning, driving)
    so that an unexpected exception inside it is captured instead of aborting the
    whole mission. The fault is logged with a full traceback and mirrored into
    the event log together with the tick/state context needed to trace it back
    to a specific sensor frame or transition.

    Args:
        stage (str): Human-readable stage name for the fault record.
        logger (logging.Logger): Logger to report the fault through.
        event_log (Optional[RunEventLog], optional): Structured log to also record the fault into.
        sim_time (Optional[float], optional): Simulation time at the point of the call.
        tick (Optional[int], optional): Control-loop iteration counter.
        state (Optional[str], optional): Current mission state label.
        reraise (bool, optional): If True, re-raise after logging. Defaults to False.

    Yields:
        Iterator[None]: Control to the wrapped block.
    """
    try:
        yield
    except Exception as exc:  # noqa: BLE001 - deliberate broad fault barrier
        logger.error("stage %r failed at tick=%s state=%s: %s",
                     stage, tick, state, exc, exc_info=True)
        if event_log is not None:
            event_log.event("fault", level="ERROR", sim_time=sim_time, tick=tick,
                            state=state, stage=stage,
                            error=f"{type(exc).__name__}: {exc}",
                            traceback=traceback.format_exc())
        if reraise:
            raise
