"""Uniform access wrapper over the per-maze navigation controllers.

Every maze keeps its own self-contained, tuned controller package under
``Maze<N>/controllers/<name>/``. Those packages are intentionally *not* aware of
this module — this wrapper is a pure consumer that lives outside every maze
folder, so importing or changing it can never affect the code that actually runs
in Webots.

:class:`MazeControllerWrapper` resolves a maze id to its controller package,
imports that package's ``NavigationController`` in isolation (the maze packages
all reuse the same internal module names such as ``config`` and ``mapping``, so
each load purges those names first), and then exposes **only** the methods listed
in :data:`common.controller_contract.CONTRACT_METHODS`. That gives any external
orchestration one stable interface to drive or introspect any maze without
reaching into a controller's private state.

Instantiating a controller requires the Webots ``controller`` runtime (the
robot's motors, lidar and cameras). For discovery, type checking, and CI-style
smoke tests without a simulator, pass ``stub_hardware=True`` to install a minimal
fake ``controller`` module — the same trick the controllers' own ``selftest.py``
uses.
"""
from __future__ import annotations

import importlib
import os
import sys
import types
from pathlib import Path
from typing import Any, Dict, List, Optional

_HERE = Path(__file__).resolve().parent
if str(_HERE) not in sys.path:
    sys.path.insert(0, str(_HERE))

from controller_contract import CONTRACT_METHODS  # noqa: E402

#: Repository root that contains every ``Maze<N>/`` folder (parent of ``common/``).
_REPO_ROOT = _HERE.parent

#: Maze id -> (controller directory relative to repo root, entry module name).
#: The controller *class* is uniformly ``NavigationController`` in every maze.
_REGISTRY: Dict[str, Dict[str, str]] = {
    "Maze1": {"dir": "Maze1/controllers/mak_02_controller", "module": "mak_02_controller"},
    "Maze2": {"dir": "Maze2/controllers/mak_02_controller", "module": "mak_02_controller"},
    "Maze3": {"dir": "Maze3/controllers/mak_04_controller", "module": "mak_04_controller"},
    "Maze4": {"dir": "Maze4/controllers/mak_02_controller", "module": "mak_02_controller"},
    "Maze5": {"dir": "Maze5/controllers/mak_02_controller", "module": "mak_02_controller"},
}

_CONTROLLER_CLASS_NAME = "NavigationController"

#: Internal module names shared (by coincidence of packaging) across the maze
#: controllers; purged from ``sys.modules`` before each load so one maze's
#: ``config``/``mapping``/… cannot leak into another's import.
_SHARED_MODULE_NAMES = (
    "config", "astar", "frontier", "geometry", "odometry", "ir_lookup",
    "local_planner", "mapping", "perception", "depth_model", "robot_io",
    "viz", "observability",
    "mak_02_controller", "mak_03_controller", "mak_04_controller",
)


def _install_hardware_stub() -> None:
    """Register a minimal fake ``controller`` module for Webots-free imports.

    ``robot_io`` does ``from controller import Robot, Keyboard`` at import time;
    the stub lets that succeed so a controller class can be imported and
    inspected without a running simulator. The stubbed classes are never
    instantiated by discovery.
    """
    if "controller" in sys.modules:
        return
    stub = types.ModuleType("controller")
    stub.Robot = object          # type: ignore[attr-defined]
    stub.Keyboard = object       # type: ignore[attr-defined]
    sys.modules["controller"] = stub


class MazeControllerWrapper:
    """Uniform facade exposing only the contract methods of one maze controller.

    Args:
        maze_id: One of :meth:`available_mazes` (e.g. ``"Maze4"``).
        stub_hardware: Install a fake Webots ``controller`` module so the class
            can be imported without a simulator (for discovery / smoke tests).

    Raises:
        KeyError: If ``maze_id`` is not registered.
        ImportError: If the controller package or its ``NavigationController``
            class cannot be imported.
    """

    def __init__(self, maze_id: str, *, stub_hardware: bool = False) -> None:
        if maze_id not in _REGISTRY:
            raise KeyError(
                f"unknown maze {maze_id!r}; known: {sorted(_REGISTRY)}")
        self.maze_id = maze_id
        entry = _REGISTRY[maze_id]
        self.controller_dir = str(_REPO_ROOT / entry["dir"])
        self._module_name = entry["module"]
        if stub_hardware:
            _install_hardware_stub()
        self.controller_class = self._import_controller_class()
        self._instance: Optional[Any] = None

    # ------------------------------------------------------------------ import
    def _import_controller_class(self) -> type:
        """Import this maze's ``NavigationController`` class in isolation."""
        for name in _SHARED_MODULE_NAMES:
            sys.modules.pop(name, None)
        # Prefer this maze's directory over any other maze already on the path.
        sys.path = [p for p in sys.path if p != self.controller_dir]
        sys.path.insert(0, self.controller_dir)
        module = importlib.import_module(self._module_name)
        module = importlib.reload(module)
        try:
            return getattr(module, _CONTROLLER_CLASS_NAME)
        except AttributeError as exc:  # pragma: no cover - guards renames
            raise ImportError(
                f"{self._module_name!r} in {self.maze_id} has no "
                f"{_CONTROLLER_CLASS_NAME!r} class") from exc

    # --------------------------------------------------------------- lifecycle
    def build(self) -> "MazeControllerWrapper":
        """Instantiate the controller (needs the Webots runtime). Returns self."""
        self._instance = self.controller_class()
        return self

    @property
    def instance(self) -> Any:
        """The wrapped controller instance; call :meth:`build` first."""
        if self._instance is None:
            raise RuntimeError(
                "controller not built yet — call build() (requires Webots)")
        return self._instance

    # ----------------------------------------------------------- introspection
    @classmethod
    def available_mazes(cls) -> List[str]:
        """Return the sorted list of registered maze ids."""
        return sorted(_REGISTRY)

    @staticmethod
    def contract_methods() -> List[str]:
        """Return the method names this wrapper is allowed to expose."""
        return list(CONTRACT_METHODS)

    def satisfies_contract(self) -> bool:
        """True if the imported controller class defines every contract method."""
        return all(callable(getattr(self.controller_class, name, None))
                   for name in CONTRACT_METHODS)

    def missing_contract_methods(self) -> List[str]:
        """Contract methods the controller class does not implement (for tests)."""
        return [name for name in CONTRACT_METHODS
                if not callable(getattr(self.controller_class, name, None))]

    # -------------------------------------------------------------- delegation
    def __getattr__(self, name: str) -> Any:
        """Delegate contract methods to the built instance; block everything else.

        This is the access-control boundary: only names in
        :data:`CONTRACT_METHODS` are reachable through the wrapper, so callers
        cannot poke at a controller's private attributes.
        """
        if name in CONTRACT_METHODS:
            return getattr(self.instance, name)
        raise AttributeError(
            f"{name!r} is not exposed by MazeControllerWrapper "
            f"(allowed: {CONTRACT_METHODS})")

    def __repr__(self) -> str:  # pragma: no cover - cosmetic
        built = self._instance is not None
        return (f"<MazeControllerWrapper {self.maze_id} "
                f"class={self.controller_class.__name__} built={built}>")
