# `common/` — shared wrapper & observability (outside every maze)

This package sits **outside all `Maze*/` folders** and is imported by **none** of
the running controllers, so nothing here can affect the code that runs in Webots.
It provides two things:

1. a **uniform access wrapper** to drive or introspect any maze's controller
   through one interface, and
2. the **canonical observability module** (`observability.py`) that each
   controller vendors a copy of for logging and runtime fault tracing.

## Files

| file | role |
|---|---|
| `controller_contract.py` | `MazeControllerContract` (a `typing.Protocol`) + `CONTRACT_METHODS` — the uniform method surface every maze controller exposes. |
| `maze_controller_wrapper.py` | `MazeControllerWrapper` — resolves a maze id to its controller package, imports its `NavigationController` in isolation, and exposes **only** the contract methods. |
| `observability.py` | `get_logger`, `RunEventLog` (JSONL), `guarded_stage` — the shared telemetry primitives. This is the source copy; each controller carries a vendored duplicate so it stays self-contained for Webots. |
| `smoke_test.py` | Webots-free check that all five mazes resolve and satisfy the contract. |
| `README.md` | this file. |

## The contract

After the fleet-wide naming pass, every maze's controller class is
`NavigationController` and implements the same methods:

```
update_sensing · run_slam_step · update_perception · refresh_costmap ·
plan_path_to · select_frontier_goal · drive_along_path · step_mission ·
finalize · run
```

The controllers do **not** import or inherit from this package — the contract is
structural (`Protocol`), so they satisfy it just by defining these methods.

## Using the wrapper

```python
from maze_controller_wrapper import MazeControllerWrapper

MazeControllerWrapper.available_mazes()          # ['Maze1', ... 'Maze5']

# Discovery / CI without a simulator (installs a fake Webots 'controller' module):
w = MazeControllerWrapper("Maze4", stub_hardware=True)
w.satisfies_contract()                           # True
w.controller_class.__name__                      # 'NavigationController'

# Inside Webots (real robot runtime available):
w = MazeControllerWrapper("Maze4").build()
w.run()                                          # only contract methods are reachable
w.some_private_attr                              # AttributeError — access is blocked
```

Each maze controller reuses the same internal module names (`config`, `mapping`,
…); the wrapper purges those from `sys.modules` and prioritises the target maze's
directory on every load, so importing one maze never leaks another maze's tuning.

## Observability (vendored into each controller)

- **Console logs** via `get_logger("navctl.<maze>")` — leveled, timestamped, the
  same operator-facing messages the controllers always printed.
- **Structured run log** via `RunEventLog(maps_dir)` → `maps/run_events.jsonl`,
  one JSON object per line: run start + device inventory, mission state
  transitions, pillar acquisitions and timing, recovery entries, hardware/sensor
  faults, exceptions (with traceback), and the final timing table. This is the
  machine-readable trail for reconstructing a failed run after the fact.
- **Fault isolation** via `guarded_stage(...)` — wraps each control-loop stage so
  a single bad frame is logged (console + JSONL, with tick/state context) and the
  mission continues instead of crashing.

## Validate

```bash
cd common
python3 smoke_test.py     # asserts all 5 mazes import and satisfy the contract
```
