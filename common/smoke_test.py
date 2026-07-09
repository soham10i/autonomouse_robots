"""Webots-free smoke test for the uniform maze controller wrapper.

Confirms that every registered maze resolves to a ``NavigationController`` and
that the class satisfies the shared contract — i.e. the fleet-wide naming pass
left all five controllers exposing the same method surface, so the wrapper can
drive any of them. Instantiation (which needs the Webots runtime) is not
attempted; only discovery + contract compliance are checked.

    python3 smoke_test.py
"""
from __future__ import annotations

import os
import sys

sys.path.insert(0, os.path.dirname(os.path.abspath(__file__)))

from maze_controller_wrapper import MazeControllerWrapper


def main() -> int:
    failures = []
    for maze_id in MazeControllerWrapper.available_mazes():
        try:
            wrapper = MazeControllerWrapper(maze_id, stub_hardware=True)
        except Exception as exc:  # noqa: BLE001 - report, don't abort the sweep
            print(f"  [FAIL] {maze_id}: import error: {exc}")
            failures.append(maze_id)
            continue
        cls = wrapper.controller_class.__name__
        missing = wrapper.missing_contract_methods()
        if missing:
            print(f"  [FAIL] {maze_id}: {cls} missing {missing}")
            failures.append(maze_id)
        else:
            print(f"  [PASS] {maze_id}: {cls} satisfies the contract")

    print()
    if failures:
        print(f"SMOKE TEST FAILED: {failures}")
        return 1
    print("ALL MAZES SATISFY THE CONTRACT")
    return 0


if __name__ == "__main__":
    sys.exit(main())
