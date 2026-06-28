#!/usr/bin/env python3
"""Run the mak_02 pure-module unit tests with the stdlib (no pytest needed).

Usage::

    python3 run_tests.py

Only the Webots-independent algorithm modules are exercised here; the hardware
layer and the main controller require the Webots ``controller`` runtime.
"""
import os
import sys
import unittest

ROOT = os.path.dirname(os.path.abspath(__file__))
sys.path.insert(0, ROOT)


def main():
    loader = unittest.TestLoader()
    suite = loader.discover(os.path.join(ROOT, "tests"), pattern="test_*.py",
                            top_level_dir=ROOT)
    runner = unittest.TextTestRunner(verbosity=2)
    result = runner.run(suite)
    return 0 if result.wasSuccessful() else 1


if __name__ == "__main__":
    sys.exit(main())
