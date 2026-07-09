BACKUP: Maze4 mak_02_controller -- PRE-FIX snapshot (2026-07-01)
================================================================

This is the Maze4 mak_02_controller EXACTLY as it was BEFORE the three "drift
hardening" fixes (anti-erosion latch, poison decay, vision pillar-arrival) were
applied on 2026-07-01.

How it was reconstructed:
  - The whole controller directory was copied from the working tree, then the
    four files that the fixes touched were restored to their committed (git HEAD)
    pre-fix versions:
        config.py, mapping.py, mak_02_controller.py, selftest.py
  - local_planner.py was left as the working-tree version (it was already modified
    before the fixes; not part of the three-point fixes).
  - __pycache__/, maps/, runtime.ini were intentionally excluded.

This snapshot is the version that REACHED BLUE and YELLOW (with a wedge right next
to yellow), i.e. it did NOT have the "stuck near blue / no path to yellow"
regression that the post-fix version showed.

To restore it as the live controller:
  cp _BACKUP_README.txt /dev/null   # (ignore this file)
  cp *.py  ../../controllers/mak_02_controller/

Active (post-fix) controller remains at:
  Maze4/controllers/mak_02_controller/
