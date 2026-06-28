"""Phase C — utility-based frontier goal selection.

Mirrors the reference repo's "distance, information gain, and utility" scoring::

    utility = INFO_GAIN_W * information_gain - DIST_W * travel_distance

* ``information_gain`` is approximated by the frontier cluster size (a bigger
  frontier reveals more unmapped space).
* ``travel_distance`` is the planner's path length when a planner is supplied,
  otherwise the straight-line distance.

Frontiers that are too close, beyond the exploration radius, blacklisted from
earlier failures, or unreachable are discarded before scoring.
"""
from __future__ import annotations

import math

import settings as S


def _euclidean(robot_world, pt):
    return math.hypot(pt[0] - robot_world[0], pt[1] - robot_world[1])


def select_goal(frontiers, robot_world, start_world, blacklist=None,
                dist_fn=None):
    """Return ``(best_frontier, score)`` or ``(None, None)``.

    ``dist_fn(centroid_world) -> float | None`` supplies travel distance; return
    ``None`` to mark a frontier unreachable.  When omitted, straight-line
    distance is used.
    """
    blacklist = blacklist or []
    best = None
    best_score = -math.inf

    for fr in frontiers:
        cw = fr.centroid_world

        # --- hard filters ---
        if _euclidean(robot_world, cw) < S.FRONTIER_MIN_DIST_M:
            continue
        if _euclidean(start_world, cw) > S.EXPL_MAX_RADIUS_M:
            continue
        if any(_euclidean(bl, cw) < S.FRONTIER_BLACKLIST_RADIUS_M for bl in blacklist):
            continue

        if dist_fn is not None:
            dist = dist_fn(cw)
            if dist is None:                 # unreachable
                continue
        else:
            dist = _euclidean(robot_world, cw)

        score = S.UTIL_INFO_GAIN_W * fr.size - S.UTIL_DIST_W * dist
        if score > best_score:
            best_score = score
            best = fr

    if best is None:
        return None, None
    return best, best_score
