"""Pure-pursuit path follower (pure math).

Given the robot pose and a world-frame path, pick a lookahead point ahead on the
path and steer toward it.  Forward speed is reduced when the heading error is
large (and zeroed for near-reversals, so the skid-steer pivots in place rather
than carving a wide, wall-clipping arc) and as the goal is approached.
"""
from __future__ import annotations

import math

import settings as S
from geometry import wrap_angle


def _closest_index(pose, path):
    px, py = pose[0], pose[1]
    best_i = 0
    best_d2 = math.inf
    for i, (x, y) in enumerate(path):
        d2 = (x - px) ** 2 + (y - py) ** 2
        if d2 < best_d2:
            best_d2 = d2
            best_i = i
    return best_i


def _lookahead_point(pose, path, lookahead):
    """First path point at least ``lookahead`` ahead of the closest point."""
    i0 = _closest_index(pose, path)
    px, py = pose[0], pose[1]
    for i in range(i0, len(path)):
        x, y = path[i]
        if math.hypot(x - px, y - py) >= lookahead:
            return (x, y)
    return path[-1]


def follow(pose, path, v_max=None):
    """Return a command dict for tracking ``path`` from ``pose``.

    Keys: ``v``, ``w``, ``target`` (lookahead point), ``reached`` (goal reached),
    ``heading_err``.
    """
    v_max = S.V_MAX if v_max is None else v_max
    out = {"v": 0.0, "w": 0.0, "target": None, "reached": False, "heading_err": 0.0}
    if not path:
        return out

    x, y, th = pose
    goal = path[-1]
    dist_goal = math.hypot(goal[0] - x, goal[1] - y)
    if dist_goal < S.GOAL_REACH_TOL:
        out["reached"] = True
        return out

    lookahead = S.LOOKAHEAD_BASE + S.LOOKAHEAD_K_V * v_max
    lookahead = max(S.LOOKAHEAD_MIN, min(S.LOOKAHEAD_MAX, lookahead))
    tx, ty = _lookahead_point(pose, path, lookahead)
    out["target"] = (tx, ty)

    desired = math.atan2(ty - y, tx - x)
    herr = wrap_angle(desired - th)
    out["heading_err"] = herr

    w = S.PP_K_HEADING * herr
    w = max(-S.W_MAX, min(S.W_MAX, w))

    if abs(herr) > S.PP_BIG_HEADING_STOP:
        # Too far off heading: pivot in place.
        v = 0.0
    else:
        # Scale speed by heading alignment and proximity to the goal.
        align = max(0.0, 1.0 - abs(herr) / S.PP_BIG_HEADING_STOP)
        v = v_max * align
        v = min(v, max(S.V_MIN, dist_goal))   # ease off near the goal
        v = max(v, 0.0)

    out["v"] = v
    out["w"] = w
    return out
