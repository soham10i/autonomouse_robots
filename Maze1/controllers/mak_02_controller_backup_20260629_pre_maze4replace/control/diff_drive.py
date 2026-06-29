"""Differential-drive kinematics (pure math).

Converts body twist ``(v, w)`` to/from per-wheel angular velocities for a
skid/differential robot with wheel radius ``R`` and track width ``L``::

    w_right = (2 v + w L) / (2 R)
    w_left  = (2 v - w L) / (2 R)

and the inverse::

    v = R (w_right + w_left) / 2
    w = R (w_right - w_left) / L
"""
from __future__ import annotations

import settings as S


def cmd_to_wheels(v, w, wheel_radius=None, wheel_sep=None):
    """Body twist ``(v, w)`` -> ``(w_left, w_right)`` wheel angular velocities."""
    r = S.WHEEL_RADIUS if wheel_radius is None else wheel_radius
    sep = S.WHEEL_SEPARATION if wheel_sep is None else wheel_sep
    w_right = (2.0 * v + w * sep) / (2.0 * r)
    w_left = (2.0 * v - w * sep) / (2.0 * r)
    return w_left, w_right


def wheels_to_cmd(w_left, w_right, wheel_radius=None, wheel_sep=None):
    """``(w_left, w_right)`` -> body twist ``(v, w)``."""
    r = S.WHEEL_RADIUS if wheel_radius is None else wheel_radius
    sep = S.WHEEL_SEPARATION if wheel_sep is None else wheel_sep
    v = r * (w_right + w_left) / 2.0
    w = r * (w_right - w_left) / sep
    return v, w


def clamp_twist(v, w, v_max=None, w_max=None):
    """Clamp a twist to the configured speed envelope."""
    v_max = S.V_MAX if v_max is None else v_max
    w_max = S.W_MAX if w_max is None else w_max
    v = max(-v_max, min(v_max, v))
    w = max(-w_max, min(w_max, w))
    return v, w
