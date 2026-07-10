"""Pure-math IR distance-sensor lookup-table inversion (no Webots import).

Webots ``DistanceSensor.getValue()`` returns a raw value mapped via the proto's
``lookupTable`` (inverse-linear for the ROSbot IRs: close → high value, far →
low value).  We cache the table's two extreme points and linearly invert
between them every tick.  Kept Webots-free so it is directly unit-testable;
:mod:`robot_io` is a thin wrapper around these two functions.
"""
from __future__ import annotations

from typing import List, Optional, Sequence, Tuple, Union


def build_lookup(
    table_triples: Optional[Sequence[float]],
    max_value_fallback: float = 4.0,
) -> Tuple[float, float, float, float, float]:
    """Parse the flat Webots lookup-table into an invertible 2-point model.

    Args:
        table_triples: Flat sequence ``[d0, val0, std0, d1, val1, std1, …]``
            as returned by ``DistanceSensor.getLookupTable()`` (or ``None`` /
            empty / too-short for a sensor with no usable table).
        max_value_fallback: Distance cap returned when the table is unusable.

    Returns:
        A 5-tuple ``(d_min, d_max, val_at_dmin, val_at_dmax, d_max)`` that
        :func:`value_to_meters` consumes for per-tick inversion.
    """
    if not table_triples or len(table_triples) < 6:
        mx = float(max_value_fallback)
        return (0.0, mx, 0.0, mx, mx)
    rows = [(float(table_triples[i]), float(table_triples[i + 1]))
            for i in range(0, len(table_triples), 3)]
    rows.sort(key=lambda r: r[0])
    d_min, val_min = rows[0]
    d_max, val_max = rows[-1]
    return (d_min, d_max, val_min, val_max, d_max)


def value_to_meters(
    lookup: Tuple[float, float, float, float, float],
    value: float,
) -> float:
    """Invert one raw sensor reading to metres using a :func:`build_lookup` result.

    Performs a clamped linear interpolation between the table's two extreme
    ``(distance, value)`` points.

    Args:
        lookup: The 5-tuple returned by :func:`build_lookup`.
        value: Raw sensor reading from ``DistanceSensor.getValue()``.

    Returns:
        Estimated distance in metres, clamped to ``[d_min, d_max]``.
    """
    d_min, d_max, v_at_dmin, v_at_dmax, _ = lookup
    if abs(v_at_dmax - v_at_dmin) < 1e-6:
        return d_max
    slope = (d_max - d_min) / (v_at_dmax - v_at_dmin)
    d = d_min + (value - v_at_dmin) * slope
    return max(d_min, min(d_max, d))
