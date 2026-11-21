"""Restricted forward clothoid CSC/SC connectors at the front axle.

C is a linear-curvature ramp, optional saturated arc, and ramp to zero.
Small zero-boundary turns use the paper's CC-circle elementary path (Eq. 10).
Nonzero feedback curvature uses a truncated ramp with exact heading closure.
No cusps, CCC, or Reeds–Shepp completeness claims are included.
"""

from dataclasses import dataclass
from math import cos, sin, sqrt

import numpy as np

# Gaussian quadrature integrates exp(i*(yaw+k*s+alpha*s*s/2)).
_NODES, _WEIGHTS = np.polynomial.legendre.leggauss(12)


@dataclass(frozen=True)
class Segment:
    length: float
    curvature: float
    sharpness: float


def turn(angle, initial, limit, sharpness):
    """Return a loopless C ending at zero curvature, or None if incompatible."""
    if abs(angle) < 1e-10:
        return [] if abs(initial) < 1e-9 else None
    sign = np.sign(angle)
    k0 = sign * initial
    # The initial ramp may cross zero when feedback has the other sign.
    if abs(initial) > limit + 1e-9:
        return None
    amount = abs(angle)
    if k0 > 0 and amount < k0 * k0 / (2 * sharpness) - 1e-10:
        return None
    if abs(initial) < 1e-9 and amount < limit * limit / sharpness:
        # Fraichard–Scheuer IV-B.2, Eq. 10. Keep the endpoint on the
        # saturated turn's CC circle rather than inserting a looping arc.
        ramp = Segment(limit / sharpness, 0, sharpness)
        q = integrate((0, 0, 0, 0), ramp, ramp.length)
        center = np.array([
            q[0] - sin(q[2]) / limit,
            q[1] + cos(q[2]) / limit,
        ])
        radius = np.linalg.norm(center)
        mu = np.arctan2(center[0], center[1])
        u = (_NODES + 1) / 2
        integral = np.exp(1j * amount * u * u / 2) @ _WEIGHTS / 2
        numerator = amount * (cos(amount / 2) * integral.real +
                              sin(amount / 2) * integral.imag) ** 2
        sharpness = min(sharpness, numerator / (radius * sin(amount / 2 + mu)) ** 2)
    peak = min(limit, sqrt(sharpness * amount + k0 * k0 / 2))
    ramp_area = (2 * peak * peak - k0 * k0) / (2 * sharpness)
    arc = max(0.0, (amount - ramp_area) / peak)
    return [p for p in (
        Segment(max(0.0, (peak - k0) / sharpness), initial, sign * sharpness),
        Segment(arc, sign * peak, 0),
        Segment(peak / sharpness, sign * peak, -sign * sharpness),
    ) if p.length > 1e-10]


def integrate(pose, segment, distances):
    d = np.asarray(distances)
    u = d[..., None] * (_NODES + 1) / 2
    phase = pose[2] + segment.curvature * u + segment.sharpness * u * u / 2
    z = d * (np.exp(1j * phase) @ _WEIGHTS) / 2
    return np.stack((pose[0] + z.real, pose[1] + z.imag,
                     pose[2] + segment.curvature * d + segment.sharpness * d * d / 2,
                     segment.curvature + segment.sharpness * d), axis=-1)


def endpoint(pose, segments):
    result = np.asarray(pose)
    for segment in segments:
        result = integrate(result, segment, segment.length)
    return result


def sample(start, segments, goal_x):
    rows = [np.asarray(start)]
    for segment in segments:
        if segment.length <= 1e-10:
            continue
        count = max(1, int(np.ceil(segment.length / .02)))
        distances = np.linspace(0, segment.length, count + 1)[1:]
        rows.extend(integrate(rows[-1], segment, distances))
    join = len(rows) - 1
    end = rows[-1]
    if goal_x <= end[0] or abs(end[1]) > 1e-7 or abs(end[2]) > 1e-7:
        return None
    # Preserve the exact integrated endpoint; never snap a failed connection.
    n = max(1, int(np.ceil((goal_x - end[0]) / .02)))
    straight = Segment(goal_x - end[0], 0, 0)
    rows.extend(integrate(end, straight, np.linspace(0, straight.length, n + 1)[1:]))
    return np.asarray(rows), join


def connectors(start, goal_x, join_bounds, limit, sharpness):
    """Enumerate LL, LR, RL, RR CSC and left/right SC, including degeneracies.

    Search intermediate headings instead of fixing a join. For each heading,
    lateral closure determines the straight length exactly; the resulting join
    must lie in the declared safe interval. A finite search is not complete.
    """
    x, y, yaw, initial = start
    if not np.all(np.isfinite(start)) or abs(yaw) >= np.pi / 2:
        return
    low, high = join_bounds
    candidates = []

    def add(parts, family):
        end = endpoint(start, parts)
        if low <= end[0] <= high and abs(end[1]) < 1e-7 and abs(end[2]) < 1e-7:
            candidates.append((sum(p.length for p in parts), parts, family))

    if abs(y) < 1e-9 and abs(yaw) < 1e-9 and abs(initial) < 1e-9 and x < high:
        add([Segment(max(0., high - x), 0, 0)], 'S')
    # SC starts straight only when measured curvature is zero.
    if abs(initial) < 1e-9:
        last = turn(-yaw, 0, limit, sharpness)
        if last is not None and abs(sin(yaw)) > 1e-8:
            end = endpoint(start, last)
            length = -end[1] / sin(yaw)
            if length >= 0:
                add([Segment(length, 0, 0), *last], 'SL' if yaw < 0 else 'SR')
    # Symmetric heading grids retain the start heading for degenerate C.
    # Both grids use ten times larger spacing than the initial dense search.
    for heading in np.unique(np.r_[np.linspace(-1.35, 1.35, 28),
                                   np.linspace(min(0., yaw) - .08, max(0., yaw) + .08, 17), yaw]):
        if abs(sin(heading)) < 1e-7:
            continue
        first = turn(heading - yaw, initial, limit, sharpness)
        last = turn(-heading, 0, limit, sharpness)
        if first is None or last is None:
            continue
        middle = endpoint(start, first)
        end = endpoint(middle, last)
        length = -end[1] / sin(heading)
        if length >= 0:
            family = ('L' if heading > yaw else 'R') + 'S' + ('R' if heading > 0 else 'L')
            add([*first, Segment(length, 0, 0), *last], family)
    # Only sample spatial paths once the inexpensive endpoint search succeeds.
    for _, parts, family in sorted(candidates, key=lambda row: row[0]):
        result = sample(start, parts, goal_x)
        if result is not None:
            yield (*result, family)
