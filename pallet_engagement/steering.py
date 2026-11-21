"""Rear-wheel steering geometry for the geometric planner and tracker."""

from math import tan

import numpy as np


def curvature_limit(g, limits):
    return tan(limits.steer) / (
        g.wheelbase + max(map(abs, g.rear_y)) * tan(limits.steer)
    )


def wheels(k, g):
    k = np.asarray(k)[..., None]
    return np.arctan2(-g.wheelbase * k, 1 - k * np.asarray(g.rear_y))
