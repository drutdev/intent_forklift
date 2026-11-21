from math import atan2, cos, sin, tan

import numpy as np


def wrap(angle):
    return atan2(sin(angle), cos(angle))


def rotation(a):
    return np.array(((cos(a), -sin(a)), (sin(a), cos(a))))


def transform(pose, points):
    return np.asarray(points) @ rotation(pose[2]).T + pose[:2]


def inverse(pose):
    return (*(-rotation(-pose[2]) @ pose[:2]), -pose[2])


def compose(a, b):
    return (*transform(a, b[:2]), wrap(a[2] + b[2]))


def arc(distance, curvature):
    a = distance * curvature
    return (
        distance * np.sinc(a / np.pi),
        distance * 0.5 * a * np.sinc(0.5 * a / np.pi) ** 2,
        a,
    )


def ackermann(g, speed, k):
    return (
        tuple(atan2(-g.wheelbase * k, 1 - k * y) for y in g.rear_y),
        tuple(speed * (1 - k * y) / g.radius for y in g.front_y),
    )


def curvatures(g, angles):
    return tuple(-tan(a) / (g.wheelbase - y * tan(a)) for a, y in zip(angles, g.rear_y))


def matrix_tuple(value):
    return tuple(map(tuple, value))
