"""Nominal five-state prediction with a separately carried actuator target and FIFO."""

from dataclasses import dataclass, replace
from math import atan, exp, tan

import numpy as np

from .frames import ackermann, arc, compose, curvatures


@dataclass(frozen=True)
class PredictedMotion:
    pose: tuple
    speed: float
    rear_angle: float
    target_speed: float
    target_angles: tuple

    @classmethod
    def measured(cls, pose, feedback, geometry):
        k = np.mean(curvatures(geometry, feedback.angles))
        return cls(
            pose,
            feedback.axle_speed,
            atan(-geometry.wheelbase * k),
            feedback.axle_speed,
            feedback.angles,
        )


def slew_targets(previous_speed, previous_angles, command, dt, geometry, limits):
    cap = limits.speed - limits.drive_headroom
    speed = np.clip(command.speed, -cap, cap)
    next_target = previous_speed + np.clip(
        speed - previous_speed, -limits.target_accel * dt, limits.target_accel * dt
    )
    angles, _ = ackermann(geometry, speed, command.curvature)
    targets = np.asarray(previous_angles)
    targets = targets + np.clip(
        np.clip(angles, -limits.steer, limits.steer) - targets,
        -limits.target_steer_rate * dt,
        limits.target_steer_rate * dt,
    )
    return next_target, tuple(targets)


def advance(state, command, dt, geometry, limits):
    next_target, targets = slew_targets(
        state.target_speed, state.target_angles, command, dt, geometry, limits
    )
    equivalent = atan(-geometry.wheelbase * np.mean(curvatures(geometry, targets)))
    delta = equivalent + (state.rear_angle - equivalent) * exp(
        -dt / limits.steering_response_s
    )
    v = next_target + (state.speed - next_target) * exp(-dt / limits.speed_response_s)
    k = -tan((state.rear_angle + delta) / 2) / geometry.wheelbase
    pose = compose(state.pose, arc((state.speed + v) * dt / 2, k))
    return PredictedMotion(pose, v, delta, next_target, tuple(targets))


def predict_queue(ctx, duration_ns, issued=(), *, samples=None):
    """Predict the nominal released-command prefix and any supplied future commands."""
    from .frames import inverse

    state = PredictedMotion.measured(
        inverse(ctx.estimate.pose), ctx.feedback, ctx.geometry
    )
    if ctx.nominal_targets is not None:
        state = replace(
            state,
            target_speed=ctx.nominal_targets[0],
            target_angles=ctx.nominal_targets[1],
        )
    cmd = ctx.active
    now = ctx.stamp.time_ns
    end = now + duration_ns
    schedule = [
        (c.stamp.time_ns + ctx.limits.delay_ns, c) for c in (*ctx.pending, *issued)
    ]
    for when, following in sorted(schedule, key=lambda x: x[0]) + [(end, None)]:
        until = min(end, when)
        while now < until:
            step = min(until - now, 4_000_000)
            state = advance(state, cmd, step * 1e-9, ctx.geometry, ctx.limits)
            now += step
            if samples is not None:
                samples.append((*state.pose, state.speed))
        if now >= end:
            break
        cmd = following
    return state
