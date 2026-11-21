"""A joined quintic and straight guide in pallet coordinates, tracked at 50 Hz."""

from dataclasses import dataclass, replace
from math import cos, sin, tan

import numpy as np

from .config import CONTROL_NS
from .frames import compose
from .guard import Clearance
from .messages import Command, GuideTrajectory, MotionProposal
from .prediction import PredictedMotion, advance, predict_queue
from .steering import curvature_limit, wheels


@dataclass
class Path:
    pose: np.ndarray
    curvature: np.ndarray
    s: np.ndarray
    v: np.ndarray
    time: np.ndarray
    join: int


def timed_path(rows, join, ctx, cruise, straight_speed, initial, reverse=False):
    ds = np.linalg.norm(np.diff(rows[:, :2], axis=0), axis=1)
    s = np.r_[0, ds.cumsum()]
    angles = wheels(rows[:, 3], ctx.geometry)
    change = abs(np.diff(angles, axis=0)).max(axis=1)
    cap = ctx.limits.target_steer_rate * ds / np.maximum(change, 1e-12)
    v = np.full(len(s), min(cruise, ctx.limits.speed - ctx.limits.drive_headroom))
    v[join:] = np.minimum(v[join:], straight_speed)
    v[:-1] = np.minimum(v[:-1], cap)
    v[1:] = np.minimum(v[1:], cap)
    v[-1] = 0
    for i in reversed(range(len(ds))):
        v[i] = min(
            v[i], np.sqrt(v[i + 1] ** 2 + 2 * min(0.18, ctx.limits.brake) * ds[i])
        )
    v[0] = min(v[0], abs(initial))
    for i, step in enumerate(ds):
        v[i + 1] = min(
            v[i + 1], np.sqrt(v[i] ** 2 + 2 * min(0.18, ctx.limits.target_accel) * step)
        )
    time = np.r_[0, np.cumsum(2 * ds / np.maximum(v[:-1] + v[1:], 1e-9))]
    return Path(rows[:, :3], rows[:, 3], s, -v if reverse else v, time, join)


def joined_curve(start, join_x, goal_x, a, b):
    x, y, yaw, k = start
    tangent = np.array([cos(yaw), sin(yaw)])
    p = np.array([x, y])
    d = a * tangent
    h = 0.5 * a * a * k * np.array([-tangent[1], tangent[0]])
    offset = np.array([join_x, 0]) - p
    end = np.array([b, 0])
    coef = np.array(
        [
            p,
            d,
            h,
            10 * offset - 6 * d - 3 * h - 4 * end,
            -15 * offset + 8 * d + 3 * h + 7 * end,
            6 * offset - 3 * d - h - 3 * end,
        ]
    )
    poly = np.polynomial.polynomial
    u = np.linspace(0, 1, max(81, int(np.linalg.norm(offset) / 0.005) + 1))
    points = poly.polyval(u, coef).T
    deriv = poly.polyval(u, poly.polyder(coef)).T
    if deriv[:, 0].min() <= 1e-6:
        return None
    second = poly.polyval(u, poly.polyder(coef, 2)).T
    curvature = (
        deriv[:, 0] * second[:, 1] - deriv[:, 1] * second[:, 0]
    ) / np.linalg.norm(deriv, axis=1) ** 3
    curve = np.column_stack((points, np.arctan2(deriv[:, 1], deriv[:, 0]), curvature))
    curve[-1] = (join_x, 0, 0, 0)
    rows = np.vstack((curve, [goal_x, 0, 0, 0]))
    arc = np.r_[0, np.linalg.norm(np.diff(rows[:, :2], axis=0), axis=1).cumsum()]
    samples = np.unique(np.r_[np.arange(0, arc[-1], 0.02), arc[-2:]])
    # Avoid nearly coincident knots where a straight bend lands on the regular grid.
    regular = samples[(abs(samples - arc[-2]) > 1e-6) & (samples < arc[-1] - 1e-6)]
    samples = np.sort(np.r_[regular, arc[-2:]])
    return np.column_stack([np.interp(samples, arc, col) for col in rows.T]), int(
        np.searchsorted(samples, arc[-2])
    )


def pursuit(pose, point):
    # Rotating the travel frame by pi and negating travel curvature in reverse
    # gives the same body-frame expression for a target behind the axle.
    dx, dy = point - pose[:2]
    return 2 * (-sin(pose[2]) * dx + cos(pose[2]) * dy) / max(dx * dx + dy * dy, 1e-8)


def tracking_curvature(path, progress, pose, speed, sign, lookahead, ctx):
    look = lookahead + 0.9 * abs(speed)
    target_s = min(path.s[-1], progress + look)
    point = np.array([np.interp(target_s, path.s, axis) for axis in path.pose[:, :2].T])
    point += (
        sign
        * max(0.0, progress + look - path.s[-1])
        * np.array([cos(path.pose[-1, 2]), sin(path.pose[-1, 2])])
    )
    reference = np.array([np.interp(progress, path.s, axis) for axis in path.pose.T])
    # Preview the planned bend by one measured servo time constant. Subtract
    # nominal pursuit to avoid counting the turn twice on curved reference paths.
    preview = progress + abs(speed) * ctx.limits.steering_response_s
    feedforward = np.interp(preview, path.s, path.curvature)
    correction = pursuit(pose, point) - pursuit(reference, point)
    limit = curvature_limit(ctx.geometry, ctx.limits)
    return float(np.clip(feedforward + correction, -limit, limit))


def predict_request(state, command, ctx):
    samples = [(*state.pose, state.speed)]
    for _ in range(8):
        state = advance(state, command, 0.05, ctx.geometry, ctx.limits)
        samples.append((*state.pose, state.speed))
    return state, samples


def preview_path(path, ctx, guard, lookahead):
    """Test the complete tracking reference from its nominal application state."""
    angles = tuple(wheels(path.curvature[0], ctx.geometry))
    state = PredictedMotion(
        pose=tuple(path.pose[0]),
        speed=path.v[0],
        target_speed=path.v[0],
        rear_angle=np.arctan(-ctx.geometry.wheelbase * path.curvature[0]),
        target_angles=angles,
    )
    poses = [state.pose]
    if path.time[-1] > 120:
        return False
    for t in np.arange(0, path.time[-1] + 0.5, 0.10):
        progress = np.interp(t, path.time, path.s)
        speed = np.interp(t + 0.05, path.time, path.v)
        curvature = tracking_curvature(
            path, progress, state.pose, state.speed, 1, lookahead, ctx
        )
        state = advance(
            state, Command(ctx.stamp, speed, curvature), 0.10, ctx.geometry, ctx.limits
        )
        poses.append(state.pose)
    return guard.swept(poses) > 0


class Method:
    period_ns = CONTROL_NS

    def reset(self, settings):
        opts = dict(settings)
        self.cruise = float(opts.get("cruise", 0.22))
        self.straight = float(opts.get("straight_speed", 0.10))
        self.lookahead = float(opts.get("lookahead", 1.5))
        self.join_distance = float(opts.get("join_distance", 0.80))
        if not all(
            np.isfinite(x) and x > 0
            for x in (self.cruise, self.straight, self.join_distance, self.lookahead)
        ):
            raise ValueError(
                "geometric speeds and join distance must be positive and finite"
            )
        self.path = self.retreat = self.authorization = self.previous = None
        self.revision = -1
        self.plan = 0
        self.acquired = False
        self.wait_capture = -1
        self.anchor = None
        self.force_retreat = False
        self.progress = 0.0
        self.paused = False
        self.launch_ns = 0
        self.events = set()

    def observe(self, message):
        pass  # The runtime fuses observations into the timestamp-aligned context.

    def forward(self, start, ctx, guard, initial, diag):
        goal = ctx.criteria.depth + ctx.criteria.target_allowance - ctx.geometry.tip_x
        join_x = -self.join_distance - ctx.geometry.tip_x
        length = np.hypot(join_x - start[0], start[1])
        candidates = []
        rejected = []
        for aa, bb in ((1.0, 1.0), (0.8, 1.2), (1.2, 0.8), (1.2, 1.2), (0.8, 0.8)):
            candidate = joined_curve(start, join_x, goal, aa * length, bb * length)
            if candidate is None:
                rejected.append("nonmonotone")
                continue
            rows, join = candidate
            if abs(rows[:, 3]).max() > curvature_limit(ctx.geometry, ctx.limits):
                rejected.append("steering")
                continue
            margin = guard.swept(rows[:, :3])
            if margin <= 0:
                rejected.append("clearance")
                continue
            path = timed_path(rows, join, ctx, self.cruise, self.straight, initial)
            score = (path.time[-1], -margin)
            candidates.append((score, path, (aa * length, bb * length)))
        for _, path, tangents in sorted(candidates, key=lambda item: item[0]):
            if preview_path(path, ctx, guard, self.lookahead):
                diag["tangents_m"] = tangents
                diag["rejected"] = tuple(rejected)
                return path
            rejected.append("tracking_preview")
        diag["rejected"] = tuple(rejected)

    def plan_retreat(self, start, ctx, guard, diag):
        for distance in (0.65, 1.25, 2.0, 2.75):
            poses = np.array(
                [
                    compose(start[:3], (-d, 0, 0))
                    for d in np.linspace(0, distance, int(distance / 0.02) + 1)
                ]
            )
            if guard.swept(poses) <= 0:
                continue
            tail = self.forward((*poses[-1], 0), ctx, guard, 0, diag)
            if tail is not None:
                leg = timed_path(
                    np.column_stack((poses, np.zeros(len(poses)))),
                    0,
                    ctx,
                    ctx.limits.recovery_speed,
                    0.09,
                    0,
                    reverse=True,
                )
                self.retreat = leg
                samples = np.vstack(
                    (
                        np.column_stack((poses[1:], leg.v[1:])),
                        np.column_stack((tail.pose, tail.v)),
                    )
                )
                self.authorization = self.trajectory(ctx, samples)
                diag["retreat_m"] = distance
                return self.authorization

    def trajectory(self, ctx, samples, suffix=""):
        return GuideTrajectory(
            replace(ctx.stamp, frame="pallet"),
            f"geometric-{self.plan}{suffix}",
            ctx.target_id,
            tuple(map(tuple, samples)),
        )

    def propose(self, ctx):
        t = ctx.stamp.time_ns
        diag = {}

        def reply(status, reason, guide=None):
            self.previous = None
            return MotionProposal(
                Command(ctx.stamp),
                t + 2 * self.period_ns,
                status,
                tuple(dict(diag, reason=reason).items()),
                guide,
            )

        if not ctx.estimate.initialized:
            return reply("hold", "acquiring")
        guard = Clearance(ctx)
        goal = ctx.criteria.depth + ctx.criteria.target_allowance - ctx.geometry.tip_x
        if self.revision != ctx.revision:
            if self.revision >= 0 and not ctx.recovery:
                self.wait_capture = t
                self.retreat = self.authorization = None
            self.path = self.retreat if ctx.recovery else None
            self.revision = ctx.revision
            self.previous = None
            self.progress, self.clock = 0.0, t
            self.launch_ns = t
            self.events.clear()
            self.anchor = None
            self.force_retreat = False
        if ctx.estimate.last_capture_ns <= self.wait_capture:
            return reply("hold", "reacquiring")
        if not self.acquired:
            if (
                guard.at((goal, 0, 0)) <= 0
                or t - ctx.estimate.last_capture_ns > ctx.criteria.fresh_ns
            ):
                return reply("hold", "acquiring")
            self.acquired = True
        state = predict_queue(ctx, ctx.limits.delay_ns)
        start = (*state.pose, -tan(state.rear_angle) / ctx.geometry.wheelbase)
        if (
            self.anchor is None
            or np.linalg.norm(np.subtract(guard.origin[:2], self.anchor)) > 0.025
        ):
            self.anchor, self.last_motion = guard.origin[:2], t
        if (
            self.path is not None
            and not ctx.recovery
            and t - self.last_motion > 2_500_000_000
            and guard.origin[0] < goal - 0.025
            and t - ctx.estimate.last_capture_ns <= ctx.criteria.fresh_ns
        ):
            self.force_retreat = self.paused = True
            self.path = None
            diag["stalled_s"] = (t - self.last_motion) * 1e-9
        if self.previous and ctx.issued_history and ctx.issued_history[-1].speed == 0:
            self.paused = True
        if self.paused:
            if abs(ctx.feedback.axle_speed) > ctx.criteria.still_speed or any(
                c.speed for c in (ctx.active, *ctx.pending)
            ):
                return reply("hold", "braking")
            self.clock = t
            self.launch_ns = t
            self.paused = False
            if self.path:
                self.clock -= round(
                    np.interp(self.progress, self.path.s, self.path.time) * 1e9
                )
        if not ctx.recovery and guard.origin[0] >= goal - 0.004:
            return reply("complete", "final_stop")
        if self.path is None:
            if self.authorization is not None:
                if guard.guide(self.authorization) <= 0:
                    return reply("failed", "withdrawal_clearance")
                return reply("retry", "awaiting_authorization", self.authorization)
            self.plan += 1
            planning_guard = (
                guard.nominal()
                if guard.approach_allowed(Command(ctx.stamp, self.cruise))
                else guard
            )
            if not self.force_retreat:
                self.path = self.forward(start, ctx, planning_guard, state.speed, diag)
            if self.path is None:
                if abs(state.speed) >= ctx.criteria.still_speed:
                    return reply("hold", "stop_before_retreat")
                authorization = self.plan_retreat(start, ctx, guard, diag)
                if authorization:
                    return reply("retry", "signed_connection", authorization)
                return reply("failed", "no_safe_continuation")
            self.progress, self.clock = 0.0, t
            self.launch_ns = t
            diag["curvature"] = tuple(self.path.curvature)
            diag["wheel_angles"] = tuple(
                map(tuple, wheels(self.path.curvature, ctx.geometry))
            )
        path = self.path
        # Project onto the local chord for continuous progress near the final stop.
        lo = max(0, np.searchsorted(path.s, self.progress) - 2)
        hi = min(len(path.s), np.searchsorted(path.s, self.progress + 0.40) + 1)
        j = lo + np.argmin(
            np.linalg.norm(path.pose[lo:hi, :2] - state.pose[:2], axis=1)
        )
        sign = -1 if ctx.recovery else 1
        yaw = path.pose[j, 2]
        tangent = np.array([cos(yaw), sin(yaw)])
        error = state.pose[:2] - path.pose[j, :2]
        self.progress = min(
            path.s[-1], max(self.progress, path.s[j] + sign * (error @ tangent))
        )
        remaining = path.s[-1] - self.progress
        if ctx.recovery and remaining < 0.004:
            return reply("reacquire", "retreat_stop")
        k = tracking_curvature(
            path, self.progress, state.pose, state.speed, sign, self.lookahead, ctx
        )
        # The spatial reference bounds clock catch-up after transient lag.
        elapsed = (t - self.clock + ctx.limits.delay_ns + self.period_ns) * 1e-9
        scheduled = abs(np.interp(elapsed, path.time, path.v))
        spatial = abs(np.interp(min(path.s[-1], self.progress + 0.015), path.s, path.v))
        v = min(
            max(scheduled, abs(np.interp(self.progress, path.s, path.v))),
            max(0.025, spatial),
            np.sqrt(2 * ctx.limits.brake * remaining),
        )
        # A small crawl closes residual longitudinal error after a noisy projection.
        v = min(
            max(0.012, v),
            min(0.18, ctx.limits.target_accel)
            * (t - self.launch_ns + ctx.limits.delay_ns + self.period_ns)
            * 1e-9,
        )
        command = Command(ctx.stamp, sign * v, k)
        idx = min(len(path.s) - 1, np.searchsorted(path.s, self.progress, side="right"))
        guide = self.trajectory(ctx, np.column_stack((path.pose[idx:], path.v[idx:])))
        # Predict the measured response to this request over the feedback horizon.
        sim, samples = predict_request(state, command, ctx)
        # Require a fresh full-depth straight corridor and settled steering at entry.
        tip = guard.origin[0] + ctx.geometry.tip_x * cos(guard.origin[2])
        if not ctx.recovery and tip > -0.35:
            distance = max(0.0, goal - sim.pose[0]) / max(0.1, cos(sim.pose[2]))
            margin = guard.segment(sim.pose, compose(sim.pose, (distance, 0, 0)))
            diag["entry_margin_m"] = margin
            diag["tracking_allowance_m"] = ctx.limits.tracking
            diag["entry_heading_rad"] = sim.pose[2]
            diag["entry_yaw_travel_m"] = distance * sin(sim.pose[2])
            nominal_margin = guard.nominal().segment(
                sim.pose, compose(sim.pose, (distance, 0, 0))
            )
            diag["entry_nominal_margin_m"] = nominal_margin
            diag["entry_covariance_cost_m"] = nominal_margin - margin
            if (
                margin <= 0
                or max(map(abs, ctx.feedback.angles)) > 0.10
                or t - ctx.estimate.last_capture_ns > ctx.criteria.fresh_ns
            ):
                cap = guard.approach_speed_limit(state.pose)
                command = replace(command, speed=min(v, cap))
                if cap < 0.012 or not guard.approach_allowed(command):
                    self.paused = True
                    return reply("hold", "entry_clearance")
                _, samples = predict_request(state, command, ctx)
        rollout = self.trajectory(ctx, samples, "-rollout")
        diag.update(
            progress_m=self.progress,
            cross_track_m=float(error @ [-sin(yaw), cos(yaw)]),
            yaw_error_rad=state.pose[2] - yaw,
            remaining_m=remaining,
            approach_reserve_m=guard.approach_reserve(abs(state.speed)),
        )
        if not ctx.recovery:
            for name, crossed in (
                ("join", guard.origin[0] >= path.pose[path.join, 0]),
                (
                    "face",
                    tip
                    + (ctx.geometry.tip_y + ctx.geometry.half_width)
                    * abs(sin(guard.origin[2]))
                    >= 0,
                ),
            ):
                if crossed and name not in self.events:
                    self.events.add(name)
                    diag[name + "_time_s"] = t * 1e-9
                    diag[name + "_measured_mps"] = ctx.feedback.axle_speed
                    diag[name + "_planned_mps"] = np.interp(
                        self.progress, path.s, path.v
                    )
        self.previous = command
        return MotionProposal(
            command,
            t + 2 * self.period_ns,
            "drive",
            tuple(diag.items()),
            guide,
            rollout,
        )
