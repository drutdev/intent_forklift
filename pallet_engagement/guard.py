"""Conservative planar swept-volume checks in the stationary pallet frame."""

from dataclasses import replace
from functools import lru_cache
from math import ceil, cos, exp

import numpy as np

from .config import CONTROL_NS
from .frames import ackermann, arc, compose, curvatures, inverse, rotation
from .messages import Command, SupervisorDecision


@lru_cache(maxsize=4)
def rectangles(geometry):
    pieces = geometry.pieces
    corners = np.array([[(x, y) for x in p[:2] for y in p[2:4]] for p in pieces])
    return corners, np.array([p[4] for p in pieces]), np.sign(corners[:, 0, 1])


class Clearance:
    def __init__(self, ctx):
        self.path = []
        self.ctx = ctx
        self.origin = inverse(ctx.estimate.pose)
        J = np.zeros((3, 3))
        J[:2, :2] = -rotation(self.origin[2])
        J[:, 2] = [self.origin[1], -self.origin[0], -1]
        self.P = J @ ctx.estimate.covariance @ J.T
        self.corners, self.forks, self.side = rectangles(ctx.geometry)

    def nominal(self):
        """Check future guide geometry with the same tracking reserve."""
        return Clearance(
            replace(
                self.ctx,
                estimate=replace(self.ctx.estimate, covariance=((0.0, 0.0, 0.0),) * 3),
            )
        )

    def face_distance(self, pose=None):
        """Lower bound on distance of every projected piece from the entry plane."""
        pose = self.origin if pose is None else pose
        points = self.corners @ rotation(pose[2]).T + pose[:2]
        allowance = self._uncertainty_margin(0, self.origin[1] - points[:, :, 1])
        return float(-(points[:, :, 0] + allowance).max())

    def _uncertainty_margin(self, i, lever):
        P, lim = self.P, self.ctx.limits
        variance = P[i, i] + 2 * lever * P[i, 2] + lever**2 * P[2, 2]
        return lim.tracking + lim.sigma * np.sqrt(np.maximum(variance, 0))

    def approach_reserve(self, speed, pose=None):
        lim = self.ctx.limits
        tau = (lim.delay_ns + lim.delay_uncertainty_ns + CONTROL_NS) * 1e-9
        stopping = abs(speed) * tau + speed**2 / (2 * lim.brake)
        return (
            self.face_distance(pose)
            - lim.stop_buffer
            - lim.approach_safety_factor * stopping
        )

    def approach_speed_limit(self, pose):
        lim = self.ctx.limits
        distance = max(
            0.0,
            (self.face_distance(pose) - lim.stop_buffer) / lim.approach_safety_factor,
        )
        tau = (lim.delay_ns + lim.delay_uncertainty_ns + CONTROL_NS) * 1e-9
        return max(
            0.0,
            np.sqrt((lim.brake * tau) ** 2 + 2 * lim.brake * distance)
            - lim.brake * tau,
        )

    def approach_allowed(self, candidate):
        if self.ctx.recovery or candidate.speed < 0:
            return False
        speed = max(
            [
                abs(self.ctx.feedback.axle_speed),
                abs(self.ctx.active.speed),
                abs(candidate.speed),
                abs(self.ctx.history_base.speed) if self.ctx.history_base else 0.0,
                abs(self.ctx.nominal_targets[0]) if self.ctx.nominal_targets else 0.0,
            ]
            + [abs(c.speed) for c in (*self.ctx.pending, *self.ctx.issued_history)]
        )
        return self.approach_reserve(speed) > 0

    def admissible_guide(self, guide, candidate):
        margin = self.guide(guide)
        if margin > 0:
            return True
        return self.approach_allowed(candidate) and self.nominal().guide(guide) > 0

    def at(self, pose):
        ctx = self.ctx
        points = self.corners @ rotation(pose[2]).T + pose[:2]
        # Sensitivities of each projected corner to the inverse pose at rollout start.
        rx, ry = (points - self.origin[:2]).transpose(2, 0, 1)
        x, y = points.transpose(2, 0, 1)
        mx, my = self._uncertainty_margin(0, -ry), self._uncertainty_margin(1, rx)
        before = -(x + mx).max(axis=1)
        # Non-fork pieces use a conservative full-width pallet envelope.
        around = (
            np.maximum((y - my).min(axis=1), -(y + my).max(axis=1))
            - ctx.geometry.pallet_half_width
        )
        after = (x - mx).min(axis=1) - ctx.geometry.length
        body = np.maximum.reduce([before, around, after])
        lo, hi = ctx.geometry.opening
        side_y = self.side[:, None] * y
        lateral = np.minimum(
            (side_y - my).min(axis=1) - lo, hi - (side_y + my).max(axis=1)
        )
        corridor = np.minimum(lateral, ctx.geometry.length - (x + mx).max(axis=1))
        result = np.where(self.forks, np.maximum(before, corridor), body)
        for i in np.flatnonzero(self.forks & (before < 0.01)):
            # Clip the blade to the potentially occupied side of the face.
            # A heel still in free space need not fit between the pocket walls.
            plane = -mx[i].max() - 0.01
            blade_points = points[i]
            clipped = list(blade_points[x[i] >= plane])
            for j, k in ((0, 1), (0, 2), (1, 3), (2, 3)):
                if (x[i, j] - plane) * (x[i, k] - plane) < 0:
                    fraction = (plane - x[i, j]) / (x[i, k] - x[i, j])
                    clipped.append(
                        blade_points[j] + fraction * (blade_points[k] - blade_points[j])
                    )
            if clipped:
                q = np.asarray(clipped)
                sy = self.side[i] * q[:, 1]
                my_clip = self._uncertainty_margin(1, q[:, 0] - self.origin[0])
                corridor = min((sy - my_clip).min() - lo, hi - (sy + my_clip).max())
                result[i] = max(
                    before[i], min(corridor, ctx.geometry.length - (x[i] + mx[i]).max())
                )
        return float(result.min())

    def swept(self, poses):
        """Sample at most 5 mm of corner travel, independent of planner spacing."""
        poses = np.asarray(poses, dtype=float)
        delta = np.diff(poses, axis=0)
        delta[:, 2] = np.arctan2(np.sin(delta[:, 2]), np.cos(delta[:, 2]))
        radius = np.linalg.norm(self.corners, axis=2).max()
        travel = np.linalg.norm(delta[:, :2], axis=1) + radius * abs(delta[:, 2])
        counts = np.maximum(1, np.ceil(travel / 0.005)).astype(int)
        if counts.sum() > 20000:
            return -1.0
        indices = np.repeat(np.arange(len(counts)), counts)
        offsets = np.arange(len(indices)) - np.repeat(counts.cumsum() - counts, counts)
        points = np.vstack(
            (
                poses[indices] + delta[indices] * (offsets / counts[indices])[:, None],
                poses[-1],
            )
        )
        # Batch the same projected rectangle and clipped-blade tests as at().
        return min(
            self._batch_margin(chunk)
            for chunk in np.array_split(points, max(1, ceil(len(points) / 512)))
        )

    def _batch_margin(self, poses):
        angle = poses[:, 2, None, None]
        c, s = np.cos(angle), np.sin(angle)
        bx, by = self.corners.transpose(2, 0, 1)
        x = poses[:, 0, None, None] + c * bx - s * by
        y = poses[:, 1, None, None] + s * bx + c * by
        g = self.ctx.geometry
        mx = self._uncertainty_margin(0, self.origin[1] - y)
        my = self._uncertainty_margin(1, x - self.origin[0])
        before = -(x + mx).max(axis=2)
        after = (x - mx).min(axis=2) - g.length
        around = (
            np.maximum((y - my).min(axis=2), -(y + my).max(axis=2))
            - g.pallet_half_width
        )
        result = np.maximum.reduce([before, after, around])
        for i in np.flatnonzero(self.forks):
            # Include all blade vertices and edge intersections in the occupied half-space.
            xx, yy = x[:, i], y[:, i]
            plane = -mx[:, i].max(axis=1, keepdims=True) - 0.01
            j, k = [0, 0, 1, 2], [1, 2, 3, 3]
            dx = xx[:, k] - xx[:, j]
            frac = (plane - xx[:, j]) / np.where(dx != 0, dx, 1)
            valid = np.hstack((xx >= plane, (dx != 0) & (frac > 0) & (frac < 1)))
            sy = self.side[i] * np.hstack((yy, yy[:, j] + frac * (yy[:, k] - yy[:, j])))
            uncertainty = np.hstack(
                (
                    my[:, i],
                    np.broadcast_to(
                        self._uncertainty_margin(1, plane - self.origin[0]), frac.shape
                    ),
                )
            )
            lateral = np.minimum(
                sy - uncertainty - g.opening[0], g.opening[1] - sy - uncertainty
            )
            corridor = np.where(valid, lateral, np.inf).min(axis=1)
            # at() uses the whole rectangle outside the clipping region.
            full = self.side[i] * yy
            whole = np.minimum(
                full - my[:, i] - g.opening[0], g.opening[1] - full - my[:, i]
            ).min(axis=1)
            corridor = np.where(
                (before[:, i] < 0.01) & valid.any(axis=1), corridor, whole
            )
            result[:, i] = np.maximum(
                before[:, i],
                np.minimum(corridor, g.length - (xx + mx[:, i]).max(axis=1)),
            )
        return float(result.min())

    def segment(self, a, b):
        return self.swept((a, b))

    def guide(self, guide):
        if (
            guide is None
            or guide.target_id != self.ctx.target_id
            or not guide.samples
            or guide.stamp.episode != self.ctx.stamp.episode
            or guide.stamp.time_ns > self.ctx.stamp.time_ns
            or guide.stamp.frame != "pallet"
            or not np.isfinite(guide.samples).all()
        ):
            return -1.0
        return self.swept([self.origin, *(s[:3] for s in guide.samples)])

    def stopping(self, candidate, *, entry_guide=None, rollout=None):
        lim = self.ctx.limits
        delays = sorted(
            {
                lim.delay_ns - lim.delay_uncertainty_ns,
                lim.delay_ns + lim.delay_uncertainty_ns,
            }
        )
        # A full checked guide supplies the intended traversal through the pockets.
        # The stopping sweep remains mandatory for every candidate.
        planned_entry = False
        if (
            entry_guide is not None
            and rollout is not None
            and entry_guide.samples
            and rollout.samples
        ):
            x, y, yaw, speed = entry_guide.samples[-1]
            g, criteria = self.ctx.geometry, self.ctx.criteria
            depth = x + g.tip_x * cos(yaw) - (g.tip_y + g.half_width) * abs(np.sin(yaw))
            planned_entry = (
                speed == 0
                and rollout.samples[-1][3] == 0
                and depth >= criteria.depth
                and abs(yaw) < criteria.yaw
                and abs(y + g.tip_x * np.sin(yaw)) < criteria.lateral
                and self.guide(entry_guide) > 0
                and self.guide(rollout) > 0
            )
        worst = None
        for delay in delays:
            self.path = []
            margin = self._stop_at_delay(candidate, delay, planned_entry)
            if worst is None or margin < worst[0]:
                worst = margin, self.path
        self.path = worst[1]
        return worst[0]

    def _stop_at_delay(self, candidate, release, planned_entry=False):
        ctx = self.ctx
        lim = ctx.limits
        now = ctx.stamp.time_ns
        command = ctx.history_base or ctx.active
        history = ctx.issued_history if ctx.history_base else ctx.pending
        queue = [(max(0, c.stamp.time_ns + release - now), c) for c in history]
        queue += [(release, candidate), (release + CONTROL_NS, Command(ctx.stamp))]
        pose = self.origin
        v = ctx.feedback.axle_speed
        angles = np.array(ctx.feedback.angles)
        margin = self.at(pose)
        self.path.append(pose)
        if (
            abs(v) < 1.0e-5
            and all(c.speed == 0 for _, c in queue)
            and command.speed == 0
        ):
            return margin
        # Integrate to each application boundary. Limit corner travel per sample.
        steering_target = angles.copy()
        stop_samples = []
        entry_checked = planned_entry
        elapsed = 0
        end = release + CONTROL_NS + int((lim.speed / lim.brake + 0.5) * 1e9)
        while elapsed < end:
            while queue and queue[0][0] <= elapsed:
                _, command = queue.pop(0)
            ns = min(4_000_000, (queue[0][0] if queue else end) - elapsed)
            dt = ns * 1e-9
            target = command.speed
            rate = lim.brake if abs(target) < abs(v) else lim.target_accel
            next_v = v + np.clip(target - v, -rate * dt, rate * dt)
            desired, _ = ackermann(ctx.geometry, target, command.curvature)
            steering_target += np.clip(
                desired - steering_target,
                -lim.target_steer_rate * dt,
                lim.target_steer_rate * dt,
            )
            angles += (steering_target - angles) * (
                1 - exp(-dt / lim.steering_response_s)
            )
            k = sum(curvatures(ctx.geometry, angles)) / 2
            pose = compose(pose, arc((v + next_v) * dt / 2, k))
            stop_samples.append(pose)
            # Start the remaining-traversal check before either outer tip reaches the face.
            heading = pose[2]
            depth = pose[0] + ctx.geometry.tip_x * cos(heading)
            leading = depth + (ctx.geometry.tip_y + ctx.geometry.half_width) * abs(
                np.sin(heading)
            )
            if (
                candidate.speed > 0
                and not entry_checked
                and leading >= -lim.stop_buffer
            ):
                if cos(heading) <= 0:
                    return -1.0
                distance = max(
                    0.0, ctx.criteria.depth + ctx.criteria.target_allowance - depth
                ) / cos(heading)
                margin = min(
                    margin, self.segment(pose, compose(pose, (distance, 0, 0)))
                )
                entry_checked = True
            if not self.path or elapsed // CONTROL_NS != (elapsed + ns) // CONTROL_NS:
                self.path.append(pose)
            v = next_v
            elapsed += ns
            if not queue and abs(v) < 1.0e-8:
                break
        if stop_samples:
            margin = min(margin, self._batch_margin(np.asarray(stop_samples)))
        prefix_speed = next(
            (c.speed for c in reversed(ctx.pending) if c.speed), ctx.active.speed
        )
        if candidate.speed or ctx.feedback.axle_speed or prefix_speed:
            sign = np.sign(candidate.speed or ctx.feedback.axle_speed or prefix_speed)
            buffered = compose(pose, arc(sign * lim.stop_buffer, k))
            self.path.extend((pose, buffered))
            margin = min(margin, self.segment(pose, buffered))
        return margin


class Supervisor:
    def __init__(self):
        self.attempts = 0
        self.recovery = False
        self.still_since = None
        self.direction = 0
        self.no_proposal_since = None
        self.recovery_guide = None
        self.reacquire_after = -1
        self.revision = 0

    def decide(self, ctx, proposal):
        t = ctx.stamp.time_ns
        feedback, estimate, criteria, limits = (
            ctx.feedback,
            ctx.estimate,
            ctx.criteria,
            ctx.limits,
        )
        stopped = (
            abs(feedback.axle_speed) < criteria.still_speed
            and abs(feedback.yaw_rate) < criteria.still_yaw_rate
        )
        if not stopped:
            self.still_since = None
        elif self.still_since is None:
            self.still_since = t
        settled = (
            self.still_since is not None
            and t - self.still_since >= criteria.settle_ns
            and ctx.active.speed == 0
            and not any(cmd.speed for cmd in ctx.pending)
        )
        zero = Command(ctx.stamp)
        command = zero
        reason, terminal = "hold", ""
        margin = None
        guard = Clearance(ctx) if estimate.initialized else None
        if not estimate.initialized:
            reason = "acquiring"
            if t >= criteria.acquire_ns:
                terminal = "acquisition_timeout"
        elif t - estimate.last_capture_ns > criteria.fresh_ns and not (
            t - estimate.last_capture_ns < criteria.acquire_ns
            and proposal is not None
            and proposal.status == "drive"
            and proposal.command.speed >= 0
            and guard.approach_allowed(proposal.command)
        ):
            reason = "stale_detection"
            if t - estimate.last_capture_ns >= criteria.acquire_ns:
                terminal = "uncertainty_abort"
        elif proposal is None:
            reason = "awaiting_proposal"
        elif (
            proposal.command.stamp.episode != ctx.stamp.episode
            or not proposal.command.stamp.time_ns <= t < proposal.valid_until_ns
        ):
            reason = "expired_proposal"
        elif proposal.status == "retry":
            reason = "awaiting_retreat_guide"
            guide = proposal.guide
            if self.attempts >= criteria.retries and not self.recovery:
                terminal = "retry_exhaustion"
            elif settled and not self.recovery and guide and guard.guide(guide) > 0:
                speeds = np.array([s[3] for s in guide.samples])
                forward = np.flatnonzero(speeds > 0)
                if (
                    len(forward)
                    and speeds[0] < 0
                    and 0 in speeds[: forward[0]]
                    and np.all(speeds[forward[0] :] >= 0)
                ):
                    self.recovery_guide = guide
                    self.attempts += 1
                    self.recovery = True
                    self.revision += 1
                    reason = "recovery_authorized"
        elif proposal.status == "reacquire" and self.recovery and settled:
            self.recovery = False
            self.reacquire_after = t
            self.revision += 1
            reason = "reacquiring"
        elif estimate.last_capture_ns <= self.reacquire_after:
            reason = "reacquiring"
        elif proposal.status == "complete":
            g = ctx.geometry
            x, y, heading = guard.origin
            tip_lateral = y + g.tip_x * np.sin(heading)
            depth = (
                x
                + g.tip_x * cos(heading)
                - (g.tip_y + g.half_width) * abs(np.sin(heading))
            )
            if (
                settled
                and depth >= criteria.depth
                and abs(heading) < criteria.yaw
                and abs(tip_lateral) < criteria.lateral
                and guard.at(guard.origin) > 0
            ):
                terminal = "assess_completion"
            else:
                reason = "estimated_completion_pending"
        elif proposal.status == "failed":
            terminal = "planning_failure"
        elif proposal.status == "drive":
            v, k = proposal.command.speed, proposal.command.curvature
            if (
                not np.isfinite([v, k]).all()
                or abs(v) > limits.speed
                or (v < 0 and abs(v) > limits.recovery_speed)
            ):
                reason = "motion_limit"
            elif max(map(abs, ackermann(ctx.geometry, v, k)[0])) > limits.steer:
                reason = "steering_limit"
            elif (v < 0 and not self.recovery) or (v > 0 and self.recovery):
                reason = "direction_unauthorized"
            elif self.direction * np.sign(v) < 0 and (
                not settled or ctx.active.speed or any(p.speed for p in ctx.pending)
            ):
                reason = "direction_change"
            elif self.recovery and (
                proposal.guide is None
                or proposal.guide.plan_id != self.recovery_guide.plan_id
            ):
                reason = "recovery_guide_required"
            elif any(
                path is not None and not guard.admissible_guide(path, proposal.command)
                for path in (proposal.guide, proposal.rollout)
            ):
                reason = "guide_clearance"
            else:
                command = Command(ctx.stamp, v, k)
                reason = "released"
        if reason in ("expired_proposal", "awaiting_proposal"):
            if self.no_proposal_since is None:
                self.no_proposal_since = t
            if t - self.no_proposal_since >= criteria.acquire_ns:
                terminal = "expired_proposal"
        else:
            self.no_proposal_since = None
        entry_paths = {}
        if proposal and proposal.status == "drive" and reason == "released":
            entry_paths = {"entry_guide": proposal.guide, "rollout": proposal.rollout}
        if guard:
            margin = guard.stopping(command, **entry_paths)
            if margin <= 0:
                command, reason = zero, "clearance"
                margin = guard.stopping(zero)
        if guard and command.speed and limits.uncertainty_speed_cap:
            free = (
                ctx.geometry.opening[1] - ctx.geometry.tip_y - ctx.geometry.half_width
            )
            factor = min(1.0, max(0.0, margin / free))
            if factor < 1:
                command = Command(ctx.stamp, command.speed * factor, command.curvature)
                margin = guard.stopping(command, **entry_paths)
                reason = "uncertainty_speed_cap"
                if margin <= 0:
                    command, reason = zero, "clearance"
                    margin = guard.stopping(zero)
        if command.speed:
            self.direction = np.sign(command.speed)
        queued_speed = max(
            [abs(feedback.axle_speed), abs(ctx.active.speed), abs(command.speed)]
            + [abs(p.speed) for p in ctx.pending]
        )
        stop = (
            queued_speed
            * (limits.delay_ns + limits.delay_uncertainty_ns + CONTROL_NS)
            * 1e-9
            + queued_speed**2 / (2 * limits.brake)
            + limits.stop_buffer
        )
        return SupervisorDecision(
            command,
            reason,
            self.recovery,
            self.attempts,
            margin,
            stop,
            terminal,
            tuple(guard.path) if guard else (),
        )
