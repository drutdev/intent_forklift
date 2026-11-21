import unittest
from dataclasses import replace
from unittest.mock import patch

import numpy as np

from pallet_engagement.cc import connectors
from pallet_engagement.config import CONTROL_NS, Config
from pallet_engagement.frames import inverse
from pallet_engagement.geometric import (
    joined_curve,
    pursuit,
    timed_path,
    tracking_curvature,
)
from pallet_engagement.guard import Supervisor
from pallet_engagement.messages import Command, ControlContext, Estimate
from pallet_engagement.methods import load
from pallet_engagement.plant import Plant
from pallet_engagement.steering import curvature_limit, wheels


class GeometricChecks(unittest.TestCase):
    @classmethod
    def setUpClass(cls):
        cfg = Config()
        plant = Plant(cfg, "geometric-test")
        stamp = plant.stamp()
        cls.ctx = ControlContext(
            stamp,
            Estimate(stamp, plant.relative(), ((0.0, 0.0, 0.0),) * 3, 0, True),
            plant.feedback(),
            plant.geometry,
            cfg.limits,
            cfg.criteria,
            (),
            Command(stamp),
            False,
            0,
            "pallet/front",
            "geometric-test",
        )

    def test_join_and_complete_speed_profile(self):
        ctx = self.ctx
        start = (-5, 0.12, 0.05, -0.03)
        end = ctx.criteria.depth + ctx.criteria.target_allowance - ctx.geometry.tip_x
        x = -ctx.geometry.tip_x - 0.8
        rows, join = joined_curve(start, x, end, 2.4, 2.5)
        mirror, _ = joined_curve((-5, -0.12, -0.05, 0.03), x, end, 2.4, 2.5)
        np.testing.assert_allclose(rows, mirror * [1, -1, -1, -1], atol=1e-9)
        np.testing.assert_allclose(rows[0], start, atol=1e-10)
        np.testing.assert_allclose(rows[join], [x, 0, 0, 0])
        np.testing.assert_allclose(rows[join:, 1:], 0)
        self.assertLess(abs(rows[join - 1, 3]), 0.02)
        path = timed_path(rows, join, ctx, 0.22, 0.10, 0)
        self.assertTrue((np.diff(path.s) > 0).all())
        self.assertTrue((np.diff(path.time) > 0).all())
        self.assertEqual(path.v[0], 0)
        self.assertEqual(path.v[-1], 0)
        self.assertTrue((path.v[1:-1] > 0).all())
        self.assertAlmostEqual(path.v[join], 0.10)
        self.assertTrue((path.v[join:] <= 0.10).all())
        acceleration = np.diff(path.v**2) / (2 * np.diff(path.s))
        self.assertLessEqual(acceleration.max(), ctx.limits.target_accel)
        self.assertGreaterEqual(acceleration.min(), -ctx.limits.brake)
        angles = wheels(path.curvature, ctx.geometry)
        self.assertLess(abs(angles).max(), ctx.limits.steer)
        self.assertLessEqual(
            (abs(np.diff(angles, axis=0)) / np.diff(path.time)[:, None]).max(),
            ctx.limits.target_steer_rate + 1e-9,
        )

    def test_cc_speed_maps_both_wheels_and_lag_headroom(self):
        ctx = self.ctx
        start = (-6.0, -1.0, 0.35, 0.0)
        limit = curvature_limit(ctx.geometry, ctx.limits)
        rows, join, _ = next(
            connectors(start, -0.955, (-2.78, -1.68), 0.85 * limit, 0.6)
        )
        path = timed_path(rows, join, ctx, 0.22, 0.10, 0)
        self.assertTrue(np.all(path.v[1:-1] > 0))
        angles = wheels(path.curvature, ctx.geometry)
        slope = np.gradient(path.curvature, path.s)
        compensated = wheels(
            path.curvature + path.v * ctx.limits.steering_response_s * slope,
            ctx.geometry,
        )
        self.assertLessEqual(abs(compensated).max(), ctx.limits.steer + 1e-10)
        self.assertLessEqual(
            (abs(np.diff(angles, axis=0)) / np.diff(path.time)[:, None]).max(),
            ctx.limits.target_steer_rate,
        )
        self.assertLessEqual(abs(slope).max(), 0.601)
        self.assertGreater(path.v[join], ctx.criteria.still_speed)
        face = np.searchsorted(path.pose[:, 0], -ctx.geometry.tip_x)
        self.assertGreater(path.v[face], ctx.criteria.still_speed)

    def test_reverse_pursuit_and_invalid_curve(self):
        for lateral in (-0.05, 0.05):
            point = np.array([-0.7, lateral])
            body = pursuit((0, 0, 0), point)
            travel = pursuit((0, 0, np.pi), point)
            self.assertAlmostEqual(body, -travel)
            self.assertGreater(body * lateral, 0)
            self.assertLess(-0.09 * body * lateral, 0)
        self.assertIsNone(joined_curve((-2, 0.05, 0, 0), -2.2, -1, 0.4, 0.4))

    def test_launch_and_stale_entry(self):
        ctx = self.ctx
        method = load("geometric")
        method.reset(())
        launch = method.propose(ctx)
        self.assertEqual(launch.status, "drive")
        self.assertGreater(launch.command.speed, 0)
        self.assertLess(launch.command.speed, 0.05)
        self.assertEqual(launch.guide.samples[-1][3], 0)
        self.assertGreater(len(launch.rollout.samples), 2)
        stamp = replace(ctx.stamp, time_ns=2_000_000_000)
        pose = inverse((-ctx.geometry.tip_x - 0.01, 0, 0))
        stale = replace(
            ctx, stamp=stamp, estimate=replace(ctx.estimate, pose=pose, stamp=stamp)
        )
        self.assertEqual(
            Supervisor().decide(stale, method.propose(stale)).command.speed, 0
        )

    def test_checked_retreat_needs_stopped_authorization(self):
        ctx = self.ctx
        pose = inverse((-ctx.geometry.tip_x - 0.55, 0.06, 0))
        ctx = replace(ctx, estimate=replace(ctx.estimate, pose=pose))
        method = load("geometric")
        method.reset(())
        proposal = method.propose(ctx)
        self.assertEqual(proposal.status, "retry")
        self.assertAlmostEqual(method.retreat.s[-1], 2.5)
        self.assertAlmostEqual(-method.retreat.v.min(), 0.4)
        self.assertEqual(method.retreat.v[0], 0)
        self.assertEqual(method.retreat.v[-1], 0)
        speeds = np.array(proposal.guide.samples)[:, 3]
        self.assertLess(speeds[0], 0)
        first_forward = np.flatnonzero(speeds > 0)[0]
        self.assertIn(0, speeds[:first_forward])
        supervisor = Supervisor()
        self.assertFalse(supervisor.decide(ctx, proposal).recovery)
        stamp = replace(ctx.stamp, time_ns=ctx.criteria.settle_ns + CONTROL_NS)
        ctx = replace(ctx, stamp=stamp)
        self.assertTrue(supervisor.decide(ctx, method.propose(ctx)).recovery)
        retreat = method.propose(replace(ctx, revision=1, recovery=True))
        self.assertLess(retreat.command.speed, 0)
        self.assertEqual(retreat.guide.plan_id, proposal.guide.plan_id)

    def test_changed_withdrawal_clearance_aborts_waiting_request(self):
        ctx = self.ctx
        pose = inverse((-ctx.geometry.tip_x - 0.55, 0.06, 0))
        ctx = replace(ctx, estimate=replace(ctx.estimate, pose=pose))
        method = load("geometric")
        method.reset(())
        self.assertEqual(method.propose(ctx).status, "retry")
        # A new estimate puts an outer blade across a pocket wall.
        pose = inverse((-ctx.geometry.tip_x + 0.05, 0.09, 0))
        ctx = replace(ctx, estimate=replace(ctx.estimate, pose=pose))
        proposal = method.propose(ctx)
        self.assertEqual(proposal.command.speed, 0)
        self.assertEqual(
            Supervisor().decide(ctx, proposal).terminal, "planning_failure"
        )

    def test_infeasible_plan_waits_for_stationary_estimate_before_abort(self):
        method = load("geometric")
        method.reset(())
        ctx = self.ctx
        with (
            patch.object(method, "forward", return_value=None) as forward,
            patch.object(method, "plan_retreat", return_value=None),
        ):
            first = method.propose(ctx)
            self.assertEqual(first.status, "hold")
            self.assertEqual(first.command.speed, 0)
            stamp = replace(ctx.stamp, time_ns=500_000_000)
            method.propose(replace(ctx, stamp=stamp))
            self.assertEqual(forward.call_count, 1)
            stamp = replace(ctx.stamp, time_ns=8_000_000_000)
            self.assertEqual(method.propose(replace(ctx, stamp=stamp)).status, "failed")

    def test_planning_wait_is_not_counted_as_a_new_path_stall(self):
        ctx = self.ctx
        origin = inverse(ctx.estimate.pose)
        goal = ctx.criteria.depth + ctx.criteria.target_allowance - ctx.geometry.tip_x
        rows = np.zeros((201, 4))
        rows[:, 0] = np.linspace(origin[0], goal, len(rows))
        path = timed_path(rows, 0, ctx, 0.1, 0.1, 0)
        method = load("geometric")
        method.reset(())
        with (
            patch.object(method, "forward", return_value=None),
            patch.object(method, "plan_retreat", return_value=None),
        ):
            self.assertEqual(method.propose(ctx).status, "hold")
        stamp = replace(ctx.stamp, time_ns=7_000_000_000)
        ctx = replace(
            ctx,
            stamp=stamp,
            estimate=replace(ctx.estimate, stamp=stamp, last_capture_ns=stamp.time_ns),
        )
        with patch.object(method, "forward", return_value=path):
            self.assertEqual(method.propose(ctx).status, "drive")
        stamp = replace(stamp, time_ns=stamp.time_ns + CONTROL_NS)
        ctx = replace(ctx, stamp=stamp, estimate=replace(ctx.estimate, stamp=stamp))
        self.assertEqual(method.propose(ctx).status, "drive")

    def test_fork_tip_yaw_feedback_damps_both_travel_directions(self):
        ctx = self.ctx
        for sign in (1, -1):
            rows = np.array(
                [[0, 0, 0, 0], [sign, 0, 0, 0], [2 * sign, 0, 0, 0]], dtype=float
            )
            path = timed_path(rows, 0, ctx, 0.1, 0.1, 0.1, reverse=sign < 0)
            for yaw in (-0.02, 0.02):
                curvature = tracking_curvature(
                    path, 0, (0, 0, yaw), sign * 0.1, sign, 1.5, ctx
                )
                self.assertLess(sign * 0.1 * curvature * yaw, 0)

    def test_proposal_covers_worker_jitter_and_expires_at_its_deadline(self):
        ctx = self.ctx
        method = load("geometric")
        method.reset(())
        proposal = method.propose(ctx)
        self.assertEqual(proposal.status, "drive")
        self.assertEqual(proposal.valid_until_ns - ctx.stamp.time_ns, 100_000_000)
        delayed = replace(ctx, stamp=replace(ctx.stamp, time_ns=60_000_000))
        self.assertGreater(Supervisor().decide(delayed, proposal).command.speed, 0)
        expired = replace(ctx, stamp=replace(ctx.stamp, time_ns=100_000_000))
        decision = Supervisor().decide(expired, proposal)
        self.assertEqual(decision.reason, "expired_proposal")
        self.assertEqual(decision.command.speed, 0)
