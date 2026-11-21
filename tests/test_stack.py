import tempfile
import time
import unittest
from dataclasses import replace
from math import radians
from pathlib import Path

import numpy as np

from pallet_engagement.config import CAPTURE_NS, CONTROL_NS, Config, Scenario, Sensors
from pallet_engagement.estimation import EKF, SensorModel
from pallet_engagement.frames import ackermann, compose, curvatures, inverse
from pallet_engagement.guard import Clearance, Supervisor
from pallet_engagement.messages import (
    Command,
    ControlContext,
    GuideTrajectory,
    MotionProposal,
    OdometryIncrement,
)
from pallet_engagement.methods import REGISTRY, Hold, Registration, load
from pallet_engagement.plant import Plant
from pallet_engagement.recording import replay
from pallet_engagement.runtime import run


class Slow(Hold):
    def propose(self, context):
        time.sleep(0.12)
        return super().propose(context)


class ScriptedDrive(Hold):
    def propose(self, ctx):
        return MotionProposal(
            Command(ctx.stamp, 0.15, 0), ctx.stamp.time_ns + 2 * CONTROL_NS
        )


class StackChecks(unittest.TestCase):
    @classmethod
    def setUpClass(cls):
        cls.cfg = Config(scenario=Scenario(0.03, 0, 0, 0.8))
        cls.plant = Plant(cls.cfg, "test")
        cls.g = cls.plant.geometry
        cls.fb = cls.plant.feedback()

    def context(self, est, stamp=None):
        return ControlContext(
            stamp or est.stamp,
            est,
            self.fb,
            self.g,
            self.cfg.limits,
            self.cfg.criteria,
            (),
            Command(self.fb.stamp),
            False,
            0,
            "pallet/front",
            "test",
        )

    def test_frames_and_time(self):
        cfg = replace(self.cfg, sensors=Sensors(noise_scale=0, dropout=0))
        sensor = SensorModel(cfg, self.g)
        ekf = EKF(cfg, self.fb.stamp)
        pose = (1.25, -0.37, 3.13)
        obs = sensor.capture(self.fb.stamp, pose)
        self.assertTrue(ekf.correct(obs, True))
        np.testing.assert_allclose(ekf.x, pose, atol=1e-14)
        self.assertFalse(ekf.correct(obs, True))
        delta = (0.12, 0.03, -0.04)
        expected = compose(inverse(delta), pose)
        stamp = replace(self.fb.stamp, time_ns=CONTROL_NS, seq=10)
        future = sensor.capture(stamp, expected)
        self.assertFalse(ekf.correct(future, True))
        increment = OdometryIncrement(stamp, 0, delta, ((0, 0, 0),) * 3, 0)
        self.assertTrue(ekf.predict(increment))
        np.testing.assert_allclose(ekf.x, expected)
        ekf.deliver_due(True)
        self.assertEqual(ekf.last, CONTROL_NS)
        self.assertFalse(ekf.predict(increment))
        self.assertFalse(ekf.correct(obs, True))
        self.assertEqual(ekf.rejected, 3)
        for k in (-0.3, 0, 0.3):
            angles, _ = ackermann(self.g, 0.25, k)
            np.testing.assert_allclose(curvatures(self.g, angles), [k, k], atol=1e-14)

    def test_damped_detection_keeps_covariance_consistent_with_gain(self):
        cfg = replace(self.cfg, sensors=Sensors(dropout=0))
        sensor = SensorModel(cfg, self.g)
        normal = EKF(cfg, self.fb.stamp)
        damped = EKF(replace(cfg, moving_detection_gain=0.5), self.fb.stamp)
        initial = sensor.capture(self.fb.stamp, (2.0, 0.0, 0.0))
        for ekf in (normal, damped):
            self.assertTrue(ekf.correct(initial, True))
        stamp = replace(self.fb.stamp, time_ns=CAPTURE_NS, seq=100)
        increment = OdometryIncrement(
            stamp,
            0,
            (0.02, 0.0, 0.0),
            ((1e-5, 0.0, 0.0), (0.0, 1e-5, 0.0), (0.0, 0.0, 1e-6)),
            0.0,
        )
        for ekf in (normal, damped):
            ekf.predict(increment)
        prior, covariance = normal.x.copy(), normal.P.copy()
        observation = sensor.capture(stamp, (1.98, 0.002, 0.001))
        self.assertTrue(normal.correct(observation, False))
        self.assertTrue(damped.correct(observation, False))
        np.testing.assert_allclose(
            damped.x - prior, 0.5 * (normal.x - prior), atol=1e-12
        )
        V = np.asarray(observation.covariance)
        K = 0.5 * covariance @ np.linalg.inv(covariance + V)
        A = np.eye(3) - K
        np.testing.assert_allclose(
            damped.P, A @ covariance @ A.T + K @ V @ K.T, atol=1e-15
        )
        self.assertGreaterEqual(np.linalg.eigvalsh(damped.P - normal.P).min(), -1e-14)
        # Stationary acquisition uses the full gain.
        stationary = EKF(replace(cfg, moving_detection_gain=0.5), self.fb.stamp)
        baseline = EKF(cfg, self.fb.stamp)
        for ekf in (stationary, baseline):
            ekf.correct(initial, True)
            ekf.predict(increment)
            ekf.correct(observation, True)
        np.testing.assert_array_equal(stationary.x, baseline.x)
        np.testing.assert_array_equal(stationary.P, baseline.P)

    def test_outlier_gate_and_duplicate_sequence(self):
        sensor = SensorModel(replace(self.cfg, sensors=Sensors(dropout=0)), self.g)
        ekf = EKF(self.cfg, self.fb.stamp)
        ekf.correct(sensor.capture(self.fb.stamp, (1, 0, 0)), True)
        stamp = replace(self.fb.stamp, time_ns=CONTROL_NS, seq=10)
        increment = OdometryIncrement(stamp, 0, (0, 0, 0), ((0, 0, 0),) * 3, 0)
        ekf.predict(increment)
        previous = ekf.x.copy()
        outlier = sensor.capture(stamp, (4, 2, 1))
        self.assertFalse(ekf.correct(outlier, True))
        np.testing.assert_array_equal(previous, ekf.x)
        self.assertEqual(ekf.last, 0)
        later = replace(stamp, time_ns=2 * CONTROL_NS, seq=20)
        ekf.predict(replace(increment, stamp=later, start_ns=CONTROL_NS))
        self.assertFalse(
            ekf.correct(replace(outlier, stamp=replace(later, seq=10)), True)
        )
        self.assertEqual(ekf.rejected, 2)

    def test_seeded_sensors_and_outage(self):
        left, right = (SensorModel(self.cfg, self.g) for _ in range(2))
        messages = []
        for i in range(100):
            stamp = replace(self.fb.stamp, time_ns=i * CAPTURE_NS, seq=i * 100)
            msg = left.capture(stamp, (1, 0, 0.1))
            self.assertEqual(msg, right.capture(stamp, (1, 0, 0.1)))
            messages.append(msg)
        self.assertTrue(0 < messages.count(None) < 50)
        a = self.fb
        b = replace(a, stamp=stamp, encoders=(1.5, 1.5), angles=(0.1, 0.11))
        self.assertEqual(left.odometry(a, b), right.odometry(a, b))
        ekf = EKF(self.cfg, self.fb.stamp)
        ekf.correct(messages[0], True)
        last = ekf.last
        missed = SensorModel(replace(self.cfg, sensors=Sensors(dropout=1)), self.g)
        self.assertIsNone(missed.capture(stamp, (1, 0, 0.1)))
        prior = ekf.P.copy()
        ekf.predict(missed.odometry(a, b))
        self.assertEqual(ekf.estimate().stamp.time_ns, stamp.time_ns)
        self.assertEqual(ekf.last, last)
        self.assertGreater(ekf.P[2, 2], prior[2, 2])

    def test_stationary_acquisition_and_clearance(self):
        cfg = self.cfg
        sensor = SensorModel(cfg, self.g)
        ekf = EKF(cfg, self.fb.stamp)
        margin = -1
        previous = self.fb
        for i in range(150):
            stamp = replace(self.fb.stamp, time_ns=i * CONTROL_NS, seq=i * 10)
            fb = replace(self.fb, stamp=stamp)
            if i:
                ekf.predict(sensor.odometry(previous, fb))
            previous = fb
            if i % 10 == 0:
                obs = sensor.capture(stamp, self.plant.relative())
                if obs:
                    ekf.correct(obs, True)
            if ekf.last is not None:
                guard = Clearance(self.context(ekf.estimate()))
                end = compose(guard.origin, (0.655, 0, 0))
                margin = guard.segment(guard.origin, end)
                if margin > 0:
                    break
        print(
            f"acquisition: t={stamp.time_ns * 1e-9:.2f}s margin={margin * 1000:.2f}mm gates={ekf.rejected}"
        )
        self.assertGreater(margin, 0)
        zero = ((0.0, 0.0, 0.0),) * 3
        est = replace(ekf.estimate(), pose=(self.g.tip_x - 0.60, 0, 0), covariance=zero)
        guard = Clearance(self.context(est))
        self.assertGreater(guard.at(guard.origin), 0.019)
        self.assertLess(guard.at(compose(guard.origin, (0, 0.025, 0))), 0)
        self.assertLess(guard.at(compose(guard.origin, (0, 0, radians(3)))), 0)

    def test_supervision_and_recovery(self):
        sensor = SensorModel(
            replace(self.cfg, sensors=Sensors(dropout=0, noise_scale=0)), self.g
        )
        ekf = EKF(self.cfg, self.fb.stamp)
        ekf.correct(sensor.capture(self.fb.stamp, (3.0, 0, 0)), True)
        ctx = self.context(ekf.estimate())
        sup = Supervisor()
        reverse = MotionProposal(Command(ctx.stamp, -0.1), 1_000_000_000)
        self.assertEqual(sup.decide(ctx, reverse).reason, "direction_unauthorized")
        stamp = replace(ctx.stamp, time_ns=250_000_000)
        ctx = replace(ctx, stamp=stamp)
        guide = GuideTrajectory(
            replace(stamp, frame="pallet"),
            "retry",
            ctx.target_id,
            ((-3.0, 0, 0, -0.1), (-3.1, 0, 0, 0), (-3.0, 0, 0, 0.1)),
        )
        retry = replace(reverse, status="retry", guide=guide)
        self.assertTrue(sup.decide(ctx, retry).recovery)
        self.assertEqual(
            sup.decide(
                replace(ctx, recovery=True), replace(reverse, guide=guide)
            ).command.speed,
            -0.1,
        )
        self.assertEqual(
            sup.decide(ctx, replace(reverse, valid_until_ns=0)).reason,
            "expired_proposal",
        )
        self.assertFalse(sup.decide(ctx, replace(reverse, status="reacquire")).recovery)
        self.assertEqual(sup.decide(ctx, retry).terminal, "retry_exhaustion")

    def test_estimated_completion_is_independently_scored(self):
        from pallet_engagement.evaluation import Evaluator
        from pallet_engagement.messages import Estimate

        stamp = self.fb.stamp
        est = Estimate(stamp, self.plant.relative(), ((0.0, 0.0, 0.0),) * 3, 0, True)
        supervisor = Supervisor()
        ctx = self.context(est)
        proposal = MotionProposal(Command(stamp), 1_000_000_000, "complete")
        supervisor.decide(ctx, proposal)
        ctx = replace(ctx, stamp=replace(stamp, time_ns=250_000_000))
        self.assertEqual(
            supervisor.decide(ctx, proposal).reason, "estimated_completion_pending"
        )
        # Deliberately feed a false, overconfident estimate to the stack.
        ctx = replace(ctx, estimate=replace(est, pose=(self.g.tip_x - 0.625, 0, 0)))
        decision = supervisor.decide(ctx, proposal)
        self.assertEqual(decision.terminal, "assess_completion")
        evaluator = Evaluator(self.plant, self.cfg.criteria)
        self.assertEqual(evaluator.score(decision.terminal)[1], "terminal_pose_error")

    def test_record_and_repeat(self):
        cfg = replace(self.cfg, duration=0.4)
        with tempfile.TemporaryDirectory() as root:
            path = Path(root) / "episode"
            outcome = run(cfg, path)
            self.assertTrue(outcome.recording_complete)
            self.assertEqual(replay(path), 21)
            with np.load(path / "states.npz") as data:
                self.assertEqual(len(data["qpos"]), 11)
            self.assertGreater((path / "trace.rrd").stat().st_size, 0)
            self.assertEqual(outcome.metrics["depth"], run(cfg).metrics["depth"])

    def test_slow_worker(self):
        REGISTRY["slow_test"] = Registration(__name__, "Slow", True)
        try:
            result = run(Config(method="slow_test", mode="realtime", duration=0.5))
            self.assertGreaterEqual(result.timing["late_results"], 1)
            self.assertEqual(result.timing["control"]["count"], 26)
            self.assertLess(abs(result.metrics["depth"] + 2), 0.001)
        finally:
            del REGISTRY["slow_test"]

    def test_scripted_runtime_braking_tail(self):
        REGISTRY["drive_test"] = Registration(__name__, "ScriptedDrive", True)
        try:
            cfg = Config(method="drive_test", duration=0.6)
            result = run(cfg)
            self.assertEqual(result.reason, "episode_timeout")
            self.assertGreater(result.time_ns, 600_000_000)
            self.assertGreater(result.metrics["depth"], -1.98)
            self.assertTrue(result.metrics["settled"])
        finally:
            del REGISTRY["drive_test"]

    def test_fifo_prediction_and_guard(self):
        from pallet_engagement.messages import Estimate
        from pallet_engagement.prediction import predict_queue

        stamp = self.fb.stamp
        est = Estimate(stamp, (3.0, 0.0, 0.0), ((0.0, 0.0, 0.0),) * 3, 0, True)
        ctx = self.context(est)
        forward = Command(stamp, 0.35, 0.1)
        ctx = replace(ctx, pending=(forward,))
        self.assertEqual(
            predict_queue(ctx, ctx.limits.delay_ns).pose, inverse(est.pose)
        )
        later = predict_queue(ctx, ctx.limits.delay_ns + 200_000_000)
        self.assertGreater(later.pose[0], -3)
        self.assertGreater(later.pose[2], 0)
        stamp = replace(stamp, time_ns=CONTROL_NS)
        guard = Clearance(replace(ctx, stamp=stamp))
        guard.stopping(Command(stamp))
        self.assertGreater(max(p[0] for p in guard.path), -3)
        unsafe = replace(
            ctx,
            estimate=replace(est, pose=(self.g.tip_x + 0.001, 0.09, 0.0)),
            active=forward,
            feedback=replace(self.fb, axle_speed=0.3),
        )
        decision = Supervisor().decide(unsafe, MotionProposal(forward, 100_000_000))
        self.assertEqual(decision.reason, "clearance")
        self.assertEqual(decision.command.speed, 0)
        self.assertLess(decision.clearance, 0)

    def test_registration(self):
        self.assertEqual(load("geometric").period_ns, 20_000_000)
        for name in ("search_ilqr", "rl"):
            with self.assertRaisesRegex(ValueError, "implementation pending"):
                load(name)
