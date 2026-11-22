import unittest
from dataclasses import replace

from pallet_engagement.config import Config
from pallet_engagement.messages import Command, ControlContext, Estimate
from pallet_engagement.plant import Plant
from pallet_engagement.prediction import advance_targets


class TargetChecks(unittest.TestCase):
    @classmethod
    def setUpClass(cls):
        cfg = Config()
        plant = Plant(cfg, "targets")
        stamp = replace(plant.stamp(), time_ns=200_000_000)
        cls.ctx = ControlContext(
            stamp,
            Estimate(stamp, plant.relative(), ((0.0, 0.0, 0.0),) * 3, 0, True),
            plant.feedback(),
            plant.geometry,
            cfg.limits,
            cfg.criteria,
            (),
            Command(stamp, 0.2),
            False,
            0,
            "pallet/front",
            "targets",
            nominal_targets=(0.1, (0.0, 0.0)),
        )

    def test_fractional_release_splits_the_control_interval(self):
        pending = Command(replace(self.ctx.stamp, time_ns=60_000_000))
        ctx = replace(self.ctx, pending=(pending,))
        speed, angles = advance_targets(ctx, Command(ctx.stamp))
        # Ten milliseconds accelerating, then ten braking: no net target change.
        self.assertAlmostEqual(speed, 0.1)
        self.assertEqual(angles, (0.0, 0.0))

    def test_release_at_interval_end_has_no_early_effect(self):
        pending = Command(replace(self.ctx.stamp, time_ns=70_000_000), -0.2)
        ctx = replace(self.ctx, pending=(pending,))
        speed, _ = advance_targets(ctx, Command(ctx.stamp))
        self.assertAlmostEqual(speed, 0.11)

    def test_zero_delay_applies_current_command_immediately(self):
        ctx = replace(self.ctx, limits=replace(self.ctx.limits, delay_ns=0))
        speed, _ = advance_targets(ctx, Command(ctx.stamp))
        self.assertAlmostEqual(speed, 0.09)

    def test_unreleased_request_cannot_change_current_targets(self):
        ctx = self.ctx
        self.assertEqual(
            advance_targets(ctx, Command(ctx.stamp, -0.4, -0.2)),
            advance_targets(ctx, Command(ctx.stamp, 0.4, 0.2)),
        )
        self.assertEqual(ctx.nominal_targets, (0.1, (0.0, 0.0)))
