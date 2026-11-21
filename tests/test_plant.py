"""Scripted plant checks use privileged starts. They do not run an autonomy method."""

import unittest

import numpy as np

from pallet_engagement.config import STEP_NS, Config, Scenario
from pallet_engagement.evaluation import Evaluator
from pallet_engagement.messages import Command
from pallet_engagement.plant import Plant


class PlantChecks(unittest.TestCase):
    def test_delayed_turn_and_brake(self):
        cfg = Config(scenario=Scenario(distance=4))
        p = Plant(cfg, "limits")
        e = Evaluator(p, cfg.criteria)
        p.enqueue(Command(p.stamp(), cfg.limits.speed, -0.32))
        for _ in range(cfg.limits.delay_ns // STEP_NS):
            self.assertIsNone(p.step())
            e.update()
            self.assertEqual(p.target_v, 0)
        self.assertIsNotNone(p.step())
        e.update()
        for _ in range(1400):
            p.step()
            e.update()
        start = p.poses()[0]
        last = start
        distance = 0.0
        p.enqueue(Command(p.stamp(), 0.44, -0.2))
        p.enqueue(Command(p.stamp()))
        # Both commands mature in order; the target remains live until then.
        for _ in range(75):
            self.assertIsNone(p.step())
            e.update()
            pos = p.poses()[0]
            distance += np.linalg.norm(np.subtract(pos[:2], last[:2]))
            last = pos
        self.assertEqual(p.step().speed, 0)
        e.update()
        for _ in range(1200):
            p.step()
            e.update()
            pos = p.poses()[0]
            distance += np.linalg.norm(np.subtract(pos[:2], last[:2]))
            last = pos
        bound = (
            cfg.limits.speed * 0.15
            + cfg.limits.speed**2 / (2 * cfg.limits.brake)
            + cfg.limits.stop_buffer
        )
        print(
            f"brake path={distance:.4f} bound={bound:.4f} peak true speed={e.max_speed:.4f} peak wheel rate={np.degrees(e.max_steer_rate):.3f} deg/s"
        )
        self.assertLess(distance, bound)
        self.assertLess(e.max_speed, cfg.limits.speed)
        self.assertLess(e.max_steer_rate, cfg.limits.steer_rate)
        self.assertTrue(e.metrics()["settled"])

    def test_positive_separation(self):
        cfg = Config(scenario=Scenario(0.01, 0.028))
        plant = Plant(cfg, "proximity")
        # Ask MuJoCo for nearby, inactive contact records.
        plant.m.geom("left_fork").margin[0] = 0.01
        plant.m.geom("left_fork").gap[0] = 0.01
        evaluator = Evaluator(plant, cfg.criteria)
        plant.enqueue(Command(plant.stamp(), 0.1))
        for _ in range(1000):
            plant.step()
            evaluator.update()
            if evaluator.target_pairs:
                break
        self.assertGreater(evaluator.proximity, 0)
        self.assertIsNone(evaluator.first)

    def test_insertion_and_contact(self):
        for lateral, yaw, success in [
            (0.0, 0.0, True),
            (0.02, 0.0, True),
            (-0.02, 0.0, True),
            (0.025, 0.0, False),
            (-0.025, 0.0, False),
            (0.0, 0.075, False),
        ]:
            cfg = Config(scenario=Scenario(0.08, lateral, yaw, 0.47))
            p = Plant(cfg, "physical")
            e = Evaluator(p, cfg.criteria)
            self.assertAlmostEqual(
                e.metrics()["depth"],
                -0.08 - abs(np.sin(yaw)) * (p.geometry.tip_y + p.geometry.half_width),
                places=5,
            )
            p.enqueue(Command(p.stamp(), 0.18))
            braking = False
            for _ in range(3200):
                p.step()
                e.update()
                if (
                    p.tick % 10 == 0
                    and not braking
                    and (e.metrics()["depth"] > 0.60 or e.first)
                ):
                    p.enqueue(Command(p.stamp()))
                    braking = True
                if braking and e.metrics()["settled"]:
                    break
            ok, reason, metrics = e.score("assess_completion")
            print(
                f"lateral={lateral:.3f} heading={yaw:.3f} result={reason} depth={metrics['depth']:.3f} first_depth={e.first['depth'] if e.first else None}"
            )
            self.assertEqual(ok, success)
            if not success:
                self.assertEqual(reason, "contact")
                first = e.first.copy()
                p.enqueue(Command(p.stamp(), -0.1))
                for _ in range(1000):
                    p.step()
                    e.update()
                self.assertEqual(e.first, first)
                self.assertEqual(e.score("assess_completion")[1], "contact")
