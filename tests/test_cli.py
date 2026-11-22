import io
import json
import unittest
from contextlib import redirect_stderr, redirect_stdout
from unittest.mock import Mock, patch

from pallet_engagement.__main__ import main
from pallet_engagement.config import Sensors, scenario


class CliChecks(unittest.TestCase):
    def invoke(self, args):
        outcome = Mock()
        outcome.to_dict.return_value = {"success": True, "reason": "success"}
        output = io.StringIO()
        with (
            patch("sys.argv", ["pallet_engagement", *args]),
            patch("pallet_engagement.runtime.run", return_value=outcome) as run,
            redirect_stdout(output),
        ):
            main()
        return run.call_args_list, json.loads(output.getvalue())

    def test_ideal_run_preserves_outage_and_estimator_options(self):
        calls, result = self.invoke(
            [
                "run",
                "--method",
                "hold",
                "--ideal",
                "--pose",
                "3",
                ".1",
                ".2",
                ".47",
                "--seed",
                "11",
                "--outage-index",
                "9",
                "--outage-count",
                "3",
                "--moving-detection-gain",
                ".5",
                "--odometry-covariance-scale",
                "2",
                "--delay-uncertainty-ms",
                "40",
                "--acquire-timeout",
                "2.5",
            ]
        )
        self.assertEqual(len(calls), 1)
        cfg, directory, live = calls[0].args
        self.assertEqual(cfg.method, "hold")
        self.assertEqual(cfg.seed, 11)
        self.assertEqual(
            (
                cfg.scenario.distance,
                cfg.scenario.lateral,
                cfg.scenario.heading,
                cfg.scenario.pallet_yaw,
            ),
            (3, 0.1, 0.2, 0.47),
        )
        self.assertEqual(cfg.sensors.dropout, 0)
        self.assertEqual(cfg.sensors.noise_scale, 0)
        self.assertEqual((cfg.sensors.outage_index, cfg.sensors.outage_count), (9, 3))
        self.assertEqual(cfg.sensors.slip_std_per_sqrt_m, Sensors().slip_std_per_sqrt_m)
        self.assertEqual(cfg.moving_detection_gain, 0.5)
        self.assertEqual(cfg.odometry_covariance_scale, 2)
        self.assertEqual(cfg.limits.delay_uncertainty_ns, 40_000_000)
        self.assertEqual(cfg.criteria.acquire_ns, 2_500_000_000)
        self.assertIsNone(directory)
        self.assertFalse(live)
        self.assertTrue(result["success"])

    def test_paired_evaluation_preserves_seed_and_outage_combinations(self):
        calls, result = self.invoke(
            [
                "evaluate",
                "--methods",
                "hold",
                "--seeds",
                "1:3",
                "--outages",
                "0",
                "5",
                "--paired-speed-cap",
            ]
        )
        combinations = set()
        for call in calls:
            cfg = call.args[0]
            combinations.add(
                (cfg.seed, cfg.sensors.outage_count, cfg.limits.uncertainty_speed_cap)
            )
            self.assertEqual(cfg.scenario, scenario("comparison", cfg.seed))
        self.assertEqual(
            combinations,
            {
                (seed, outage, cap)
                for seed in (1, 2)
                for outage in (0, 5)
                for cap in (False, True)
            },
        )
        self.assertEqual(len(calls), 8)
        self.assertEqual(len(result), 4)
        self.assertTrue(all(group["total"] == group["wins"] == 2 for group in result))

    def test_invalid_acquisition_timeout_does_not_start_runtime(self):
        with (
            patch("sys.argv", ["pallet_engagement", "run", "--acquire-timeout", "0"]),
            patch("pallet_engagement.runtime.run") as run,
            redirect_stderr(io.StringIO()),
            self.assertRaises(SystemExit) as error,
        ):
            main()
        self.assertEqual(error.exception.code, 2)
        run.assert_not_called()
