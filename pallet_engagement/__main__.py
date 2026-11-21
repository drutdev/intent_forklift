import argparse
import json
from collections import Counter
from dataclasses import asdict
from math import sqrt
from pathlib import Path

from .config import Config, Criteria, Limits, Scenario, Sensors, scenario
from .methods import REGISTRY, load
from .recording import playback, replay


def wilson(wins, n):
    z = 1.96
    p = wins / n
    half = z * sqrt((p * (1 - p) + z * z / (4 * n)) / n)
    return [(p + z * z / (2 * n) + sign * half) / (1 + z * z / n) for sign in (-1, 1)]


def main():
    parser = argparse.ArgumentParser()
    sub = parser.add_subparsers(dest="action", required=True)
    one = sub.add_parser("run")
    batch = sub.add_parser("evaluate")
    for p in (one, batch):
        p.add_argument(
            "--mode", choices=["deterministic", "realtime"], default="deterministic"
        )
        p.add_argument("--duration", type=float, default=60)
        p.add_argument("--dropout", type=float, default=0.1)
        p.add_argument("--outage-count", type=int, default=0)
        p.add_argument(
            "--moving-detection-gain",
            type=float,
            default=1.0,
            help="fraction of EKF correction used during motion",
        )
        p.add_argument(
            "--odometry-covariance-scale",
            type=float,
            default=1.0,
            help="estimator multiplier for odometry covariance",
        )
        p.add_argument(
            "--delay-uncertainty-ms",
            type=int,
            default=0,
            help="symmetric episode delay variation in whole 2 ms steps",
        )
        p.add_argument("--outage-index", type=int, default=5)
        p.add_argument(
            "--ideal",
            action="store_true",
            help="disable sensing noise, drift and random dropouts",
        )
        p.add_argument("--acquire-timeout", type=float, default=3.0, metavar="SECONDS")
    one.add_argument(
        "--pose",
        nargs=4,
        type=float,
        metavar=("DISTANCE", "LATERAL", "HEADING", "PALLET_YAW"),
        help="pallet-frame start in metres and radians",
    )
    one.add_argument("--method", choices=REGISTRY, default="geometric")
    one.add_argument("--seed", type=int, default=7)
    one.add_argument("--scenario", choices=["nominal", "comparison"], default="nominal")
    one.add_argument("--record", nargs="?", const="")
    one.add_argument("--live", action="store_true")
    batch.add_argument("--methods", nargs="+", choices=REGISTRY, default=["geometric"])
    batch.add_argument(
        "--suite", choices=["comparison", "nominal"], default="comparison"
    )
    batch.add_argument("--seeds", default="0:50")
    batch.add_argument("--outages", type=int, nargs="+")
    batch.add_argument("--paired-speed-cap", action="store_true")
    for name in ("replay", "playback"):
        p = sub.add_parser(name)
        p.add_argument("directory", type=Path)
        if name == "playback":
            p.add_argument("--video", type=Path)
    args = parser.parse_args()
    try:
        if args.action == "replay":
            result = dict(matching_estimates=replay(args.directory))
        elif args.action == "playback":
            result = dict(frames=playback(args.directory, args.video))
        else:
            from .runtime import run

            def config(method, seed, name, outage, speed_cap=False):
                sensors = Sensors(
                    dropout=args.dropout,
                    outage_index=args.outage_index,
                    outage_count=outage,
                )
                if args.ideal:
                    sensors = Sensors(
                        dropout=0,
                        noise_scale=0,
                        outage_index=args.outage_index,
                        outage_count=outage,
                        scale_walk=0,
                        heading_walk=0,
                        encoder_noise=0,
                        yaw_noise=0,
                    )
                if args.acquire_timeout <= 0:
                    raise ValueError("--acquire-timeout must be positive")
                pose = getattr(args, "pose", None)
                return Config(
                    method,
                    args.mode,
                    seed,
                    args.duration,
                    Scenario(*pose) if pose else scenario(name, seed),
                    sensors,
                    Limits(
                        uncertainty_speed_cap=speed_cap,
                        delay_uncertainty_ns=args.delay_uncertainty_ms * 1_000_000,
                    ),
                    Criteria(acquire_ns=round(args.acquire_timeout * 1e9)),
                    moving_detection_gain=args.moving_detection_gain,
                    odometry_covariance_scale=args.odometry_covariance_scale,
                )

            if args.action == "run":
                load(args.method)
                if args.record == "":
                    args.record = f"recordings/{args.method}"
                if args.live and not args.record:
                    parser.error("--live requires --record")
                result = run(
                    config(args.method, args.seed, args.scenario, args.outage_count),
                    args.record,
                    args.live,
                ).to_dict()
            else:
                for method in args.methods:
                    load(method)
                a, b = map(int, args.seeds.split(":"))
                if b <= a or a < 0:
                    raise ValueError("expected nonnegative start:stop seed range")
                result = []
                for n in args.outages or [args.outage_count]:
                    for method in args.methods:
                        for cap in [False, True] if args.paired_speed_cap else [False]:
                            configs = [
                                config(method, i, args.suite, n, cap)
                                for i in range(a, b)
                            ]
                            runs = [
                                dict(
                                    **run(cfg).to_dict(),
                                    seed=cfg.seed,
                                    scenario=asdict(cfg.scenario),
                                )
                                for cfg in configs
                            ]
                            wins = sum(r["success"] for r in runs)
                            result.append(
                                dict(
                                    method=method,
                                    mode=args.mode,
                                    outage_count=n,
                                    uncertainty_speed_cap=cap,
                                    method_period_ns=load(method).period_ns,
                                    wins=wins,
                                    total=len(runs),
                                    rate=wins / len(runs),
                                    wilson_95=wilson(wins, len(runs)),
                                    runs=runs,
                                    failures=dict(
                                        Counter(
                                            r["reason"]
                                            for r in runs
                                            if not r["success"]
                                        )
                                    ),
                                )
                            )

    except (ValueError, FileExistsError) as exc:
        parser.error(str(exc))
    print(json.dumps(result, indent=2, allow_nan=False))


if __name__ == "__main__":
    main()
