"""Measure crossings from recorded truth and feedback. Optionally plot the traces."""

import argparse
import json
from math import cos, sin
from pathlib import Path

import numpy as np


def measure(directory, join_distance):
    manifest = json.loads((directory / "manifest.json").read_text())
    outcome = json.loads((directory / "outcome.json").read_text())
    if not outcome["recording_complete"]:
        raise ValueError(f"incomplete recording in {directory}")
    geometry = manifest["geometry"]
    samples = []
    active_join = 0.8 if join_distance is None else join_distance
    previous_axle = previous_face = None
    speed = 0.0
    crossing = {}
    errors, margins, entry = [], [], []
    with (directory / "events.jsonl").open() as rows:
        for line in rows:
            row = json.loads(line)
            data = row["data"]
            time = row["time_ns"] * 1e-9
            if row["kind"] == "feedback":
                speed = data["axle_speed"]
            elif row["kind"] == "evaluator_truth":
                x, y, yaw = data["pose"]
                axle_x = -cos(yaw) * x - sin(yaw) * y
                middle = axle_x + geometry["tip_x"] * cos(yaw)
                edge = (geometry["tip_y"] + geometry["half_width"]) * abs(sin(yaw))
                # Require approach through each plane. A close start can already
                # be beyond the join and move out again before a forward retry.
                values = (axle_x + geometry["tip_x"] + active_join, middle + edge)
                if previous_axle is not None:
                    previous = (previous_axle + geometry["tip_x"] + active_join, previous_face)
                    for i, name in enumerate(("join", "face")):
                        if previous[i] < 0 <= values[i] and speed > 0:
                            crossing.setdefault(name, []).append(
                                {"time_s": time, "feedback_mps": speed}
                            )
                samples.append((time, speed, middle - edge, *values))
                previous_axle, previous_face = axle_x, middle + edge
            elif row["kind"] == "decision":
                if data["clearance"] is not None:
                    margins.append(data["clearance"])
            elif row["kind"] == "proposal":
                diag = dict(data["diagnostics"])
                if join_distance is None and "join_distance_m" in diag:
                    active_join = diag["join_distance_m"]
                if "cross_track_m" in diag:
                    errors.append(diag["cross_track_m"])
                entry.append((time, diag.get("entry_margin_m", np.nan)))
                for name in ("join", "face"):
                    if name + "_time_s" in diag:
                        crossing.setdefault(name + "_estimate", []).append(
                            {
                                "time_s": time,
                                "planned_mps": diag[name + "_planned_mps"],
                                "feedback_mps": diag[name + "_measured_mps"],
                            }
                        )
    trace = np.array(samples)
    moving = np.flatnonzero(
        abs(trace[:, 1]) >= manifest["config"]["criteria"]["still_speed"]
    )
    standstill = (
        abs(trace[moving[0] : moving[-1] + 1, 1])
        < manifest["config"]["criteria"]["still_speed"]
        if len(moving)
        else []
    )
    report = {
        "reason": outcome["reason"],
        "depth_m": outcome["metrics"]["depth"],
        "retries": outcome["retries"],
        "crossings": crossing,
        "intermediate_still_ticks": int(np.sum(standstill)),
        "peak_cross_track_m": max(map(abs, errors), default=None),
        "min_stopping_margin_m": min(margins, default=None),
        "method_time_ns": outcome["timing"]["method"],
    }
    return report, trace, np.array(entry)


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("directories", nargs="+", type=Path)
    parser.add_argument("--join-distance", type=float, default=None)
    parser.add_argument("--plot", type=Path)
    args = parser.parse_args()
    results = {}
    if args.plot:
        import matplotlib.pyplot as plt

        fig, axes = plt.subplots(
            3, 1, figsize=(9, 8), sharex=True, layout="constrained"
        )
    for directory in args.directories:
        report, trace, entry = measure(directory, args.join_distance)
        results[directory.name] = report
        if args.plot:
            style = {"linestyle": ["-", "--", "-.", ":"][(len(results) - 1) % 4]}
            axes[0].plot(
                trace[:, 0],
                trace[:, 1],
                label=directory.name.removeprefix("geometric-"),
                **style,
            )
            axes[1].plot(trace[:, 0], trace[:, 2], **style)
            axes[1].scatter(
                trace[-1, 0],
                trace[-1, 2],
                s=25,
                color=axes[1].lines[-1].get_color(),
                zorder=3,
            )
            if len(entry):
                axes[2].plot(entry[:, 0], 1000 * entry[:, 1], **style)
    if args.plot:
        axes[0].set_ylabel("Measured axle speed (m/s)")
        axes[0].legend()
        axes[1].set_ylabel("Minimum planar tip depth (m)")
        axes[1].axhline(0.60, color="black", linestyle=":")
        axes[2].set_ylabel("Full-depth entry margin (mm)")
        axes[2].axhline(0, color="black", linestyle=":")
        axes[2].set_xlabel("Simulation time (s)")
        for ax in axes:
            ax.grid(alpha=0.2)
        fig.savefig(args.plot, dpi=150)
    print(json.dumps(results, indent=2))


if __name__ == "__main__":
    main()
