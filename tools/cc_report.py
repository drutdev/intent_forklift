"""Plot measured axle and derived tip paths, and compare matched CC outcomes."""
import argparse
import json
from pathlib import Path

import matplotlib
matplotlib.use('Agg')
import matplotlib.pyplot as plt
import numpy as np

from geometric_evidence import measure


def trace(directory):
    geometry = json.loads((directory / 'manifest.json').read_text())['geometry']
    rows, crossings, plans = [], [], []
    speed = 0.
    with (directory / 'events.jsonl').open() as file:
        for line in file:
            row = json.loads(line)
            data = row['data']
            t = row['time_ns'] * 1e-9
            if row['kind'] == 'feedback':
                speed = data['axle_speed']
            elif row['kind'] == 'evaluator_truth':
                x, y, yaw = data['pose']
                a = -yaw
                axle = np.array([-np.cos(yaw)*x-np.sin(yaw)*y,
                                 np.sin(yaw)*x-np.cos(yaw)*y])
                tip = axle + geometry['tip_x'] * np.array([np.cos(a), np.sin(a)])
                rows.append((t, *axle, *tip, speed))
            elif row['kind'] == 'proposal':
                diag = dict(data['diagnostics'])
                if 'cc_family' in diag and data.get('guide'):
                    plans.append(np.array(data['guide']['samples']))
                for name in ('join', 'face'):
                    if name + '_time_s' in diag:
                        crossings.append(dict(event=name, time_s=t,
                            measured_mps=diag[name+'_measured_mps'],
                            planned_mps=diag[name+'_planned_mps']))
    return np.array(rows), crossings, plans, geometry


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('directory', type=Path)
    parser.add_argument('--baseline', type=Path)
    args = parser.parse_args()
    results = json.loads((args.directory / 'results.json').read_text())
    baseline = {r['case']: r for r in json.loads((args.baseline / 'results.json').read_text())} if args.baseline else {}
    report = []
    names = [r['case'] for r in results if r['case'] in ('case2-ideal', 'case2-nominal', 'case6-ideal', 'random-3', 'stale')]
    if not names:
        names = [r['case'] for r in results[:5]]
    fig, axes = plt.subplots(len(names), 2, figsize=(12, 3.2*len(names)), squeeze=False, layout='constrained')
    for i, name in enumerate(names):
        rows, crossings, plans, geometry = trace(args.directory / name)
        ax, speed = axes[i]
        for j, plan in enumerate(plans):
            ax.plot(plan[:, 0], plan[:, 1], color='green', alpha=.35,
                    label='Checked axle guides' if j == 0 else None)
        ax.plot(rows[:, 1], rows[:, 2], label='Measured front axle')
        ax.plot(rows[:, 3], rows[:, 4], '--', label='Derived fork-tip midpoint')
        ax.axvline(0, color='black', label='Pallet face')
        ax.axvspan(0, geometry['length'], color='gray', alpha=.12)
        ax.set(title=name, xlabel='Pallet x (m)', ylabel='Pallet y (m)')
        ax.set_aspect('equal', adjustable='datalim')
        ax.legend(fontsize=7)
        speed.plot(rows[:, 0], rows[:, 5])
        for event in crossings:
            speed.axvline(event['time_s'], color='green' if event['event']=='join' else 'orange',
                          linestyle=':', label=event['event'] + ' (estimate)')
        speed.set(xlabel='Simulation time (s)', ylabel='Measured axle speed (m/s)')
        if crossings:
            speed.legend(fontsize=7)
    fig.savefig(args.directory / 'paths.png', dpi=150)
    for outcome in results:
        name = outcome['case']
        _, crossings, _, _ = trace(args.directory / name)
        old = baseline.get(name)
        physical, _, _ = measure(args.directory / name, None)
        report.append(dict(case=name, reason=outcome['reason'], time_s=outcome['time_ns']*1e-9,
            within_60s=outcome['success'] and outcome['time_ns']<=60_000_000_000,
            depth_m=outcome['metrics']['depth'], retries=outcome['retries'],
            contact=outcome['first_contact'] is not None,
            crossings_estimate=crossings,
            crossings_truth={k: v for k, v in physical['crossings'].items() if k in ('join', 'face')},
            method_timing=outcome['timing']['method'],
            baseline_reason=old['reason'] if old else None,
            baseline_time_s=old['time_ns']*1e-9 if old else None))
    (args.directory / 'measurements.json').write_text(json.dumps(report, indent=2))
    print(json.dumps(report, indent=2))


if __name__ == '__main__':
    main()
