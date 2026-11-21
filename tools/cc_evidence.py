"""Run matched CC runtime scenarios and archive complete local evidence."""
import argparse
import json
from dataclasses import replace
from pathlib import Path
import sys

sys.path.insert(0, str(Path(__file__).resolve().parents[1]))
from pallet_engagement.config import Config, Scenario, Sensors, scenario
from pallet_engagement.runtime import run
from pallet_engagement.recording import replay

parser = argparse.ArgumentParser(description=__doc__)
parser.add_argument('directory', type=Path, help='new output directory')
parser.add_argument('reference', type=Path, nargs='?', help='historical geometric.py for a matched baseline')
parser.add_argument('--match-recovery', action='store_true',
                    help='match fixed retreat, reverse speed, and stall allowance in a reference')
parser.add_argument('--jobs', type=int, default=3)
parser.add_argument('--duration', type=float, default=90)
parser.add_argument('--cases', nargs='+', help='optional subset of named cases')
args = parser.parse_args()
if args.jobs < 1:
    parser.error('--jobs must be positive')
ROOT, REFERENCE = args.directory, args.reference
if args.match_recovery and REFERENCE is None:
    parser.error('--match-recovery requires a reference')
if REFERENCE is not None:
    import importlib.util
    from pallet_engagement.methods import REGISTRY, Registration
    spec = importlib.util.spec_from_file_location('pallet_engagement._cc_reference', REFERENCE)
    module = importlib.util.module_from_spec(spec)
    sys.modules[spec.name] = module
    reference_source = REFERENCE.read_text()
    if args.match_recovery:
        reference_source = reference_source.replace(
            'for distance in (3.5,):', 'for distance in (2.5,):')
        for old, new in (
            ('for distance in (0.30, 0.65, 1.25, 2.0, 2.75):', 'for distance in (2.5,):'),
            ('                    0.09,', '                    ctx.limits.recovery_speed,'),
            ('t - self.last_motion > 2_500_000_000', 't - self.last_motion > 6_000_000_000'),
        ):
            if old not in reference_source and new not in reference_source:
                parser.error('reference does not contain the expected recovery setting')
            reference_source = reference_source.replace(old, new)
    exec(compile(reference_source, str(REFERENCE), 'exec'), module.__dict__)
    REGISTRY['geometric'] = Registration(spec.name, 'Method', True)
ideal = Sensors(dropout=0, noise_scale=0, scale_walk=0, heading_walk=0,
                encoder_noise=0, yaw_noise=0)
base = Config(duration=args.duration)
cases = []
for label, pose in [('center', Scenario()),
                    ('case2', Scenario(2, -.5, .3490658503988659, 0)),
                    ('case6', Scenario(2, .5, -.3490658503988659, 0)),
                    ('rotated', Scenario(2, -.5, .3490658503988659, .47)),
                    ('close', Scenario(.55, .06, 0, .47))]:
    cases.append((label + '-ideal', replace(base, scenario=pose, sensors=ideal)))
    cases.append((label + '-nominal', replace(base, scenario=pose, seed=8)))
for seed in range(6):
    cases.append((f'random-{seed}', replace(base, seed=seed, scenario=scenario('comparison', seed))))
cases.extend([
    ('delay', replace(base, seed=8, limits=replace(base.limits, delay_uncertainty_ns=40_000_000))),
    ('stale', replace(base, seed=8, sensors=replace(base.sensors, outage_index=70, outage_count=24))),
])
if args.cases:
    names = {name for name, _ in cases}
    if set(args.cases) - names:
        parser.error('unknown cases: ' + ', '.join(sorted(set(args.cases) - names)))
    cases = [(name, cfg) for name, cfg in cases if name in args.cases]


def execute(case):
    name, cfg = case
    directory = ROOT / name
    if (directory / 'outcome.json').exists():
        result = json.loads((directory / 'outcome.json').read_text())
    else:
        result = run(cfg, str(directory)).to_dict()
    result['case'] = name
    result['replayed_estimates'] = replay(directory)
    return result


if __name__ == '__main__':
    from concurrent.futures import ProcessPoolExecutor, as_completed
    import shutil
    ROOT.mkdir(parents=True, exist_ok=False)
    source = ROOT / 'source'
    if not source.exists():
        shutil.copytree('pallet_engagement', source / 'pallet_engagement',
                        ignore=shutil.ignore_patterns('__pycache__'))
        if REFERENCE is not None:
            (source / 'pallet_engagement' / 'geometric.py').write_text(reference_source)
    results = []
    with ProcessPoolExecutor(max_workers=args.jobs) as pool:
        futures = [pool.submit(execute, case) for case in cases]
        for future in as_completed(futures):
            result = future.result()
            results.append(result)
            (ROOT / 'results.json').write_text(json.dumps(results, indent=2))
            print(result['case'], result['reason'], round(result['time_ns'] * 1e-9, 2),
                  round(result['metrics']['depth'], 4), result['retries'], flush=True)
