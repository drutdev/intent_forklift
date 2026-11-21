# Forklift pallet engagement

This repository runs a geometric pallet-engagement stack for a rear-steered forklift and one stationary rackable pallet. A quintic approach joins straight insertion with a continuous speed profile. Pure pursuit tracks the path. The runtime supplies timestamped sensing, a relative-pose EKF, delayed actuation, uncertainty-aware supervision, physical scoring, and recording.

`geometric` is the default for both `run` and `evaluate`. The stationary `hold` method remains available for runtime diagnostics.

## Run an engagement

From the repository root with Python 3.12 and venv support, create the environment and run a centered case with nominal sensing. Each recording directory must be new.

```sh
python3 -m venv .venv
.venv/bin/python -m pip install -r requirements.txt
.venv/bin/python -m pallet_engagement run --seed 8 --record recordings/my-geometric-run
```

The cleanup verification run for seed 8 completed in 23.62 simulated seconds. It reached a settled minimum fork-tip depth of 627.7 mm with both blades inside their pockets and zero pallet contact. Measured speed stayed positive through the approach join and pallet face. This single run does not establish reliability across the starting envelope. The [geometric design and measured results](DESIGN_GEOMETRIC.md) describe broader trials and failures.

The JSON outcome reports physical success, contact, terminal errors, timing, and recording completeness. Deterministic mode pauses simulation during computation. Realtime mode paces physics against a monotonic clock and runs motion generation in a worker. A late or expired proposal triggers a checked braking continuation. Realtime performance requires separate measurement.

## Vary the start and sensing

`--pose` supplies fork-tip distance, lateral offset, heading error, and pallet yaw. Distances are in metres and angles are in radians. `--ideal` disables detector noise, random dropouts, and sensor drift. Physical dynamics, actuation delay, slew limits, and scoring remain active.

```sh
.venv/bin/python -m pallet_engagement run --ideal
.venv/bin/python -m pallet_engagement run --ideal --pose 3.2 .08 .04 .47
.venv/bin/python -m pallet_engagement run --ideal --pose 3.2 -.08 -.04 -.47
.venv/bin/python -m pallet_engagement run --ideal --pose .55 .06 0 .47
.venv/bin/python -m pallet_engagement run --seed 8 --delay-uncertainty-ms 40
.venv/bin/python -m pallet_engagement run --seed 8 --outage-index 70 --outage-count 24
```

The close start exercises authorized withdrawal and reacquisition. Recovery still needs a checked retreat and forward continuation. Acquisition, clearance, freshness, proposal expiry, and retry limits remain active.

## Inspect a recording

```sh
.venv/bin/python tools/geometric_evidence.py recordings/my-geometric-run
.venv/bin/python -m pallet_engagement replay recordings/my-geometric-run
.venv/bin/rerun rrd verify recordings/my-geometric-run/trace.rrd
.venv/bin/rerun recordings/my-geometric-run/trace.rrd
.venv/bin/python -m pallet_engagement playback recordings/my-geometric-run
MUJOCO_GL=egl .venv/bin/python -m pallet_engagement playback recordings/my-geometric-run --video recordings/my-geometric-run/scene.mp4
```

The evidence command reconstructs pallet-plane crossings independently and pairs them with measured axle speed. Replay compares every archived estimate and covariance. Playback uses copied joint poses at 25 Hz and the terminal pose. On macOS, use MuJoCo's `mjpython` launcher for native playback.

Rerun shows the remaining joined path in green, the tracking prediction in orange, and the stopping path in red. The 3D view includes articulated meshes, pocket detections at capture time, and the estimated entry face. The 3D model uses evaluator truth for visualization. The controller consumes estimated observations through its declared messages.

Add `--live` to a recorded run to view telemetry during execution. On this Linux workstation, the Rerun viewer can use the X11 backend with an explicit window size.

```sh
env -u WAYLAND_DISPLAY XDG_SESSION_TYPE=x11 .venv/bin/rerun --bind 127.0.0.1 --window-size 1280x800 recordings/my-geometric-run/trace.rrd
```

## Evaluate repeated runs

```sh
.venv/bin/python -m pallet_engagement evaluate --suite comparison --seeds 0:6
.venv/bin/python -m pallet_engagement evaluate --suite comparison --seeds 0:3 --outages 0 5 --paired-speed-cap
.venv/bin/python -m pallet_engagement run --mode realtime --seed 8
```

The seed range has an exclusive stop. Evaluation reports counts, Wilson 95% intervals, terminal reasons, and individual outcomes. The optional paired run applies a speed cap as uncertainty consumes clearance. Compare success, contact, completion time, and interventions on matched scenarios.

## Run diagnostics and checks

```sh
.venv/bin/python -m pallet_engagement run --method hold --duration 3 --record recordings/hold-check
.venv/bin/python probe.py
.venv/bin/python -m unittest discover -s tests -v
```

The hold diagnostic should end with `episode_timeout`, `settled`, and `recording_complete`. The original probe checks steering direction, centered insertion to 600 mm, and contact at a deliberately misaligned 40 mm offset. The tests cover geometry, sensing, estimation, supervision, tracking, recovery, recording, and visualization.

Open the bundled scene on a machine with a graphical display.

```sh
.venv/bin/python -m mujoco.viewer --mjcf=forklift_c.xml
```

## Design and recovery

- [Geometric planning and pure-pursuit control](DESIGN_GEOMETRIC.md)
- [Runtime, messages, sensing, safety, and evaluation](DESIGN_SCAFFOLDING.md)
- [First week on the real forklift](REAL_FORKLIFT_FIRST_WEEK.md)
- [Recover the shelved methods and original working state](RECOVERY.md)

The inactive methods, their tests and design documents, conversion scripts, and unused legacy assets are preserved in local Git stashes. An archive branch retains the previous commit. Existing ignored recordings and the virtual environment remain untouched.

## Asset credits

NVIDIA's Isaac Sim 6.2 ForkliftC supplies the forklift geometry. The MuJoCo adaptation uses simplified materials, estimated inertias, and rear Ackermann steering commands. [ForkliftC source description](https://docs.isaacsim.omniverse.nvidia.com/6.2.0/controllers/tutorial_ackermann.html) · [NVIDIA asset terms](https://docs.isaacsim.omniverse.nvidia.com/6.2.0/common/license-isaac-sim-additional.html)

NVIDIA's SimReady Warehouse 01 supplies the black rackable pallet. The MuJoCo adaptation uses a flat material and convex collision pieces. [Original pallet](https://huggingface.co/datasets/nvidia/PhysicalAI-SimReady-Warehouse-01/tree/c7fe115cb79c7ddbd0532630d7768b5736b0ecc4/Props/general/SM_RackablePallet_A01_BlackA_01) · [CC BY 4.0](https://creativecommons.org/licenses/by/4.0/)
