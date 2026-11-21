# Forklift pallet engagement

This repository runs a geometric pallet-engagement stack for a rear-steered forklift and one stationary rackable pallet. A restricted continuous-curvature CSC or SC approach joins straight insertion with a continuous speed profile. Turns use clothoids and optional circular arcs. Pure pursuit tracks the path. The runtime supplies timestamped sensing, a relative-pose EKF, delayed actuation, uncertainty-aware supervision, physical scoring, and recording.

`geometric` is the default for both `run` and `evaluate`. The stationary `hold` method remains available for runtime diagnostics.

## Pallet clearance

The rackable pallet is scaled by 1.12 across its width in both the visible mesh and all collision pieces. Its width is approximately 1.142 m. With the fixed fork spacing, the calibrated outer-side clearance is 66.5 mm. The previous unscaled asset provided 22.6 mm.

This modified simulation fixture accommodates a combined 30 mm entrance offset and 2° yaw through at least 600 mm of insertion. Physical regression checks cover all four sign combinations. The evaluator still requires final lateral error below 30 mm, final yaw error below 2°, and zero contact. A constant-yaw drive can fit physically while exceeding the final lateral-error criterion.

Earlier recordings and results describe their recorded geometry. Check each manifest before comparing outcomes across this asset change.

## Run an engagement

Install [uv](https://docs.astral.sh/uv/getting-started/installation/) once. After cloning, run this command from the repository root:

```sh
uv run --locked python -m pallet_engagement run --seed 8 --duration 90
```

uv downloads Python 3.12.3 if needed, creates `.venv`, and installs the dependencies from `uv.lock`. No environment activation is required. The first run needs internet access. The command runs a centered case with nominal sensing and prints a JSON outcome. It runs without a graphical display.

To save telemetry and joint states, use a new recording directory:

```sh
uv run --locked python -m pallet_engagement run --seed 8 --duration 90 --record recordings/my-geometric-run
```

Native viewers require a graphical display and working graphics drivers. Video export also requires an OpenGL backend. The commands below use EGL for headless video on Linux.

`pyproject.toml` and `uv.lock` define the uv environment. `requirements.txt` remains available for pip users.

The CC connector supports forward LSL, LSR, RSL, RSR, SL, and SR variants,
plus stopped, supervisor-authorized 2.5 m straight withdrawal at a 0.4 m/s cruise cap, followed by those same
forward families. It does not implement the full Reeds–Shepp set. The
[geometric design](DESIGN_GEOMETRIC.md) defines the small-turn construction,
finite search, actuator reserves, and measured limitations. Historical quintic
results are labeled separately there.

The final matched 90 s study scores 16/18 successes for CC and 16/18 for the
quintic reference with the same fixed recovery policy, with zero CC contact.
The six randomized cases remain 5/6 for CC versus 6/6 for that reference.
Realtime engagement is not validated. See the
[measured outcomes and limitations](DESIGN_GEOMETRIC.md#cc-runtime-validation).

The JSON outcome reports physical success, contact, terminal errors, timing, and recording completeness. Deterministic mode pauses simulation during computation. Realtime mode paces physics against a monotonic clock and runs motion generation in a worker. A late or expired proposal triggers a checked braking continuation. Realtime performance requires separate measurement.

## Vary the start and sensing

`--pose` supplies fork-tip distance, lateral offset, heading error, and pallet yaw. Distances are in metres and angles are in radians. `--ideal` disables detector noise, random dropouts, and sensor drift. Physical dynamics, actuation delay, slew limits, and scoring remain active.

```sh
uv run --locked python -m pallet_engagement run --ideal
uv run --locked python -m pallet_engagement run --ideal --pose 3.2 .08 .04 .47
uv run --locked python -m pallet_engagement run --ideal --pose 3.2 -.08 -.04 -.47
uv run --locked python -m pallet_engagement run --ideal --pose .55 .06 0 .47
uv run --locked python -m pallet_engagement run --seed 8 --delay-uncertainty-ms 40
uv run --locked python -m pallet_engagement run --seed 8 --outage-index 70 --outage-count 24
```

The close start exercises authorized withdrawal and reacquisition. Recovery still needs a checked retreat and forward continuation. Acquisition, clearance, freshness, proposal expiry, and retry limits remain active.

## Inspect a recording

```sh
uv run --locked python tools/geometric_evidence.py recordings/my-geometric-run
uv run --locked python -m pallet_engagement replay recordings/my-geometric-run
uv run --locked rerun rrd verify recordings/my-geometric-run/trace.rrd
uv run --locked rerun recordings/my-geometric-run/trace.rrd
uv run --locked python -m pallet_engagement playback recordings/my-geometric-run
MUJOCO_GL=egl uv run --locked python -m pallet_engagement playback recordings/my-geometric-run --video recordings/my-geometric-run/scene.mp4
```

The evidence command reconstructs pallet-plane crossings independently and pairs them with measured axle speed. Replay compares every archived estimate and covariance. Playback uses copied joint poses at 25 Hz and the terminal pose. On macOS, use MuJoCo's `mjpython` launcher for native playback.

Rerun shows the remaining joined path in green, the tracking prediction in orange, and the stopping path in red. The 3D view includes articulated meshes, pocket detections at capture time, and the estimated entry face. The 3D model uses evaluator truth for visualization. The controller consumes estimated observations through its declared messages.

Add `--live` to a recorded run to open Rerun and stream telemetry during execution.
The launcher finds Rerun in the Python environment used to run the stack, so
activating the virtual environment is optional. Use a new recording directory.

```sh
env -u WAYLAND_DISPLAY XDG_SESSION_TYPE=x11 uv run --locked python -m pallet_engagement run --seed 8 --duration 90 --record recordings/my-live-run --live
```

On this Linux workstation, the Rerun viewer can use the X11 backend with an explicit window size.

```sh
env -u WAYLAND_DISPLAY XDG_SESSION_TYPE=x11 uv run --locked rerun --bind 127.0.0.1 --window-size 1280x800 recordings/my-geometric-run/trace.rrd
```

## Evaluate repeated runs

```sh
uv run --locked python -m pallet_engagement evaluate --suite comparison --seeds 0:6 --duration 90
uv run --locked python -m pallet_engagement evaluate --suite comparison --seeds 0:3 --outages 0 5 --paired-speed-cap
uv run --locked python -m pallet_engagement run --mode realtime --seed 8
```

For complete local CC traces, estimator replay, and the targeted regression
suite, run the following command with a new output directory. The suite uses
90 s per episode, including acquisition, recovery, safety holds, and settling. The regular runtime default
remains 60 s.

```sh
OPENBLAS_NUM_THREADS=1 uv run --locked python tools/cc_evidence.py recordings/my-cc-suite
uv run --locked --group analysis python tools/cc_report.py recordings/my-cc-suite
# Matched historical baseline with the same 90 s budget and seeds.
git show 0839e89:pallet_engagement/geometric.py > recordings/quintic-reference.py
OPENBLAS_NUM_THREADS=1 uv run --locked python tools/cc_evidence.py recordings/my-quintic-suite recordings/quintic-reference.py --match-recovery
uv run --locked --group analysis python tools/cc_report.py recordings/my-cc-suite --baseline recordings/my-quintic-suite
```

The seed range has an exclusive stop. Evaluation reports counts, Wilson 95% intervals, terminal reasons, and individual outcomes. The optional paired run applies a speed cap as uncertainty consumes clearance. Compare success, contact, completion time, and interventions on matched scenarios.

## Run diagnostics and checks

```sh
uv run --locked python -m pallet_engagement run --method hold --duration 3 --record recordings/hold-check
uv run --locked python probe.py
uv run --locked python -m unittest discover -s tests -v
```

The hold diagnostic should end with `episode_timeout`, `settled`, and `recording_complete`. The original probe checks steering direction, centered insertion to 600 mm, and contact at a deliberately misaligned 80 mm offset. The tests cover geometry, sensing, estimation, supervision, tracking, recovery, recording, and visualization.

Open the bundled scene on a machine with a graphical display.

```sh
uv run --locked python -m mujoco.viewer --mjcf=forklift_c.xml
```

## Design and recovery

- [Pipeline explanation for a beginner control engineer](PIPELINE.md)
- [Geometric planning and pure-pursuit control](DESIGN_GEOMETRIC.md)
- [Runtime, messages, sensing, safety, and evaluation](DESIGN_SCAFFOLDING.md)
- [First week on the real forklift](REAL_FORKLIFT_FIRST_WEEK.md)
- [Recover the shelved methods and original working state](RECOVERY.md)

The inactive methods, their tests and design documents, conversion scripts, and unused legacy assets are preserved in local Git stashes. An archive branch retains the previous commit. Existing ignored recordings and the virtual environment remain untouched.

## Asset credits

NVIDIA's Isaac Sim 6.2 ForkliftC supplies the forklift geometry. The MuJoCo adaptation uses simplified materials, estimated inertias, and rear Ackermann steering commands. [ForkliftC source description](https://docs.isaacsim.omniverse.nvidia.com/6.2.0/controllers/tutorial_ackermann.html) · [NVIDIA asset terms](https://docs.isaacsim.omniverse.nvidia.com/6.2.0/common/license-isaac-sim-additional.html)

NVIDIA's SimReady Warehouse 01 supplies the black rackable pallet. The MuJoCo adaptation uses a flat material and convex collision pieces. [Original pallet](https://huggingface.co/datasets/nvidia/PhysicalAI-SimReady-Warehouse-01/tree/c7fe115cb79c7ddbd0532630d7768b5736b0ecc4/Props/general/SM_RackablePallet_A01_BlackA_01) · [CC BY 4.0](https://creativecommons.org/licenses/by/4.0/)
