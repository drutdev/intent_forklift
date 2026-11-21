# Recorded engagement attempts

These four animations replay saved MuJoCo joint states at 2x speed. Each video includes a perspective view, a pocket close-up, simulation time, and measured pose errors. Exporting them does not rerun the controller or simulation.

The recordings use the CC controller with a fixed 2.5 m recovery withdrawal and the widened pallet fixture. All four episodes use nominal sensing, a 150 ms actuation delay, and a 90 s simulation budget. None makes robot-pallet contact.

## Failed attempts

| Animation | Terminal reason | Simulation time | Depth | Lateral error | Yaw error |
| --- | --- | ---: | ---: | ---: | ---: |
| [Seed 10](failed/seed-10.mp4) | `terminal_pose_error` | 33.40 s | 610.7 mm | +27.78 mm | +2.094° |
| [Seed 14](failed/seed-14.mp4) | `episode_timeout` | 90.00 s | 600.6 mm | -44.95 mm | -3.701° |

Seed 10 exceeds the 2° yaw threshold. Seed 14 exceeds both the 30 mm lateral threshold and the 2° yaw threshold.

Seed 14 has an incomplete recording. Its animation marks gaps and holds the last saved pose during them. Motion within those gaps is unknown. Its last saved pose is at 89.88 s. The reported terminal outcome is measured at 90.00 s.

## Successful attempts

| Animation | Motion | Simulation time | Depth | Lateral error | Yaw error |
| --- | --- | ---: | ---: | ---: | ---: |
| [Seed 7](succeeded/seed-7.mp4) | Direct engagement, no recovery retry | 28.48 s | 624.8 mm | -1.65 mm | -0.055° |
| [Seed 8](succeeded/seed-8.mp4) | Engagement after one recovery retry | 48.02 s | 628.9 mm | -1.72 mm | +0.158° |

Both successes have complete recordings. Both finish settled, with the forks inside the pockets, insertion depth at least 600 mm, and lateral and yaw errors below the acceptance thresholds.

## Recorded evidence and export

Each video has a matching `seed-N.json` file with the original configuration, physical outcome, recording coverage, video checksum, and source hashes. The original local archives are under `recordings/robustness-20261122/random-NN`. Raw episode archives remain outside Git.

The export command requires those local archives and the repository's uv environment. It refuses to overwrite existing videos. To reproduce the four animations in a new directory, run this command from the repository root:

```sh
MUJOCO_GL=egl OPENBLAS_NUM_THREADS=1 uv run --locked --group analysis python tools/render_attempt_animations.py recordings/robustness-20261122 --seeds 7 8 10 14 --output recordings/attempt-animation-export
```

The renderer uses the DejaVu Sans font installed at `/usr/share/fonts/truetype/dejavu/DejaVuSans.ttf` on Linux. It also exports start, middle, and final preview images.
