"""Render archived engagement attempts at 2x speed without stepping physics."""
import argparse
import json
from math import degrees
from pathlib import Path
import sys

import imageio_ffmpeg
import mujoco
import numpy as np
from PIL import Image, ImageDraw, ImageFont

sys.path.insert(0, str(Path.cwd()))
from pallet_engagement.plant import Plant
from pallet_engagement.recording import configuration
from pallet_engagement.evaluation import Evaluator

FONT = '/usr/share/fonts/truetype/dejavu/DejaVuSans.ttf'
font = ImageFont.truetype(FONT, 20)
small = ImageFont.truetype(FONT, 17)


def render(seed, root, output_root):
    directory = root / f'random-{seed:02}'
    outcome = json.loads((directory / 'outcome.json').read_text())
    cfg = configuration(directory)
    plant = Plant(cfg, 'archived-animation')
    plant.m.vis.headlight.ambient[:] = .5
    plant.m.vis.headlight.diffuse[:] = .7
    evaluator = Evaluator(plant, cfg.criteria)
    with np.load(directory / 'states.npz') as data:
        times = data['time_ns'] * 1e-9
        poses = data['qpos'].copy()
    perspective, top = mujoco.MjvCamera(), mujoco.MjvCamera()
    perspective.distance = 6.3
    perspective.azimuth = degrees(cfg.scenario.pallet_yaw) + 115
    perspective.elevation = -30
    top.azimuth = degrees(cfg.scenario.pallet_yaw) + 180
    top.elevation = -89
    category = 'succeeded' if outcome['success'] else 'failed'
    output = output_root / category / f'seed-{seed}.mp4'
    output.parent.mkdir(parents=True, exist_ok=True)
    if output.exists():
        raise FileExistsError(output)
    writer = imageio_ffmpeg.write_frames(str(output), (1280, 608), fps=25,
        codec='libx264', quality=7, output_params=['-pix_fmt', 'yuv420p', '-loglevel', 'error'])
    writer.send(None)
    saved_preview = set()
    try:
        with mujoco.Renderer(plant.m, height=480, width=640) as renderer:
            for tick in range(int(np.ceil(times[-1] / .08)) + 51):
                t = min(tick * .08, times[-1])
                index = min(len(times) - 1, max(0, int(np.searchsorted(times, t + 1e-8, side='right') - 1)))
                plant.d.qpos[:] = poses[index]
                plant.d.time = float(times[index])
                mujoco.mj_forward(plant.m, plant.d)
                pallet = plant.d.body('pallet').xpos.copy()
                chassis = plant.d.body('chassis').xpos.copy()
                perspective.lookat[:] = (chassis + pallet) / 2
                perspective.distance = max(8.0, float(np.linalg.norm(chassis[:2] - pallet[:2])) * 1.5 + 5)
                top.lookat[:] = pallet
                top.distance = 3.6
                renderer.update_scene(plant.d, camera=perspective)
                left = renderer.render().copy()
                renderer.update_scene(plant.d, camera=top)
                right = renderer.render().copy()
                image = Image.new('RGB', (1280, 608), '#171d25')
                image.paste(Image.fromarray(left), (0, 0))
                image.paste(Image.fromarray(right), (640, 0))
                draw = ImageDraw.Draw(image)
                for x, label in [(12, 'Perspective'), (652, 'Pocket close-up')]:
                    draw.rectangle((x - 4, 8, x + 210, 38), fill='#171d25')
                    draw.text((x, 10), label, font=font, fill='white')
                m = evaluator.metrics()
                draw.text((14, 487), f'Seed {seed}  |  simulation {t:05.2f} s  |  playback 2x  |  saved poses only', font=font, fill='white')
                draw.text((14, 517), f"Depth {1000*m['depth']:.1f} mm    lateral {1000*m['lateral']:+.1f} mm    yaw {degrees(m['yaw']):+.2f} deg", font=font, fill='white')
                age = max(0, t - times[index])
                if tick * .08 > times[-1] + .01:
                    note = f"Outcome: {outcome['reason']} at {outcome['time_ns']*1e-9:.2f} s. Last saved pose: {times[-1]:.2f} s. No contact."
                elif age > .081:
                    note = f'RECORDING GAP: holding last saved pose ({age:.2f} s old); motion during gap is unknown.'
                elif not outcome['recording_complete']:
                    note = 'INCOMPLETE RECORDING: gaps hold the last saved pose. Limits: depth >=600 mm, |lateral| <30 mm, |yaw| <2 deg.'
                else:
                    note = 'Limits: depth >=600 mm, |lateral| <30 mm, |yaw| <2 deg. No contact during episode.'
                draw.text((14, 550), note, font=small, fill='#ffcf83')
                writer.send(np.asarray(image))
                for tag, target in [('start', 0), ('middle', times[-1] / 2), ('end', times[-1])]:
                    if tag not in saved_preview and t >= target:
                        image.save(output.with_name(f'seed-{seed}-{tag}.png'))
                        saved_preview.add(tag)
    finally:
        writer.close()
    print(output, flush=True)


if __name__ == '__main__':
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('recordings', type=Path)
    parser.add_argument('--output', type=Path, default=Path('docs/attempts'))
    parser.add_argument('--seeds', type=int, nargs='+', required=True)
    args = parser.parse_args()
    for seed in args.seeds:
        render(seed, args.recordings, args.output)
