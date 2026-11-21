"""Run open-loop infrastructure checks. Units are meters, seconds and radians."""
from math import atan2, radians, tan
from pathlib import Path

import mujoco
import numpy as np


def run(case, lateral_offset=0.0):
    model = mujoco.MjModel.from_xml_path(str(Path(__file__).with_name("forklift_c.xml")))
    data = mujoco.MjData(model)
    arc = case == "arc"
    data.qpos[1] = 2.0 if arc else lateral_offset
    speed = 0.25
    target = radians(18) if arc else 0.0
    # Positive rear steering produces clockwise yaw when driving forward.
    curvature = -tan(target) / 1.652
    steer_targets = {}
    for side, rear_y, front_y in (("left", 0.570, 0.522), ("right", -0.570, -0.522)):
        angle = atan2(tan(target), 1 - curvature * rear_y)
        steer_targets[f"steer_{side}"] = angle
        data.actuator(f"turn_{side}").ctrl[0] = angle
        data.actuator(f"drive_{side}").ctrl[0] = speed * (1 - curvature * front_y) / 0.325
    pallet = model.body("pallet").id
    forks = [model.geom(name) for name in ("left_fork", "right_fork")]
    mujoco.mj_forward(model, data)
    face = data.site("pallet_face").xpos[0]
    hits = set()
    while data.time < 8:
        mujoco.mj_step(model, data)
        mujoco.mj_forward(model, data)
        for contact in data.contact:
            a, b = (model.geom(int(i)) for i in contact.geom)
            if pallet in (a.bodyid[0], b.bodyid[0]):
                hits.add(tuple(sorted((a.name, b.name))))
        # Minimum insertion depth across the eight vertices of each tip face.
        corners = [
            data.geom_xpos[g.id] + data.geom_xmat[g.id].reshape(3, 3)
            @ (g.size * [1, y, z])
            for g in forks for y in (-1, 1) for z in (-1, 1)
        ]
        depth = min(point[0] for point in corners) - face
        if hits or (not arc and depth >= 0.60):
            break
    rotation = data.body("chassis").xmat.reshape(3, 3)
    yaw = atan2(rotation[1, 0], rotation[0, 0])
    if arc:
        print("Rear wheel angles (degrees):", {j: round(float(np.degrees(data.joint(j).qpos[0])), 2) for j in steer_targets})
        passed = (not hits and data.qpos[0] > 1 and yaw < -0.15 and
                  data.joint("steer_right").qpos[0] > data.joint("steer_left").qpos[0] and
                  all(abs(data.joint(j).qpos[0] - angle) < 0.05
                      for j, angle in steer_targets.items()))
        detail = f"travel x={data.qpos[0]:.3f} m, yaw={np.degrees(yaw):.1f} deg"
    elif case == "centered":
        passed = not hits and depth >= 0.60
        detail = f"minimum tip depth={depth:.3f} m, contacts={sorted(hits)}"
    else:
        expected_fork = "left_fork" if lateral_offset > 0 else "right_fork"
        passed = any(expected_fork in pair for pair in hits)
        detail = f"first contact at t={data.time:.3f} s, depth={depth:.3f} m, pairs={sorted(hits)}"
    print(f"{case}: {'PASS' if passed else 'FAIL'} | {detail}")
    return passed


if __name__ == "__main__":
    outcomes = [run("arc"), run("centered"), run("offset", lateral_offset=0.08)]
    raise SystemExit(0 if all(outcomes) else 1)
