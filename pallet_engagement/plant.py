"""Simulation-only geometry calibration, actuator adapter, and feedback."""

from collections import deque
from itertools import product
from math import atan2, cos, sin
from pathlib import Path

import mujoco
import numpy as np

from .config import STEP_NS
from .frames import ackermann, compose, inverse, rotation, transform
from .messages import Command, Geometry, Stamp, VehicleFeedback

MODEL = Path(__file__).resolve().parent.parent / "forklift_c.xml"
SIDES = ("left", "right")
BOX_EDGES = [(i, i ^ bit) for i in range(8) for bit in (1, 2, 4) if i < (i ^ bit)]
SIGNS = np.array(list(product((-1, 1), repeat=3)))


def mesh_points(model, data, name):
    geom = model.geom(name)
    mesh = model.mesh(int(geom.dataid[0]))
    start = int(mesh.vertadr[0])
    verts = model.mesh_vert[start : start + int(mesh.vertnum[0])]
    return verts @ data.geom(name).xmat.reshape(3, 3).T + data.geom(name).xpos


def calibrate(m, d):
    front = np.array([d.xanchor[m.joint("front_" + s).id] for s in SIDES])
    rear = np.array([d.xanchor[m.joint("steer_" + s).id] for s in SIDES])
    axle = front.mean(axis=0)
    fork = m.geom("left_fork")
    top = d.geom(fork.name).xpos[2] + fork.size[2]
    # Intersect the free intervals of every row that overlaps blade height.
    center, outer, floor, roof = [], [], [], []
    for row in range(3):
        for layer in range(4):
            pts = mesh_points(m, d, f"pallet_{layer}_{row}_1")
            if layer == 0:
                floor.append(pts[:, 2].max())
            elif layer == 3:
                roof.append(pts[:, 2].min())
            elif pts[:, 2].min() <= top:
                center.append(pts[:, 1].max())
                outer.append(mesh_points(m, d, f"pallet_{layer}_{row}_2")[:, 1].min())
    pieces = []
    for i in range(m.ngeom):
        g = m.geom(i)
        if not g.contype[0] or m.body_rootid[int(g.bodyid[0])] != m.body("chassis").id:
            continue
        # Conservative horizontal bounding rectangles for every collision piece.
        box = m.geom_aabb[i]
        corners = (box[:3] + SIGNS * box[3:]) @ d.geom_xmat[i].reshape(
            3, 3
        ).T + d.geom_xpos[i]
        lo, hi = corners.min(axis=0) - axle, corners.max(axis=0) - axle
        pieces.append(
            (lo[0], hi[0], lo[1], hi[1], g.name in ("left_fork", "right_fork"))
        )
    tip = d.geom(fork.name).xpos + [fork.size[0], 0, 0] - axle
    face = d.site("pallet_face").xpos
    return Geometry(
        front[0, 0] - rear[0, 0],
        tuple(rear[:, 1]),
        tuple(front[:, 1]),
        front[0, 2] - d.body("chassis").xpos[2],
        tip[0],
        tip[1],
        fork.size[1],
        (max(center), min(outer)),
        (max(floor), min(roof)),
        2 * (d.body("pallet").xpos[0] - face[0]),
        mesh_points(m, d, "pallet_3_0_2")[:, 1].max(),
        tuple(pieces),
    )


class Plant:
    def __init__(self, cfg, episode):
        self.m = mujoco.MjModel.from_xml_path(str(MODEL))
        self.d = mujoco.MjData(self.m)
        m, d = self.m, self.d
        mujoco.mj_forward(m, d)
        self.geometry = calibrate(m, d)
        self.axle_offset = (
            d.xanchor[m.joint("front_left").id] + d.xanchor[m.joint("front_right").id]
        ) / 2 - d.body("chassis").xpos
        self.front_x = self.axle_offset[0]
        self.limits = cfg.limits
        for side in SIDES:
            drive = m.actuator("drive_" + side)
            drive.gainprm[0] = self.limits.drive_gain
            drive.biasprm[2] = -self.limits.drive_gain
            m.actuator("turn_" + side).biasprm[2] = -self.limits.steering_damping
        self.episode = episode
        self.tick = 0
        self.queue = deque()
        self.active = Command(self.stamp())
        self.target_v = 0.0
        self.target_angles = np.zeros(2)
        uncertainty = self.limits.delay_uncertainty_ns // STEP_NS
        offset = np.random.default_rng([cfg.seed, 5]).integers(
            -uncertainty, uncertainty + 1
        )
        self.delay = self.limits.delay_ns + int(offset) * STEP_NS
        s = cfg.scenario
        # Keep the entry face fixed and rotate the pallet about it.
        face = d.site("pallet_face").xpos[:2].copy()
        pallet = m.body("pallet")
        pallet.pos[:2] = face - rotation(s.pallet_yaw) @ m.site("pallet_face").pos[:2]
        pallet.quat[:] = [cos(s.pallet_yaw / 2), 0, 0, sin(s.pallet_yaw / 2)]
        tip = transform((*face, s.pallet_yaw), (-s.distance, s.lateral))
        yaw = s.pallet_yaw + s.heading
        d.qpos[:2] = tip - rotation(yaw) @ (self.front_x + self.geometry.tip_x, 0)
        d.qpos[3:7] = [cos(yaw / 2), 0, 0, sin(yaw / 2)]
        mujoco.mj_forward(m, d)

    def stamp(self):
        return Stamp(self.episode, self.tick, self.tick * STEP_NS, "front_axle")

    def poses(self):
        chassis = self.d.body("chassis")
        rot = chassis.xmat.reshape(3, 3)
        axle = chassis.xpos + rot @ self.axle_offset
        face = self.d.site("pallet_face")
        mat = face.xmat.reshape(3, 3)
        return (
            (*axle[:2], atan2(rot[1, 0], rot[0, 0])),
            (*face.xpos[:2], atan2(mat[1, 0], mat[0, 0])),
        )

    def relative(self):
        body, pallet = self.poses()
        return compose(inverse(body), pallet)

    def feedback(self):
        drive = [self.d.joint("front_" + s) for s in SIDES]
        enc = tuple(float(j.qpos[0]) for j in drive)
        omega = tuple(float(j.qvel[0]) for j in drive)
        angles = tuple(float(self.d.joint("steer_" + s).qpos[0]) for s in SIDES)
        v = sum(omega) * self.geometry.radius / 2
        yaw_rate = (
            (omega[1] - omega[0])
            * self.geometry.radius
            / (2 * self.geometry.front_y[0])
        )
        rates = tuple(float(self.d.joint("steer_" + side).qvel[0]) for side in SIDES)
        return VehicleFeedback(self.stamp(), enc, omega, angles, v, yaw_rate, rates)

    def enqueue(self, command):
        self.queue.append(command)

    def step(self):
        now = self.tick * STEP_NS
        applied = None
        while self.queue and self.queue[0].stamp.time_ns + self.delay <= now:
            self.active = applied = self.queue.popleft()
        dt = STEP_NS * 1e-9
        lim = self.limits
        cap = lim.speed - lim.drive_headroom
        desired_v = np.clip(self.active.speed, -cap, cap)
        self.target_v += np.clip(
            desired_v - self.target_v, -lim.target_accel * dt, lim.target_accel * dt
        )
        angles, speeds = ackermann(self.geometry, self.target_v, self.active.curvature)
        desired = np.clip(angles, -lim.steer, lim.steer)
        self.target_angles += np.clip(
            desired - self.target_angles,
            -lim.target_steer_rate * dt,
            lim.target_steer_rate * dt,
        )
        for i, s in enumerate(SIDES):
            self.d.actuator("turn_" + s).ctrl[0] = self.target_angles[i]
            self.d.actuator("drive_" + s).ctrl[0] = speeds[i]
        mujoco.mj_step(self.m, self.d)
        mujoco.mj_forward(self.m, self.d)
        self.tick += 1
        return applied
