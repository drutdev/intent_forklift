"""Privileged physical scorer. The controller never receives this object."""

from collections.abc import Mapping
from dataclasses import dataclass, fields
from types import MappingProxyType

import numpy as np

from .config import STEP_NS
from .frames import wrap
from .plant import BOX_EDGES, SIGNS


def copy_values(value, mapping_type):
    if isinstance(value, Mapping):
        return mapping_type({k: copy_values(v, mapping_type) for k, v in value.items()})
    if isinstance(value, (list, tuple)):
        return tuple(copy_values(v, mapping_type) for v in value)
    return value


@dataclass(frozen=True)
class EpisodeOutcome:
    episode: str
    time_ns: int
    success: bool
    reason: str
    first_contact: Mapping | None
    metrics: Mapping
    timing: Mapping
    recording_complete: bool | None
    overrides: Mapping
    retries: int
    execution_mode: str
    schema: int = 1
    frame: str = "pallet"

    def __post_init__(self):
        for name in ("metrics", "timing", "overrides", "first_contact"):
            object.__setattr__(
                self, name, copy_values(getattr(self, name), MappingProxyType)
            )

    def to_dict(self):
        return {f.name: copy_values(getattr(self, f.name), dict) for f in fields(self)}


class Evaluator:
    def __init__(self, plant, criteria):
        self.p, self.c = plant, criteria
        m = plant.m
        self.roots = m.body_rootid[m.geom_bodyid]
        self.pallet = m.body("pallet").id
        self.robot = m.body("chassis").id
        self.floor = m.geom("floor").id
        self.first = None
        self.proximity = None
        self.floor_contacts = 0
        self.target_pairs = []
        self.last_pose = plant.poses()[0]
        self.still_since = 0
        self.max_speed = self.max_steer_rate = 0.0
        self.forks = [m.geom(s + "_fork") for s in ("left", "right")]
        self.steer_dofs = [
            int(m.joint("steer_" + s).dofadr[0]) for s in ("left", "right")
        ]

    def metrics(self):
        d, g = self.p.d, self.p.geometry
        site = d.site("pallet_face")
        rot = site.xmat.reshape(3, 3)
        depths, ys = [], []
        inside = True
        for fork, side in zip(self.forks, (1, -1)):
            world = (SIGNS * fork.size) @ d.geom_xmat[fork.id].reshape(
                3, 3
            ).T + d.geom_xpos[fork.id]
            pts = (world - site.xpos) @ rot
            pts[:, 2] += site.xpos[2] - d.body("pallet").xpos[2]
            tip = pts[4:]
            depths.append(float(tip[:, 0].min()))
            ys.append(float(tip[:, 1].mean()))
            # Intersect the full blade box with the half-space past the mouth.
            samples = list(pts[pts[:, 0] >= 0])
            for j, k in BOX_EDGES:
                heel, end = pts[j], pts[k]
                if heel[0] * end[0] < 0:
                    samples.append(
                        heel + (end - heel) * (-heel[0] / (end[0] - heel[0]))
                    )
            if not samples:
                inside = False
                continue
            q = np.array(samples)
            y = side * q[:, 1]
            inside &= bool(
                y.min() >= g.opening[0]
                and y.max() <= g.opening[1]
                and q[:, 2].min() >= g.pocket_z[0]
                and q[:, 2].max() <= g.pocket_z[1]
                and q[:, 0].max() <= g.length
            )
        chassis, pallet = self.p.poses()
        return dict(
            depth=min(depths),
            lateral=sum(ys) / 2,
            yaw=wrap(chassis[2] - pallet[2]),
            inside=inside,
            settled=self.still_since is not None
            and self.p.tick * STEP_NS - self.still_since >= self.c.settle_ns,
            peak_true_speed=self.max_speed,
            peak_steer_rate=self.max_steer_rate,
            minimum_positive_separation=self.proximity,
            floor_contact_samples=self.floor_contacts,
        )

    def update(self):
        p = self.p
        now = p.tick * STEP_NS
        pose = p.poses()[0]
        dt = STEP_NS * 1e-9
        speed = np.linalg.norm(np.subtract(pose[:2], self.last_pose[:2])) / dt
        rate = abs(wrap(pose[2] - self.last_pose[2])) / dt
        self.last_pose = pose
        self.max_speed = max(self.max_speed, float(speed))
        self.max_steer_rate = max(
            self.max_steer_rate, float(np.abs(p.d.qvel[self.steer_dofs]).max())
        )
        if speed >= self.c.still_speed or rate >= self.c.still_yaw_rate:
            self.still_since = None
        elif self.still_since is None:
            self.still_since = now
        contacts = p.d.contact
        pairs = contacts.geom
        self.floor_contacts += int((pairs == self.floor).any(axis=1).sum())
        roots = self.roots[pairs]
        mask = (roots == self.robot).any(axis=1) & (roots == self.pallet).any(axis=1)
        self.target_pairs = []
        for pair, dist in zip(pairs[mask], contacts.dist[mask]):
            info = dict(
                time_ns=now,
                pair=[p.m.geom(int(i)).name for i in pair],
                distance=float(dist),
            )
            self.target_pairs.append(info)
            if dist > 0:
                self.proximity = min(dist, self.proximity or dist)
            elif self.first is None:
                self.first = dict(**info, depth=self.metrics()["depth"])

    def score(self, reason):
        metrics = self.metrics()
        if self.first is not None:
            return False, "contact", metrics
        success = (
            metrics["settled"]
            and metrics["inside"]
            and metrics["depth"] >= self.c.depth
            and abs(metrics["lateral"]) < self.c.lateral
            and abs(metrics["yaw"]) < self.c.yaw
        )
        if success and reason in ("assess_completion", "episode_timeout"):
            return True, "success", metrics
        return (
            False,
            "terminal_pose_error" if reason == "assess_completion" else reason,
            metrics,
        )
