"""Rerun scene geometry and planar observations in world coordinates."""

import mujoco
import numpy as np

from .frames import compose, rotation, transform

PATH_STYLES = (
    ("guide", [70, 200, 90], 4, "Joined path"),
    ("rollout", [255, 180, 30], 2, "Tracking prediction"),
)


class Scene3D:
    def __init__(self, model, geometry):
        import rerun as rr

        self.rr = rr
        self.model = model
        self.data = mujoco.MjData(model)
        self.pocket_height = float(np.mean(geometry["pocket_z"]))
        self.half_width = geometry["pallet_half_width"]
        self.detection = None
        self.estimate = None
        self.geoms = []
        self.meshes = {}
        for geom_id in range(model.ngeom):
            geom = model.geom(geom_id)
            if (
                geom.type[0] != mujoco.mjtGeom.mjGEOM_MESH
                or geom.contype[0]
                or geom.conaffinity[0]
            ):
                continue
            mesh_id = int(geom.dataid[0])
            path = f"scene/model/geom_{geom_id}"
            self.geoms.append((geom_id, path))
            mesh = model.mesh(mesh_id)
            first_vertex = int(mesh.vertadr[0])
            first_face = int(mesh.faceadr[0])
            vertices = model.mesh_vert[
                first_vertex : first_vertex + int(mesh.vertnum[0])
            ]
            faces = model.mesh_face[first_face : first_face + int(mesh.facenum[0])]
            normals = np.zeros_like(vertices)
            face_normals = np.cross(
                vertices[faces[:, 1]] - vertices[faces[:, 0]],
                vertices[faces[:, 2]] - vertices[faces[:, 0]],
            )
            for corner in range(3):
                np.add.at(normals, faces[:, corner], face_normals)
            lengths = np.linalg.norm(normals, axis=1, keepdims=True)
            np.divide(normals, lengths, out=normals, where=lengths > 0)
            self.meshes[path] = rr.Mesh3D(
                vertex_positions=vertices,
                triangle_indices=faces,
                vertex_normals=normals,
                albedo_factor=geom.rgba,
            )

    def log_static(self, streams):
        for stream in streams:
            stream.log("scene", self.rr.ViewCoordinates.FLU, static=True)
            for path, mesh in self.meshes.items():
                stream.log(path, mesh, static=True)

    def log_pose(self, streams, qpos):
        self.data.qpos[:] = qpos
        mujoco.mj_forward(self.model, self.data)
        for geom_id, path in self.geoms:
            pose = self.rr.Transform3D(
                translation=self.data.geom_xpos[geom_id].copy(),
                mat3x3=self.data.geom_xmat[geom_id].reshape(3, 3).copy(),
            )
            for stream in streams:
                stream.log(path, pose)

    def log_event(self, streams, kind, time_ns, value):
        rr = self.rr
        if kind == "detection":
            # Truth follows the capture event at the same simulation tick.
            self.detection = value
        elif kind == "estimate":
            self.estimate = value if value["initialized"] else None
        elif kind == "evaluator_truth":
            pallet = value["pallet_world"]
            planar_rotation = rotation(pallet[2])
            mat = np.eye(3)
            mat[:2, :2] = planar_rotation
            pallet_pose = rr.Transform3D(translation=[*pallet[:2], 0], mat3x3=mat)
            for stream in streams:
                stream.log("scene/pallet_frame", pallet_pose)
            if self.estimate is not None:
                estimate = compose(value["axle_world"], self.estimate["pose"])
                entry = transform(
                    estimate, [[0, -self.half_width], [0, self.half_width]]
                )
                entry = np.column_stack((entry, np.full(2, self.pocket_height)))
                for stream in streams:
                    stream.log(
                        "scene/estimate/entry",
                        rr.LineStrips3D(
                            [entry],
                            colors=[50, 200, 255],
                            radii=0.007,
                            labels="Estimated entry face",
                        ),
                    )
            if (
                self.detection is not None
                and self.detection["stamp"]["time_ns"] == time_ns
            ):
                points = transform(
                    value["axle_world"],
                    [self.detection["left"], self.detection["right"]],
                )
                points = np.column_stack((points, np.full(2, self.pocket_height)))
                for stream in streams:
                    stream.log(
                        "scene/detections",
                        rr.Points3D(
                            points,
                            radii=0.025,
                            colors=[[255, 90, 70], [210, 80, 255]],
                            labels=["Left pocket", "Right pocket"],
                        ),
                    )
                self.detection = None
        elif kind == "dropout":
            self.detection = None
            for stream in streams:
                stream.log("scene/detections", rr.Clear(recursive=True))
        elif kind in ("proposal", "decision"):
            if kind == "decision":
                paths = [
                    (
                        "stopping_path",
                        value["stopping_path"],
                        [255, 80, 70],
                        1.5,
                        "Stop",
                    )
                ]
            else:
                paths = [
                    (
                        name,
                        (value.get(name) or {}).get("samples", []),
                        color,
                        width,
                        label,
                    )
                    for name, color, width, label in PATH_STYLES
                ]
            for name, samples, color, width, label in paths:
                path = f"scene/pallet_frame/{name}"
                if len(samples) > 1:
                    points = np.column_stack(
                        (np.asarray(samples)[:, :2], np.full(len(samples), 0.025))
                    )
                    visible = rr.LineStrips3D(
                        [points],
                        colors=color,
                        radii=rr.Radius.ui_points(width),
                        labels=label,
                    )
                else:
                    visible = rr.Clear(recursive=True)
                for stream in streams:
                    stream.log(path, visible)
