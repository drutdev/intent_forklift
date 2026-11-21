import unittest
from dataclasses import asdict
from math import pi

import numpy as np
import rerun as rr

from pallet_engagement.config import Config
from pallet_engagement.plant import Plant
from pallet_engagement.recording import log_rerun
from pallet_engagement.visualization import Scene3D


class CaptureStream:
    def __init__(self):
        self.values = {}
        self.static = set()

    def log(self, path, value, *, static=False):
        self.values[path] = value
        if static:
            self.static.add(path)


class SceneChecks(unittest.TestCase):
    @classmethod
    def setUpClass(cls):
        cls.plant = Plant(Config(), "visualization")
        cls.scene = Scene3D(cls.plant.m, asdict(cls.plant.geometry))

    def setUp(self):
        self.stream = CaptureStream()
        self.scene.detection = None
        self.scene.estimate = None

    def test_detection_keeps_capture_world_position(self):
        capture = dict(stamp=dict(time_ns=200), left=[1, 0.2], right=[1, -0.2])
        streams = [self.stream, CaptureStream()]
        self.scene.log_event(streams, "detection", 200, capture)
        self.assertNotIn("scene/detections", self.stream.values)
        truth = dict(axle_world=[2, 3, pi / 2], pallet_world=[5, 3, 0])
        self.scene.log_event(streams, "evaluator_truth", 200, truth)
        detection = self.stream.values["scene/detections"]
        points = detection.positions.as_arrow_array().to_pylist()
        np.testing.assert_allclose(
            points,
            [[1.8, 4, self.scene.pocket_height], [2.2, 4, self.scene.pocket_height]],
        )
        np.testing.assert_array_equal(
            points,
            streams[1].values["scene/detections"].positions.as_arrow_array().to_pylist(),
        )
        truth["axle_world"] = [3, 3, pi / 2]
        self.scene.log_event(streams, "evaluator_truth", 220, truth)
        self.assertIs(self.stream.values["scene/detections"], detection)
        self.scene.log_event(streams, "dropout", 400, {})
        self.assertIsInstance(self.stream.values["scene/detections"], rr.Clear)

    def test_mesh_pose_snapshots_do_not_mutate_simulation(self):
        self.scene.log_static([self.stream])
        self.assertTrue(self.scene.geoms)
        self.assertTrue(set(self.scene.meshes).issubset(self.stream.static))
        source = self.plant.d.qpos.copy()
        self.scene.log_pose([self.stream], source)
        _, path = self.scene.geoms[0]
        first = self.stream.values[path].translation.as_arrow_array().to_pylist()
        captured = self.stream.values[path]
        moved = source.copy()
        moved[0] += 0.5
        self.scene.log_pose([self.stream], moved)
        latest = self.stream.values[path].translation.as_arrow_array().to_pylist()
        np.testing.assert_allclose(
            np.subtract(latest, first), [[0.5, 0, 0]], atol=1e-6
        )
        np.testing.assert_array_equal(
            captured.translation.as_arrow_array().to_pylist(), first
        )
        np.testing.assert_array_equal(self.plant.d.qpos, source)

    def test_missing_guide_clears_previous_path(self):
        self.scene.log_event(
            [self.stream],
            "proposal",
            0,
            dict(guide=dict(samples=[[-2, 0, 0, 0], [-1, 0.1, 0, 0]])),
        )
        path = self.stream.values["scene/pallet_frame/guide"]
        np.testing.assert_allclose(
            path.strips.as_arrow_array().to_pylist(),
            [[[-2, 0, 0.025], [-1, 0.1, 0.025]]],
        )
        self.scene.log_event(
            [self.stream], "proposal", 20, dict(guide=None, rollout=None)
        )
        self.assertIsInstance(
            self.stream.values["scene/pallet_frame/guide"], rr.Clear
        )
        self.scene.log_event([self.stream], "decision", 20, dict(stopping_path=[]))
        self.assertIsInstance(
            self.stream.values["scene/pallet_frame/stopping_path"], rr.Clear
        )

    def test_path_comparison_keeps_separate_data_and_clears_stale_paths(self):
        proposal = dict(
            command=dict(speed=0.1, curvature=0),
            guide=dict(samples=[[-2, 0, 0, 0.1], [-1, 0.2, 0, 0]]),
            rollout=dict(samples=[[-2, 0, 0, 0.1], [-1.8, 0.05, 0, 0.1]]),
        )
        log_rerun(rr, self.stream, "proposal", proposal)
        for name in ("guide", "rollout"):
            path = self.stream.values[f"pallet/{name}"]
            endpoint = self.stream.values[f"pallet/{name}/endpoint"]
            np.testing.assert_allclose(
                path.strips.as_arrow_array().to_pylist(),
                [[sample[:2] for sample in proposal[name]["samples"]]],
            )
            np.testing.assert_allclose(
                endpoint.positions.as_arrow_array().to_pylist(),
                [proposal[name]["samples"][-1][:2]],
            )
        log_rerun(
            rr,
            self.stream,
            "proposal",
            dict(command=proposal["command"], guide=None, rollout=None),
        )
        for name in ("guide", "rollout"):
            self.assertIsInstance(self.stream.values[f"pallet/{name}"], rr.Clear)

    def test_sensing_pockets_share_the_detection_frame(self):
        geometry = asdict(self.plant.geometry)
        offset = self.plant.geometry.landmark_y
        for kind in ("estimate", "evaluator_truth"):
            value = dict(pose=[2, 3, pi / 2])
            if kind == "estimate":
                value.update(
                    covariance=np.eye(3),
                    stamp=dict(time_ns=200),
                    last_capture_ns=100,
                )
            else:
                value.update(axle_world=[0, 0, 0], pallet_world=[2, 3, pi / 2])
            log_rerun(rr, self.stream, kind, value, geometry)
            points = self.stream.values[f"front_axle/{kind}/pockets"]
            np.testing.assert_allclose(
                points.positions.as_arrow_array().to_pylist(),
                [[2 - offset, 3], [2 + offset, 3]],
            )

    def test_tracking_metrics_preserve_signed_error_and_path_distance(self):
        log_rerun(
            rr,
            self.stream,
            "proposal",
            dict(
                command=dict(speed=0.1, curvature=0),
                diagnostics=[["cross_track_m", -0.025], ["remaining_m", 1.2]],
            ),
        )
        for name, expected in (("lateral_error_m", -0.025), ("distance_to_end_m", 1.2)):
            value = self.stream.values[f"tracking/{name}"]
            np.testing.assert_allclose(
                value.scalars.as_arrow_array().to_pylist(), [expected]
            )


if __name__ == "__main__":
    unittest.main()
