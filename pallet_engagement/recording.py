"""Local recording, estimator replay, and scene playback."""

import json
import shutil
import sysconfig
import time
from copy import deepcopy
from dataclasses import asdict
from pathlib import Path
from queue import Full, Queue
from threading import Thread

import numpy as np

from .config import Config, Criteria, Limits, Scenario, Sensors
from .estimation import EKF
from .frames import transform
from .messages import OdometryIncrement, PocketObservation, Stamp
from .visualization import PATH_STYLES, Scene3D


def write_json(path, value):
    path.write_text(json.dumps(value, indent=2, allow_nan=False) + "\n")


def log_rerun(rr, stream, kind, v, geometry=None):
    def scalar(path, value):
        if value is not None:
            stream.log(path, rr.Scalars(value))

    if kind in ("estimate", "evaluator_truth") and v.get("initialized", True):
        stream.log(f"front_axle/{kind}", rr.Points2D([v["pose"][:2]]))
        if geometry is not None:
            landmark_y = sum(geometry["opening"]) / 2
            stream.log(
                f"front_axle/{kind}/pockets",
                rr.Points2D(
                    transform(v["pose"], [(0, landmark_y), (0, -landmark_y)]),
                    labels=[f"{kind} left", f"{kind} right"],
                    colors=[50, 200, 255] if kind == "estimate" else [70, 200, 90],
                ),
            )
        scalar(kind + "/yaw", v["pose"][2])
        if kind == "evaluator_truth":
            stream.log("world/evaluator/axle", rr.Points2D([v["axle_world"][:2]]))
            stream.log("world/evaluator/pallet", rr.Points2D([v["pallet_world"][:2]]))
        if kind == "estimate":
            for axis, var in zip("xyθ", np.diag(v["covariance"])):
                scalar("variance/" + axis, var)
            scalar(
                "detector/age_s", (v["stamp"]["time_ns"] - v["last_capture_ns"]) * 1e-9
            )
    elif kind == "detection":
        stream.log(
            "front_axle/pockets",
            rr.Points2D([v["left"], v["right"]], labels=["left", "right"]),
        )
    elif kind == "feedback":
        scalar("motion/measured_speed", v["axle_speed"])
        for i, a in enumerate(v["angles"]):
            scalar(f"motion/rear_angle_{i}", a)
    elif kind in ("proposal", "decision"):
        scalar(f"motion/{kind}_speed", v["command"]["speed"])
        scalar(f"motion/{kind}_curvature", v["command"]["curvature"])
        stream.log(kind, rr.TextLog(v.get("reason", v.get("status", ""))))
        if kind == "decision":
            if len(v["stopping_path"]) > 1:
                stream.log(
                    "pallet/stopping_path",
                    rr.LineStrips2D([[p[:2] for p in v["stopping_path"]]]),
                )
            scalar("guard/clearance", v["clearance"])
            scalar("guard/stopping_distance", v["stopping_distance"])
        else:
            if v.get("diagnostics"):
                stream.log("solver/diagnostics", rr.TextLog(str(v["diagnostics"])))
            diagnostics = dict(v.get("diagnostics", ()))
            scalar("tracking/lateral_error_m", diagnostics.get("cross_track_m"))
            scalar("tracking/distance_to_end_m", diagnostics.get("remaining_m"))
            goal_label = "Guide endpoint"
            horizon_label = "Prediction horizon"
            for name, color, width, label in PATH_STYLES:
                samples = (v.get(name) or {}).get("samples", [])
                path = f"pallet/{name}"
                if samples:
                    stream.log(
                        path,
                        rr.LineStrips2D(
                            [[s[:2] for s in samples]],
                            colors=color,
                            radii=rr.Radius.ui_points(width),
                            labels=label,
                            draw_order=1 if name == "guide" else 2,
                        ),
                    )
                    stream.log(
                        path + "/endpoint",
                        rr.Points2D(
                            [samples[-1][:2]],
                            colors=color,
                            radii=rr.Radius.ui_points(5),
                            labels=goal_label if name == "guide" else horizon_label,
                        ),
                    )
                else:
                    stream.log(path, rr.Clear(recursive=True))
    elif kind == "odometry":
        scalar("odometry/curvature_disagreement", v["curvature_disagreement"])
    elif kind == "dropout":
        stream.log("detector/dropout", rr.TextLog("missed capture"))
    elif kind == "late_result":
        scalar("solver/late_latency_ms", v["latency_ns"] * 1e-6)
    elif kind == "applied":
        scalar("motion/applied_request", v["speed"])
    elif kind == "targets":
        scalar("motion/target_speed", v["speed"])
    elif kind == "contact":
        stream.log("evaluator/contact", rr.TextLog(str(v)))


def diagnostic_views():
    import rerun.blueprint as bp

    sensing = bp.Tabs(
        bp.Spatial2DView(
            origin="/front_axle",
            name="Sensing: detections, estimate and ground truth",
        ),
        bp.TimeSeriesView(origin="/variance", name="Uncertainty: pose variances"),
        bp.TimeSeriesView(name="Detection age (s)", contents=["/detector/age_s"]),
        bp.Spatial2DView(origin="/world", name="Evaluator world poses"),
    )
    tracking = bp.Tabs(
        bp.Spatial2DView(
            origin="/pallet", name="Tracking: guide, prediction and stopping path"
        ),
        bp.TimeSeriesView(
            name="Desired vs measured speed (m/s)",
            contents=[
                "/motion/measured_speed",
                "/motion/proposal_speed",
                "/motion/decision_speed",
                "/motion/target_speed",
            ],
        ),
        bp.TimeSeriesView(
            name="Lateral error to path (m)",
            contents=["/tracking/lateral_error_m"],
        ),
        bp.TimeSeriesView(
            name="Distance to end (m)",
            contents=["/tracking/distance_to_end_m"],
        ),
        bp.TimeSeriesView(
            name="Rear steering (rad)",
            contents=["/motion/rear_angle_0", "/motion/rear_angle_1"],
        ),
        bp.TimeSeriesView(origin="/guard", name="Clearance and stopping distance (m)"),
        bp.TextLogView(name="Method and supervisor messages"),
    )
    scene = bp.Tabs(
        bp.Spatial3DView(
            origin="/scene",
            name="3D scene",
            background=[235, 238, 242],
            line_grid=True,
            eye_controls=bp.EyeControls3D(
                position=[2, -7, 4.5],
                look_target=[0, 0, 0.4],
                eye_up=[0, 0, 1],
            ),
        ),
        bp.Spatial3DView(
            origin="/scene",
            contents=[
                "$origin/pallet_frame/guide/**",
                "$origin/pallet_frame/rollout/**",
                "$origin/detections",
                "$origin/estimate/**",
            ],
            name="3D paths",
            background=[235, 238, 242],
            line_grid=True,
        ),
    )
    return bp.Blueprint(
        bp.Horizontal(
            scene,
            bp.Vertical(sensing, tracking),
            column_shares=[3, 2],
        ),
        collapse_panels=True,
    )


class Recorder:
    def __init__(self, directory, manifest, live=False, model=None):
        import rerun as rr

        self.dir = Path(directory)
        self.dir.mkdir(parents=True, exist_ok=False)
        write_json(self.dir / "manifest.json", manifest)
        self.q = Queue(maxsize=8192)
        self.dropped = 0
        self.error = None
        self.start = time.monotonic_ns()
        self.rr = rr
        self.geometry = manifest.get("geometry")
        self.stream = rr.RecordingStream("pallet_engagement")
        self.stream.save(self.dir / "trace.rrd")
        self.live = rr.RecordingStream("pallet_engagement_live") if live else None
        if self.live:
            # Explicit .venv/bin/python does not put .venv/bin on PATH.
            rr.spawn(
                recording=self.live,
                executable_path=shutil.which("rerun", path=sysconfig.get_path("scripts")),
            )
        self.streams = [stream for stream in (self.stream, self.live) if stream]
        self.scene = Scene3D(model, manifest["geometry"]) if model is not None else None
        if self.scene:
            self.scene.log_static(self.streams)
        views = diagnostic_views()
        self.stream.send_blueprint(views)
        if self.live:
            self.live.send_blueprint(views)
        self.thread = Thread(target=self._write)
        self.thread.start()

    def emit(self, kind, t, value):
        if kind == "proposal":
            value = deepcopy(value)
        try:
            self.q.put_nowait((kind, t, time.monotonic_ns() - self.start, value))
        except Full:
            self.dropped += 1

    def _write(self):
        times, poses = [], []
        try:
            with (self.dir / "events.jsonl").open("w") as out:
                while True:
                    item = self.q.get()
                    if item is None:
                        break
                    kind, t, wall, value = item
                    for stream in self.streams:
                        stream.set_time("simulation", duration=np.timedelta64(t, "ns"))
                        stream.set_time(
                            "elapsed_wall", duration=np.timedelta64(wall, "ns")
                        )
                    if kind == "qpos":
                        times.append(t)
                        poses.append(value)
                        if self.scene:
                            self.scene.log_pose(self.streams, value)
                        continue
                    v = (
                        asdict(value)
                        if hasattr(value, "__dataclass_fields__")
                        else value
                    )
                    if kind == "odometry":
                        v["end_ns"] = value.end_ns
                    elif kind == "detection":
                        v["capture_ns"] = value.stamp.time_ns
                    out.write(
                        json.dumps(
                            dict(schema=1, kind=kind, time_ns=t, wall_ns=wall, data=v),
                            allow_nan=False,
                        )
                        + "\n"
                    )
                    for stream in self.streams:
                        if kind != "targets" or t % 20_000_000 == 0:
                            log_rerun(self.rr, stream, kind, v, self.geometry)
                    if self.scene:
                        self.scene.log_event(self.streams, kind, t, v)
            np.savez_compressed(self.dir / "states.npz", time_ns=times, qpos=poses)
        except Exception as exc:
            self.error = repr(exc)
        finally:
            try:
                self.stream.flush()
                self.stream.disconnect()
                if self.live:
                    self.live.disconnect()
            except Exception as exc:
                self.error = repr(exc)

    def close(self):
        while self.thread.is_alive():
            try:
                self.q.put(None, timeout=0.1)
                break
            except Full:
                continue
        self.thread.join()
        return not self.error and self.dropped == 0


def configuration(directory):
    data = json.loads((Path(directory) / "manifest.json").read_text())["config"]
    # Original shared recordings predate the wheel-slip covariance term.
    data["sensors"].setdefault("slip_std_per_sqrt_m", (0.0, 0.0, 0.0))
    for key, cls in [
        ("scenario", Scenario),
        ("limits", Limits),
        ("criteria", Criteria),
        ("sensors", Sensors),
    ]:
        data[key] = cls(**data[key])
    return Config(**data)


def replay(directory):
    outcome = json.loads((Path(directory) / "outcome.json").read_text())
    if not outcome["recording_complete"]:
        raise ValueError("incomplete recording")
    cfg = configuration(directory)
    ekf = None
    count = 0
    with (Path(directory) / "events.jsonl").open() as events:
        for line in events:
            row = json.loads(line)
            kind, v = row["kind"], row["data"]
            if kind not in ("feedback", "odometry", "detection", "estimate"):
                continue
            v["stamp"] = Stamp(**v["stamp"])
            if kind == "feedback":
                if ekf is None:
                    ekf = EKF(cfg, v["stamp"])
                stopped = (
                    abs(v["axle_speed"]) < cfg.criteria.still_speed
                    and abs(v["yaw_rate"]) < cfg.criteria.still_yaw_rate
                )
            elif kind == "odometry":
                if v.pop("end_ns") != v["stamp"].time_ns:
                    raise ValueError("odometry endpoint mismatch")
                ekf.predict(OdometryIncrement(**v))
            elif kind == "detection":
                if v.pop("capture_ns") != v["stamp"].time_ns:
                    raise ValueError("capture timestamp mismatch")
                ekf.correct(PocketObservation(**v), stopped)
            else:
                e = ekf.estimate()
                if not (
                    e.stamp == v["stamp"]
                    and e.last_capture_ns == v["last_capture_ns"]
                    and np.array_equal(e.pose, v["pose"])
                    and np.array_equal(e.covariance, v["covariance"])
                ):
                    raise ValueError(
                        f"estimator replay diverged at {row['time_ns']} ns"
                    )
                count += 1
    if not count:
        raise ValueError("recording contains no estimates")
    return count


def playback(directory, video=None):
    import os
    import sys

    # The pinned GLFW Wayland window can keep its render loop alive on close.
    # Select the wheel's X11 backend when the host also exposes XWayland.
    if (
        not video
        and sys.platform.startswith("linux")
        and os.environ.get("DISPLAY")
        and os.environ.get("XDG_SESSION_TYPE") == "wayland"
    ):
        os.environ.setdefault("PYGLFW_LIBRARY_VARIANT", "x11")
    import mujoco

    from .plant import Plant

    plant = Plant(configuration(directory), "playback")
    with np.load(Path(directory) / "states.npz") as data:
        times, poses = data["time_ns"], data["qpos"]

    def restore(i):
        plant.d.time = times[i] * 1e-9
        plant.d.qpos[:] = poses[i]
        mujoco.mj_forward(plant.m, plant.d)

    if video:
        import imageio_ffmpeg

        writer = imageio_ffmpeg.write_frames(
            str(video), (640, 480), fps=25, output_params=["-loglevel", "error"]
        )
        writer.send(None)
        camera = mujoco.MjvCamera()
        camera.distance, camera.azimuth, camera.elevation = 6.0, 115.0, -28.0
        try:
            with mujoco.Renderer(plant.m, height=480, width=640) as renderer:
                for i in range(len(poses)):
                    restore(i)
                    camera.lookat[:] = (
                        plant.d.body("chassis").xpos + plant.d.body("pallet").xpos
                    ) / 2
                    renderer.update_scene(plant.d, camera=camera)
                    writer.send(renderer.render().copy())
        finally:
            writer.close()
    else:
        import threading

        import mujoco.viewer

        existing = set(threading.enumerate())
        with mujoco.viewer.launch_passive(plant.m, plant.d) as viewer:
            # MuJoCo 4.0.1 closes its native window asynchronously. Let that
            # thread destroy the window before Python runs GLFW's exit hook.
            render_threads = [
                thread
                for thread in threading.enumerate()
                if thread not in existing and "_launch_internal" in thread.name
            ]
            start = time.monotonic()
            for i, t in enumerate(times):
                if not viewer.is_running():
                    break
                time.sleep(max(0, start + t * 1e-9 - time.monotonic()))
                with viewer.lock():
                    restore(i)
                viewer.sync()
        for thread in render_threads:
            thread.join()
    return len(poses)
