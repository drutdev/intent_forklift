import json
import multiprocessing as mp
import subprocess
import time
from collections import Counter, deque
from dataclasses import asdict, replace
from hashlib import sha256
from importlib.metadata import version
from pathlib import Path

import numpy as np

from .config import CAPTURE_NS, CONTROL_NS, STEP_NS
from .estimation import EKF, SensorModel
from .evaluation import EpisodeOutcome, Evaluator
from .frames import wrap
from .guard import Supervisor
from .messages import Command, ControlContext, MotionProposal
from .methods import load
from .plant import MODEL, Plant
from .prediction import slew_targets
from .recording import Recorder, write_json


def compute(method, ctx, updates):
    start = time.monotonic_ns()
    try:
        for msg in updates:
            method.observe(msg)
        proposal = method.propose(ctx)
    except (
        Exception
    ) as exc:  # A method failure must still pass through braking and scoring.
        proposal = MotionProposal(
            Command(ctx.stamp),
            ctx.stamp.time_ns + CONTROL_NS,
            "failed",
            (("exception", type(exc).__name__),),
        )
    return proposal, time.monotonic_ns() - start


def worker(conn, method, settings):
    try:
        method.reset(settings)
        conn.send("ready")
        while True:
            ctx, updates = conn.recv()
            proposal, latency = compute(method, ctx, updates)
            conn.send(
                (
                    (ctx.stamp.episode, ctx.target_id, ctx.config_id, ctx.revision),
                    proposal,
                    latency,
                )
            )
    except EOFError:
        pass
    finally:
        conn.close()


def stats(values):
    return (
        dict(
            count=len(values),
            p50_ns=int(np.median(values)),
            p95_ns=int(np.percentile(values, 95)),
            max_ns=int(max(values)),
        )
        if values
        else dict(count=0)
    )


def manifest(cfg, plant, episode):
    def git(*args):
        return subprocess.check_output(
            ["git", "-C", str(MODEL.parent), *args], text=True
        ).strip()

    return dict(
        schema=1,
        episode=episode,
        config=asdict(cfg),
        geometry=asdict(plant.geometry),
        seeds=dict(
            scenario=[cfg.seed, 0],
            noise=[cfg.seed, 1],
            dropout=[cfg.seed, 2],
            odometry=[cfg.seed, 3],
            plant=[cfg.seed, 5],
            method=[cfg.seed, 6],
        ),
        revision=git("rev-parse", "HEAD"),
        dirty=bool(git("status", "--porcelain")),
        dependencies={
            n: version(n) for n in ("numpy", "mujoco", "rerun-sdk", "imageio-ffmpeg")
        },
        method_period_ns=load(cfg.method).period_ns,
        physics_step_ns=STEP_NS,
        control_period_ns=CONTROL_NS,
        capture_period_ns=CAPTURE_NS,
    )


def run(cfg, directory=None, live=False):
    method = load(cfg.method)
    config_id = sha256(json.dumps(asdict(cfg), sort_keys=True).encode()).hexdigest()[
        :16
    ]
    episode = f"{cfg.method}-{cfg.seed}-{config_id}"
    plant = Plant(cfg, episode)
    sensor = SensorModel(cfg, plant.geometry)
    evaluator = Evaluator(plant, cfg.criteria)
    ekf = EKF(cfg, plant.stamp())
    sup = Supervisor()
    proc = conn = rec = None

    def emit(kind, t, msg):
        if rec:
            rec.emit(kind, t, msg)

    latencies, overshoots, wall_cost = [], [], []
    issued = deque()
    nominal_targets = (0.0, (0.0, 0.0))
    history_base = Command(plant.stamp())
    proposal = previous = None
    updates = []
    busy = False
    next_job = 0
    late = expired = missed = 0
    overrides = Counter()
    revision = 0
    reason = "episode_timeout"
    start = 0
    coverage = []
    outage_coverage = []
    terminal_ns = None
    last_snapshot = None
    outage_distance = None
    try:
        settings = (
            *cfg.method_settings,
            (
                "method_seed",
                int(np.random.SeedSequence([cfg.seed, 6]).generate_state(1)[0]),
            ),
        )
        if directory:
            rec = Recorder(directory, manifest(cfg, plant, episode), live, model=plant.m)
        if cfg.mode == "realtime":
            context = mp.get_context("spawn")
            conn, child = context.Pipe()
            proc = context.Process(
                target=worker, args=(child, method, settings), daemon=True
            )
            proc.start()
            child.close()
            if not conn.poll(20) or conn.recv() != "ready":
                raise RuntimeError("motion worker failed to initialize")
        else:
            method.reset(settings)
        if rec:
            rec.start = time.monotonic_ns()
        start = time.monotonic_ns()
        while True:
            stamp = plant.stamp()
            t = stamp.time_ns
            if proc:
                wait = start + t - time.monotonic_ns()
                if wait > 0:
                    time.sleep(wait * 1e-9)
            if evaluator.first:
                break
            if t >= round(cfg.duration * 1e9) and terminal_ns is None:
                terminal_ns = t
            if t % CONTROL_NS == 0:
                begin = time.monotonic_ns()
                if proc:
                    slip = max(0, begin - start - t)
                    overshoots.append(slip)
                    missed += int(slip >= CONTROL_NS)
                feedback = plant.feedback()
                stopped = (
                    abs(feedback.axle_speed) < cfg.criteria.still_speed
                    and abs(feedback.yaw_rate) < cfg.criteria.still_yaw_rate
                )
                emit("feedback", t, feedback)
                updates.append(feedback)
                if previous is not None:
                    odom = sensor.odometry(previous, feedback)
                    ekf.predict(odom)
                    emit("odometry", t, odom)
                    updates.append(odom)
                previous = feedback
                ekf.deliver_due(stopped)
                truth = plant.relative()
                if t % CAPTURE_NS == 0:
                    if (
                        sensor.index == cfg.sensors.outage_index
                        and cfg.sensors.outage_count
                    ):
                        outage_distance = -evaluator.metrics()["depth"]
                    detection = sensor.capture(stamp, truth)
                    if detection:
                        ekf.correct(detection, stopped)
                        emit("detection", t, detection)
                        emit(
                            "filter_update",
                            t,
                            dict(accepted=ekf.last == t, rejected_total=ekf.rejected),
                        )
                        updates.append(detection)
                    else:
                        emit("dropout", t, dict(capture_ns=t))
                est = ekf.estimate()
                updates.append(est)
                emit("estimate", t, est)
                emit(
                    "evaluator_truth",
                    t,
                    dict(
                        pose=truth,
                        axle_world=plant.poses()[0],
                        pallet_world=plant.poses()[1],
                    ),
                )
                if est.initialized:
                    error = np.subtract(est.pose, truth)
                    error[2] = wrap(error[2])
                    covered = bool(
                        np.all(abs(error) <= 2 * np.sqrt(np.diag(est.covariance)))
                    )
                    coverage.append(covered)
                    if t - est.last_capture_ns >= CAPTURE_NS:
                        outage_coverage.append(covered)
                max_delay = cfg.limits.delay_ns + cfg.limits.delay_uncertainty_ns
                while issued and issued[0].stamp.time_ns + max_delay <= t:
                    history_base = issued.popleft()
                active = history_base
                pending = []
                for command in issued:
                    if command.stamp.time_ns + cfg.limits.delay_ns <= t:
                        active = command
                    else:
                        pending.append(command)
                ctx = ControlContext(
                    stamp,
                    est,
                    feedback,
                    plant.geometry,
                    cfg.limits,
                    cfg.criteria,
                    tuple(pending),
                    active,
                    sup.recovery,
                    sup.revision,
                    "pallet/front",
                    config_id,
                    tuple(issued),
                    history_base,
                    nominal_targets,
                )
                key = (episode, ctx.target_id, config_id, ctx.revision)
                if ctx.revision != revision:
                    proposal = None
                    revision = ctx.revision
                if proc and busy and conn.poll():
                    try:
                        result_key, result, latency = conn.recv()
                    except (EOFError, OSError):
                        result_key, result, latency = key, None, 0
                        reason, terminal_ns = "planning_failure", t
                    if result is None:
                        result = MotionProposal(
                            Command(stamp), t + CONTROL_NS, "failed"
                        )
                    latencies.append(latency)
                    busy = False
                    if result_key == key and result.valid_until_ns > max(
                        t, time.monotonic_ns() - start
                    ):
                        proposal = result
                        emit("proposal", t, result)
                    else:
                        late += 1
                        emit(
                            "late_result",
                            t,
                            dict(
                                latency_ns=latency,
                                issue_ns=result.command.stamp.time_ns,
                            ),
                        )
                if terminal_ns is None and t >= next_job and not busy:
                    next_job = (t // method.period_ns + 1) * method.period_ns
                    if proc:
                        conn.send((ctx, tuple(updates)))
                        busy = True
                    else:
                        proposal, latency = compute(method, ctx, updates)
                        latencies.append(latency)
                        emit("proposal", t, proposal)
                    updates.clear()
                if len(updates) > 1024:
                    reason = "worker_observation_overflow"
                    terminal_ns = t if terminal_ns is None else terminal_ns
                    updates.clear()
                checked = proposal
                if (
                    proc
                    and checked
                    and checked.valid_until_ns <= time.monotonic_ns() - start
                ):
                    checked = replace(checked, valid_until_ns=t)
                decision = sup.decide(ctx, None if terminal_ns is not None else checked)
                if (
                    proc
                    and checked
                    and decision.command.speed
                    and checked.valid_until_ns <= time.monotonic_ns() - start
                ):
                    late += 1
                    decision = sup.decide(ctx, replace(checked, valid_until_ns=t))
                if terminal_ns is not None:
                    decision = replace(decision, reason="terminal_stop", terminal="")
                expired += int(decision.reason == "expired_proposal")
                if decision.reason not in ("released", "hold", "terminal_stop"):
                    overrides[decision.reason] += 1
                emit("decision", t, decision)
                plant.enqueue(decision.command)
                issued.append(decision.command)
                # Reconstruct nominal actuator targets solely from released requests.
                # Include fractional control intervals at application boundaries.
                time_cursor, request = t, active
                for future in (*pending, decision.command):
                    when = future.stamp.time_ns + cfg.limits.delay_ns
                    if when > t + CONTROL_NS:
                        break
                    nominal_targets = slew_targets(
                        *nominal_targets,
                        request,
                        (when - time_cursor) * 1e-9,
                        plant.geometry,
                        cfg.limits,
                    )
                    time_cursor, request = when, future
                nominal_targets = slew_targets(
                    *nominal_targets,
                    request,
                    (t + CONTROL_NS - time_cursor) * 1e-9,
                    plant.geometry,
                    cfg.limits,
                )
                wall_cost.append(time.monotonic_ns() - begin)
                if decision.terminal:
                    reason = decision.terminal
                    terminal_ns = t
            if plant.tick % 20 == 0 and rec:
                emit("qpos", t, plant.d.qpos.copy())
                last_snapshot = t
            if terminal_ns is not None:
                # Only the evaluator/plant can see the actual FIFO. Continue scoring
                # the physical braking tail after the stack has requested termination.
                moving_command = plant.active.speed or any(c.speed for c in plant.queue)
                if not moving_command and evaluator.metrics()["settled"]:
                    break
                if t - terminal_ns > 3_000_000_000:
                    reason = "braking_timeout"
                    break
            applied = plant.step()
            if applied:
                emit("applied", t, applied)
            if rec:
                emit(
                    "targets",
                    t,
                    dict(speed=plant.target_v, angles=tuple(plant.target_angles)),
                )
            evaluator.update()
            if terminal_ns is None and (
                evaluator.max_speed > cfg.limits.speed
                or evaluator.max_steer_rate > cfg.limits.steer_rate
            ):
                reason = "measured_motion_limit"
                terminal_ns = t + STEP_NS
            for info in evaluator.target_pairs:
                emit("contact", t + STEP_NS, info)
        if rec and last_snapshot != plant.stamp().time_ns:
            emit("qpos", plant.stamp().time_ns, plant.d.qpos.copy())
        elapsed = time.monotonic_ns() - start
    finally:
        if proc:
            proc.terminate()
            proc.join()
            conn.close()
        complete = rec.close() if rec else None
    success, reason, metrics = evaluator.score(reason)
    metrics.update(
        gate_or_time_rejections=ekf.rejected,
        estimates_within_two_sigma=sum(coverage),
        estimates_scored=len(coverage),
        outage_start_distance=outage_distance,
        outage_estimates_scored=len(outage_coverage),
        outage_within_two_sigma=sum(outage_coverage),
    )
    timing = dict(
        wall_ns=elapsed,
        method=stats(latencies),
        compute_over_period=sum(v > method.period_ns for v in latencies),
        control=stats(wall_cost),
        control_lateness=stats(overshoots),
        missed_control_deadlines=missed,
        late_results=late,
        expired_proposal_ticks=expired,
        worker_pending=busy,
    )
    if rec:
        metrics.update(recording_drops=rec.dropped, recording_error=rec.error)
    outcome = EpisodeOutcome(
        episode,
        plant.tick * STEP_NS,
        success,
        reason,
        evaluator.first,
        metrics,
        timing,
        complete,
        dict(overrides),
        sup.attempts,
        cfg.mode,
    )
    if rec:
        write_json(Path(directory) / "outcome.json", outcome.to_dict())
    return outcome
