from math import atan2, radians, sqrt

import numpy as np

from .frames import arc, curvatures, matrix_tuple, rotation, transform, wrap
from .messages import Estimate, OdometryIncrement, PocketObservation


class SensorModel:
    def __init__(self, cfg, geometry):
        self.s, self.g = cfg.sensors, geometry
        self.noise, self.drop, self.drift = (
            np.random.default_rng([cfg.seed, i]) for i in (1, 2, 3)
        )
        self.bias = np.zeros(2)
        self.index = 0

    def capture(self, stamp, relative):
        ratio = min(1.0, np.linalg.norm(relative[:2]) / 4)
        std = self.s.noise_scale * np.array(
            [0.01 + 0.02 * ratio] * 2 + [radians(1 + ratio)]
        )
        noisy = relative + std * self.noise.standard_normal(3)
        idx = self.index
        self.index += 1
        random_drop = self.drop.random() < self.s.dropout
        if (
            random_drop
            or self.s.outage_index <= idx < self.s.outage_index + self.s.outage_count
        ):
            return None
        left, right = transform(
            noisy, [(0, self.g.landmark_y), (0, -self.g.landmark_y)]
        )
        return PocketObservation(
            stamp, tuple(left), tuple(right), matrix_tuple(np.diag(std**2))
        )

    def odometry(self, old, new):
        dt = (new.stamp.time_ns - old.stamp.time_ns) * 1e-9
        self.bias += (
            sqrt(dt)
            * np.array([self.s.scale_walk, self.s.heading_walk])
            * self.drift.standard_normal(2)
        )
        distance = self.g.radius * (sum(new.encoders) - sum(old.encoders)) / 2
        k = curvatures(self.g, (np.array(old.angles) + new.angles) / 2)
        noise = sqrt(dt) * np.array([self.s.encoder_noise, self.s.yaw_noise])
        dn, an = noise * self.drift.standard_normal(2)
        measured = distance * (1 + self.bias[0]) + dn
        angle = measured * sum(k) / 2 + abs(distance) * self.bias[1] + an
        # Use the measured arc angle even for a near-stationary encoder interval.
        dx, dy, _ = arc(1.0, angle)
        cov = np.diag([noise[0] ** 2, (0.5 * measured * noise[1]) ** 2, noise[1] ** 2])
        # Wheel travel is a proxy for axle motion. Turning and braking leave
        # millimetre-scale slip even with noiseless encoder readings.
        cov += abs(distance) * np.diag(np.square(self.s.slip_std_per_sqrt_m))
        return OdometryIncrement(
            new.stamp,
            old.stamp.time_ns,
            (measured * dx, measured * dy, angle),
            matrix_tuple(cov),
            abs(k[0] - k[1]),
        )


class EKF:
    def __init__(self, cfg, stamp):
        self.x = np.zeros(3)
        self.P = np.zeros((3, 3))
        self.stamp = stamp
        self.last = None
        self.seen_capture = -1
        self.capture_seq = -1
        self.rejected = 0
        self.cfg = cfg
        self.future = {}

    def predict(self, odom):
        if (
            odom.stamp.episode != self.stamp.episode
            or odom.stamp.frame != self.stamp.frame
            or odom.stamp.seq <= self.stamp.seq
            or odom.start_ns != self.stamp.time_ns
            or odom.stamp.time_ns <= odom.start_ns
        ):
            self.rejected += 1
            return False
        dt = (odom.stamp.time_ns - odom.start_ns) * 1e-9
        self.stamp = odom.stamp
        if self.last is not None:
            a = odom.delta[2]
            R = rotation(-a)
            self.x[:2] = R @ (self.x[:2] - odom.delta[:2])
            self.x[2] = wrap(self.x[2] - a)
            F = np.eye(3)
            F[:2, :2] = R
            G = -F.copy()
            G[:2, 2] = [self.x[1], -self.x[0]]
            self.P = (
                F @ self.P @ F.T
                + self.cfg.odometry_covariance_scale * G @ odom.covariance @ G.T
                + dt * np.diag(self.cfg.drift_variance_per_s)
            )
        return True

    def correct(self, obs, stopped):
        t = obs.stamp.time_ns
        if (
            obs.stamp.episode != self.stamp.episode
            or obs.stamp.frame != self.stamp.frame
            or obs.stamp.seq <= self.capture_seq
        ):
            self.rejected += 1
            return False
        if t > self.stamp.time_ns:
            self.future.setdefault(t, obs)
            return False
        if t < self.stamp.time_ns or t <= self.seen_capture:
            self.rejected += 1
            return False
        self.seen_capture = t
        self.capture_seq = obs.stamp.seq
        pair = np.array([obs.left, obs.right])
        dx, dy = pair[0] - pair[1]
        z = np.array([*pair.mean(axis=0), atan2(-dx, dy)])
        V = np.asarray(obs.covariance)
        if self.last is None:
            if not stopped:
                return False
            self.x, self.P = z, V.copy()
        else:
            innovation = z - self.x
            innovation[2] = wrap(innovation[2])
            Sinv = np.linalg.pinv(self.P + V, hermitian=True)
            if innovation @ Sinv @ innovation > self.cfg.gate_squared:
                self.rejected += 1
                return False
            gain = 1.0 if stopped else self.cfg.moving_detection_gain
            K = gain * self.P @ Sinv
            self.x += K @ innovation
            self.x[2] = wrap(self.x[2])
            A = np.eye(3) - K
            self.P = A @ self.P @ A.T + K @ V @ K.T
        self.last = t
        return True

    def deliver_due(self, stopped):
        for t in sorted(t for t in self.future if t <= self.stamp.time_ns):
            self.correct(self.future.pop(t), stopped)

    def estimate(self):
        return Estimate(
            self.stamp,
            tuple(self.x),
            matrix_tuple(self.P),
            self.last,
            self.last is not None,
        )
