from dataclasses import dataclass, field
from math import isfinite, radians

import numpy as np

STEP_NS = 2_000_000
CONTROL_NS = 10 * STEP_NS
CAPTURE_NS = 100 * STEP_NS


@dataclass(frozen=True)
class Scenario:
    distance: float = 2.0
    lateral: float = 0.0
    heading: float = 0.0
    pallet_yaw: float = 0.0


@dataclass(frozen=True)
class Sensors:
    dropout: float = 0.10
    noise_scale: float = 1.0
    outage_index: int = 5
    outage_count: int = 0
    scale_walk: float = 0.001  # fractional distance error / sqrt(s)
    heading_walk: float = 0.001  # rad/m / sqrt(s)
    encoder_noise: float = 0.0001  # m / sqrt(s)
    yaw_noise: float = 0.0001  # rad / sqrt(s)
    slip_std_per_sqrt_m: tuple = (0.006, 0.005, 0.004)  # body dx, dy, yaw


@dataclass(frozen=True)
class Limits:
    delay_ns: int = 150_000_000
    delay_uncertainty_ns: int = 0
    speed: float = 0.45
    recovery_speed: float = 0.10
    steer: float = radians(45)
    steer_rate: float = radians(30)
    target_steer_rate: float = radians(16)
    drive_gain: float = 800.0
    steering_damping: float = 3000.0
    drive_headroom: float = 0.01
    speed_response_s: float = 0.105
    steering_response_s: float = 0.92
    uncertainty_speed_cap: bool = False
    target_accel: float = 0.5
    brake: float = 0.30
    stop_buffer: float = 0.025
    tracking: float = 0.002
    sigma: float = 2.0
    approach_safety_factor: float = 1.5


@dataclass(frozen=True)
class Criteria:
    depth: float = 0.60
    target_allowance: float = 0.025
    lateral: float = 0.03
    yaw: float = radians(2)
    still_speed: float = 0.01
    still_yaw_rate: float = 0.01
    settle_ns: int = 200_000_000
    fresh_ns: int = 600_000_000
    acquire_ns: int = 3_000_000_000
    retries: int = 1


@dataclass(frozen=True)
class Config:
    method: str = "geometric"
    mode: str = "deterministic"
    seed: int = 7
    duration: float = 60.0
    scenario: Scenario = field(default_factory=Scenario)
    sensors: Sensors = field(default_factory=Sensors)
    limits: Limits = field(default_factory=Limits)
    criteria: Criteria = field(default_factory=Criteria)
    drift_variance_per_s: tuple = (1.0e-7, 1.0e-7, 1.0e-8)
    gate_squared: float = 16.27
    method_settings: tuple = ()
    moving_detection_gain: float = 1.0
    odometry_covariance_scale: float = 1.0

    def __post_init__(self):
        sensor, limits = self.sensors, self.limits
        if (
            not 0 < self.moving_detection_gain <= 1
            or not isfinite(self.odometry_covariance_scale)
            or self.odometry_covariance_scale <= 0
        ):
            raise ValueError("invalid estimator gain or odometry covariance scale")
        if self.seed < 0 or not isfinite(self.duration) or self.duration <= 0:
            raise ValueError(
                "duration must be finite and positive and seed must be nonnegative"
            )
        if (
            not 0 <= sensor.dropout <= 1
            or sensor.noise_scale < 0
            or min(sensor.outage_count, sensor.outage_index) < 0
        ):
            raise ValueError("invalid detector noise or outage settings")
        if (
            limits.delay_ns < limits.delay_uncertainty_ns
            or limits.delay_uncertainty_ns < 0
            or any(t % STEP_NS for t in (limits.delay_ns, limits.delay_uncertainty_ns))
        ):
            raise ValueError(
                "delay and uncertainty must be whole physics steps with a nonnegative minimum delay"
            )
        if (
            not 0 <= limits.drive_headroom < limits.speed
            or min(
                limits.brake,
                limits.speed_response_s,
                limits.steering_response_s,
                limits.target_accel,
            )
            <= 0
        ):
            raise ValueError("invalid motion limits or response times")
        if (
            not isfinite(limits.approach_safety_factor)
            or limits.approach_safety_factor < 1
        ):
            raise ValueError("approach safety factor must be finite and at least one")
        if self.mode not in ("deterministic", "realtime"):
            raise ValueError(f"unknown execution mode {self.mode}")


def scenario(name, seed):
    if name == "nominal":
        return Scenario()
    if name != "comparison":
        raise ValueError(f"unknown scenario {name}")
    rng = np.random.default_rng([seed, 0])
    return Scenario(
        rng.uniform(2, 4),
        rng.uniform(-0.5, 0.5),
        rng.uniform(-radians(20), radians(20)),
        rng.uniform(-np.pi, np.pi),
    )
