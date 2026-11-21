"""Immutable snapshots. time_ns is acquisition or issue time on the episode clock."""

from dataclasses import dataclass
from typing import ClassVar, Protocol

from .config import Criteria, Limits


@dataclass(frozen=True)
class Stamp:
    episode: str
    seq: int
    time_ns: int
    frame: str
    schema: int = 1


@dataclass(frozen=True)
class VehicleFeedback:
    stamp: Stamp
    encoders: tuple
    speeds: tuple
    angles: tuple
    axle_speed: float
    yaw_rate: float
    steering_rates: tuple


@dataclass(frozen=True)
class OdometryIncrement:
    stamp: Stamp
    start_ns: int
    delta: tuple
    covariance: tuple
    curvature_disagreement: float

    @property
    def end_ns(self):
        return self.stamp.time_ns


@dataclass(frozen=True)
class PocketObservation:
    stamp: Stamp
    left: tuple
    right: tuple
    covariance: tuple


@dataclass(frozen=True)
class Estimate:
    stamp: Stamp
    pose: tuple
    covariance: tuple
    last_capture_ns: int | None
    initialized: bool


@dataclass(frozen=True)
class Command:
    stamp: Stamp
    speed: float = 0.0
    curvature: float = 0.0


@dataclass(frozen=True)
class GuideTrajectory:
    stamp: Stamp
    plan_id: str
    target_id: str
    samples: tuple  # (axle x, y, yaw in pallet frame, signed speed)


@dataclass(frozen=True)
class MotionProposal:
    command: Command
    valid_until_ns: int
    status: str = "drive"
    diagnostics: tuple = ()
    guide: GuideTrajectory | None = None
    rollout: GuideTrajectory | None = None


@dataclass(frozen=True)
class ControlContext:
    stamp: Stamp
    estimate: Estimate
    feedback: VehicleFeedback
    geometry: "Geometry"
    limits: Limits
    criteria: Criteria
    pending: tuple
    active: Command
    recovery: bool
    revision: int
    target_id: str
    config_id: str
    issued_history: tuple = ()
    history_base: Command | None = None
    nominal_targets: tuple | None = None  # speed and two wheel angles at stamp


@dataclass(frozen=True)
class SupervisorDecision:
    command: Command
    reason: str
    recovery: bool
    attempts: int
    clearance: float | None
    stopping_distance: float
    terminal: str = ""
    stopping_path: tuple = ()


@dataclass(frozen=True)
class Geometry:
    wheelbase: float
    rear_y: tuple
    front_y: tuple
    radius: float
    tip_x: float
    tip_y: float
    half_width: float
    opening: tuple  # positive y inner and outer bounds
    pocket_z: tuple  # height above pallet base
    length: float
    pallet_half_width: float
    pieces: tuple  # local (xmin, xmax, ymin, ymax, is_fork) for height-relevant pieces

    @property
    def landmark_y(self):
        return sum(self.opening) / 2


class Method(Protocol):
    period_ns: ClassVar[int]

    def reset(self, settings: tuple): ...
    def observe(self, message): ...
    def propose(self, context: ControlContext) -> MotionProposal: ...
