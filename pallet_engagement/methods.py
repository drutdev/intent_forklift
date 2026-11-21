from dataclasses import dataclass
from importlib import import_module

from .config import CONTROL_NS
from .messages import Command, MotionProposal


class Hold:
    period_ns = CONTROL_NS

    def reset(self, settings):
        pass

    def observe(self, message):
        pass

    def propose(self, context):
        return MotionProposal(
            Command(context.stamp), context.stamp.time_ns + 2 * self.period_ns, "hold"
        )


@dataclass(frozen=True)
class Registration:
    module: str
    factory: str
    available: bool


REGISTRY = {
    name: Registration(f"pallet_engagement.{name}", "Method", False)
    for name in ("geometric", "search_ilqr", "rl")
}
REGISTRY["hold"] = Registration(__name__, "Hold", True)


def load(name):
    entry = REGISTRY[name]
    if not entry.available:
        raise ValueError(f"{name}: registered, implementation pending")
    return getattr(import_module(entry.module), entry.factory)()
