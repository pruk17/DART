"""Config loader for the ROS2 layer.

Reads the YAML parameter file (robot speed/tolerances, decider epsilon, and the
list of targets) and turns it into the plain dataclasses that `dart.scoring`
and `dart.decider` already expect. Keeps all tuning out of the node code.
"""

from __future__ import annotations

import math
import os
from dataclasses import dataclass, field

import yaml

from dart.decider import DeciderParams
from dart.scoring import Target


@dataclass
class RobotParams:
    speed: float = 0.22
    goal_tol: float = 0.10
    safe_distance: float = 0.25
    wall_offset: float = 0.25   # distance kept from the wall WHILE following (<= safe_distance)
    max_ang: float = 2.84       # angular speed cap (rad/s); Burger hardware max is 2.84
    wall_follow_speed: float = 1.0  # forward speed while wall-following, as a fraction of speed


@dataclass
class DartConfig:
    robot: RobotParams
    decider: DeciderParams
    replan_rate: float
    expire_threshold: float
    targets: list[Target]
    service_time: float = 0.0   # seconds stopped at each delivery point
    target_types: dict = field(default_factory=dict)  # id -> food type name (for display)
    deadline: float = 0.0       # seconds the whole run should finish within (0 = none)


def _decay_rate(src: dict) -> float | None:
    """Decay rate from a dict: `half_life` (seconds) wins, else `lam`, else None."""
    if src.get("half_life") is not None:
        hl = float(src["half_life"])
        return math.log(2) / hl if hl > 0 else 0.0
    if src.get("lam") is not None:
        return float(src["lam"])
    return None


def _target_from_dict(d: dict, type_defaults: dict) -> Target:
    """Build a Target, filling v / decay from the named task_type when omitted.

    Decay can be given as `half_life` (seconds, preferred: lam = ln2 / half_life)
    or directly as `lam`. A value on the target overrides its type's value.
    """
    tdef = type_defaults.get(d.get("type", "default"), {})
    pos = d["position"]
    lam = _decay_rate(d)
    if lam is None:
        lam = _decay_rate(tdef)
    return Target(
        id=str(d["id"]),
        position=(float(pos[0]), float(pos[1])),
        v=float(d.get("v", tdef.get("v", 1.0))),
        lam=lam if lam is not None else 0.0,
    )


def _deep_merge(base: dict, over: dict) -> dict:
    """Return base updated with over (nested dicts merged, other keys replaced)."""
    out = dict(base)
    for k, v in over.items():
        if isinstance(v, dict) and isinstance(out.get(k), dict):
            out[k] = _deep_merge(out[k], v)
        else:
            out[k] = v
    return out


def _load_raw(path: str) -> dict:
    """Load a YAML file, resolving an optional `include:` base (relative path).

    This lets sim.yaml / real.yaml share a common.yaml of tuning while each
    keeps its own target list (and any per-mode overrides).
    """
    with open(path, "r") as f:
        raw = yaml.safe_load(f) or {}
    inc = raw.pop("include", None)
    if inc:
        base_path = inc if os.path.isabs(inc) else os.path.join(os.path.dirname(path), inc)
        base = _load_raw(base_path)
        raw = _deep_merge(base, raw)
    return raw


def load_config(path: str) -> DartConfig:
    """Load and validate a DART YAML config file into a DartConfig."""
    raw = _load_raw(path)

    r = raw.get("robot", {})
    robot = RobotParams(
        speed=float(r.get("speed", 0.22)),
        goal_tol=float(r.get("goal_tol", 0.10)),
        safe_distance=float(r.get("safe_distance", 0.25)),
        wall_offset=float(r.get("wall_offset", 0.25)),
        max_ang=float(r.get("max_ang", 2.84)),
        wall_follow_speed=float(r.get("wall_follow_speed", 1.0)),
    )

    d = raw.get("decider", {})
    decider = DeciderParams(epsilon=float(d.get("epsilon", 0.1)))
    replan_rate = float(d.get("replan_rate", 5.0))

    # Value fraction below which a target is considered expired (matches the sim).
    expire_threshold = float(raw.get("expire_threshold", 0.05))

    type_defaults = raw.get("task_types", {})
    targets = [_target_from_dict(t, type_defaults) for t in raw.get("targets", [])]

    return DartConfig(
        robot=robot,
        decider=decider,
        replan_rate=replan_rate,
        expire_threshold=expire_threshold,
        targets=targets,
        service_time=float(raw.get("service_time", 0.0)),
        deadline=float(raw.get("deadline", 0.0)),
        target_types={str(t["id"]): str(t.get("type", "")) for t in raw.get("targets", [])},
    )
