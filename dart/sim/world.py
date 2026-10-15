"""Headless 2D world for validating DART (no ROS2, no rendering).

The world owns the true robot position and the list of remaining targets,
advances time in fixed steps, decays target values, and serves targets the
robot reaches. It knows nothing about which policy is driving - it just
executes motion and bookkeeping so DART and nearest-first can be compared
fairly on the same scenario.

Value convention: each Target.v holds the target's CURRENT (already-decayed)
value. decay_targets() lowers it one dt at a time; scoring then projects it
forward by the estimated travel time. The original values are snapshotted in
`initial_value` so expiry can be judged as a fraction of the starting value.
"""

from __future__ import annotations

import math
import random
from dataclasses import dataclass, field

from ..scoring import RobotState, Target, euclidean_distance, value_remaining


@dataclass
class WorldConfig:
    """Scenario + simulation settings."""

    dt: float = 0.1                 # simulation timestep (s).
    speed: float = 0.22             # robot speed (m/s).
    reach_tol: float = 0.15         # distance (m) within which a target is "reached".
    expire_threshold: float = 0.05  # value fraction below which a target is "expired".
    max_time: float = 300.0         # safety cap on total sim time (s).
    area_half_size: float = 4.0     # targets scattered in [-area, +area] on each axis.


@dataclass
class SimResult:
    """Outcome of one policy run, for comparison."""

    total_value_collected: float = 0.0
    expired_count: int = 0
    served_count: int = 0
    finish_time: float = 0.0


@dataclass
class World:
    """Mutable simulation state.

    Attributes
    ----------
    robot : RobotState
        Current robot pose + speed.
    targets : list[Target]
        Remaining (unserved) targets; served/expired ones are removed.
    config : WorldConfig
        Scenario settings.
    time : float
        Elapsed simulation time (s).
    initial_value : dict[str, float]
        Snapshot of each target's starting value, keyed by target id. Used to
        judge expiry as a fraction of the original value.
    """

    robot: RobotState
    targets: list[Target]
    config: WorldConfig = field(default_factory=WorldConfig)
    time: float = 0.0
    initial_value: dict[str, float] = field(default_factory=dict)

    def __post_init__(self) -> None:
        # Snapshot starting values once, so expiry is measured against them.
        if not self.initial_value:
            self.initial_value = {t.id: t.v for t in self.targets}

    def is_done(self) -> bool:
        """Return True when no targets remain or max_time is exceeded."""
        return not self.targets or self.time >= self.config.max_time

    def step_robot_toward(self, goal_pos: tuple[float, float]) -> None:
        """Move the robot one dt toward goal_pos and advance the clock.

        Straight-line move of length speed * dt (no obstacles in the first
        scenario). If the remaining distance is within one step, snap exactly
        onto the goal.
        """
        rx, ry = self.robot.position
        gx, gy = goal_pos
        dx, dy = gx - rx, gy - ry
        dist = math.hypot(dx, dy)
        step = self.config.speed * self.config.dt
        if dist <= step or dist == 0.0:
            self.robot.position = (gx, gy)
        else:
            self.robot.position = (rx + dx / dist * step, ry + dy / dist * step)
        self.time += self.config.dt

    def decay_targets(self) -> int:
        """Decay every remaining target by one dt and drop expired ones.

        Standard form per step: v <- v * exp(-lambda * dt).
        A target is expired when its value falls below
        expire_threshold * initial_value. Returns how many expired this step.
        """
        expired = 0
        survivors: list[Target] = []
        for t in self.targets:
            t.v = value_remaining(t.v, t.lam, self.config.dt)
            v0 = self.initial_value.get(t.id, t.v)
            if t.v < self.config.expire_threshold * v0:
                expired += 1
            else:
                survivors.append(t)
        self.targets = survivors
        return expired

    def try_serve(self, goal_pos: tuple[float, float]) -> float:
        """Serve a target if the robot has reached one.

        Returns the value collected (the target's current, decayed value), or
        0.0 if no target is within reach_tol of the robot. The served target is
        removed from the list.
        """
        for i, t in enumerate(self.targets):
            if euclidean_distance(self.robot.position, t.position) <= self.config.reach_tol:
                collected = t.v
                del self.targets[i]
                return collected
        return 0.0


def make_scenario(n_targets: int, seed: int, config: WorldConfig | None = None) -> World:
    """Build a reproducible scenario with N targets of mixed decay rates.

    - Robot starts at the origin with the configured speed.
    - Targets are scattered uniformly in a square area.
    - Each target is randomly tagged fast-decaying or slow-decaying, so the
      basket has a mix (this is what separates DART from nearest-first).
    """
    cfg = config if config is not None else WorldConfig()
    rng = random.Random(seed)

    fast_lams = [0.15, 0.25]        # spoil quickly
    slow_lams = [0.01, 0.03]        # spoil slowly
    values = [1.0, 2.0, 3.0, 5.0]   # mixed base values (importance)
    half = cfg.area_half_size

    targets: list[Target] = []
    for k in range(n_targets):
        x = rng.uniform(-half, half)
        y = rng.uniform(-half, half)
        lam = rng.choice(fast_lams if rng.random() < 0.5 else slow_lams)
        v = rng.choice(values)
        targets.append(Target(id=f"T{k}", position=(x, y), v=v, lam=lam))

    robot = RobotState(position=(0.0, 0.0), speed=cfg.speed)
    return World(robot=robot, targets=targets, config=cfg)
