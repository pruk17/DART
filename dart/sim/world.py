"""Headless 2D world for validating DART (no ROS2, no rendering).

The world owns the true robot position and the list of remaining targets,
advances time in fixed steps, decays target values, and serves targets the
robot reaches. It knows nothing about which policy is driving - it just
executes motion and bookkeeping so DART and nearest-first can be compared
fairly on the same scenario.
"""

from __future__ import annotations

from dataclasses import dataclass, field

from ..scoring import RobotState, Target


@dataclass
class WorldConfig:
    """Scenario + simulation settings."""

    dt: float = 0.1                 # simulation timestep (s).
    speed: float = 0.22             # robot speed (m/s).
    expire_threshold: float = 0.05  # value fraction below which a target is "expired".
    max_time: float = 300.0         # safety cap on total sim time (s).


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
    """

    robot: RobotState
    targets: list[Target]
    config: WorldConfig = field(default_factory=WorldConfig)
    time: float = 0.0

    def is_done(self) -> bool:
        """Return True when no targets remain or max_time is exceeded."""
        # TODO: return not self.targets or self.time >= self.config.max_time.
        raise NotImplementedError

    def step_robot_toward(self, goal_pos: tuple[float, float]) -> None:
        """Move the robot one dt toward goal_pos and advance the clock.

        For the sim we use a straight-line move (no obstacles in the first
        scenario). Obstacles / Bug2 can be layered in later via navigator.
        """
        # TODO: advance robot position by speed * dt, update self.time.
        raise NotImplementedError

    def decay_targets(self) -> None:
        """Drop the value of every remaining target by one dt of decay.

        Uses value_i(tau) with tau = dt applied each step. Targets whose value
        fraction falls below expire_threshold are removed and counted expired.
        """
        # TODO: apply exponential decay per step; flag expirations.
        raise NotImplementedError

    def try_serve(self, goal_pos: tuple[float, float]) -> float:
        """Serve the target at goal_pos if the robot reached it.

        Returns the value actually collected (0.0 if not reached). Removes the
        served target from the list.
        """
        # TODO: if at goal_pos within tol, collect value and remove target.
        raise NotImplementedError


def make_scenario(n_targets: int, seed: int) -> World:
    """Build a reproducible scenario with N targets of mixed decay rates.

    Steps (to implement):
        1. seed an RNG.
        2. scatter n_targets positions in a bounded area.
        3. assign mixed (v, lam) - some fast-decaying, some slow.
        4. place the robot at a fixed start.
        5. return a World.
    """
    # TODO: implement scenario generation.
    raise NotImplementedError
