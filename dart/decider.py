"""Decision layer - target selection with anti-oscillation (concept section 5).

Turns the Net scores from `scoring` into a chosen target, and decides when it
is worth abandoning the current target mid-travel (step 7, continuous
re-decision). A switching margin epsilon (hysteresis) prevents flip-flop
between two close targets.
"""

from __future__ import annotations

from dataclasses import dataclass

from .scoring import RobotState, Target


@dataclass
class DeciderParams:
    """Tunable parameters for the decider.

    Attributes
    ----------
    epsilon : float
        Switching margin / hysteresis (fraction, e.g. 0.1). A new target must
        beat the current one's Net by this margin before the robot switches.
    """

    epsilon: float = 0.1


def select_target(all_targets: list[Target], robot: RobotState) -> Target | None:
    """Return the target with the highest Net score (i* = argmax Net_i).

    Returns None when `all_targets` is empty.

    Steps (to implement):
        1. scores = scoring.score_all(all_targets, robot)
        2. pick the target id with the maximum Net score
        3. return the matching Target object
    """
    # TODO: implement argmax over Net scores.
    raise NotImplementedError


def should_switch(
    current: Target,
    candidate: Target,
    robot: RobotState,
    params: DeciderParams,
) -> bool:
    """Return True if the robot should abandon `current` for `candidate`.

    Hysteresis rule (to implement):
        switch only if  Net(candidate) > Net(current) * (1 + epsilon)

    This keeps the robot committed unless the new option is clearly better,
    avoiding rapid target oscillation.
    """
    # TODO: compare Net(candidate) against Net(current) with the epsilon margin.
    raise NotImplementedError


def nearest_target(all_targets: list[Target], robot: RobotState) -> Target | None:
    """Return the closest target by Euclidean distance (baseline policy).

    This is the "nearest-first" baseline that DART is compared against in the
    sim. It ignores value and decay entirely. Returns None when empty.
    """
    # TODO: implement argmin over euclidean_distance to robot.
    raise NotImplementedError
