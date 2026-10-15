"""Decision layer - target selection with anti-oscillation (concept section 5).

Turns the Net scores from `scoring` into a chosen target, and decides when it
is worth abandoning the current target mid-travel (step 7, continuous
re-decision). A switching margin epsilon (hysteresis) prevents flip-flop
between two close targets.
"""

from __future__ import annotations

from dataclasses import dataclass

from .scoring import RobotState, Target, euclidean_distance, net_score


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
    """
    if not all_targets:
        return None
    return max(all_targets, key=lambda t: net_score(t, all_targets, robot))


def should_switch(
    current: Target,
    candidate: Target,
    all_targets: list[Target],
    robot: RobotState,
    params: DeciderParams,
) -> bool:
    """Return True if the robot should abandon `current` for `candidate`.

    Hysteresis rule (additive margin, robust to negative Net):

        switch only if  Net(candidate) - Net(current) > epsilon * |Net(current)|

    The candidate must beat the current target by a margin of epsilon times the
    current score's magnitude. This keeps the robot committed unless the new
    option is clearly better, avoiding rapid target oscillation (flip-flop).

    Note: `all_targets` is needed because net_score includes the basket-bleed
    term, which sums over the other targets.
    """
    if candidate is current:
        return False
    net_current = net_score(current, all_targets, robot)
    net_candidate = net_score(candidate, all_targets, robot)
    margin = params.epsilon * abs(net_current)
    return (net_candidate - net_current) > margin


def nearest_target(all_targets: list[Target], robot: RobotState) -> Target | None:
    """Return the closest target by Euclidean distance (baseline policy).

    This is the "nearest-first" baseline that DART is compared against in the
    sim. It ignores value and decay entirely. Returns None when empty.
    """
    if not all_targets:
        return None
    return min(
        all_targets,
        key=lambda t: euclidean_distance(robot.position, t.position),
    )
