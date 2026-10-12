"""Scoring layer - the DART decision rule (concept section 4).

This module computes the Net score for each candidate target. It is pure
math: no robot I/O, no ROS2. Everything here should be easy to unit-test.

Standard forms used in this module
-----------------------------------
Value of a target after elapsed time tau (exponential decay):

    value_i(tau) = v_i * exp(-lambda_i * tau)

Estimated travel time to target i (Euclidean estimate, NO A*):

    t_i = d_i / s

Net score of target i (the locked DART formula):

    Net_i = v_i * exp(-lambda_i * t_i)                          # gain: value of i left on arrival
          - sum_{j != i} v_j * (1 - exp(-lambda_j * t_i))       # basket-bleed: value others lose while we travel

Selection rule:

    i* = argmax_i Net_i
"""

from __future__ import annotations

from dataclasses import dataclass


@dataclass
class Target:
    """A candidate task the robot may serve.

    Attributes
    ----------
    id : str
        Unique identifier for the target.
    position : tuple[float, float]
        (x, y) coordinate in the map frame (metres).
    v : float
        Base value / importance of the target (default 1.0 for all).
    lam : float
        Decay rate lambda_i per time unit; higher = spoils faster.
        Calibrate from half-life: lambda = ln(2) / t_halflife.
    """

    id: str
    position: tuple[float, float]
    v: float = 1.0
    lam: float = 0.0


@dataclass
class RobotState:
    """Minimal robot state needed for scoring.

    Attributes
    ----------
    position : tuple[float, float]
        Current (x, y) pose in the map frame (metres).
    speed : float
        Nominal speed s (m/s) used to estimate travel time.
    """

    position: tuple[float, float]
    speed: float


def euclidean_distance(a: tuple[float, float], b: tuple[float, float]) -> float:
    """Return the straight-line distance d between points a and b.

    Standard form: d = sqrt((x_a - x_b)^2 + (y_a - y_b)^2)
    """
    # TODO: implement Euclidean distance.
    raise NotImplementedError


def travel_time(distance: float, speed: float) -> float:
    """Return estimated travel time t_i = d_i / s.

    No A*; this is a straight-line time estimate only.
    """
    # TODO: implement t_i = distance / speed (guard speed > 0).
    raise NotImplementedError


def value_remaining(v: float, lam: float, tau: float) -> float:
    """Return value_i(tau) = v * exp(-lambda * tau).

    The amount of a target's value still collectable after elapsed time tau.
    """
    # TODO: implement v * exp(-lam * tau).
    raise NotImplementedError


def basket_bleed(target: Target, others: list[Target], t_i: float) -> float:
    """Return the opportunity-cost term for travelling to `target`.

    Standard form:
        sum_{j != i} v_j * (1 - exp(-lambda_j * t_i))

    This is how much every OTHER target decays during the trip of length t_i.
    `others` must already exclude `target`.
    """
    # TODO: sum value lost by each other target over t_i.
    raise NotImplementedError


def net_score(target: Target, all_targets: list[Target], robot: RobotState) -> float:
    """Return Net_i for one target against the whole remaining basket.

    Standard form:
        Net_i = v_i * exp(-lambda_i * t_i)
              - sum_{j != i} v_j * (1 - exp(-lambda_j * t_i))

    Steps (to implement):
        1. d_i   = euclidean_distance(robot.position, target.position)
        2. t_i   = travel_time(d_i, robot.speed)
        3. gain  = value_remaining(target.v, target.lam, t_i)
        4. bleed = basket_bleed(target, others, t_i)   # others = all_targets minus target
        5. return gain - bleed
    """
    # TODO: implement using the helpers above.
    raise NotImplementedError


def score_all(all_targets: list[Target], robot: RobotState) -> dict[str, float]:
    """Return {target_id: Net_i} for every target in `all_targets`."""
    # TODO: call net_score for each target and collect into a dict.
    raise NotImplementedError
