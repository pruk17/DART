"""Route planner - lookahead over visiting orders.

Replaces greedy one-step selection. Instead of picking the single argmax-Net
target, it SIMULATES decay over multi-leg sequences and chooses the ordering
that collects the most total value, then returns the FIRST target of that order.
Pure Python (no ROS); reuses scoring's decay + distance so the sim and the robot
agree.

Objective (Orienteering with decaying profits, solved online):
    value of an order = sum over targets of their value AT ARRIVAL, where arrival
    time is the cumulative travel time along the order. A target that would decay
    below expire_threshold * initial_value before we reach it yields 0 - so the
    best order naturally skips the ones that cannot be saved (it does not abandon
    unnecessarily, but physics-forced losses remain).

Search (chosen by target count in plan_sequence):
    * N <= full_perm_max : plan_full_permutation - try every order, exact optimum
    * N >  full_perm_max : plan_two_opt - nearest-neighbour start + 2-opt local
                           search (fast, but can stop at a local optimum)
"""

from __future__ import annotations

from dataclasses import dataclass
from itertools import permutations

from .scoring import RobotState, Target, euclidean_distance, value_remaining


@dataclass
class PlannerParams:
    expire_threshold: float = 0.05   # fraction of initial value below which a leg yields 0
    max_2opt_passes: int = 20        # safety cap on local-search sweeps
    full_perm_max: int = 7           # use exhaustive search up to this many targets (7! = 5040)
    service_time: float = 0.0        # seconds the robot waits at each delivery point


def simulate_sequence(
    order: list[Target],
    robot: RobotState,
    initial_value: dict[str, float],
    expire_threshold: float,
    service_time: float = 0.0,
) -> float:
    """Total value collected if the robot visits `order` in sequence.

    Walks a clock: each leg adds its travel time, and the visited target yields
    its value decayed by the cumulative time to reach it (0 if it has fallen
    below expire_threshold * initial_value by then). After each delivery the
    robot waits `service_time`, which delays every later target.
    """
    cur = robot.position
    speed = robot.speed if robot.speed > 0 else 1e-6
    clock = 0.0
    total = 0.0
    for t in order:
        clock += euclidean_distance(cur, t.position) / speed
        arrive_v = value_remaining(t.v, t.lam, clock)
        v0 = initial_value.get(t.id, t.v)
        if arrive_v >= expire_threshold * v0:
            total += arrive_v
        # else: expired before arrival in this order -> contributes 0
        clock += service_time
        cur = t.position
    return total


def _nearest_neighbour(targets: list[Target], robot: RobotState) -> list[Target]:
    """Construction heuristic: repeatedly append the nearest remaining target."""
    remaining = list(targets)
    order: list[Target] = []
    cur = robot.position
    while remaining:
        nxt = min(remaining, key=lambda t: euclidean_distance(cur, t.position))
        order.append(nxt)
        remaining.remove(nxt)
        cur = nxt.position
    return order


def plan_full_permutation(
    targets: list[Target],
    robot: RobotState,
    params: PlannerParams,
    initial_value: dict[str, float],
) -> list[Target]:
    """Exhaustive search: simulate EVERY visiting order and return the best.

    Guaranteed optimum (for the simulation model). Cost grows as N!, so use it
    only for small N (7 targets = 5040 orders).
    """
    if not targets:
        return []
    best_order: list[Target] = []
    best = -1.0
    for perm in permutations(targets):
        score = simulate_sequence(list(perm), robot, initial_value, params.expire_threshold, params.service_time)
        if score > best + 1e-9:
            best_order, best = list(perm), score
    return best_order


def plan_two_opt(
    targets: list[Target],
    robot: RobotState,
    params: PlannerParams,
    initial_value: dict[str, float],
) -> list[Target]:
    """Heuristic: nearest-neighbour start, then 2-opt segment reversals while the
    collected value improves. Fast for many targets, but may stop at a local
    optimum (a good order that is not the best one).
    """
    if not targets:
        return []
    order = _nearest_neighbour(targets, robot)
    best = simulate_sequence(order, robot, initial_value, params.expire_threshold, params.service_time)

    improved = True
    passes = 0
    n = len(order)
    while improved and passes < params.max_2opt_passes:
        improved = False
        passes += 1
        for i in range(n - 1):
            for k in range(i + 1, n):
                candidate = order[:i] + order[i:k + 1][::-1] + order[k + 1:]
                score = simulate_sequence(candidate, robot, initial_value, params.expire_threshold, params.service_time)
                if score > best + 1e-9:
                    order, best = candidate, score
                    improved = True
    return order


def plan_sequence(
    targets: list[Target],
    robot: RobotState,
    params: PlannerParams,
    initial_value: dict[str, float],
) -> list[Target]:
    """Return the best visiting order, picking the search method by target count:
    exhaustive when N <= params.full_perm_max, 2-opt otherwise.
    """
    if len(targets) <= params.full_perm_max:
        return plan_full_permutation(targets, robot, params, initial_value)
    return plan_two_opt(targets, robot, params, initial_value)


def plan_next(
    targets: list[Target],
    robot: RobotState,
    params: PlannerParams,
    initial_value: dict[str, float],
) -> Target | None:
    """Convenience: the first target of the best order (None if no targets)."""
    order = plan_sequence(targets, robot, params, initial_value)
    return order[0] if order else None
