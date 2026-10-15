"""Run DART vs nearest-first on the same scenario and report the difference.

This is the first test to build (concept section 12): it validates that the
decision rule actually saves more value than the naive baseline, before any
hardware is involved.

Usage:
    python -m dart.sim.run_comparison --targets 20 --seed 0
"""

from __future__ import annotations

import argparse

from ..decider import DeciderParams, nearest_target, select_target, should_switch
from .world import SimResult, World, make_scenario


def run_policy(world: World, policy: str, params: DeciderParams | None = None) -> SimResult:
    """Run one policy to completion on `world` and return its SimResult.

    policy:
        "dart"          -> select_target (Net-score argmax) + hysteresis switching
        "nearest_first" -> nearest_target (distance argmin)

    One cycle mirrors concept section 5: pick a goal, step toward it, decay all
    targets, then try to serve. DART keeps a committed target and only switches
    when should_switch clears the epsilon margin (anti-oscillation).
    """
    p = params if params is not None else DeciderParams()
    result = SimResult()
    current: object = None  # the committed target (DART) / current pick (nearest)

    while not world.is_done():
        if policy == "dart":
            candidate = select_target(world.targets, world.robot)
            if current is None or current not in world.targets:
                current = candidate
            elif candidate is not current and should_switch(
                current, candidate, world.targets, world.robot, p
            ):
                current = candidate
        else:  # nearest_first
            current = nearest_target(world.targets, world.robot)

        if current is None:
            break

        goal = current.position
        world.step_robot_toward(goal)
        result.expired_count += world.decay_targets()
        collected = world.try_serve(goal)
        if collected > 0.0:
            result.total_value_collected += collected
            result.served_count += 1
            current = None  # need a fresh target next cycle

    result.finish_time = world.time
    return result


def compare(n_targets: int, seed: int) -> dict[str, SimResult]:
    """Run both policies on identical scenarios and return their results.

    Each policy gets its OWN fresh World built from the same seed, so they face
    exactly the same target layout and decay rates.
    """
    return {
        "dart": run_policy(make_scenario(n_targets, seed), "dart"),
        "nearest_first": run_policy(make_scenario(n_targets, seed), "nearest_first"),
    }


def print_report(results: dict[str, SimResult]) -> None:
    """Print a short side-by-side report: value collected, served, expired."""
    d = results["dart"]
    n = results["nearest_first"]
    print(f"{'metric':<22}{'DART':>12}{'nearest-first':>16}")
    print("-" * 50)
    print(f"{'value collected':<22}{d.total_value_collected:>12.3f}{n.total_value_collected:>16.3f}")
    print(f"{'targets served':<22}{d.served_count:>12d}{n.served_count:>16d}")
    print(f"{'targets expired':<22}{d.expired_count:>12d}{n.expired_count:>16d}")
    print(f"{'finish time (s)':<22}{d.finish_time:>12.1f}{n.finish_time:>16.1f}")


def main() -> None:
    """CLI entry point: parse args, run compare, print the report."""
    parser = argparse.ArgumentParser(description="Compare DART vs nearest-first.")
    parser.add_argument("--targets", type=int, default=20, help="number of targets.")
    parser.add_argument("--seed", type=int, default=0, help="scenario RNG seed.")
    args = parser.parse_args()

    results = compare(args.targets, args.seed)
    print_report(results)


if __name__ == "__main__":
    main()
