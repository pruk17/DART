"""Run DART vs nearest-first on the same scenario and report the difference.

This is the first test to build (concept section 12): it validates that the
decision rule actually saves more value than the naive baseline, before any
hardware is involved.

Usage (once implemented):
    python -m dart.sim.run_comparison --targets 20 --seed 0
"""

from __future__ import annotations

from ..decider import nearest_target, select_target
from ..scoring import RobotState
from .world import SimResult, World, make_scenario


def run_policy(world: World, policy: str) -> SimResult:
    """Run one policy to completion on `world` and return its SimResult.

    Parameters
    ----------
    world : World
        A fresh scenario (do not share one World between two runs).
    policy : str
        "dart"          -> decider.select_target (Net-score argmax)
        "nearest_first" -> decider.nearest_target (distance argmin)

    Loop (to implement), mirroring concept section 5:
        while not world.is_done():
            1. choose goal via the selected policy
            2. step robot toward goal (world.step_robot_toward)
            3. decay all targets (world.decay_targets)
            4. try to serve the goal (world.try_serve) and accumulate value
            5. (DART) optionally re-decide / switch target each step
        return the collected SimResult
    """
    # TODO: implement the per-policy run loop.
    raise NotImplementedError


def compare(n_targets: int, seed: int) -> dict[str, SimResult]:
    """Run both policies on identical scenarios and return their results.

    Both runs must use the SAME scenario (same make_scenario seed) so the
    comparison is fair.
    """
    # TODO: build two identical worlds, run each policy, return both results.
    raise NotImplementedError


def print_report(results: dict[str, SimResult]) -> None:
    """Print a short side-by-side report: total value + expired count."""
    # TODO: format results["dart"] vs results["nearest_first"].
    raise NotImplementedError


def main() -> None:
    """CLI entry point: parse args, run compare, print the report."""
    # TODO: argparse (--targets, --seed); call compare + print_report.
    raise NotImplementedError


if __name__ == "__main__":
    main()
