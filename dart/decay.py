"""Value decay + expiry (concept section 9) - pure, ROS-agnostic.

The single source of truth for how a target basket loses value as real time
passes. Both the headless sim (`world.py`) and the ROS2 node call this, so the
robot and the simulator decay identically.

Value convention (same as everywhere in DART): each Target.v holds the target's
CURRENT, already-decayed value. One elapsed interval dt lowers it by

    v <- v * exp(-lambda * dt)

A target is EXPIRED once its current value drops below a fraction of its
original value (snapshotted separately), at which point it is dropped from the
basket - the robot must not keep chasing a dead task.
"""

from __future__ import annotations

from .scoring import Target, value_remaining


def snapshot_initial_values(targets: list[Target]) -> dict[str, float]:
    """Record each target's starting value, keyed by id.

    Call once when the basket is created; expiry is judged against this.
    """
    return {t.id: t.v for t in targets}


def decay_basket(
    targets: list[Target],
    initial_value: dict[str, float],
    dt: float,
    expire_threshold: float = 0.05,
) -> tuple[list[Target], list[Target]]:
    """Decay every target by one interval dt and split off the expired ones.

    Mutates each surviving target's ``v`` in place (v <- v * exp(-lambda * dt)).

    Parameters
    ----------
    targets : list[Target]
        The remaining basket. Not mutated as a list; individual ``v`` fields are.
    initial_value : dict[str, float]
        Starting value per target id (from ``snapshot_initial_values``). A
        target missing here is measured against its own post-decay value (never
        counts as expired on that step).
    dt : float
        Elapsed time since the last decay (seconds). Must be >= 0.
    expire_threshold : float
        Expire when current value < expire_threshold * initial_value.

    Returns
    -------
    (survivors, expired) : tuple[list[Target], list[Target]]
        Targets still alive, and those that expired this step.
    """
    if dt < 0.0:
        raise ValueError("dt must be non-negative")

    survivors: list[Target] = []
    expired: list[Target] = []
    for t in targets:
        t.v = value_remaining(t.v, t.lam, dt)
        v0 = initial_value.get(t.id, t.v)
        if t.v < expire_threshold * v0:
            expired.append(t)
        else:
            survivors.append(t)
    return survivors, expired
