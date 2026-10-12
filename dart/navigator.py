"""Navigation layer - reactive modified Bug2 (concept section 6, steps 5-6).

Drives the robot toward the currently chosen target along the m-line, and
follows obstacle boundaries when the LiDAR detects something. No global map /
no A*: this layer is purely reactive. The value-aware target switching lives
in `decider`; this module only executes motion for whatever goal it is given.

Standard Bug2 idea
------------------
    m-line   = straight line from start position to goal position
    hit point   = where the robot first touches an obstacle
    leave point = where it re-crosses the m-line closer to the goal
    rule        = follow the boundary until a leave point, then rejoin m-line
"""

from __future__ import annotations

from dataclasses import dataclass
from enum import Enum, auto


class NavMode(Enum):
    """Reactive navigation mode."""

    GO_TO_GOAL = auto()       # moving along the m-line toward the goal
    WALL_FOLLOW = auto()      # circumnavigating an obstacle boundary


@dataclass
class MLine:
    """Straight start-goal line used by Bug2.

    Attributes
    ----------
    start : tuple[float, float]
        Position where the current m-line was defined.
    goal : tuple[float, float]
        Target position the m-line points at.
    """

    start: tuple[float, float]
    goal: tuple[float, float]


def make_mline(robot_pos: tuple[float, float], goal_pos: tuple[float, float]) -> MLine:
    """Create a fresh m-line from the robot to the goal.

    Called whenever the target changes (step 4 / step 7 redefines the m-line).
    """
    # TODO: build and return an MLine.
    raise NotImplementedError


def at_goal(robot_pos: tuple[float, float], goal_pos: tuple[float, float], tol: float) -> bool:
    """Return True if the robot is within `tol` metres of the goal."""
    # TODO: distance(robot_pos, goal_pos) <= tol.
    raise NotImplementedError


def detect_obstacle(scan: object, safe_distance: float) -> bool:
    """Return True if the LiDAR scan shows an obstacle within safe_distance.

    `scan` is a placeholder for the sensor reading (e.g. list of ranges or a
    ROS2 LaserScan later). Works for static OR moving obstacles - they are
    simply sensed and avoided each cycle.
    """
    # TODO: inspect scan ranges against safe_distance.
    raise NotImplementedError


def go_to_goal_step(robot_pos: tuple[float, float], mline: MLine, speed: float, dt: float) -> tuple[float, float]:
    """Advance one timestep along the m-line toward the goal.

    Returns the new robot position after moving `speed * dt` along the m-line.
    """
    # TODO: move along the m-line direction.
    raise NotImplementedError


def wall_follow_step(robot_pos: tuple[float, float], scan: object, speed: float, dt: float) -> tuple[float, float]:
    """Advance one timestep following the obstacle boundary.

    Keeps the standard Bug2 leave rule unchanged: leave the wall when the
    robot re-crosses the m-line closer to the goal (checked by the caller via
    `crossed_mline_closer`).
    """
    # TODO: follow boundary (left or right) by one step.
    raise NotImplementedError


def crossed_mline_closer(
    robot_pos: tuple[float, float],
    mline: MLine,
    hit_distance_to_goal: float,
) -> bool:
    """Return True if the robot re-crossed the m-line nearer the goal than the hit point.

    This is the Bug2 leave condition: only then is it safe to stop wall
    following and resume GO_TO_GOAL.
    """
    # TODO: test m-line membership AND distance-to-goal < hit_distance_to_goal.
    raise NotImplementedError


class ReactiveNavigator:
    """Stateful reactive Bug2 driver for a single moving goal.

    Holds the current NavMode, the active m-line, and the hit-point distance
    while wall following. The outer online loop (decider) may call
    `set_goal` at any time to switch targets; this navigator just re-plans the
    m-line and keeps driving reactively.
    """

    def __init__(self, goal_tol: float, safe_distance: float) -> None:
        self.mode: NavMode = NavMode.GO_TO_GOAL
        self.mline: MLine | None = None
        self.goal_tol = goal_tol
        self.safe_distance = safe_distance
        self._hit_distance_to_goal: float | None = None
        # TODO: any extra state needed for wall following.

    def set_goal(self, robot_pos: tuple[float, float], goal_pos: tuple[float, float]) -> None:
        """Redefine the m-line for a new goal (called on target switch)."""
        # TODO: self.mline = make_mline(...); reset mode to GO_TO_GOAL.
        raise NotImplementedError

    def step(self, robot_pos: tuple[float, float], scan: object, speed: float, dt: float) -> tuple[float, float]:
        """Run one reactive navigation cycle; return the new robot position.

        Sketch (to implement):
            - if detect_obstacle -> enter WALL_FOLLOW, record hit distance
            - if WALL_FOLLOW and crossed_mline_closer -> back to GO_TO_GOAL
            - dispatch to go_to_goal_step or wall_follow_step by self.mode
        """
        # TODO: implement the reactive state machine.
        raise NotImplementedError
