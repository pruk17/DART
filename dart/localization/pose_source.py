"""Pose source abstraction (concept section 6, localization).

The core DART code only needs "where is the robot now". This interface hides
whether that answer comes from a sim's ground truth or from AMCL on hardware,
so `scoring`, `decider`, and `navigator` do not change between sim and robot.
"""

from __future__ import annotations

from abc import ABC, abstractmethod


class PoseSource(ABC):
    """Abstract source of the robot's current pose in the map frame."""

    @abstractmethod
    def get_pose(self) -> tuple[float, float]:
        """Return the current (x, y) position in metres."""
        raise NotImplementedError


class SimPoseSource(PoseSource):
    """Ground-truth pose for the headless sim.

    The sim world owns the true robot position and reports it directly - no
    noise, no scan-matching.
    """

    def __init__(self, initial_pos: tuple[float, float]) -> None:
        self._pos = initial_pos

    def set_pose(self, pos: tuple[float, float]) -> None:
        """Update the stored ground-truth pose (called by the sim each step)."""
        self._pos = pos

    def get_pose(self) -> tuple[float, float]:
        return self._pos


class AmclPoseSource(PoseSource):
    """AMCL-backed pose for TurtleBot (ROS2) - stub for later.

    On hardware this will subscribe to /amcl_pose and return the latest
    estimate. Not implemented in the pure-Python package.
    """

    def get_pose(self) -> tuple[float, float]:
        # TODO: read latest AMCL pose when the ROS2 layer is added.
        raise NotImplementedError
