"""ROS2-backed pose sources.

Implements the `PoseSource` interface from the DART core so the scoring/decider
code is unchanged between sim and hardware. Two concrete sources:

* OdomPoseSource  - subscribes /odom (good for Gazebo, no map needed).
* AmclPoseSource  - map frame: smoothed AMCL map->odom + live odom (real or sim).

Both also expose the latest yaw, which the Bug2 navigator needs for heading
control (the core PoseSource only promises (x, y)).
"""

from __future__ import annotations

import math

from nav_msgs.msg import Odometry
from rclpy.time import Time
from tf2_ros import Buffer, TransformListener, TransformException

from dart.localization.pose_source import PoseSource


def _yaw_from_quat(q) -> float:
    """Yaw (rad) from a geometry_msgs quaternion."""
    siny = 2.0 * (q.w * q.z + q.x * q.y)
    cosy = 1.0 - 2.0 * (q.y * q.y + q.z * q.z)
    return math.atan2(siny, cosy)


class _RosPoseSourceBase(PoseSource):
    def __init__(self) -> None:
        self._pos: tuple[float, float] | None = None
        self._yaw: float = 0.0
        self._clock = None
        self._fresh_ns: int | None = None   # local clock time of the last fresh pose data

    def age(self) -> float:
        """Seconds since fresh pose data last arrived (math.inf if never).

        Measured on this PC's clock, not message stamps, so a robot/PC clock
        offset cannot hide stale data.
        """
        if self._fresh_ns is None or self._clock is None:
            return math.inf
        return (self._clock.now().nanoseconds - self._fresh_ns) * 1e-9

    def ready(self) -> bool:
        return self._pos is not None

    def get_pose(self) -> tuple[float, float]:
        if self._pos is None:
            raise RuntimeError("pose not received yet; check the topic is publishing")
        return self._pos

    def get_yaw(self) -> float:
        return self._yaw


class OdomPoseSource(_RosPoseSourceBase):
    """Pose from /odom — used in Gazebo and odometry-only setups."""

    def __init__(self, node, topic: str = "/odom") -> None:
        super().__init__()
        self._clock = node.get_clock()
        self._sub = node.create_subscription(Odometry, topic, self._cb, 10)

    def _cb(self, msg: Odometry) -> None:
        self._fresh_ns = self._clock.now().nanoseconds
        p = msg.pose.pose.position
        self._pos = (p.x, p.y)
        self._yaw = _yaw_from_quat(msg.pose.pose.orientation)


class AmclPoseSource(_RosPoseSourceBase):
    """Pose in the MAP frame = smoothed AMCL correction (``map -> odom``)
    composed with live odometry (``odom -> base_footprint``).

    Uses TF rather than the ``/amcl_pose`` topic on purpose: AMCL publishes that
    topic only on motion, which deadlocks a stationary robot. Works the same in
    sim and on hardware.

    Smoothing: AMCL applies its corrections as jumps (a few cm / degrees each
    update). Fed straight to the heading controller they make the robot weave
    left-right, most visibly near a goal. Odometry is continuous, so only the
    correction is low-pass filtered (time constant ``smoothing_tau``): the pose
    stays as smooth as odometry and still converges to AMCL within ~1 s. A
    correction larger than ``snap_dist`` / ``snap_yaw`` (a re-localization,
    e.g. RViz 2D Pose Estimate) is applied at once.

    Freshness: age() follows odometry (the control input). If AMCL stops
    updating ``map -> odom`` for longer than ``amcl_timeout`` the pose is
    reported stale too, since it is then dead reckoning only.
    """

    def __init__(self, node, map_frame: str = "map", base_frame: str = "base_footprint",
                 odom_frame: str = "odom", smoothing_tau: float = 0.7,
                 snap_dist: float = 0.30, snap_yaw: float = 0.35,
                 amcl_timeout: float = 3.0) -> None:
        super().__init__()
        self._clock = node.get_clock()
        self._map_frame = map_frame
        self._odom_frame = odom_frame
        self._base_frame = base_frame
        self._tau = smoothing_tau
        self._snap_dist = snap_dist
        self._snap_yaw = snap_yaw
        self._amcl_timeout = amcl_timeout
        self._mo: tuple[float, float, float] | None = None   # filtered map->odom (x, y, yaw)
        self._mo_ns: int | None = None                        # when the filter last ran
        self._mo_stamp: tuple[int, int] | None = None
        self._mo_fresh_ns: int | None = None                  # when AMCL last published a new map->odom
        self._odom_stamp: tuple[int, int] | None = None
        self._buffer = Buffer()
        self._listener = TransformListener(self._buffer, node)

    def _filter_map_odom(self, raw: tuple[float, float, float], now_ns: int) -> None:
        if self._mo is None:
            self._mo, self._mo_ns = raw, now_ns
            return
        dx, dy = raw[0] - self._mo[0], raw[1] - self._mo[1]
        dyaw = math.atan2(math.sin(raw[2] - self._mo[2]), math.cos(raw[2] - self._mo[2]))
        dt = max(0.0, (now_ns - self._mo_ns) * 1e-9)  # type: ignore[operator]
        self._mo_ns = now_ns
        if math.hypot(dx, dy) > self._snap_dist or abs(dyaw) > self._snap_yaw or self._tau <= 0.0:
            self._mo = raw   # re-localization: take it at once
            return
        a = 1.0 - math.exp(-dt / self._tau)
        self._mo = (self._mo[0] + a * dx, self._mo[1] + a * dy, self._mo[2] + a * dyaw)

    def _refresh(self) -> None:
        try:
            mo = self._buffer.lookup_transform(self._map_frame, self._odom_frame, Time())
            ob = self._buffer.lookup_transform(self._odom_frame, self._base_frame, Time())
        except TransformException:
            return  # not available (yet / any more): age() keeps growing
        now_ns = self._clock.now().nanoseconds
        mo_stamp = (mo.header.stamp.sec, mo.header.stamp.nanosec)
        if mo_stamp != self._mo_stamp:
            self._mo_stamp, self._mo_fresh_ns = mo_stamp, now_ns
        ob_stamp = (ob.header.stamp.sec, ob.header.stamp.nanosec)
        if ob_stamp != self._odom_stamp:
            self._odom_stamp, self._fresh_ns = ob_stamp, now_ns

        t = mo.transform.translation
        self._filter_map_odom((t.x, t.y, _yaw_from_quat(mo.transform.rotation)), now_ns)
        mx, my, myaw = self._mo  # type: ignore[misc]
        o = ob.transform.translation
        oyaw = _yaw_from_quat(ob.transform.rotation)
        self._pos = (mx + o.x * math.cos(myaw) - o.y * math.sin(myaw),
                     my + o.x * math.sin(myaw) + o.y * math.cos(myaw))
        self._yaw = math.atan2(math.sin(myaw + oyaw), math.cos(myaw + oyaw))

    def age(self) -> float:
        odom_age = super().age()
        if self._mo_fresh_ns is None:
            return math.inf
        amcl_age = (self._clock.now().nanoseconds - self._mo_fresh_ns) * 1e-9
        # dead reckoning alone is fine for a moment, not for long
        return odom_age if amcl_age <= self._amcl_timeout else max(odom_age, amcl_age)

    def ready(self) -> bool:
        self._refresh()
        return self._pos is not None
