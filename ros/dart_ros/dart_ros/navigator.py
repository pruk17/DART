"""Reactive navigator, ROS2 flavour — standard Bug2 (DART_concept.md section 6, 8.1).

The DART core `navigator.py` integrates positions for the headless sim (returns
a new (x, y)). On a real robot / Gazebo we command velocities instead, so this
is a ROS-oriented navigator that produces (linear, angular) velocity from the
current pose + LaserScan.

Per the concept, the wall-follow is **standard Bug2, leave rule kept unchanged**:
follow the obstacle boundary until the robot re-crosses the m-line closer to the
goal than the hit point. The DART "modification" (value-aware target switching
that redraws the m-line mid-travel) lives in the decider/node layer, NOT here —
this navigator just drives whatever goal it is given.

    m-line     = straight line from where set_goal() was called to the goal
    hit point  = where the robot first touches an obstacle (we store its dist-to-goal)
    follow     = the boundary of THAT obstacle (tracked point to point), not
                 whatever happens to be nearest in the scan
    leave      = re-cross m-line AND nearer the goal than the hit point AND the
                 way to the goal is free ("able to move towards the goal")
    give up    = back at the hit point after going round -> goal unreachable
"""

from __future__ import annotations

import math
from dataclasses import dataclass
from enum import Enum, auto


class NavMode(Enum):
    GO_TO_GOAL = auto()
    WALL_FOLLOW = auto()


@dataclass
class MLine:
    start: tuple[float, float]
    goal: tuple[float, float]


def _angle_wrap(a: float) -> float:
    """Wrap an angle to (-pi, pi]."""
    return math.atan2(math.sin(a), math.cos(a))


class ReactiveNavigatorROS:
    """Drives one goal at a time via standard Bug2; outputs (linear, angular) vel.

    Parameters
    ----------
    max_lin, max_ang : float
        Velocity caps (TurtleBot3 Burger: ~0.22 m/s, ~2.84 rad/s).
    goal_tol : float
        Distance (m) at which the goal counts as reached.
    safe_distance : float
        LiDAR distance (m) that triggers / maintains wall following.
    wall_side : str
        Default side to keep the obstacle on ("right" or "left") when both
        ways round look equally open; otherwise the side is chosen per hit.
    """

    def __init__(
        self,
        max_lin: float,
        max_ang: float,
        goal_tol: float,
        safe_distance: float,
        front_arc_deg: float = 30.0,
        wall_side: str = "right",
        leave_margin: float = 0.05,
        follow_range: float = 1.5,
        lookahead: float = 0.15,
        wall_offset: float = 0.25,
        body_clear: float = 0.19,
        track_gate: float = 0.15,
        hit_return_radius: float = 0.15,
        hit_depart_radius: float = 0.30,
        wall_follow_speed: float = 1.0,
        half_width: float = 0.14,
    ) -> None:
        self.max_lin = max_lin
        self.max_ang = max_ang
        self.goal_tol = goal_tol
        self.safe_distance = safe_distance     # trigger: start avoiding when this close ahead
        # how close to hug the wall WHILE following (< safe_distance -> tighter orbit)
        self.wall_offset = min(wall_offset, safe_distance)
        self.front_arc_deg = front_arc_deg
        self.wall_side = wall_side.lower()
        self.leave_margin = leave_margin
        self.follow_range = follow_range      # only hug obstacle points within this range
        self.lookahead = lookahead            # pure-pursuit lookahead along the boundary (m)
        # anything closer than this (LiDAR centre; Burger radius 0.105 + margin)
        # is never driven toward, and always counts as part of the followed
        # boundary (a gap the robot cannot pass merges the two obstacles)
        self.body_clear = body_clear
        self.track_gate = track_gate          # max jump (m) of the tracked boundary point per cycle
        self.hit_return_radius = hit_return_radius  # back within this of the hit point -> give up
        self.hit_depart_radius = hit_depart_radius  # ...but only after having gone this far from it
        self.wall_follow_speed = wall_follow_speed  # fraction of max_lin used while wall-following
        self.half_width = half_width          # corridor swept when driving straight (Burger 0.105 + margin)

        self.mode = NavMode.GO_TO_GOAL
        self.mline: MLine | None = None
        self._goal: tuple[float, float] | None = None
        self._hit_distance_to_goal: float | None = None
        self._prev_signed: float | None = None  # signed m-line distance last cycle (crossing test)
        # boundary point being followed, in the ROBOT frame (x ahead, y left):
        # tracked scan-to-scan, so map-frame pose corrections can't break it
        self._track_pt: tuple[float, float] | None = None
        self._blocking_pt: tuple[float, float] | None = None  # obstacle point on the way to the goal (robot frame)
        self._side = self.wall_side           # side the obstacle is kept on, chosen at each hit
        self._hit_pos: tuple[float, float] | None = None
        self._departed_hit = False
        self.unreachable = False              # Bug2 went round and came back to its hit point

    # ------------------------------------------------------------------ goal
    def set_goal(self, robot_pos: tuple[float, float], goal_pos: tuple[float, float]) -> None:
        """Redraw the m-line for a new goal and reset to GO_TO_GOAL.

        Called when the committed target changes (step 4 / step 7 switch), with
        the robot's current position as the m-line start.
        """
        self._goal = goal_pos
        self.mline = MLine(start=robot_pos, goal=goal_pos)
        self.mode = NavMode.GO_TO_GOAL
        self._hit_distance_to_goal = None
        self._prev_signed = None
        self._track_pt = None
        self._hit_pos = None
        self._departed_hit = False
        self.unreachable = False
        self._side = self.wall_side

    def at_goal(self, pos: tuple[float, float]) -> bool:
        if self._goal is None:
            return False
        return math.hypot(pos[0] - self._goal[0], pos[1] - self._goal[1]) <= self.goal_tol

    # ---------------------------------------------------------------- sensing
    def _sector_nearest(
        self,
        ranges: list[float],
        angle_min: float,
        angle_inc: float,
        center_deg: float,
        width_deg: float,
    ) -> tuple[float, float]:
        """(range, bearing) of the nearest valid reading in a sector centred on
        center_deg; (math.inf, 0.0) if the sector has no valid readings.
        """
        if not ranges or angle_inc == 0.0:
            return (math.inf, 0.0)
        center = math.radians(center_deg)
        half = math.radians(width_deg) / 2.0
        best, best_a = math.inf, 0.0
        for i, r in enumerate(ranges):
            if r is None or math.isinf(r) or math.isnan(r) or r <= 0.0:
                continue
            ang = _angle_wrap(angle_min + i * angle_inc)
            if abs(_angle_wrap(ang - center)) <= half and r < best:
                best, best_a = r, ang
        return (best, best_a)

    def _sector_min(self, ranges, angle_min, angle_inc, center_deg, width_deg) -> float:
        """Minimum valid range within a sector (math.inf if none)."""
        return self._sector_nearest(ranges, angle_min, angle_inc, center_deg, width_deg)[0]

    def _goal_blocked(self, pos, yaw, ranges, angle_min, angle_inc) -> bool:
        """Bug2 'hit an obstacle': something lies on the way TO THE GOAL.

        Checked in the direction of the goal (not the robot's current heading)
        and only counts if it is closer than the goal itself. A wall the robot
        merely faces while turning toward a goal behind it -- or a wall beyond
        the goal -- is not an obstacle on the path, so it must not trigger
        wall-following (textbook: "move toward the goal; if you hit an obstacle").
        """
        gx, gy = self._goal  # type: ignore[misc]
        goal_dist = math.hypot(gx - pos[0], gy - pos[1])
        bearing = _angle_wrap(math.atan2(gy - pos[1], gx - pos[0]) - yaw)
        d, a = self._sector_nearest(ranges, angle_min, angle_inc, math.degrees(bearing), self.front_arc_deg)
        self._blocking_pt = (d * math.cos(a), d * math.sin(a)) if not math.isinf(d) else None
        return d < self.safe_distance and d < goal_dist

    def front_clearance(self, ranges, angle_min, angle_inc) -> float:
        """Nearest obstacle distance in the front arc (math.inf if clear). For debug."""
        return self._sector_min(ranges, angle_min, angle_inc, 0.0, self.front_arc_deg)

    def _body_blocked_toward_goal(self, pos, yaw, ranges, angle_min, angle_inc) -> bool:
        """Facing the goal but something within body_clear in the forward arc
        stops the robot: it cannot move toward the goal, i.e. it has hit an
        obstacle. Catches obstacles inside the LiDAR's minimum range dead
        ahead (LDS-02: 0.16 m), which _goal_blocked cannot see."""
        gx, gy = self._goal  # type: ignore[misc]
        err = _angle_wrap(math.atan2(gy - pos[1], gx - pos[0]) - yaw)
        if abs(err) > math.radians(60.0):
            return False   # still turning toward the goal; the guard just pauses forward motion
        p = self._in_corridor(ranges, angle_min, angle_inc)
        if p is not None:
            self._blocking_pt = p
            return True
        return False

    def _choose_side(self, pos, yaw, ranges, angle_min, angle_inc) -> str:
        """Which way round to go (Bug2: "follow the boundary, moving either
        left or right"). Go round on the side that is more open beside the
        goal direction, so a gap the robot cannot pass (e.g. the obstacle near
        a wall) is not entered and the whole room boundary is not followed.
        Going round to the LEFT keeps the obstacle on the RIGHT.
        """
        gx, gy = self._goal  # type: ignore[misc]
        gb = math.degrees(_angle_wrap(math.atan2(gy - pos[1], gx - pos[0]) - yaw))
        left = min(self._sector_min(ranges, angle_min, angle_inc, gb + 70.0, 80.0), self.follow_range)
        right = min(self._sector_min(ranges, angle_min, angle_inc, gb - 70.0, 80.0), self.follow_range)
        if abs(left - right) < 0.05:
            return self.wall_side
        return "right" if left > right else "left"

    def _corridor_points(self, ranges, angle_min, angle_inc):
        """Points (robot frame) that driving straight on would hit: less than
        body_clear ahead and within half_width of the centre line (the strip
        the body sweeps). Points off to the side are not in the way, however
        close they are."""
        pts = []
        for i, r in enumerate(ranges):
            if r is None or math.isinf(r) or math.isnan(r) or r <= 0.0 or r >= self.body_clear:
                continue
            a = angle_min + i * angle_inc
            x, y = r * math.cos(a), r * math.sin(a)
            if x > 0.0 and abs(y) < self.half_width:
                pts.append((x, y))
        return pts

    def _in_corridor(self, ranges, angle_min, angle_inc):
        """Nearest point in the swept strip, or None if it is clear."""
        pts = self._corridor_points(ranges, angle_min, angle_inc)
        return min(pts) if pts else None

    def _body_blocked(self, ranges, angle_min, angle_inc) -> bool:
        """Driving straight on would sweep the robot's body into something."""
        return bool(self._corridor_points(ranges, angle_min, angle_inc))

    # ------------------------------------------------------------------- step
    def step(
        self,
        pos: tuple[float, float],
        yaw: float,
        scan_ranges: list[float],
        angle_min: float,
        angle_inc: float,
    ) -> tuple[float, float]:
        """Run one Bug2 cycle; return (linear, angular) velocity."""
        if self._goal is None or self.at_goal(pos) or self.unreachable:
            return (0.0, 0.0)

        if self.mode == NavMode.GO_TO_GOAL:
            if (self._goal_blocked(pos, yaw, scan_ranges, angle_min, angle_inc)
                    or self._body_blocked_toward_goal(pos, yaw, scan_ranges, angle_min, angle_inc)):
                # Hit an obstacle (Bug2 line 3): record the hit point's distance
                # to the goal and start following the boundary.
                self.mode = NavMode.WALL_FOLLOW
                self._hit_distance_to_goal = math.hypot(
                    pos[0] - self._goal[0], pos[1] - self._goal[1]
                )
                self._prev_signed = self._signed_mline_distance(pos)
                self._hit_pos = pos
                self._departed_hit = False
                self._track_pt = self._blocking_pt   # follow the obstacle that blocked us
                self._side = self._choose_side(pos, yaw, scan_ranges, angle_min, angle_inc)
                return self._wall_follow(pos, yaw, scan_ranges, angle_min, angle_inc)
            lin, ang = self._go_to_goal(pos, yaw)
            # Collision guard while still turning toward the goal: never drive
            # the body into something in its way.
            if self._body_blocked(scan_ranges, angle_min, angle_inc):
                lin = 0.0
            return (lin, ang)

        # WALL_FOLLOW. Give up if we went round and are back at the hit point.
        d_hit = math.hypot(pos[0] - self._hit_pos[0], pos[1] - self._hit_pos[1])  # type: ignore[index]
        if not self._departed_hit:
            self._departed_hit = d_hit > self.hit_depart_radius
        elif d_hit < self.hit_return_radius:
            self.unreachable = True
            return (0.0, 0.0)
        # Leave: re-crossed the m-line closer to the goal AND able to move
        # towards the goal from here (textbook Bug2, all three conditions).
        crossed = self._crossed_mline_closer(pos)
        if crossed and not self._goal_blocked(pos, yaw, scan_ranges, angle_min, angle_inc):
            self.mode = NavMode.GO_TO_GOAL
            return self._go_to_goal(pos, yaw)
        return self._wall_follow(pos, yaw, scan_ranges, angle_min, angle_inc)

    # --------------------------------------------------------------- go-to-goal
    def _go_to_goal(self, pos: tuple[float, float], yaw: float) -> tuple[float, float]:
        gx, gy = self._goal  # type: ignore[misc]
        desired = math.atan2(gy - pos[1], gx - pos[0])
        err = _angle_wrap(desired - yaw)
        ang = max(-self.max_ang, min(self.max_ang, 1.5 * err))
        # Rotate in place when badly misaligned; drive straight when aligned.
        lin = self.max_lin * max(0.0, math.cos(err))
        return (max(0.0, min(self.max_lin, lin)), ang)

    # -------------------------------------------------------------- wall-follow
    def _tracked_point(self, scan_ranges, angle_min, angle_inc):
        """(range, bearing) of the nearest point on the boundary being followed,
        or (None, None) if nothing is in range. Bearing is relative to the heading.

        Bug2 follows the boundary of the obstacle it hit. Only scan points
        within track_gate of the last tracked boundary point belong to it, so
        the follower cannot hop onto another obstacle (e.g. a room wall) that
        merely becomes nearer while going round. A point closer than
        body_clear from ANOTHER obstacle joins the boundary only if it is ahead
        of the robot and the gap between it and the followed boundary is too
        narrow for the body to pass (< 2*half_width + 2 cm): then the two
        really are one obstacle for the robot. A wider gap stays
        open; the collision guards keep the robot off the nearer obstacle.
        Points behind the robot (> 120 deg) are boundary already passed.

        Tracking is done in the robot frame from one cycle to the next (the
        point moves < 2 cm per cycle), using the scan only, so jumps of the
        localization pose cannot make the follower lose the boundary.
        """
        if not scan_ranges or angle_inc == 0.0:
            return (None, None)
        side_sign = -1.0 if self._side == "right" else 1.0
        pts = []
        for i, r in enumerate(scan_ranges):
            if r is None or math.isinf(r) or math.isnan(r) or r <= 0.0 or r > self.follow_range:
                continue
            a = _angle_wrap(angle_min + i * angle_inc)
            if abs(a) > math.radians(120.0):
                continue   # behind the robot: boundary already passed, not the one to follow
            pts.append((r, a, r * math.cos(a), r * math.sin(a)))
        if not pts:
            return (None, None)
        tp = self._track_pt
        best = None
        if tp is not None:
            narrow = 2.0 * self.half_width + 0.02         # the body (+ margin) does not fit through
            cand = []
            for p in pts:
                gap = math.hypot(p[2] - tp[0], p[3] - tp[1])
                # another obstacle joins only if it closes the way AHEAD (|bearing| <= 90 deg);
                # one beside/behind the robot does not block where it is going
                intrudes = p[0] < self.body_clear and abs(p[1]) <= math.pi / 2 and gap < narrow
                if gap <= self.track_gate or intrudes:
                    cand.append(p)
            if cand:
                best = min(cand, key=lambda p: p[0])
        if best is None:
            # Contact lost (or no boundary yet): re-acquire the nearest point
            # on the wall side, else the nearest point at all, instead of
            # steering blind.
            side = [p for p in pts if side_sign * p[1] > 0.0]
            best = min(side or pts, key=lambda p: p[0])
        self._track_pt = (best[2], best[3])
        return (best[0], best[1])

    def _wall_follow(self, pos, yaw, scan_ranges, angle_min, angle_inc) -> tuple[float, float]:
        """Boundary following by pure pursuit around the tracked boundary point.

        P = nearest point of the followed boundary, n = unit vector P -> robot,
        t = boundary tangent (obstacle kept on the chosen side). The robot
        steers to the carrot  P + wall_offset*n + lookahead*t, i.e. a point on
        the offset path a little ahead. Along a wall the carrot is straight
        ahead; round a convex corner P stays put while n turns, so the carrot
        swings round it and the robot turns on the wall_offset circle instead
        of drifting outward. Too far / too close simply moves the carrot in /
        out. Velocities: pure pursuit w = 2 v sin(alpha) / L.
        """
        s = -1.0 if self._side == "right" else 1.0      # obstacle on the right = -y side
        # track every cycle (also while turning in place): the point is kept in the robot frame
        r, bearing = self._tracked_point(scan_ranges, angle_min, angle_inc)

        # Something in the strip the body would sweep: turn in place away from
        # it (rotating a round robot cannot hit anything). Across the path or
        # on the obstacle side it is the next face of the boundary (concave
        # corner): follow it from now on, turn away from the obstacle side.
        # Wholly on the free side it is something else (e.g. a wall across a
        # passable gap): keep the boundary and turn away from it, i.e. toward
        # the boundary.
        blk = self._corridor_points(scan_ranges, angle_min, angle_inc)
        if blk:
            if any(s * y >= 0.0 for _, y in blk):
                self._track_pt = min(blk)
                return (0.0, -s * 0.5 * self.max_ang)
            return (0.0, s * 0.5 * self.max_ang)
        if r is None:
            # Nothing at all within follow_range: arc toward the obstacle side.
            return (self.wall_follow_speed * self.max_lin * 0.3, s * 0.8)

        px, py = r * math.cos(bearing), r * math.sin(bearing)
        nx, ny = -px / r, -py / r                  # boundary point -> robot
        tx, ty = -s * ny, s * nx                   # along the boundary
        cx = px + self.wall_offset * nx + self.lookahead * tx
        cy = py + self.wall_offset * ny + self.lookahead * ty
        alpha = math.atan2(cy, cx)
        # Carrot well to the side/behind: turn toward it first.
        if abs(alpha) > math.radians(60.0):
            return (0.0, max(-self.max_ang, min(self.max_ang, 1.5 * alpha)))
        v = self.wall_follow_speed * self.max_lin * math.cos(alpha)
        w = 2.0 * v * math.sin(alpha) / max(math.hypot(cx, cy), 1e-3)
        return (v, max(-self.max_ang, min(self.max_ang, w)))

    # ------------------------------------------------------------- leave rule
    def _signed_mline_distance(self, pos: tuple[float, float]) -> float:
        """Signed perpendicular distance from pos to the m-line (start->goal).

        Sign flips when the robot crosses the line, which is how we detect
        'encountered the start-goal line again' robustly at speed.
        """
        sx, sy = self.mline.start      # type: ignore[union-attr]
        gx, gy = self.mline.goal       # type: ignore[union-attr]
        dx, dy = gx - sx, gy - sy
        denom = math.hypot(dx, dy)
        if denom == 0.0:
            return 0.0
        return (dx * (pos[1] - sy) - dy * (pos[0] - sx)) / denom

    def _crossed_mline_closer(self, pos: tuple[float, float]) -> bool:
        """Bug2 leave condition: re-crossed the m-line AND nearer the goal than
        the hit point (textbook: 'encounter the start-goal line again closer').
        """
        if self.mline is None or self._hit_distance_to_goal is None:
            return False
        cur = self._signed_mline_distance(pos)
        prev = self._prev_signed
        self._prev_signed = cur
        if prev is None:
            return False
        crossed = (cur == 0.0) or ((prev < 0.0) != (cur < 0.0))
        dist_goal = math.hypot(pos[0] - self.mline.goal[0], pos[1] - self.mline.goal[1])
        closer = dist_goal < (self._hit_distance_to_goal - self.leave_margin)
        return crossed and closer
