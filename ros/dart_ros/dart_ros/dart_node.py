"""DART online decision loop (the robot's brain).

This is the single insertion point where the ROS2 world meets the DART core:
it imports the pure-Python `dart.*` modules UNCHANGED and drives them each cycle.
Per tick it:

    1. reads the latest pose  -> builds a RobotState
    2. decays the basket (wall-clock dt); drops expired targets
    3. plan_sequence()        -> lookahead: best visiting ORDER (2-opt local search)
    4. commit with hysteresis -> switch target only if clearly better; may
                                 abandon mid-travel, also mid wall-follow
                                 (DART_concept step 7 / 8.1)
    5. navigator.set_goal/step-> reactive Bug2 -> (v, w); m-line redrawn only on
                                 an actual target change
    6. publishes /cmd_vel; drops a target once reached (the current one, or any
                                 other one passed within goal_tol on the way)

Parameters (ROS params, override in the launch file):
    config_file   : path to the DART YAML (targets + tuning)
    pose_source   : "odom" (Gazebo) or "amcl" (real robot with a map)
    cmd_vel_topic : default /cmd_vel
    scan_topic    : default /scan
    use_twist_stamped : publish TwistStamped (ROS2 Jazzy) instead of Twist (Humble)
"""

from __future__ import annotations

import rclpy
from rclpy.node import Node
from sensor_msgs.msg import LaserScan
from geometry_msgs.msg import Twist, TwistStamped

import math
import os

from dart.scoring import RobotState, euclidean_distance
from dart.decay import decay_basket, snapshot_initial_values
from dart.planner import PlannerParams, plan_sequence, simulate_sequence

from dart_ros.config import load_config
from dart_ros.navigator import ReactiveNavigatorROS
from dart_ros.pose_source import AmclPoseSource, OdomPoseSource

from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data



class DartNode(Node):
    def __init__(self) -> None:
        super().__init__("dart_node")

        self.declare_parameter("config_file", "")
        self.declare_parameter("pose_source", "odom")
        self.declare_parameter("cmd_vel_topic", "/cmd_vel")
        self.declare_parameter("scan_topic", "/scan")
        self.declare_parameter("use_twist_stamped", False)
        self.declare_parameter("debug", True)          # print the periodic state block
        self.declare_parameter("debug_period", 1.0)    # seconds between debug blocks
        self.declare_parameter("log_dir", "")          # where run summaries are saved ("" -> ./dart_logs)
        self.declare_parameter("keep_logs", 3)         # how many past run logs to keep
        self.declare_parameter("map_yaml", "")         # optional map drawn under the path plot
        self.declare_parameter("stale_timeout", 1.0)   # stop if pose/scan older than this (s)
        self.declare_parameter("pose_smoothing_tau", 0.7)  # AMCL correction low-pass (s), 0 = off

        config_file = self.get_parameter("config_file").value
        if not config_file:
            raise RuntimeError("config_file parameter is required")
        self.cfg = load_config(config_file)
        self.targets = list(self.cfg.targets)
        # Snapshot starting values so expiry is judged against them, and remember
        # when we last decayed so each tick uses real elapsed wall-clock time.
        self._initial_value = snapshot_initial_values(self.targets)
        self._last_decay_ns: int | None = None
        self.planner_params = PlannerParams(
            expire_threshold=self.cfg.expire_threshold,
            service_time=self.cfg.service_time,
        )

        pose_kind = str(self.get_parameter("pose_source").value).lower()
        if pose_kind == "amcl":
            self.pose_source = AmclPoseSource(
                self, smoothing_tau=float(self.get_parameter("pose_smoothing_tau").value))
        else:
            self.pose_source = OdomPoseSource(self)

        self.navigator = ReactiveNavigatorROS(
            max_lin=self.cfg.robot.speed,
            max_ang=self.cfg.robot.max_ang,
            goal_tol=self.cfg.robot.goal_tol,
            safe_distance=self.cfg.robot.safe_distance,
            wall_offset=self.cfg.robot.wall_offset,
            wall_follow_speed=self.cfg.robot.wall_follow_speed,
        )

        self._scan: LaserScan | None = None
        self._scan_rx_ns: int | None = None   # local receive time of the latest scan
        self._stale_timeout = float(self.get_parameter("stale_timeout").value)
        self._stale = False                   # currently stopped for stale data
        self.create_subscription(
            LaserScan, str(self.get_parameter("scan_topic").value), self._on_scan, qos_profile_sensor_data
        )

        self._use_stamped = bool(self.get_parameter("use_twist_stamped").value)
        msg_type = TwistStamped if self._use_stamped else Twist
        self._cmd_pub = self.create_publisher(
            msg_type, str(self.get_parameter("cmd_vel_topic").value), 10
        )

        self._current = None  # currently committed Target
        self._nav_target_id = None  # target id last handed to the navigator
        self._debug = bool(self.get_parameter("debug").value)
        self._debug_period = float(self.get_parameter("debug_period").value)
        self._last_debug_ns = 0
        self._start_ns = self.get_clock().now().nanoseconds
        self._mission_start_ns: int | None = None   # set on the first active tick

        # --- run-log state: a summary file is written when a run completes, and
        #     only the newest `keep_logs` files are retained (oldest deleted). ---
        _ld = str(self.get_parameter("log_dir").value)
        self._log_dir = _ld if _ld else os.path.join(os.getcwd(), "dart_logs")
        self._keep_logs = int(self.get_parameter("keep_logs").value)
        self._n_targets0 = len(self.targets)
        self._events: list[tuple[float, str]] = []
        self._served = 0
        self._expired_count = 0
        self._collected = 0.0
        self._quality_pcts: list[float] = []   # % of initial value kept, per delivery
        self._log_written = False
        self._map_yaml = str(self.get_parameter("map_yaml").value)
        self._path: list[tuple[float, float]] = []         # robot trajectory (pose frame)
        self._target_pos0 = {t.id: t.position for t in self.targets}
        self._delivered: dict[str, tuple[int, float]] = {}  # id -> (visit order, quality %)
        self._expired_ids: list[str] = []
        self._unreachable_ids: list[str] = []               # Bug2 gave up on these (skipped)
        self._dwell_until_ns: int | None = None             # stopped at a delivery point
        period = 1.0 / max(0.1, self.cfg.replan_rate)
        self.create_timer(period, self._tick)
        self.get_logger().info(
            f"DART up: {len(self.targets)} targets, pose={pose_kind}, "
            f"replan={self.cfg.replan_rate} Hz"
        )

    def _on_scan(self, msg: LaserScan) -> None:
        self._scan = msg
        self._scan_rx_ns = self.get_clock().now().nanoseconds

    def _scan_age(self) -> float:
        if self._scan_rx_ns is None:
            return math.inf
        return (self.get_clock().now().nanoseconds - self._scan_rx_ns) * 1e-9

    def _data_stale(self) -> bool:
        """Stop instead of driving blind when pose or scan stopped updating.

        Without this the node keeps acting on the last pose/scan forever (e.g.
        AMCL stops publishing map->odom, or the robot's Wi-Fi drops).
        """
        pose_age = self.pose_source.age()
        scan_age = self._scan_age()
        stale = pose_age > self._stale_timeout or scan_age > self._stale_timeout
        if stale and not self._stale:
            self._event(f"STOP: data stale (pose age {pose_age:.1f}s, scan age {scan_age:.1f}s)")
        elif not stale and self._stale:
            self._event("data fresh again, resuming")
        if stale:
            self.get_logger().warn(
                f"waiting for fresh data: pose age {pose_age:.1f}s, scan age {scan_age:.1f}s "
                f"(limit {self._stale_timeout:.1f}s)",
                throttle_duration_sec=1.0,
            )
        self._stale = stale
        return stale

    def _publish(self, lin: float, ang: float) -> None:
        if self._use_stamped:
            msg = TwistStamped()
            msg.header.stamp = self.get_clock().now().to_msg()
            msg.twist.linear.x = lin
            msg.twist.angular.z = ang
        else:
            msg = Twist()
            msg.linear.x = lin
            msg.angular.z = ang
        self._cmd_pub.publish(msg)

    def _sim_t(self) -> float:
        """Seconds since the mission started (first tick with pose + scan)."""
        t0 = self._mission_start_ns if self._mission_start_ns is not None else self._start_ns
        return (self.get_clock().now().nanoseconds - t0) * 1e-9

    @staticmethod
    def _half_life_str(lam: float) -> str:
        return f"{math.log(2) / lam:.0f}s" if lam > 0 else "inf"

    def _event(self, msg: str) -> None:
        """Record an important event (for the run log) and echo it to ROS log."""
        self._events.append((self._sim_t(), msg))
        self.get_logger().info(msg)

    def _deadline_note(self) -> str:
        if self.cfg.deadline <= 0:
            return ""
        t = self._sim_t()
        if t <= self.cfg.deadline:
            return f" / deadline {self.cfg.deadline:.0f} s -> ON TIME ({self.cfg.deadline - t:.1f} s spare)"
        return f" / deadline {self.cfg.deadline:.0f} s -> LATE by {t - self.cfg.deadline:.1f} s"

    def _write_run_log(self) -> None:
        """Write a summary of the finished run, then keep only the newest N files."""
        import datetime
        import glob

        try:
            os.makedirs(self._log_dir, exist_ok=True)
            ts = datetime.datetime.now().strftime("%Y%m%d_%H%M%S")
            path = os.path.join(self._log_dir, f"dart_run_{ts}.log")
            lines = [
                "DART run summary",
                f"saved         : {datetime.datetime.now().isoformat(timespec='seconds')}",
                f"duration (sim): {self._sim_t():.1f} s",
                f"deadline      : " + (f"{self.cfg.deadline:.0f} s" if self.cfg.deadline > 0 else "none"),
                f"targets       : {self._n_targets0}",
                f"config        : replan={self.cfg.replan_rate}Hz eps={self.cfg.decider.epsilon} "
                f"safe={self.cfg.robot.safe_distance} wall_offset={self.cfg.robot.wall_offset} "
                f"max_ang={self.cfg.robot.max_ang} "
                f"service={self.cfg.service_time}s",
                "",
                "events (sim time):",
            ]
            for t, m in self._events:
                lines.append(f"  t={t:7.1f}  {m}")
            lines += [
                "",
                "summary:",
                f"  finished  : {self._sim_t():.1f} s" + self._deadline_note(),
                f"  delivered : {self._served}/{self._n_targets0}",
                f"  expired   : {self._expired_count}",
                f"  unreachable: {len(self._unreachable_ids)}"
                + (f" ({', '.join(self._unreachable_ids)})" if self._unreachable_ids else ""),
                f"  collected : {self._collected:.2f}",
            ]
            avg_q = None
            if self._quality_pcts:
                avg_q = sum(self._quality_pcts) / len(self._quality_pcts)
                lines.append(f"  quality   : {avg_q:.0f}% (average remaining of delivered)")

            # path map image, same timestamp as the log
            png = os.path.join(self._log_dir, f"dart_run_{ts}.png")
            try:
                from dart_ros.path_map import render_path_map
                title = (f"DART run {ts}  |  delivered {self._served}/{self._n_targets0}  "
                         f"collected {self._collected:.2f}"
                         + (f"  quality {avg_q:.0f}%" if avg_q is not None else ""))
                render_path_map(png, self._path, self._target_pos0, self.cfg.target_types,
                                self._delivered, self._expired_ids, title, self._map_yaml,
                                unreachable_ids=self._unreachable_ids)
                lines.append(f"  path map  : {os.path.basename(png)}")
            except Exception as e:
                png = None
                self.get_logger().warn(f"could not draw path map: {e}")

            with open(path, "w") as f:
                f.write("\n".join(lines) + "\n")

            # rotation: keep only the newest `keep_logs` runs (log + png together)
            runs = sorted(
                os.path.basename(p)[len("dart_run_"):-len(".log")]
                for p in glob.glob(os.path.join(self._log_dir, "dart_run_*.log"))
            )
            for old_ts in runs[:-max(1, self._keep_logs)]:
                for ext in (".log", ".png"):
                    try:
                        os.remove(os.path.join(self._log_dir, f"dart_run_{old_ts}{ext}"))
                    except OSError:
                        pass
            self.get_logger().info(
                f"run log saved: {path}" + (f" + {os.path.basename(png)}" if png else "")
                + f"  (keeping newest {self._keep_logs})"
            )
        except Exception as e:  # never let logging crash the node
            self.get_logger().warn(f"could not write run log: {e}")

    def _apply_decay(self) -> None:
        """Decay the live basket by the real time elapsed since the last tick.

        Mirrors the sim's step 3: lowers each target's current value and drops
        expired ones so the robot never chases a dead task. Uses wall-clock dt
        so decay tracks actual elapsed time, not the replan rate.
        """
        now_ns = self.get_clock().now().nanoseconds
        if self._last_decay_ns is None:
            self._last_decay_ns = now_ns
            return  # first tick: establish the clock, no elapsed time yet
        dt = (now_ns - self._last_decay_ns) * 1e-9
        self._last_decay_ns = now_ns
        if dt <= 0.0 or not self.targets:
            return

        survivors, expired = decay_basket(
            self.targets, self._initial_value, dt, self.cfg.expire_threshold
        )
        if expired:
            self.targets = survivors
            self._expired_count += len(expired)
            for t in expired:
                self._expired_ids.append(t.id)
                self._event(f"expired {t.id}")
            if self._current in expired:
                self._current = None
                self._nav_target_id = None

    def _tick(self) -> None:
        if not self.pose_source.ready() or self._scan is None:
            return  # wait for first pose + scan
        if self._mission_start_ns is None:
            self._mission_start_ns = self.get_clock().now().nanoseconds

        # Step 3 (decay) runs before selection so Net scores use fresh values.
        # It keeps running while the robot waits at a delivery point.
        self._apply_decay()

        if self._data_stale():
            self._publish(0.0, 0.0)
            return

        pos = self.pose_source.get_pose()
        yaw = self.pose_source.get_yaw()
        # record the trajectory for the end-of-run path map (skip sub-cm jitter)
        if not self._path or math.dist(self._path[-1], pos) > 0.01:
            self._path.append(pos)

        # waiting at a delivery point (service time): stay still, then continue
        if self._dwell_until_ns is not None:
            if self.get_clock().now().nanoseconds < self._dwell_until_ns:
                self._publish(0.0, 0.0)
                return
            self._dwell_until_ns = None

        if not self.targets:
            # run complete -> write the summary once, then keep only newest N
            if not self._log_written:
                self._write_run_log()
                self._log_written = True
            self._publish(0.0, 0.0)
            return

        robot = RobotState(position=pos, speed=self.cfg.robot.speed)

        # --- planner: lookahead best visiting ORDER (re-planned every tick) ---
        order = plan_sequence(self.targets, robot, self.planner_params, self._initial_value)
        best = order[0] if order else None

        if self._current is None or self._current not in self.targets:
            # starting a new leg -> take the planner's first target
            self._current = best
        elif best is not None and best is not self._current:
            # hysteresis: switch only if the planned order clearly beats keeping
            # the current target first (avoids flip-flop between close options).
            eps = self.cfg.decider.epsilon
            exp = self.planner_params.expire_threshold
            best_total = simulate_sequence(order, robot, self._initial_value, exp, self.planner_params.service_time)
            cur_first = [self._current] + [t for t in order if t is not self._current]
            cur_total = simulate_sequence(cur_first, robot, self._initial_value, exp, self.planner_params.service_time)
            if best_total > cur_total * (1.0 + eps):
                self._event(f"switch {self._current.id} -> {best.id}")
                self._current = best

        if self._current is None:
            self._publish(0.0, 0.0)
            return

        # --- reactive navigation toward the committed target --------------
        # Only redraw the m-line when the target actually changes (step 4/7).
        # Calling set_goal every tick would reset Bug2 and wipe wall-follow state.
        if self._current.id != self._nav_target_id:
            self.navigator.set_goal(pos, self._current.position)
            self._nav_target_id = self._current.id

        if self.navigator.unreachable:
            # Bug2 went round the obstacle and came back to its hit point: the
            # goal cannot be reached. Skip it and carry on with the others.
            self._event(f"unreachable {self._current.id}: Bug2 returned to its hit point, skipping")
            self._unreachable_ids.append(self._current.id)
            self.targets.remove(self._current)
            self._current = None
            self._nav_target_id = None
            self._publish(0.0, 0.0)
            return

        if self.navigator.at_goal(pos):
            self._deliver(self._current)
            self._current = None
            self._nav_target_id = None
            return

        # Deliver on the way: passing within goal_tol of another target (e.g.
        # while going round an obstacle) serves it there and then. Like after
        # any delivery, the current target is then restarted from here: new
        # m-line, Bug2 state cleared (the old m-line / hit point may be far
        # behind, and following on from them can circle an obstacle forever).
        for tg in self.targets:
            if tg is not self._current and euclidean_distance(pos, tg.position) <= self.cfg.robot.goal_tol:
                self._deliver(tg, note=" on the way to " + self._current.id)
                self._nav_target_id = None   # -> set_goal(pos, current) on the next cycle
                return

        try:
            lin, ang = self.navigator.step(
                pos,
                yaw,
                list(self._scan.ranges),
                self._scan.angle_min,
                self._scan.angle_increment,
            )
        except NotImplementedError:
            # Wall-follow not implemented yet: stop safely instead of crashing.
            self.get_logger().warn("obstacle ahead; wall-follow not implemented — stopping")
            lin, ang = 0.0, 0.0

        self._publish(lin, ang)
        self._debug_block(robot, pos, yaw, order, lin, ang)

    def _deliver(self, tg, note: str = "") -> None:
        """Serve target tg here: count it, log it, drop it, then stop for the
        service time (handing over the food)."""
        self._served += 1
        self._collected += tg.v
        v0 = self._initial_value.get(tg.id, tg.v)
        pct = 100.0 * tg.v / v0 if v0 > 0 else 0.0
        self._quality_pcts.append(pct)
        self._event(
            f"reached {tg.id}{note} "
            f"(v={tg.v:.2f} / {v0:.2f} | {pct:.0f}%) "
            f"( λ={tg.lam:.4f} | t½={self._half_life_str(tg.lam)})"
        )
        self._delivered[tg.id] = (self._served, pct)
        self.targets.remove(tg)
        self._publish(0.0, 0.0)
        if self.cfg.service_time > 0.0:
            self._dwell_until_ns = (
                self.get_clock().now().nanoseconds + int(self.cfg.service_time * 1e9)
            )
            self.get_logger().info(f"serving: waiting {self.cfg.service_time:.1f} s")

    def _debug_block(self, robot, pos, yaw, order, lin, ang) -> None:
        """Throttled, human-readable snapshot of the full decision/nav state."""
        if not self._debug:
            return
        now_ns = self.get_clock().now().nanoseconds
        if (now_ns - self._last_debug_ns) * 1e-9 < self._debug_period:
            return
        self._last_debug_ns = now_ns

        t = self._sim_t()
        front = self.navigator.front_clearance(
            list(self._scan.ranges), self._scan.angle_min, self._scan.angle_increment
        )
        front_s = "inf" if math.isinf(front) else f"{front:.2f}m"
        cur_id = self._current.id if self._current else "-"

        lines = []
        lines.append(f"╔══ DART  t={t:6.1f}s  ═══════════════════════════════════════")
        lines.append(
            f"║ pose   x={pos[0]:+.2f} y={pos[1]:+.2f} yaw={math.degrees(yaw):+6.1f}°"
            f"   mode={self.navigator.mode.name}"
        )
        lines.append(
            f"║ drive  current={cur_id:<3}  cmd v={lin:+.2f} w={ang:+.2f}"
            f"   front={front_s}"
        )
        # the lookahead plan: the order the robot intends to visit
        plan_str = " -> ".join(t.id for t in order) if order else "-"
        lines.append(f"║ plan   {plan_str}")
        lines.append(
            f"║ data   pose age {self.pose_source.age():.2f}s   scan age {self._scan_age():.2f}s"
        )
        lines.append(f"║ targets ({len(self.targets)} left)   [*]=current  [1..]=plan order")
        lines.append(f"║   id   position        v      dist   tag")
        rank = {t.id: i + 1 for i, t in enumerate(order)}
        for tg in self.targets:
            dist = euclidean_distance(pos, tg.position)
            tag = ("*" if (self._current is not None and tg is self._current) else " ")
            tag += str(rank.get(tg.id, "-"))
            lines.append(
                f"║   {tg.id:<3} ({tg.position[0]:+.2f},{tg.position[1]:+.2f})"
                f"  {tg.v:5.2f}  {dist:5.2f}   {tag}"
            )
        lines.append("╚═══════════════════════════════════════════════════════════")
        self.get_logger().info("\n" + "\n".join(lines))


def main(args=None) -> None:
    import signal
    import time
    from rclpy.signals import SignalHandlerOptions

    # Handle Ctrl+C / SIGTERM ourselves instead of rclpy: rclpy's handler shuts
    # the context down first, so the final stop could not be published and the
    # robot kept driving on its last /cmd_vel.
    rclpy.init(args=args, signal_handler_options=SignalHandlerOptions.NO)
    node = DartNode()

    stop_sig = []
    signal.signal(signal.SIGINT, lambda signum, _f: stop_sig.append(signum))
    signal.signal(signal.SIGTERM, lambda signum, _f: stop_sig.append(signum))
    try:
        # spin_once with a timeout hands control back to Python every 0.1 s, so
        # a signal is acted on promptly even when no ROS data arrives at all
        # (plain rclpy.spin can sit inside the wait and never see it).
        while not stop_sig and rclpy.ok():
            rclpy.spin_once(node, timeout_sec=0.1)
    finally:
        # No more spinning -> no tick can publish again. Command a stop several
        # times (RELIABLE /cmd_vel; gives a lost sample time to be repaired).
        node.get_logger().info("shutting down: publishing zero /cmd_vel")
        try:
            for _ in range(5):
                node._publish(0.0, 0.0)
                time.sleep(0.05)
        except Exception as e:
            print(f"[dart_node] could not publish stop: {e}", flush=True)
        node.destroy_node()
        rclpy.try_shutdown()


if __name__ == "__main__":
    main()
