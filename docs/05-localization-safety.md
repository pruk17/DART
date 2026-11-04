# 5. Localization and safety

## 5.1 Map

A SLAM map is built once with the TurtleBot3 SLAM tools (saved with
`map_saver_cli`) and used only for localization and target coordinates.

| Map | Room | Notes |
|---|---|---|
| `map` | ~2.2 × 2.2 m | first test room |
| `map_test` | 1.33 × 1.33 m | walls tilted −11.5° in the map frame |
| `map_test_0deg` | same | `map_test` rotated +11.5° with `tools/rotate_map.py` so walls are axis-aligned; used for one round of tests, **no longer in the repo** (regenerate with the command in document 8) |
| `map_small` | 1.30 × 1.30 m | new SLAM of the small room, **launch default** |

`map_small` was recorded with the robot starting in the room's bottom-left
corner facing "up", so in RViz the map appears rotated by 90° relative to how
the room was described earlier; target coordinates were transferred by
matching the walls of the two maps. Because the room is almost square, the
wall match fits equally well at 0°, 90°, 180° and 270° (1.2–1.4 cm mean
misfit); the correct rotation was fixed from the known SLAM start pose.

## 5.2 AMCL (`real.launch.py`)

| Setting | Value | Reason |
|---|---|---|
| `set_initial_pose`, `initial_pose` | from `x0 y0 yaw0` | start pose without RViz |
| `update_min_d` / `update_min_a` | 0.1 m / 0.1 rad | Nav2 defaults (0.25 m / 0.2 rad) leave a 0.6 m leg on odometry alone; 0.05 resampled so often that scans corrupted by an unmapped obstacle pulled the particle cloud off within seconds |
| `resample_interval` | 2 | same reason |
| `laser_model_type` | `likelihood_field_prob` | required for beam skipping |
| `do_beamskip` (distance 0.5, threshold 0.3, error 0.9) | on | ignore beams that most particles agree do not fit the map, i.e. obstacles placed in the room |
| `laser_min_range` / `laser_max_range` | 0.16 / 3.5 m | LDS-02 range |

Evidence for beam skipping: with obstacles in the room, path maps showed the
robot "inside" a wall by 10–15 cm, and target points appeared shifted in the
real room. After the change, the outline of obstacles reconstructed from the
rosbag stayed sharp and consistent through whole runs.

## 5.3 Pose used for control (`AmclPoseSource`)

AMCL applies its corrections as jumps of a few centimetres / degrees. Fed
straight into the heading controller they made the robot weave left-right,
most visibly near a target. The pose is therefore built as

```
pose = filtered(map→odom) ∘ odom→base_footprint
```

- odometry is continuous and used as is;
- the AMCL correction `map→odom` is low-pass filtered with time constant
  τ = 0.7 s (`pose_smoothing_tau`); it still converges in about a second;
- a correction larger than 0.30 m or 20° (re-localization, *2D Pose
  Estimate*) is applied at once.

In the kinematic simulator with AMCL-like jumps, filtering halved the steering
sign flips (19.4 → 9.6 per leg) and the steering jerk (1.30 → 0.59 rad/s²) and
shortened legs from 6.9 to 5.1 s.

## 5.4 Stale-data stop

Both pose sources report an **age** measured on the PC clock (not from message
stamps, so a clock offset between robot and PC cannot hide stale data):

- odometry pose: time since the TF stamp last advanced;
- AMCL: additionally, if `map→odom` has not updated for 3 s the pose counts as
  stale (dead reckoning only).

If pose or scan age exceeds `stale_timeout` (1 s) the node publishes zero,
logs `STOP: data stale (...)` once, and resumes automatically with
`data fresh again`. Without it the node kept commanding its last action on a
frozen pose (one run: 40 s of `w = 2.84` with an unchanged pose).

## 5.5 Shutdown

`rclpy`'s default signal handler shuts the context down before user code runs,
so the final stop command could not be published and the robot kept its last
`/cmd_vel`. The node installs its own SIGINT/SIGTERM handlers, spins with a
0.1 s timeout so a signal is handled even when no data arrives, then publishes
a zero `/cmd_vel` five times. Measured: the whole process group exits in
0.43–0.48 s with five zero commands received.

## 5.6 Speed limits

`max_ang = 1.2 rad/s` on the real robot (2.84 in simulation). At 2.84 rad/s
the robot turns ~30° between two LiDAR scans and wheels slip; one run lost its
pose right after turning on the spot at a target. Nav2's TurtleBot3 default is
1.0 rad/s.

## 5.7 Communication pitfalls found on the robot

| Symptom | Cause | Remedy |
|---|---|---|
| Robot never moves; node log: *"New publisher discovered on /scan, offering incompatible QoS … RELIABILITY"* | node subscribed reliably, LDS driver publishes best effort | sensor-data QoS subscription |
| Robot weaves and turns slowly; `/cmd_vel` at 30 msg/s with zeros every 0.1 s | `teleop_keyboard` left running publishes zero velocity at 10 Hz | close teleop; check `ros2 topic info /cmd_vel -v` shows one publisher |
| Short pose freezes (~1–2 s), RViz/AMCL *"Message Filter dropping message … queue is full / earlier than all the data in the transform cache"* | over Wi-Fi a lost packet on the **reliable** `/tf` topic blocks later samples until repaired (Fast DDS heartbeat 3 s); measured: odometry TF silent 2.35 s then 47 messages at once while `/scan` (best effort) kept flowing | handled by the stale-data stop; a Fast DDS heartbeat profile was tried and not adopted |
| Host `ros2` commands misbehave | host Jazzy daemon on the same network | `ros2 daemon stop` on the host or `--no-daemon` |
| Robot stops; odometry TF stops while scans continue; on the Pi `turtlebot3_node`: *"There is no status packet!"*, *"stack smashing detected"* | OpenCR not answering, typically low battery | charge battery, re-seat USB, restart bringup |
| *"Lookup would require extrapolation into the future"* when setting a pose in RViz | RViz stamps with PC time, latest odometry is a little older | harmless |
