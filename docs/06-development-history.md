# 6. Development history

A chronological record of what was built, what went wrong, how the cause was
established, and what was changed. It explains *why* the current design looks
the way it does. Evidence comes from run logs, path maps, the node's debug
output and rosbags (see [Experiments](07-experiments.md) for the methods).

---

## Phase 1 — Environment

| Problem | Cause | Fix |
|---|---|---|
| Robot needs Humble, host is Ubuntu 24.04 (Jazzy) | Humble not packaged for 24.04 | Docker container from `osrf/ros:humble-desktop` |
| Upgrading setuptools broke colcon | Humble's colcon needs the pinned setuptools 59.6 | `dart` uses a classic `setup.py` (no PEP 621/660) |
| Gazebo GL errors | no GPU passthrough | software rendering in the image (`DART_GPU=1` to pass the GPU) |
| `spawn_entity` hangs | Gazebo tries to download models | empty `GAZEBO_MODEL_DATABASE_URI` |
| Robot falls through the ground | model path missing ground plane models | `GAZEBO_MODEL_PATH` includes `gazebo-11/models` |
| Robot keeps moving after the node stops (sim) | diff-drive latches the last `/cmd_vel`; publishing in shutdown failed | own signal handling + final zero commands (phase 4) |

The pure-Python core was separated from ROS packaging (pip package `dart`);
the ROS package `dart_ros` is only the glue layer.

## Phase 2 — Decision layer

1. **Greedy Net (concept).** Implemented with decay/expiry and a headless
   comparison against nearest-first. DART collected more value but expired
   more targets (it "sacrificed" cheap ones).
2. **Lookahead planner.** Requirement: evaluate whole visiting orders with
   decay along every leg. First version used nearest-neighbour + 2-opt for all
   sizes; twice it produced visibly worse orders (local optimum), so ≤ 7
   targets are now solved by full permutation.
3. **Service time and deadline.** A 3 s stop per delivery (included in the
   planner's clock) and a 60 s deadline reported in the run log.
4. **Half-lives from the deadline.** Values and half-lives per food type were
   derived from the quality wanted at 60 s; configs accept `half_life`
   directly and logs show `t½` next to `λ`.

## Phase 3 — Bug2 in simulation

| Problem | Cause | Fix |
|---|---|---|
| Robot turned *into* the wall while following | sign error in the wall-follow steering | corrected |
| Endless circling in open space | side-distance regulator lost the wall and kept turning | follow the **nearest boundary point** at the side bearing; leave by m-line crossing (sign change) |
| Wedged in dense obstacle grids | following distance too small | separate `wall_offset` (hug distance) from `safe_distance` (trigger) |
| Robot sat at start, then drove into a corner and fell over | hit triggered by a wall in front while turning toward a goal behind; hug distance below robot radius + margin | **hit only in the goal direction and only if closer than the goal**; safe 0.30 / offset 0.25 |
| Robot never moved with AMCL | `/amcl_pose` only published on motion → deadlock | pose from TF `map→odom→base_footprint` |

## Phase 4 — First real-robot runs (2.2 m room)

| Observation | Evidence | Cause | Fix |
|---|---|---|---|
| Robot does not move at all | node: *incompatible QoS … RELIABILITY* on `/scan` | reliable subscription vs best-effort LiDAR | sensor-data QoS |
| Robot reached the first target, then stood still for 40 s commanding `w = 2.84` | pose, yaw and `front` identical to the digit for 40 s; pose outside the mapped room | input data frozen, node kept acting on it | **stale-data stop** (pose/scan age, measured on the PC clock) |
| Same run: pose jumped after turning on the spot at the target | 2.84 rad/s ≈ 30° between scans | AMCL loses track during fast spins | `max_ang = 1.2` on the real robot |
| Ctrl+C left the robot driving | rclpy shut the context down first | own SIGINT/SIGTERM handling, spin with timeout, 5 zero commands |
| Periodic 1–2 s pose freezes | rosbag probe: odometry TF silent 2.35 s then 47 messages at once, scans unaffected | reliable `/tf` over Wi-Fi blocks until a lost sample is repaired | handled by the stale-data stop (DDS tuning tried, not kept) |
| PC/robot clock suspected | measured offset ≈ 0.26 s | not the cause | — |

A smaller test room (`map_test`, 1.3 m) was mapped to iterate faster.

## Phase 5 — Obstacles on the real robot

### 5a. Following the wrong thing and getting stuck

With an obstacle on the path the robot "followed the edge until it left the
map". A kinematic simulator built on the real map (document 7) reproduced it:
**0 of 18** obstacle cases reached the goal (12 stuck, 6 collisions). Causes
and fixes (F1–F5):

| | Cause | Fix |
|---|---|---|
| F1 | the follower tracked the nearest point in the whole scan, so it hopped onto room walls | track the boundary of the hit obstacle point-to-point; merge another obstacle only if the gap is impassable |
| F2 | in concave corners the two controller terms cancelled → `w ≈ 0`, robot frozen | turn in place away from the obstacle side |
| F3 | leave lacked the textbook "able to move towards the goal" | three-condition leave |
| F4 | no termination when going all the way round | back at the hit point → unreachable, target skipped |
| F5 | the body swept into corners at 30–60° | no forward motion when a close point is ahead of the side bearing |

Plus two found by the simulator: corner **feed-forward** (proportional
control lagged ~24° and drifted 0.1–0.2 m outward at convex corners) and
turning away when forward motion is blocked. Result: 10/18 (0.35/0.30) and
15/18 (0.30/0.25) reached, no collisions.

### 5b. Localization disturbed by obstacles

Path maps showed the robot inside walls by 10–15 cm and targets displaced in
the room when obstacles were present. Cause: AMCL matched beams that hit the
(unmapped) obstacle to the nearest walls, amplified by very frequent
resampling. Fix: beam skipping, `update_min 0.1`, `resample_interval 2`.

### 5c. Weaving between two states

The robot alternated `v = 0, w = +0.6` and `v = 0.06, w = −0.8` and slid along
walls. These are exactly the "turn away" and "contact lost" outputs: the
tracked boundary point was kept in the **map frame** and lost after pose jumps.
Fix: track it in the **robot frame**, scan to scan; on loss, re-acquire on the
obstacle side instead of arcing blindly. Also: obstacles inside the LiDAR's
0.16 m blind range straight ahead now count as a hit.

### 5d. Clearances vs room size

A rosbag replay (93 % agreement with recorded commands) showed the robot
treating a real 0.54 m gap as closed because the merge threshold was
`wall_offset + body_clear` = 0.49 m and the robot drifted a few cm while
turning; with walls on both sides it then followed the room perimeter and
declared the target unreachable. With the real obstacle shape in the
simulator: 0.35/0.30 → 0/6 reached, 0.30/0.25 → 4/6, **0.28/0.22 → 6/6**.
Chosen for the small room: `safe_distance 0.28`, `wall_offset 0.20` (the
floor, see document 4) and 80 % speed while following.

### 5e. Wall on the opposite side treated as boundary

Replay of another run: at t = 37 s the robot, 0.36 m from the obstacle after a
corner, came within 0.18 m of the room wall on the **opposite** side; the old
merge rule ("anything closer than 0.19 m") adopted the wall and the robot
turned back along it, although the gap was 0.56 m. Fix: merge depends on the
**gap width** between the two obstacles, not on the robot's distance.

### 5f. Target switching during detours

A run where the planner preferred another target for about a minute while the
robot followed a boundary — switching was only allowed while driving straight.
Changed to the concept: switching allowed at any time, m-line redrawn.

## Phase 6 — Steering quality and map handling

| Observation | Cause | Fix |
|---|---|---|
| Robot weaves left-right constantly | rosbag: `/cmd_vel` 30 msg/s, zero commands every 0.1 s from `teleop_keyboard` | close teleop |
| Residual weaving near targets | AMCL correction jumps | smoothed correction composed with odometry (τ 0.7 s) |
| Hard to edit target coordinates in a tilted map | walls at −11.5° | `rotate_map.py` → `map_test_0deg` (later superseded by `map_small` and removed) |
| Targets closer to walls requested | — | 0.18 m from both walls is the closest where every leg was reachable in simulation (0.17 got stuck once, 0.15 collides) |
| New map `map_small` | different SLAM start pose | wall registration + known SLAM start to transfer all points; `real.launch.py` defaults to it |

## Phase 7 — Simplifying boundary following

User report: large turning radius round obstacles, then following the wall
instead of the obstacle. In simulation on `map_small` the old controller hugged
at 0.17 m instead of 0.20, stopped-and-turned 51 % of the time, and in one leg
followed the whole room because it always went round to the left even when
that side was a narrow gap to a wall.

Removed: two-gain controller, corner feed-forward and its slide estimator, the
diagonal-ahead special case, the doubled tracking gate, the separate ±15°
front check. Added/strengthened: **pure pursuit** around the tracked point,
**side choice** per hit, a single **swept-corridor** check, ignoring boundary
points behind the robot, a merge threshold from the robot width (0.30 m).
Result: 22/24 reached (was 20/24), median hug 0.19–0.20 m, stop-and-turn 19 %,
mean leg 10.7 s (was 13.5 s).

During this phase a rule for corridor points on the **free side** was removed
for simplicity. A later rosbag (99 % replay agreement) showed exactly that
moment: the robot had passed between a box and the room wall toward the target
when the wall entered the corridor on the free side; it was adopted as the
boundary and the robot turned back and later declared the target unreachable.
The rule was restored ("turn away from it, keep the boundary").

Real-robot runs after this phase: obstacles avoided with tight arcs and no
wall hopping (median distance to obstacle while following 0.20–0.25 m).

## Phase 8 — Decision refinements

### Why a nearby target was not served

Analysis in [document 3, §3.6](03-decision.md#36-worked-example-why-the-robot-passed-a-target-without-serving-it):
the planner's straight-line estimate made serving the passed target first look
only ~6 % better, below ε = 15 %.

<a id="deliver-on-the-way"></a>
### Deliver on the way

Passing within `goal_tol` of any target now delivers it. The first version kept
the old m-line and hit point after such a delivery; in the next run the robot
circled an obstacle for ~40 s because the old m-line (along the opposite side
of the room) was never crossed and the hit point never revisited. Fixed by
restarting the current target from the delivery point, as after any delivery.
