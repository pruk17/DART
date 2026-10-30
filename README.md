# DART — Decision-Adaptive Reward Targeting

Online target-selection + reactive navigation for a **single robot** that has
**many tasks whose value decays over time** (perishable reward). Target
platform: **TurtleBot3** (ROS2). This repo currently holds the **pure-Python
core + a headless simulator**; the ROS2 node layer is not written yet.

> Full concept/design spec: `DART_concept.md` (kept locally, git-ignored — not
> on GitHub). The essentials are summarized below so work can continue without
> it.

---

## 1. The idea in one paragraph

Classic planners pick the **nearest** task and assume value is constant. When
task value decays over time, nearest-first lets high-value / fast-decaying
tasks expire. DART instead scores each task by **value collectable on arrival
minus the value the whole remaining basket loses while travelling there**, and
re-decides continuously. Navigation is reactive (modified Bug2), **no A\***.

## 2. The locked decision rule (concept section 4)

For each candidate target `i`:

```
Net_i = v_i * exp(-lambda_i * t_i)                       # gain: value of i left on arrival
      - sum_{j != i} v_j * (1 - exp(-lambda_j * t_i))    # basket-bleed: value others lose en route

t_i = d_i / s        # travel-time estimate (Euclidean distance / speed, NO A*)
```

Choose `i* = argmax_i Net_i`. Symbols: `v` base value, `lambda` decay rate
(`lambda = ln(2)/half_life`), `d` straight-line distance, `s` robot speed.

---

## 3. Status (what is done vs stub)

| File | Role | Status |
|---|---|---|
| `dart/scoring.py` | Net_i math (section 4) | ✅ **implemented + tested** |
| `dart/decay.py` | value decay + expiry (shared sim + ROS) | ✅ **implemented + tested** |
| `dart/planner.py` | lookahead ordering (full-permutation <=7, 2-opt above) — drives the robot | ✅ **implemented + tested** |
| `dart/decider.py` | argmax + hysteresis + nearest-first | ✅ kept as the **sim baseline** (no longer drives the robot) |
| `dart/sim/world.py`, `run_comparison.py` | headless DART vs nearest-first | ✅ **implemented** |
| `dart/localization/pose_source.py` | pure `PoseSource` + `SimPoseSource` | ✅ (ROS Odom/AMCL sources live in `dart_ros`) |

The ROS2 node layer, Bug2 navigator, and AMCL/Odom pose sources are implemented
in the **`dart_ros`** package (see `../dart_ros`), not here. The old
`dart/navigator.py` stub and `AmclPoseSource` stub were removed.

---

## 4. Key design decisions already made (do not silently revert)

1. **Hysteresis is additive, not multiplicative.** `should_switch` uses
   `Net_cand - Net_cur > epsilon * |Net_cur|`. The original multiplicative form
   `Net_cur*(1+eps)` breaks because Net is frequently **negative** in DART.
2. **`should_switch` signature** is
   `should_switch(current, candidate, all_targets, robot, params)` — it needs
   the target list because `net_score` includes the basket-bleed sum.
3. **`Target.v` holds the CURRENT (already-decayed) value.** The world decays
   it each step; `World.initial_value` snapshots the starting values so expiry
   is judged as a fraction of the original. `scoring` projects `v` forward by
   travel time.
4. **`make_scenario` assigns VARIED base values** (`v` from {1,2,3,5}). This
   matters: with uniform `v=1`, DART does NOT beat nearest-first (see results).

---

## 5. How to run the simulator

```bash
# from the repo root
python -m dart.sim.run_comparison --targets 20 --seed 0
```

Reports value collected, targets served, targets expired, finish time — for
DART vs nearest-first on the same scenario.

### Current results (10 seeds per config, varied v)

| targets | DART avg value | nearest-first avg | DART win-rate |
|---:|---:|---:|---:|
| 10 | 8.71 | 8.29 | 80% |
| 20 | 15.19 | 13.97 | 60% |
| 30 | 18.49 | 18.52 | 40% |
| 40 | 24.21 | 22.89 | 70% |

**Interpretation:** with varied value DART wins modestly on ~3/4 configs. It
consistently **expires MORE targets but collects MORE total value** — i.e. it
sacrifices low-value tasks to grab high-value ones (the intended behaviour).
With uniform value (`v=1`) DART ties or loses, because then max-throughput
(nearest-first) is already near-optimal. The win margin should widen in
parameter regimes where value-ordering pays (higher speed, wider value range,
weaker decay) — tuning this is an open task.

---

## 6. Target hardware / OS (decisions)

- **Robot:** TurtleBot3 (Burger/Waffle). An older unit (4+ yrs) likely has a
  Raspberry Pi 3B+ (1 GB RAM); newer ones have Pi 4. Check with
  `cat /proc/device-tree/model`.
- **Chosen stack: Ubuntu 22.04 + ROS2 Humble** (most documented, lighter on
  old SBCs). Flash **Ubuntu Server 22.04 LTS (64-bit)** with Raspberry Pi
  Imager (Server, not Desktop).
- **Dev laptop currently runs Ubuntu 24.04 + ROS2 Jazzy.** ROS2 versions must
  match between PC and robot, so Humble must be made available on the laptop
  (via Docker `ros:humble` or a 22.04 VM — native Humble on 24.04 is not
  supported).
- **⚠️ `/cmd_vel` message type:** Humble uses `geometry_msgs/Twist`; Jazzy
  defaults to `TwistStamped`. The future `navigator.py` publisher must match
  whichever distro it runs on.

---

## 7. Intended runtime flow on TurtleBot (concept section 5)

```
startup: load config -> init pose source (AMCL) -> init navigator
loop while targets remain:
  1 sense      pose (/amcl_pose) + targets + LiDAR (/scan)
  2-3 score    scoring.net_score for every target
  4 select     decider.select_target -> i*  ; navigator.set_goal (m-line)
  5-6 drive    navigator reactive Bug2 toward i*, avoid obstacles -> /cmd_vel
  7 re-decide  recompute Net; decider.should_switch (epsilon) may switch target
  8 serve      on reaching i*, do the task, remove it from the list
```

Localization: SLAM map built **once** at setup, then **AMCL** localizes in real
time. The map is used ONLY for localization + goal coordinates, **not** for
global path planning.

---

## 8. Status / next steps

Core algorithm + ROS2 layer are implemented (see `../dart_ros`). Decision now
uses lookahead planning (`planner.py`), not greedy. Open research: parameter
sweeps for where DART's advantage is largest, and a receding-horizon vs greedy
comparison.

## 9. Module layout

```
dart/
  __init__.py
  scoring.py            # Net_i computation            [done]
  decay.py              # value decay + expiry         [done]
  planner.py            # lookahead ordering (perm/2-opt) [done, drives the robot]
  decider.py            # argmax + hysteresis + nearest-first [done, sim baseline]
  config.yaml           # parameters (s, epsilon, v, lambda)
  localization/
    __init__.py
    pose_source.py      # PoseSource + SimPoseSource   [ROS sources live in dart_ros]
  sim/
    __init__.py
    world.py            # headless world              [done]
    run_comparison.py   # DART vs nearest-first       [done]
```

## 10. Repository layout (ROS 2 workspace)

The repo root is the pure-Python `dart` package. The ROS 2 side lives in subfolders:

```
dart/            pip package (scoring, decay, planner, sim)
ros/dart_ros/    ROS 2 Humble package: dart_node, Bug2 navigator, configs, launch, rviz
docker/          Humble container (Dockerfile, run.sh, SETUP.md)
tools/           map2world.py (map -> Gazebo world), rotate_map.py (axis-align a map)
maps/            SLAM maps (map_small is the launch default)
```

colcon does not look inside a package folder, so the workspace links the ROS
package and the helper folders next to it:

```bash
mkdir -p ~/DART_ws/src && cd ~/DART_ws/src && git clone https://github.com/pruk17/DART.git
ln -s DART/ros/dart_ros dart_ros
cd ~/DART_ws && ln -s src/DART/docker docker && ln -s src/DART/tools tools && ln -s src/DART/maps maps
./docker/run.sh build && ./docker/run.sh up          # then: colcon build --symlink-install
```

Real robot (PC side, map_small, start = RViz bottom-left facing +x):

```bash
ros2 launch dart_ros real.launch.py                                                   # localization + RViz
ros2 launch dart_ros real.launch.py localization:=false rviz:=false start_dart:=true  # DART
```
