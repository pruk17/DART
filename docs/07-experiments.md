# 7. Experiments and results

## 7.1 Methods

### Real-robot runs

TurtleBot3 Burger with LDS-02, PC in Docker, Wi-Fi. Each run writes a run log
(events with mission time, summary) and a path map. Values reported:
*finish* (s, incl. last service stop), *delivered*, *collected* (sum of values
at delivery), *quality* (mean % of base value kept). Deadline 60 s, service 3 s.

### Rosbag recording and reconstruction

```bash
ros2 bag record -o /ws/bags/run_<time> /scan /tf /tf_static /odom /cmd_vel /rosout
```

`tools/analysis/bag_extract.py` reads the bag; `bag_report.py` rebuilds the
robot pose at every scan (`map→odom` from AMCL ∘ `odom→base_footprint`
interpolated ∘ static `base_scan` offset), projects scans into the map, marks
returns more than 10–12 cm from any mapped wall as **obstacle points**, colours
the trajectory by mode (from the debug blocks in `/rosout`), and measures data
rates, command rates and distances to obstacles. Plotting obstacle points per
time window reveals obstacles that were moved during a run.

### Replay

`tools/analysis/bag_replay.py` feeds the recorded scans and poses of one leg
into the current `navigator.py` and compares its output with the recorded
`/cmd_vel`. Agreement of 82–99 % (|Δv| < 0.03–0.04, |Δw| < 0.15–0.2) was
obtained, so the replay can be used to see *why* the robot did something
(tracked point, mode changes, side). It is open-loop: after the first
different decision the poses are still the recorded ones.

### Kinematic simulator

`tools/analysis/sim_harness.py`: the real `navigator.py` on the real occupancy
grid at 20 Hz; unicycle kinematics; a 360-beam LiDAR model with LDS-02
behaviour (returns < 0.16 m become 0); square or rectangular boxes added to the
grid; collision = any occupied cell within 0.105 m of the centre. Optional
AMCL-like pose noise: an offset random walk updated every 0.5 s (σ = 0.02–0.06 m,
capped), applied to the pose the navigator sees while the scan comes from the
true pose. Outcomes: REACHED, UNREACHABLE, TIMEOUT (60–90 s), COLLISION, LEFT ROOM.
Limitations: no wheel slip, static obstacles, ideal timing.

## 7.2 Real-robot results

### `map_test` room (1.33 m), no obstacles

| Run | Delivered | Finish | Quality |
|---|---|---:|---:|
| 065712 | 3/3 | 19.1 s | 88 % |
| 070933 | 3/3 | 20.8 s | 84 % |
| 071117 | 3/3 | 21.9 s | 84 % |
| 071225 | 3/3 | 21.4 s | 83 % |
| 072506 | 3/3 | 23.2 s | 82 % |
| 072650 | 3/3 | 23.3 s | 82 % |
| 094238 | 3/3 | 28.7 s | 74 % (one obstacle, avoided) |

### `map_test`, obstacles, during the Bug2 rework (phase 5)

| Run | Situation | Result |
|---|---|---|
| 081925, 082023 | obstacle on a leg; localization drifted | 3/3, 21.0 s / 26.3 s |
| 082756 | obstacle near m1 | 2/3, m1 unreachable after a room lap (weaving bug, 5c) |
| 090739 | obstacle between m2 and m3 | 2/3, m3 unreachable (clearances too large, 5d) |
| 094610 | obstacle inserted abruptly | 2/3, m2 unreachable (opposite-wall merge, 5e) |

### `map_small` room (1.30 m), current navigator family

| Run | Situation | Delivered | Finish | Quality |
|---|---|---|---:|---:|
| 050128 | no obstacle | 3/3 | 21.9 s | 83 % |
| 050251 | no obstacle | 3/3 | 23.3 s | 86 % |
| 051812 | teleop left running (weaving) | 3/3 | 54.4 s | 65 % |
| 053944 | no obstacle | 3/3 | 23.9 s | 82 % |
| 054341 | no obstacle | 3/3 | 25.7 s | 79 % |
| 061128 | box touching right wall | 3/3 | 25.8 s | 84 % |
| 061238 | obstacle top middle | 3/3 | 27.3 s | 84 % |
| 061437 | two obstacles | 3/3 | 39.9 s | 74 % |
| 063640 | obstacles moved 3× during the run; free-side rule absent | 3/4 (m4 unreachable) | 95.4 s | 69 % |
| 065800 | obstacles moved, every leg blocked | 4/4 | 71.3 s | 57 % |
| 070244 | long detour passing m4 (before deliver-on-the-way) | 4/4 | 107.1 s | 47 % |
| 070634 | obstacles | 4/4 | 59.5 s | 62 % |
| 073630 | deliver-on-the-way, before the restart fix (40 s loop) | 4/4 | 87.9 s | 58 % |

Measured in the rosbags of runs 061059–061353 (static obstacles): command rate
20 Hz (DART only), distance to the nearest obstacle while following median
0.24–0.25 m, 10th percentile 0.18–0.20 m, max 0.29 m; no wall hopping.

## 7.3 Simulation results

### Bug2 rework on `map_test` (18 cases: 3 legs × box 16/24 cm × 3 positions)

| Version | Reached | Stuck | Collision |
|---|---:|---:|---:|
| before F1–F5 | 0 | 12 | 6 |
| F1–F5 + feed-forward, 0.35/0.30 | 10 | 1 | 0 (7 unreachable) |
| same, 0.30/0.25 | 15 | 0 | 0 (3 unreachable) |

### Clearance vs a real obstacle shape (from a rosbag), 6 runs each

| safe / wall_offset | Reached |
|---|---:|
| 0.35 / 0.30 | 0/6 |
| 0.30 / 0.25 | 4/6 |
| 0.28 / 0.22 | 6/6 |

### All three real obstacle layouts + 12 boxes, 4 runs each (0.28/0.20, 80 %)

59/60 reached, no collisions (gap-based merge).

### Boundary follower comparison on `map_small` (24 box cases)

| Version | Reached | Median hug (target 0.20) | Stop-and-turn | Mean time |
|---|---|---:|---:|---:|
| two-term controller | 20 (4 stuck) | 0.17 m | 51 % | 13.5 s |
| pure pursuit + side choice + corridor | 22 (2 unreachable) | 0.19–0.20 m | 19–25 % | 10.7 s |

Without obstacles 18/18; with AMCL-like noise 66/72 reached, 6 unreachable
(pocket cases), no collisions.

### Target distance from walls (`map_test_0deg`, 16 legs)

| Distance from both corner walls | Reached |
|---|---:|
| 0.15 m | 10/16 (collisions) |
| 0.17 m | 15/16 |
| 0.18 m | 16/16 |
| 0.20–0.22 m | 16/16 |

### Pose smoothing (30 legs with AMCL-like jumps)

| | Steering sign flips / leg | Steering jerk | Mean leg time |
|---|---:|---:|---:|
| unfiltered | 19.4 | 1.30 rad/s² | 6.9 s |
| τ = 0.7 s | 9.6 | 0.59 rad/s² | 5.1 s |

### Headless greedy-vs-nearest-first (`dart.sim`)

| Targets | DART avg value | Nearest-first | DART win rate |
|---:|---:|---:|---:|
| 10 | 8.71 | 8.29 | 80 % |
| 20 | 15.19 | 13.97 | 60 % |
| 30 | 18.49 | 18.52 | 40 % |
| 40 | 24.21 | 22.89 | 70 % |

## 7.4 Main findings

1. With a path available, the robot delivers every target; without obstacles
   a 3-target mission takes ~20–24 s at 82–88 % quality in a 1.3 m room.
2. Mission quality is dominated by detours: a blocked shortest path doubles
   or triples leg times and food decays meanwhile.
3. In a small room, clearances decide whether gaps are passable; 0.20 m hug /
   0.28 m trigger was needed.
4. Most real-robot failures were system issues (QoS, stale TF, a second
   `/cmd_vel` publisher, battery), each identified from logs and rosbags.
