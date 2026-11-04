# 8. Operations guide

## 8.1 One-time setup

See the top-level [README](../README.md#6-getting-started) (workspace links,
Docker image, `colcon build --symlink-install`) and
[docker/SETUP.md](../docker/SETUP.md). The robot needs ROS 2 Humble and
`turtlebot3_bringup`, with the same `ROS_DOMAIN_ID` (30) as the container.

## 8.2 A run on the real robot

1. Robot: `ros2 launch turtlebot3_bringup robot.launch.py` (check the battery).
2. PC terminal 1 — localization + RViz:
   ```bash
   ros2 launch dart_ros real.launch.py
   ```
   Place the robot at the start pose (default x = 0.19, y = −0.78, facing +x on
   `map_small`). In RViz the red LiDAR points must lie on the map walls; if
   not, use **2D Pose Estimate**.
3. Optional terminal — record:
   ```bash
   ros2 bag record -o /ws/bags/run_$(date +%H%M%S) /scan /tf /tf_static /odom /cmd_vel /rosout
   ```
4. PC terminal 2 — start DART:
   ```bash
   ros2 launch dart_ros real.launch.py localization:=false rviz:=false start_dart:=true
   ```
5. Ctrl+C in terminal 2 stops the robot at any time.

Check before step 4 that nothing else publishes velocity:
`ros2 topic info /cmd_vel -v` should list only `dart_node` once it runs.

## 8.3 Changing targets and food

Edit `ros/dart_ros/config/real_small.yaml` and restart DART (no build).

```yaml
targets:
  - { id: m1, position: [0.97, -0.93], type: ice_cream }            # use a type
  - { id: m2, position: [0.97,  0.01], type: main_dish, v: 6.0 }    # override value
  - { id: m3, position: [0.03,  0.01], type: hot_soup, half_life: 45 }
```

- `position` is `[x, y]` in metres in the **map frame**; in RViz x grows to the
  right and y upwards. Use *Publish Point* in RViz to read coordinates.
- Keep targets ≥ 0.18 m from walls; closer does not make the robot stop closer
  (it counts as reached within `goal_tol` = 0.15 m) and can trigger the
  obstacle logic.
- New food type: add `name: { v: …, half_life: … }` under `task_types` in
  `common.yaml`. From a wanted quality `q` at 60 s:
  `half_life = 60·ln2 / ln(1/q)`.
- A misspelt type silently becomes `default` (v = 1, t½ = 60 s); check the
  initial values in the first debug block.

What needs what after an edit:

| Edited | Action |
|---|---|
| targets, types, robot values in existing YAML | restart DART |
| start pose | `x0:= y0:= yaw0:=` when starting terminal 1, or 2D Pose Estimate |
| AMCL settings in `real.launch.py` | restart both terminals |
| a new file (config, map, launch) | `colcon build --symlink-install`, then restart |

## 8.4 Another room

1. Build a SLAM map and save it to `maps/<name>.yaml/.pgm`.
2. If the walls are tilted, optionally axis-align it:
   `python3 tools/rotate_map.py maps/<name>.yaml maps/<name>_0deg <deg>`.
3. Copy `real_small.yaml` to a new config with targets in that map's frame.
4. Start with `map:=/ws/maps/<name>.yaml config_file:=…/<new>.yaml x0:= y0:= yaw0:=`.
5. For simulation: `python3 tools/map2world.py maps/<name>.yaml ros/dart_ros/worlds/<name>.world`.

## 8.5 Reading the debug block

```
╔══ DART  t=  45.4s  ═════════════════════
║ pose   x=-0.71 y=+0.22 yaw=+124.1°   mode=WALL_FOLLOW
║ drive  current=m1   cmd v=+0.06 w=-0.80   front=0.32m
║ plan   m2 -> m1 -> m3
║ data   pose age 0.00s   scan age 0.09s
║ targets (3 left)   [*]=current  [1..]=plan order
║   id   position        v      dist   tag
║   m1  (-0.25,-0.08)   1.40   0.55   *2
```

`mode` is the Bug2 state; `cmd` the velocity sent; `front` the nearest return
within ±15°; `plan` the planner's best order (it may differ from `current`
until the switching margin is exceeded); `data` ages > 1 s stop the robot;
`v` is the current (decayed) value.

## 8.6 Analysing a run

```bash
# inside the container
python3 tools/analysis/bag_extract.py /ws/bags/run_X /tmp/run_X.pkl
# anywhere with numpy / scipy / matplotlib
python3 tools/analysis/bag_report.py /tmp/run_X.pkl --map maps/map_small.yaml --out run_X.png
python3 tools/analysis/bag_replay.py /tmp/run_X.pkl --goal 0.03,0.01 --from 40.3 --map-config ros/dart_ros/config/real_small.yaml
```

## 8.7 Troubleshooting

| Symptom | Check |
|---|---|
| Robot does not move | `DART up` printed? pose/scan ages in the debug block; `ros2 topic hz /scan`; QoS warnings |
| `STOP: data stale` often | Wi-Fi quality; robot battery; `odom` still publishing on the Pi |
| Weaving, slow turning | another `/cmd_vel` publisher (teleop) |
| LiDAR points not on the walls in RViz | wrong start pose → 2D Pose Estimate |
| A target marked unreachable | obstacle closes all gaps narrower than 0.30 m, or the target lies in a narrow pocket |
| Robot stops and does nothing on the Pi side | `turtlebot3_node` crashed (*no status packet*); charge battery, restart bringup |
| Host `ros2` tools act strangely | `ros2 daemon stop` on the host |
