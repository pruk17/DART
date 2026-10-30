"""REAL robot (PC side): localize on a SLAM map with AMCL, then run DART.

Runs ONLY the PC-side processing: map_server + AMCL (localization on your SLAM
map) + dart_node. The robot's Raspberry Pi runs `turtlebot3_bringup` separately
and publishes /scan and the odom->base_footprint TF over the network; AMCL adds
map->odom and /amcl_pose, and dart_node drives via /cmd_vel.

Defaults: map_small + config/real_small.yaml, start in the room's bottom-right
corner facing room-up (x0=0.19 y0=-0.78 yaw0=0.0). Two steps:

    ros2 launch dart_ros real.launch.py                                   # localization + RViz
    ros2 launch dart_ros real.launch.py localization:=false rviz:=false start_dart:=true   # DART

Other map/config/start: map:=... config_file:=... x0:=... y0:=... yaw0:=...
Set x0/y0/yaw0 to where the robot actually starts on the map (its initial pose),
or re-localize later with RViz "2D Pose Estimate".
"""

import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, GroupAction
from launch.conditions import IfCondition
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description() -> LaunchDescription:
    dart_share = get_package_share_directory("dart_ros")
    default_cfg = os.path.join(dart_share, "config", "real_small.yaml")

    map_yaml = LaunchConfiguration("map")
    config_file = LaunchConfiguration("config_file")
    x0 = LaunchConfiguration("x0")
    y0 = LaunchConfiguration("y0")
    yaw0 = LaunchConfiguration("yaw0")

    # AMCL tuned for TurtleBot3 (differential, 360 LiDAR on /scan).
    amcl_params = {
        "use_sim_time": False,
        "global_frame_id": "map",
        "odom_frame_id": "odom",
        "base_frame_id": "base_footprint",
        "scan_topic": "scan",
        "robot_model_type": "nav2_amcl::DifferentialMotionModel",
        "set_initial_pose": True,
        "initial_pose": {"x": x0, "y": y0, "z": 0.0, "yaw": yaw0},
        # Small rooms: correct from the scan every 10 cm / ~6 deg instead of the
        # Nav2 defaults (0.25 m / 0.2 rad), which leave a 0.6 m leg on odometry.
        # (0.05 updated + resampled so often that scans corrupted by an
        # unmapped obstacle pulled the particle cloud off within seconds.)
        "update_min_d": 0.1,
        "update_min_a": 0.1,
        "resample_interval": 2,
        # Obstacles that are not in the map: beam skipping ignores the beams
        # most particles agree do not fit the map, instead of letting them
        # drag the pose toward the nearest wall. Needs the _prob model.
        "laser_model_type": "likelihood_field_prob",
        "do_beamskip": True,
        "beam_skip_distance": 0.5,
        "beam_skip_threshold": 0.3,
        "beam_skip_error_threshold": 0.9,
        # LDS-02 measuring range (160-8000 mm; the robot reports "FOUND LDS-02").
        "laser_min_range": 0.16,
        "laser_max_range": 3.5,
    }

    map_server = Node(
        package="nav2_map_server", executable="map_server", name="map_server", output="screen",
        parameters=[{"use_sim_time": False, "yaml_filename": map_yaml}],
    )
    amcl = Node(
        package="nav2_amcl", executable="amcl", name="amcl", output="screen",
        parameters=[amcl_params],
    )
    lifecycle = Node(
        package="nav2_lifecycle_manager", executable="lifecycle_manager",
        name="lifecycle_manager_localization", output="screen",
        parameters=[{
            "use_sim_time": False,
            "autostart": True,
            "node_names": ["map_server", "amcl"],
        }],
    )

    dart_node = Node(
        package="dart_ros", executable="dart_node", name="dart_node", output="screen",
        parameters=[{
            "config_file": config_file,
            "pose_source": "amcl",      # pose from TF map->base_footprint (MAP frame)
            "map_yaml": map_yaml,       # drawn under the end-of-run path map
            "cmd_vel_topic": "/cmd_vel",
            "scan_topic": "/scan",
            "use_twist_stamped": False,  # Humble -> Twist
            "use_sim_time": False,
        }],
    )

    rviz = Node(
        package="rviz2", executable="rviz2", name="rviz2", output="log",
        arguments=["-d", os.path.join(dart_share, "rviz", "real.rviz")],
        condition=IfCondition(LaunchConfiguration("rviz")),
    )

    return LaunchDescription([
        DeclareLaunchArgument("map", default_value="/ws/maps/map_small.yaml"),
        DeclareLaunchArgument("config_file", default_value=default_cfg),
        # start: room bottom-right corner of map_small, facing room-up (= map +x)
        DeclareLaunchArgument("x0", default_value="0.19"),
        DeclareLaunchArgument("y0", default_value="-0.78"),
        DeclareLaunchArgument("yaw0", default_value="0.0"),
        # Method B: localize first (RViz 2D Pose Estimate), start DART afterwards
        # with start_dart:=true in a second launch (rviz:=false), or set it here.
        DeclareLaunchArgument("start_dart", default_value="false"),
        DeclareLaunchArgument("localization", default_value="true"),
        DeclareLaunchArgument("rviz", default_value="true"),
        GroupAction([map_server, amcl, lifecycle],
                    condition=IfCondition(LaunchConfiguration("localization"))),
        rviz,
        GroupAction([dart_node], condition=IfCondition(LaunchConfiguration("start_dart"))),
    ])
