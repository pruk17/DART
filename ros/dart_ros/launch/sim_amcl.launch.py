"""SIM + AMCL on YOUR map: test the real localization pipeline in Gazebo.

Uses dart_room.world (auto-generated from maps/map.pgm by tools/map2world.py, so
the Gazebo geometry == your SLAM map) + map_server + AMCL on YOUR map
(maps/map.yaml) + dart_node via /amcl_pose. This mirrors the real robot's setup
as closely as a sim can: same map, same localization, your room's geometry.

    ros2 launch dart_ros sim_amcl.launch.py

Regenerate the world if the map changes:
    python3 tools/map2world.py maps/map.yaml src/dart_ros/worlds/dart_room.world
"""

import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription, TimerAction
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description() -> LaunchDescription:
    dart_share = get_package_share_directory("dart_ros")
    tb3_launch = os.path.join(get_package_share_directory("turtlebot3_gazebo"), "launch")
    pkg_gazebo_ros = get_package_share_directory("gazebo_ros")

    sim = {"use_sim_time": True}
    config_file = LaunchConfiguration("config_file")
    map_yaml = LaunchConfiguration("map")
    default_cfg = os.path.join(dart_share, "config", "sim_amcl.yaml")
    world = os.path.join(dart_share, "worlds", "dart_room.world")

    # Start pose (lower-right corner of the room): spawn here and start AMCL here.
    X0, Y0 = 1.75, -1.65

    gzserver = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(os.path.join(pkg_gazebo_ros, "launch", "gzserver.launch.py")),
        launch_arguments={"world": world}.items(),
    )
    gzclient = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(os.path.join(pkg_gazebo_ros, "launch", "gzclient.launch.py"))
    )
    rsp = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(os.path.join(tb3_launch, "robot_state_publisher.launch.py")),
        launch_arguments={"use_sim_time": "true"}.items(),
    )
    spawn = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(os.path.join(tb3_launch, "spawn_turtlebot3.launch.py")),
        launch_arguments={"x_pose": str(X0), "y_pose": str(Y0)}.items(),
    )

    map_server = Node(
        package="nav2_map_server", executable="map_server", name="map_server", output="screen",
        parameters=[{**sim, "yaml_filename": map_yaml}],
    )
    amcl = Node(
        package="nav2_amcl", executable="amcl", name="amcl", output="screen",
        parameters=[{
            **sim,
            "global_frame_id": "map", "odom_frame_id": "odom", "base_frame_id": "base_footprint",
            "scan_topic": "scan",
            "robot_model_type": "nav2_amcl::DifferentialMotionModel",
            "set_initial_pose": True,
            "initial_pose": {"x": X0, "y": Y0, "z": 0.0, "yaw": 0.0},
        }],
    )
    lifecycle = Node(
        package="nav2_lifecycle_manager", executable="lifecycle_manager",
        name="lifecycle_manager_localization", output="screen",
        parameters=[{**sim, "autostart": True, "node_names": ["map_server", "amcl"]}],
    )

    dart_node = Node(
        package="dart_ros", executable="dart_node", name="dart_node", output="screen",
        parameters=[{
            "config_file": config_file,
            "pose_source": "amcl",
            "map_yaml": map_yaml,       # drawn under the end-of-run path map
            "cmd_vel_topic": "/cmd_vel",
            "scan_topic": "/scan",
            "use_twist_stamped": False,
            **sim,
        }],
    )

    return LaunchDescription([
        DeclareLaunchArgument("config_file", default_value=default_cfg),
        DeclareLaunchArgument("map", default_value="/ws/maps/map.yaml"),
        gzserver, gzclient, rsp, spawn,
        map_server, amcl, lifecycle,
        TimerAction(period=10.0, actions=[dart_node]),
    ])
