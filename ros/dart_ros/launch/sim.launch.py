"""SIM: DART in the Gazebo cube world, localized by /odom (no map).

Loads dart_cubes.world (open ground + 4 cube obstacles + 7 delivery pads),
spawns the TurtleBot3 at the ORIGIN so /odom matches the sim.yaml target frame,
and runs dart_node with lookahead planning + Bug2.

    ros2 launch dart_ros sim.launch.py
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

    use_sim_time = LaunchConfiguration("use_sim_time", default="true")
    config_file = LaunchConfiguration("config_file")
    default_cfg = os.path.join(dart_share, "config", "sim.yaml")
    world = os.path.join(dart_share, "worlds", "dart_cubes.world")

    gzserver = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(os.path.join(pkg_gazebo_ros, "launch", "gzserver.launch.py")),
        launch_arguments={"world": world}.items(),
    )
    gzclient = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(os.path.join(pkg_gazebo_ros, "launch", "gzclient.launch.py"))
    )
    robot_state_publisher = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(os.path.join(tb3_launch, "robot_state_publisher.launch.py")),
        launch_arguments={"use_sim_time": use_sim_time}.items(),
    )
    spawn = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(os.path.join(tb3_launch, "spawn_turtlebot3.launch.py")),
        launch_arguments={"x_pose": "0.0", "y_pose": "0.0"}.items(),
    )

    dart_node = Node(
        package="dart_ros", executable="dart_node", name="dart_node", output="screen",
        parameters=[{
            "config_file": config_file,
            "pose_source": "odom",
            "cmd_vel_topic": "/cmd_vel",
            "scan_topic": "/scan",
            "use_twist_stamped": False,
        }],
    )

    return LaunchDescription([
        DeclareLaunchArgument("config_file", default_value=default_cfg),
        gzserver, gzclient, robot_state_publisher, spawn,
        TimerAction(period=8.0, actions=[dart_node]),
    ])
