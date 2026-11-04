import os
from glob import glob

from setuptools import find_packages, setup

package_name = "dart_ros"

setup(
    name=package_name,
    version="0.1.0",
    packages=find_packages(exclude=["test"]),
    data_files=[
        ("share/ament_index/resource_index/packages", ["resource/" + package_name]),
        ("share/" + package_name, ["package.xml"]),
        (os.path.join("share", package_name, "launch"), glob("launch/*.launch.py")),
        (os.path.join("share", package_name, "config"), glob("config/*.yaml")),
        (os.path.join("share", package_name, "worlds"), glob("worlds/*.world")),
        (os.path.join("share", package_name, "rviz"), glob("rviz/*.rviz")),
    ],
    install_requires=["setuptools"],
    zip_safe=True,
    maintainer="chaiyapruk",
    maintainer_email="chaiyapruk6494@gmail.com",
    description="ROS2 node layer for DART on TurtleBot3.",
    license="MIT",
    entry_points={
        "console_scripts": [
            # The online decision loop node (DART brain on the robot/sim).
            "dart_node = dart_ros.dart_node:main",
        ],
    },
)
