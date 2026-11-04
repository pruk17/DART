# DART workspace — ROS2 Humble on Ubuntu 22.04 (matches the pruk17/DART target).
# The host here is Ubuntu 24.04 / Jazzy, so Humble runs inside this container.
#
# Build:  docker/run.sh build
# Run:    docker/run.sh up
FROM osrf/ros:humble-desktop

ARG DEBIAN_FRONTEND=noninteractive

# TurtleBot3 + Gazebo + Nav2/AMCL stack. These are declared here but only
# installed when you actually build the image (nothing is installed on the host).
RUN apt-get update && apt-get install -y --no-install-recommends \
        python3-pip \
        python3-colcon-common-extensions \
        python3-yaml \
        ros-humble-turtlebot3 \
        ros-humble-turtlebot3-msgs \
        ros-humble-turtlebot3-gazebo \
        ros-humble-turtlebot3-simulations \
        ros-humble-nav2-bringup \
        ros-humble-navigation2 \
        ros-humble-gazebo-ros-pkgs \
    && rm -rf /var/lib/apt/lists/*

# NOTE: do NOT upgrade setuptools here — Humble's colcon/ament_python builds
# require the distro setuptools (59.6). The `dart` pip package uses a classic
# setup.py so `pip install -e` works on that version without PEP 660.

# Default TurtleBot3 model (override at runtime with -e TURTLEBOT3_MODEL=waffle).
ENV TURTLEBOT3_MODEL=burger

# Force software OpenGL (llvmpipe) so Gazebo renders without GPU passthrough.
# run.sh sets this to 0 when DART_GPU=1 to use the host GPU instead.
ENV LIBGL_ALWAYS_SOFTWARE=1

# Stop gzserver hanging on first launch while it queries the online model
# database (models.gazebosim.org); all TurtleBot3 models are local anyway.
ENV GAZEBO_MODEL_DATABASE_URI=""
# Must include the DEFAULT gazebo models (ground_plane, sun) or the robot falls
# through the floor; plus the TurtleBot3 models the world references.
ENV GAZEBO_MODEL_PATH=/usr/share/gazebo-11/models:/opt/ros/humble/share/turtlebot3_gazebo/models

# Auto-source ROS + the overlay for interactive shells, and make the pure-Python
# `dart` library importable via an editable pip install (it is NOT a ROS package).
# The workspace is bind-mounted at run time, so the install runs on first shell.
RUN echo 'source /opt/ros/humble/setup.bash' >> /root/.bashrc && \
    echo '[ -f /ws/install/setup.bash ] && source /ws/install/setup.bash' >> /root/.bashrc && \
    echo 'export TURTLEBOT3_MODEL=${TURTLEBOT3_MODEL:-burger}' >> /root/.bashrc && \
    echo 'pip show dart >/dev/null 2>&1 || pip install -e /ws/src/DART >/dev/null 2>&1' >> /root/.bashrc

WORKDIR /ws
# The workspace (DART_ws) is bind-mounted to /ws at run time — see run.sh.
CMD ["bash"]
