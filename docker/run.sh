#!/usr/bin/env bash
# Helper for the DART Humble container.
#   ./docker/run.sh build     build the image
#   ./docker/run.sh up         start a shell in the container (GUI + net ready)
#   ./docker/run.sh exec       open another shell in the running container
#   ./docker/run.sh down       stop/remove the container
#
# Nothing runs unless you invoke it — this just wraps docker commands.
set -euo pipefail

IMAGE="dart:humble"
CONTAINER="dart_humble"
WS_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")/.." && pwd)"
TURTLEBOT3_MODEL="${TURTLEBOT3_MODEL:-burger}"
ROS_DOMAIN_ID="${ROS_DOMAIN_ID:-30}"   # set to match your real TurtleBot3

cmd="${1:-up}"

case "$cmd" in
  build)
    docker build -t "$IMAGE" "$WS_DIR/docker"
    ;;
  up)
    # X11 for Gazebo/RViz; --network host so a real TurtleBot3 on the LAN is reachable.
    xhost +local:root >/dev/null 2>&1 || true

    # Graphics: default to software rendering (llvmpipe) so Gazebo works without
    # GPU passthrough. Set DART_GPU=1 to pass the Intel/host GPU for speed.
    GFX=()
    # Software GL is baked into the image (ENV LIBGL_ALWAYS_SOFTWARE=1). With
    # DART_GPU=1 we instead pass the host GPU and turn software GL back off.
    if [ "${DART_GPU:-0}" = "1" ] && [ -d /dev/dri ]; then
      GFX+=(-e LIBGL_ALWAYS_SOFTWARE=0 --device /dev/dri)
      for g in video render; do
        gid="$(getent group "$g" | cut -d: -f3)"
        [ -n "$gid" ] && GFX+=(--group-add "$gid")
      done
    fi

    docker run -it --rm \
      --name "$CONTAINER" \
      --network host \
      --ipc host \
      -e DISPLAY="${DISPLAY:-:0}" \
      -e QT_X11_NO_MITSHM=1 \
      -e TURTLEBOT3_MODEL="$TURTLEBOT3_MODEL" \
      -e ROS_DOMAIN_ID="$ROS_DOMAIN_ID" \
      "${GFX[@]}" \
      -v /tmp/.X11-unix:/tmp/.X11-unix:rw \
      -v "$WS_DIR":/ws:rw \
      "$IMAGE" bash
    ;;
  exec)
    docker exec -it "$CONTAINER" bash
    ;;
  down)
    docker rm -f "$CONTAINER" >/dev/null 2>&1 || true
    ;;
  *)
    echo "usage: $0 {build|up|exec|down}" >&2
    exit 1
    ;;
esac
