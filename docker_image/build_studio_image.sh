#!/usr/bin/env bash
# Builds the Studio base image blueprint-studio:humble.
# Run from any folder. Needs internet (the first build takes ~10-20 minutes).
set -euo pipefail
here="$(cd "$(dirname "$0")" && pwd)"
docker build -t blueprint-studio:humble -f "$here/Dockerfile.studio" "$here"
echo
echo "Done. Check:"
docker run --rm blueprint-studio:humble bash -c \
  "source /opt/ros/humble/setup.bash && ros2 pkg list | grep -E 'controller_manager|moveit_ros_move_group|foxglove_bridge|turtlesim|kdl_parser'"
