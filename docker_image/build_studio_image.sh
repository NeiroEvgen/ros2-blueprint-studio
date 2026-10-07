#!/usr/bin/env bash
# Собирает базовый образ студии blueprint-studio:humble.
# Запускать из любой папки. Нужен интернет (первая сборка ~10-20 минут).
set -euo pipefail
here="$(cd "$(dirname "$0")" && pwd)"
docker build -t blueprint-studio:humble -f "$here/Dockerfile.studio" "$here"
echo
echo "Готово. Проверка:"
docker run --rm blueprint-studio:humble bash -c \
  "source /opt/ros/humble/setup.bash && ros2 pkg list | grep -E 'controller_manager|moveit_ros_move_group|foxglove_bridge|turtlesim|kdl_parser'"
