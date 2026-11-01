#!/usr/bin/env bash
# Fetch and (when possible) build LI-Init / LiDAR_IMU_Init.
#
# The original hku-mars package is ROS1-oriented.  If the cloned branch is a
# ROS2/ament package it is built with colcon; otherwise follow the printed
# instructions and run it in a ROS1 Noetic environment/container.
set -euo pipefail

WS="$(cd "$(dirname "${BASH_SOURCE[0]}")/../../../.." && pwd)"
DEST="$WS/src/calibration/third_party/LiDAR_IMU_Init"
REPO="${LI_INIT_REPO:-https://github.com/hku-mars/LiDAR_IMU_Init.git}"

mkdir -p "$(dirname "$DEST")"
if [ ! -d "$DEST/.git" ]; then
  echo "cloning $REPO -> $DEST"
  git clone --depth 1 "$REPO" "$DEST"
else
  echo "already cloned: $DEST"
fi

if [ ! -f "$DEST/package.xml" ]; then
  echo "package.xml not found; check the repository layout."
  exit 1
fi

NAME="$(sed -n 's:.*<name>\(.*\)</name>.*:\1:p' "$DEST/package.xml" | head -1)"
if grep -qE "ament_cmake|ament_python" "$DEST/package.xml"; then
  echo "ROS2 package '$NAME' detected; building with colcon"
  cd "$WS"
  colcon build --packages-select "$NAME"
  echo "run: ros2 launch $NAME ..."
else
  echo "This is a ROS1/catkin package ('$NAME')."
  echo "Run it in a ROS1 Noetic workspace or container, for example:"
  echo "  mkdir -p ~/li_init_ws/src && cd ~/li_init_ws/src"
  echo "  ln -s $DEST ."
  echo "  cd ~/li_init_ws && catkin_make"
  echo "  source devel/setup.bash && roslaunch $NAME li_init.launch"
fi
