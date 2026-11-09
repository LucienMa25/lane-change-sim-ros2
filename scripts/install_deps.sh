#!/usr/bin/env bash
set -euo pipefail

echo "[install_deps] Install ROS1 runtime deps..."
sudo apt-get update
sudo apt-get install -y \
  ros-noetic-rviz \
  ros-noetic-robot-state-publisher \
  ros-noetic-joint-state-publisher \
  ros-noetic-xacro

echo "[install_deps] Done."
echo "[install_deps] You can now build:"
echo "  cd ~/catkin_ws && source /opt/ros/noetic/setup.bash && catkin_make -DCMAKE_POLICY_VERSION_MINIMUM=3.5"
