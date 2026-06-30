#!/usr/bin/env bash
# Install build tooling + binary dependencies for the FR3 hand-eye calibration
# workspace on ROS 2 Jazzy. Run ONCE (uses sudo). Safe to re-run.
set -euo pipefail

if [[ "${ROS_DISTRO:-}" != "jazzy" ]]; then
  echo "ROS_DISTRO is not jazzy (got '${ROS_DISTRO:-unset}')." >&2
  exit 1
fi

echo ">> Installing colcon / rosdep + MoveIt 2 + Orbbec SDK system deps"
sudo apt-get update
sudo apt-get install -y \
  python3-colcon-common-extensions \
  python3-rosdep \
  ros-jazzy-moveit \
  ros-jazzy-rviz2 \
  ros-jazzy-ros2-control \
  ros-jazzy-ros2-controllers \
  ros-jazzy-controller-manager \
  ros-jazzy-xacro \
  ros-jazzy-joint-state-publisher \
  libgflags-dev \
  nlohmann-json3-dev \
  libgoogle-glog-dev \
  libssl-dev \
  libusb-1.0-0-dev

echo ">> Initializing rosdep"
if [[ ! -f /etc/ros/rosdep/sources.list.d/20-default.list ]]; then
  sudo rosdep init
fi
rosdep update
