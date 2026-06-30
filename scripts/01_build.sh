#!/usr/bin/env bash
# Fetch source submodules, resolve dependencies, and build the workspace.
# Prereq: scripts/00_install_tools.sh has been run and ROS 2 Jazzy is sourced.
set -euo pipefail

WS="$(cd "$(dirname "${BASH_SOURCE[0]}")/.." && pwd)"
cd "$WS"

if [[ "${ROS_DISTRO:-}" != "jazzy" ]]; then
  echo "ROS_DISTRO is not jazzy (got '${ROS_DISTRO:-unset}')." >&2
  exit 1
fi

echo ">> Fetching source submodules (franka_ros2, franka_description, moveit_calibration, OrbbecSDK_ROS2)"
# libfranka is intentionally NOT a submodule; the system install is used.
# moveit_calibration points at a fork (hesic73/moveit_calibration, branch ros2)
# that accepts CameraInfo with a non-standard distortion vector length (Orbbec
# publishes 8 coefficients labeled "plumb_bob"); see docs/setup.md.
git submodule update --init --recursive

echo ">> Resolving binary dependencies with rosdep"
# Skip libfranka (provided system-wide) and zed_wrapper (unused franka_ros2 dep).
rosdep install --from-paths src --ignore-src -r -y \
  --rosdistro jazzy \
  --skip-keys "libfranka zed_wrapper"

echo ">> Building"
# --packages-ignore: franka_ros2's monorepo bundles Gazebo/mobile/spine/example
# packages that pull heavy deps (e.g. franka_gazebo_hardware -> gz-sim8); skip them.
# --allow-overriding: franka_ros2 ships its own realtime_tools that shadows the
# one from the ROS install.
colcon build \
  --symlink-install \
  --allow-overriding realtime_tools \
  --cmake-args -DCMAKE_BUILD_TYPE=Release -DBUILD_TESTS=OFF \
  --packages-ignore \
    franka_ros2 \
    franka_gazebo_hardware \
    franka_gazebo_bringup \
    franka_example_controllers \
    franka_mobile \
    franka_mobile_fr3_duo_moveit_config \
    franka_mobile_sensors \
    mobile_fr3_duo_trajectory_controller \
    franka_spine_examples \
    franka_spine_msgs \
    franka_spine_server \
    franka_vision_and_manipulation_kit \
    franka_selfcollision \
  --packages-up-to \
    fr3_handeye_calibration \
    franka_fr3_moveit_config \
    franka_robot_state_broadcaster \
    moveit_calibration_gui \
    orbbec_camera

echo ">> Build complete."
