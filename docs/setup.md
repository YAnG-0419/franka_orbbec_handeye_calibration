# Setup & Build

## Prerequisites

- Ubuntu 24.04 with ROS 2 Jazzy.
- libfranka installed system-wide and discoverable by CMake (`find_package(Franka)` resolves). The workspace uses it as-is and never rebuilds it.
- A real-time kernel.
- Network access to the FR3 (FCI IP) and to the Orbbec Gemini 435Le camera(s).
- Build tools are NOT included in `ros-jazzy-desktop`: `colcon` and `rosdep` ship separately (`ros-dev-tools`, or the `python3-colcon-*` / `python3-rosdep` debs). `scripts/00_install_tools.sh` installs them.

Always start from a clean shell. Examples use zsh; use `setup.bash` if your shell is bash.

```bash
conda deactivate
source /opt/ros/jazzy/setup.zsh
```

## 1. Install tools and dependencies

From the repository root:

```bash
./scripts/00_install_tools.sh
```

Installs `colcon`, `rosdep`, MoveIt 2, ros2_control, RViz, and the Orbbec SDK system libraries (`libgflags`, `nlohmann-json3`, `glog`, `ssl`, `usb`), then runs `rosdep init/update`. Uses sudo.

## 2. Fetch sources and build

The third-party sources are git submodules. If you cloned this repo without `--recurse-submodules`, the build script initializes them for you.

```bash
./scripts/01_build.sh
source install/setup.zsh
```

What the build script does:

1. `git submodule update --init --recursive` — checks out the pinned commits of `franka_ros2` (jazzy), `franka_description` (tag 2.8.0), `moveit_calibration` (ros2), and `OrbbecSDK_ROS2` (v2-main). libfranka is intentionally not a submodule; the system install is used.
2. `rosdep install ... --skip-keys "libfranka zed_wrapper"` — pulls binary deps.
3. `colcon build` with `--packages-up-to` the packages we need, `--packages-ignore` the unrelated `franka_ros2` packages (gazebo / mobile / spine / vision-kit / example controllers), and `--allow-overriding realtime_tools` (franka_ros2 bundles its own `realtime_tools` that shadows the one from the ROS install).

## 3. Run

Run the camera in its own terminal and the robot + RViz in another. Both terminals need `conda deactivate && source /opt/ros/jazzy/setup.zsh && source install/setup.zsh` first.

Terminal 1 — camera only. Close OrbbecViewer first; the 435Le is a network camera with exclusive access.

```bash
ros2 launch fr3_handeye_calibration camera.launch.py enumerate_net_device:=false net_device_ip:=<CAMERA_IP>
```

Confirm it is streaming: `ros2 topic hz /camera/color/image_raw`.

Terminal 2 — robot + MoveIt + RViz (no camera). Use `mode:=eye_in_hand` for the wrist-mounted configuration.

```bash
ros2 launch fr3_handeye_calibration calibration.launch.py mode:=eye_to_hand robot_ip:=<FCI_IP> launch_camera:=false
```

RViz opens with the HandEyeCalibration display preloaded (frames and sensor mount type already set for the chosen `mode`). `calibration.launch.py` can also start the camera itself with `launch_camera:=true` plus the camera selectors, but the two-terminal split is easier to debug.

## 4. (Optional) Orbbec udev rules

Only needed for USB enumeration (network cameras don't strictly need it):

```bash
sudo cp src/OrbbecSDK_ROS2/orbbec_camera/udev/*.rules /etc/udev/rules.d/
sudo udevadm control --reload-rules && sudo udevadm trigger
```

## Why these versions

- `franka_ros2`'s CMake requires `find_package(Franka 0.19.0)`. A system libfranka of 0.19.0 or newer satisfies this (AnyNewerVersion), so no rebuild is needed — do not rebuild libfranka or add it as a submodule.
- `moveit_calibration`'s `ros2` branch was updated for Jazzy (Nov 2025) and is the only branch that builds here.
- `OrbbecSDK_ROS2` must be the `v2-main` branch (the repo's default branch / modern v2 SDK). The legacy `main` branch does not ship `gemini435_le.launch.py` and cannot drive the 435Le. The Gemini 435Le is a network camera, so we build the driver from source (the `ros-jazzy-orbbec-camera` debian doesn't have the 435Le launch either).

## Troubleshooting

- `colcon` / `rosdep` not found. They are not part of `ros-jazzy-desktop`. Run `scripts/00_install_tools.sh` (or `sudo apt install ros-dev-tools`).
- Build fails with `ModuleNotFoundError: No module named 'catkin_pkg'` (CMake calling a conda Python). A conda env is active and CMake's FindPython picks the conda interpreter via `CONDA_PREFIX`. Run `conda deactivate` before building — removing conda from `PATH` alone is not enough, `CONDA_PREFIX` must be unset too.
- Camera node starts but there are no `/camera/...` image topics (only `/camera/device_status`). Almost always another client is holding the camera — the 435Le has exclusive access, so close OrbbecViewer (and any other client) first. A real connection logs `Connecting to device with net ip: <ip>`. If you pass camera args by hand, note that with `enumerate_net_device:=false` the `net_device_port` must be non-zero (our launch defaults it to 8090).
- `libfranka: UDP receive: Timeout`. The FR3 needs a real-time kernel and FCI activated (Franka Desk, joints unlocked, green light); check the FCI IP is reachable (`ping <robot_ip>`).
- Markers are detected but the drawn target axis is wrong / does not follow the board, with a HandEyeCalibration warning `Target detector has not received reasonable intrinsics`. The Orbbec driver publishes 8 distortion coefficients but labels the model `plumb_bob` (nominally 5); stock moveit_calibration rejects that CameraInfo and falls back to an identity camera matrix, so the pose is garbage. The `moveit_calibration` submodule points at a fork (`hesic73/moveit_calibration`, branch `ros2`) that accepts the published distortion length as-is. After (re)building, restart RViz so the new plugin loads.

## Useful commands

```bash
# list connected Orbbec devices (serials / IPs) to pick the right one
ros2 run orbbec_camera list_devices_node
src/OrbbecSDK_ROS2/orbbec_camera/scripts/list_ob_devices.sh

# verify the camera is actually streaming
ros2 topic hz /camera/color/image_raw
```
