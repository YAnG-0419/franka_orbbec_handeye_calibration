# FR3 + Orbbec Hand-Eye Calibration (ROS 2 Jazzy)

MoveIt hand-eye calibration for a Franka Research 3 arm and an Orbbec Gemini 435Le (networked) camera. Supports both eye-in-hand (camera on the wrist) and eye-to-hand (camera fixed, target on the hand).

It builds on three upstream sources, fetched as git submodules:

- `franka_ros2` — FR3 hardware bringup and the `franka_fr3_moveit_config` MoveIt configuration.
- `OrbbecSDK_ROS2` — the Orbbec camera driver (provides the Gemini 435Le launch).
- `moveit_calibration` — the RViz HandEyeCalibration plugin (solver and sample-collection UI).

The only first-party package is `src/fr3_handeye_calibration`, which provides the launch files and RViz presets that wire these together.

## Layout

```
scripts/00_install_tools.sh    # colcon/rosdep + MoveIt 2 + Orbbec SDK system deps
scripts/01_build.sh            # git submodule update + rosdep + colcon build
src/fr3_handeye_calibration/   # first-party package: launch files + RViz presets
docs/setup.md                  # install, build, run, troubleshooting
```

## Getting started

See [docs/setup.md](docs/setup.md) for installation, build, how to launch the camera and robot, and a Troubleshooting list of the traps worth knowing.
