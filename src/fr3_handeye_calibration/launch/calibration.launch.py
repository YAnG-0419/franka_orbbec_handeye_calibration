# One-shot bringup for FR3 + Orbbec Gemini 435Le hand-eye calibration.
#
# Starts:
#   * the Orbbec camera (camera.launch.py)
#   * FR3 hardware + MoveIt move_group + RViz preloaded with the
#     HandEyeCalibration display (fr3_moveit.launch.py)
#
# The `mode` arg selects the RViz preset (which also pre-sets the calibration
# "Sensor configuration": eye-in-hand vs eye-to-hand) -- you can still change
# everything in the GUI afterwards.
#
# Examples:
#   # Eye-to-hand, camera pinned by IP, real robot:
#   ros2 launch fr3_handeye_calibration calibration.launch.py \
#       mode:=eye_to_hand robot_ip:=169.254.67.230 \
#       enumerate_net_device:=false net_device_ip:=192.168.1.10
#
#   # Eye-in-hand, camera by serial:
#   ros2 launch fr3_handeye_calibration calibration.launch.py \
#       mode:=eye_in_hand robot_ip:=169.254.67.230 serial_number:=AY12345678
#
#   # Bring up only the robot (camera already running elsewhere):
#   ros2 launch fr3_handeye_calibration calibration.launch.py \
#       robot_ip:=169.254.67.230 launch_camera:=false

import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import (
    DeclareLaunchArgument,
    IncludeLaunchDescription,
    OpaqueFunction,
)
from launch.conditions import IfCondition
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration


def launch_setup(context, *args, **kwargs):
    pkg = get_package_share_directory('fr3_handeye_calibration')
    launch_dir = os.path.join(pkg, 'launch')

    mode = LaunchConfiguration('mode').perform(context)
    if mode not in ('eye_in_hand', 'eye_to_hand'):
        raise RuntimeError(
            f"Invalid mode '{mode}'. Use 'eye_in_hand' or 'eye_to_hand'.")
    rviz_config = os.path.join(pkg, 'rviz', f'handeye_{mode}.rviz')

    camera = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(os.path.join(launch_dir, 'camera.launch.py')),
        condition=IfCondition(LaunchConfiguration('launch_camera')),
        launch_arguments={
            'camera_name': LaunchConfiguration('camera_name'),
            'serial_number': LaunchConfiguration('serial_number'),
            'enumerate_net_device': LaunchConfiguration('enumerate_net_device'),
            'net_device_ip': LaunchConfiguration('net_device_ip'),
            'net_device_port': LaunchConfiguration('net_device_port'),
        }.items(),
    )

    robot = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(os.path.join(launch_dir, 'fr3_moveit.launch.py')),
        condition=IfCondition(LaunchConfiguration('launch_robot')),
        launch_arguments={
            'robot_ip': LaunchConfiguration('robot_ip'),
            'load_gripper': LaunchConfiguration('load_gripper'),
            'ee_id': LaunchConfiguration('ee_id'),
            'use_fake_hardware': LaunchConfiguration('use_fake_hardware'),
            'use_rviz': LaunchConfiguration('use_rviz'),
            'rviz_config': rviz_config,
        }.items(),
    )

    return [camera, robot]


def generate_launch_description():
    args = [
        DeclareLaunchArgument(
            'mode', default_value='eye_to_hand',
            description="Calibration configuration: 'eye_in_hand' or 'eye_to_hand'."),
        DeclareLaunchArgument(
            'robot_ip', default_value='172.16.26.3',
            description='FCI IP of the FR3 (override with robot_ip:= for a different robot).'),

        # camera selection (two cameras on the LAN -> identify one)
        DeclareLaunchArgument('launch_camera', default_value='true'),
        DeclareLaunchArgument('camera_name', default_value='camera'),
        DeclareLaunchArgument('serial_number', default_value=''),
        DeclareLaunchArgument('enumerate_net_device', default_value='true'),
        DeclareLaunchArgument('net_device_ip', default_value=''),
        DeclareLaunchArgument('net_device_port', default_value='8090'),

        # robot / moveit
        DeclareLaunchArgument('launch_robot', default_value='true'),
        DeclareLaunchArgument('load_gripper', default_value='false'),
        DeclareLaunchArgument('ee_id', default_value='none'),
        DeclareLaunchArgument('use_fake_hardware', default_value='false'),
        DeclareLaunchArgument('use_rviz', default_value='true'),
    ]

    return LaunchDescription(args + [OpaqueFunction(function=launch_setup)])
