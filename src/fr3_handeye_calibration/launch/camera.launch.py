# Brings up a single Orbbec Gemini 435Le (network) camera for hand-eye calibration.
#
# This is a thin wrapper around OrbbecSDK_ROS2's gemini435_le.launch.py. Because
# two cameras are connected on the LAN, you MUST identify which one to use, by
# either serial_number or net_device_ip (see args below).
#
# Examples:
#   # Auto-enumerate (only safe with a single camera on the LAN):
#   ros2 launch fr3_handeye_calibration camera.launch.py
#
#   # Select a specific camera by static IP (recommended for 2 cameras):
#   ros2 launch fr3_handeye_calibration camera.launch.py \
#       enumerate_net_device:=false net_device_ip:=192.168.1.10
#
#   # Or select by serial number:
#   ros2 launch fr3_handeye_calibration camera.launch.py serial_number:=AY12345678
#
# Resulting (with default camera_name:=camera):
#   color image   : /camera/color/image_raw
#   color info    : /camera/color/camera_info
#   optical frame : camera_color_optical_frame

import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration


def generate_launch_description():
    orbbec_launch = os.path.join(
        get_package_share_directory('orbbec_camera'),
        'launch',
        'gemini435_le.launch.py',
    )

    args = [
        DeclareLaunchArgument(
            'camera_name', default_value='camera',
            description='Namespace/prefix for camera topics and TF frames.'),
        DeclareLaunchArgument(
            'serial_number', default_value='',
            description='Select a specific camera by serial number (blank = ignore).'),
        DeclareLaunchArgument(
            'enumerate_net_device', default_value='true',
            description='true: auto-discover the camera on the LAN. '
                        'Set false and provide net_device_ip to pin a specific camera.'),
        DeclareLaunchArgument(
            'net_device_ip', default_value='',
            description='Static IP of the target camera (used when enumerate_net_device:=false).'),
        DeclareLaunchArgument(
            'net_device_port', default_value='8090',
            description='Network port of the target camera. MUST be non-zero when '
                        'enumerate_net_device:=false, or the driver silently skips '
                        'connecting (8090 is the Orbbec default). Ignored when auto-enumerating.'),
        DeclareLaunchArgument(
            'enable_point_cloud', default_value='false',
            description='Point cloud is not needed for calibration; off by default to save CPU.'),
        DeclareLaunchArgument(
            'enable_colored_point_cloud', default_value='false'),
        DeclareLaunchArgument(
            'depth_registration', default_value='true'),
    ]

    forwarded = {
        'camera_name': LaunchConfiguration('camera_name'),
        'serial_number': LaunchConfiguration('serial_number'),
        'enumerate_net_device': LaunchConfiguration('enumerate_net_device'),
        'net_device_ip': LaunchConfiguration('net_device_ip'),
        'net_device_port': LaunchConfiguration('net_device_port'),
        'enable_point_cloud': LaunchConfiguration('enable_point_cloud'),
        'enable_colored_point_cloud': LaunchConfiguration('enable_colored_point_cloud'),
        'depth_registration': LaunchConfiguration('depth_registration'),
    }

    camera = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(orbbec_launch),
        launch_arguments=forwarded.items(),
    )

    return LaunchDescription(args + [camera])
