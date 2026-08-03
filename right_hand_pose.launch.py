""" Static transform publisher acquired via MoveIt 2 hand-eye calibration """
""" EYE-TO-HAND: fr3_link0 -> camera_color_optical_frame """
from launch import LaunchDescription
from launch_ros.actions import Node


def generate_launch_description() -> LaunchDescription:
    nodes = [
        Node(
            package="tf2_ros",
            executable="static_transform_publisher",
            output="log",
            arguments=[
                "--frame-id",
                "fr3_link0",
                "--child-frame-id",
                "camera_color_optical_frame",
                "--x",
                "-0.024517",
                "--y",
                "0.421091",
                "--z",
                "0.256964",
                "--qx",
                "0.682676",
                "--qy",
                "-0.383116",
                "--qz",
                "0.604902",
                "--qw",
                "-0.145841",
                # "--roll",
                # "2.27734",
                # "--pitch",
                # "1.21581",
                # "--yaw",
                # "1.64385",
            ],
        ),
    ]
    return LaunchDescription(nodes)
