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
                "-0.0434372",
                "--y",
                "-0.39801",
                "--z",
                "0.292056",
                "--qx",
                "-0.38606",
                "--qy",
                "0.705046",
                "--qz",
                "-0.16033",
                "--qw",
                "0.572854",
                # "--roll",
                # "0.636974",
                # "--pitch",
                # "1.94288",
                # "--yaw",
                # "-1.44506",
            ],
        ),
    ]
    return LaunchDescription(nodes)
