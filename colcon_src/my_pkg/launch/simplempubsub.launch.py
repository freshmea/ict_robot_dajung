from launch import LaunchDescription
from launch_ros.actions import Node


def generate_launch_description():
    return LaunchDescription(
        [
            Node(package="my_pkg", node_executable="mpub", output="screen"),
            Node(package="my_pkg", node_executable="msub", output="screen"),
        ]
    )
