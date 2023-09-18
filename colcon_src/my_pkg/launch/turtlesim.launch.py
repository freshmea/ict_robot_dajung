from launch import LaunchDescription
from launch_ros.actions import Node
from launch.substitutions import LaunchConfiguration
from ament_index_python.packages import get_package_share_directory
from launch.actions import DeclareLaunchArgument, ExecuteProcess
import os


def generate_launch_description():
    param_dir = LaunchConfiguration(
        "param_dir",
        default=os.path.join(
            get_package_share_directory("my_pkg"), "param", "turtlesim.yaml"
        ),
    )

    return LaunchDescription(
        [
            DeclareLaunchArgument(
                "param_dir",
                default_value=param_dir,
            ),
            Node(
                package="turtlesim",
                node_executable="turtlesim_node",
                parameters=[param_dir],
                output="screen",
            ),
            ExecuteProcess(
                cmd=[
                    "ros2 service call",
                    "/spawn",
                    "turtlesim/srv/Spawn",
                    '"{x: 3, y: 7, theta: 0.2}"',
                ],
                shell=True,
            ),
            Node(
                package="move_turtle",
                node_executable="mturtle",
                output="screen",
            ),
        ]
    )
