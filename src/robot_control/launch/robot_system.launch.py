"""Start the communication and control nodes for the complete robot stack."""

from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import PathJoinSubstitution
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare


def generate_launch_description():
    robot_comm_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            PathJoinSubstitution([
                FindPackageShare("robot_comm"),
                "launch",
                "robot.launch.py",
            ])
        )
    )

    return LaunchDescription([
        robot_comm_launch,
        Node(
            package="robot_control",
            executable="commmux_node",
            name="comm_mux_node",
            output="screen",
        ),
        Node(
            package="robot_control",
            executable="tracing_node",
            name="tracing_node",
            output="screen",
        ),
    ])
