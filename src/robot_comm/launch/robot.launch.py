"""
Start the three robot_comm nodes.

数据链：
    bluetooth_receive_node 发布 /cmd_controller
        -> comm_mux_node（robot_control）发布 /cmd_chassis
        -> usb2uart_node 发往 STM32 (/dev/tty_stm32h7)

    STM32 轮速逆运动学反馈
        -> usb2uart_node 发布 /chassis_feedback
        -> wheel_velocity_node 发布 /wheel_velocity
"""

from launch import LaunchDescription
from launch_ros.actions import Node


def generate_launch_description():
    return LaunchDescription([
        Node(
            package='robot_comm',
            executable='usb2uart_node',
            name='usb2uart_node',
            output='screen',
        ),
        Node(
            package='robot_comm',
            executable='wheel_velocity_node',
            name='wheel_velocity_node',
            output='screen',
        ),
        Node(
            package='robot_comm',
            executable='usb2topic_node',
            # C++ main() 中的节点名是 bluetooth_receive_node；这里显式保持一致。
            name='bluetooth_receive_node',
            output='screen',
        ),
    ])
