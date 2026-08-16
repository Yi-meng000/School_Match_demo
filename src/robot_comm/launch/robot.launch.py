"""一次性启动 robot_comm 的两个节点。

数据链：
    蓝牙接收 bluetooth_receive_node (可执行文件 usb2topic_node) 发布 /cmd_chassis
        -> usb2uart_node 订阅 /cmd_chassis -> 发往 STM32 (/dev/tty_stm32h7)
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
            executable='usb2topic_node',
            # 关键：C++ main() 里硬编码节点名是 'bluetooth_receive_node'，
            # 而 launch 默认把 name 设成可执行名 usb2topic_node 并注入 __node:= 重映射，
            # 会悄悄把节点名改成 /usb2topic_node。这里显式写 name 保持原名不变。
            name='bluetooth_receive_node',
            output='screen',
        ),
    ])
