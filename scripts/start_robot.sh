#!/usr/bin/env bash
# 注意：不能加 `set -u`（nounset）。ROS 的 setup.bash/setup.sh 在 systemd 的
# 极简环境下会引用未定义变量（如 AMENT_TRACE_SETUP_FILES），nounset 会让它直接报
# "unbound variable" 退出。这里只用 errexit + pipefail。
set -eo pipefail

# systemd 是非登录 shell、环境极简，必须在这里显式 source。
# 注意：这里不等价于交互式终端的 startros alias——只 source 本项目需要的环境，
# 不引入 ros2_ws / moveit2_ws，也不加载 .bashrc 里的 MuJoCo 变量，避免环境冲突。
source /opt/ros/humble/setup.bash
source /home/yimeng/rc/School_Match_ws/install/setup.bash

STM=/dev/tty_stm32h7
BT=/dev/tty_bluetooth

# 等两个 udev 符号链接都出现（while 条件不受 set -e 影响，安全）
while [ ! -e "$STM" ] || [ ! -e "$BT" ]; do
    echo "[robot-bringup] waiting for serial devices: $STM and $BT"
    sleep 2
done

# 符号链接出现可能早于 CDC 设备真正可 open，静置 3s 再启动
echo "[robot-bringup] both devices present; settling 3s before launch"
sleep 3

# exec 让 ros2 launch 成为 systemd 的主进程，信号/退出码正确
exec ros2 launch robot_comm robot.launch.py
