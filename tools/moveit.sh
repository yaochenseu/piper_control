#!/usr/bin/env bash
# MoveIt 快捷启动：自动加载 ROS 环境，启动 MoveIt + RViz。
#
# 前置条件：机械臂驱动已在运行（另开终端或后台），且 ROS_DOMAIN_ID 一致。
#   ros2 launch agx_arm_ctrl start_single_agx_arm.launch.py \
#       can_port:=can0 arm_type:=piper_x effector_type:=agx_gripper speed_percent:=15
#
# 用法：
#   moveit                 启动 MoveIt + RViz（默认开启执行期门控）
#   moveit use_rviz:=false 只启动 MoveIt，不开 RViz
#   moveit auto_control_gate:=false  显式关闭门控
#   moveit                 Ctrl+C 退出
#
# 驱动当前以 ROS_DOMAIN_ID=42 启动；若驱动换了域，
# 用 ROS_DOMAIN_ID=xx moveit 覆盖。

export PATH=/usr/bin:/bin:$PATH
export ROS_DOMAIN_ID="${ROS_DOMAIN_ID:-42}"
export LC_NUMERIC=en_US.UTF-8

source /opt/ros/humble/setup.bash
source "$HOME/agx_arm_ws/install/setup.bash"

# 默认开启执行期门控；用户显式传 auto_control_gate:=false 时以用户为准。
GATE_ARGS=()
HAS_GATE=0
for arg in "$@"; do
    case "$arg" in
        auto_control_gate:=*) HAS_GATE=1 ;;
    esac
done
if [ "$HAS_GATE" -eq 0 ]; then
    GATE_ARGS+=(auto_control_gate:=true)
fi

exec ros2 launch agx_arm_moveit demo.launch.py \
    arm_type:=piper_x \
    effector_type:=agx_gripper \
    follow:=true \
    "${GATE_ARGS[@]}" \
    "$@"
