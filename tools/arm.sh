#!/usr/bin/env bash
# 机械臂控制快捷入口：自动加载 ROS 环境并调用 arm.py。
#
# 用法：
#   arm j x x -30 x x x            关节控制
#   arm j x+10 x x x x x 0.5 -f 1.0
#   arm p x x x+10                位姿控制
#   arm p 96.9 0 300 -85 0 -90 -l
#   arm j -n x x -30 x x x        只预演
#   arm                           查看帮助
#
# 驱动当前以 ROS_DOMAIN_ID=42 启动；若驱动换了域，
# 用 ROS_DOMAIN_ID=xx arm ... 覆盖。

export PATH=/usr/bin:/bin:$PATH
export ROS_DOMAIN_ID="${ROS_DOMAIN_ID:-42}"

source /opt/ros/humble/setup.bash
source "$HOME/agx_arm_ws/install/setup.bash"

ARM_PY="$HOME/agx_arm_ws/tools/arm.py"

if [ $# -eq 0 ]; then
    exec /usr/bin/python3 "$ARM_PY" --help
fi

exec /usr/bin/python3 "$ARM_PY" "$@"
