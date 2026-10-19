#!/usr/bin/env bash
# 打印当前关节角、夹爪、法兰位姿和指尖位置。
# 用法：arm_state

export PATH=/usr/bin:/bin:$PATH
export ROS_DOMAIN_ID="${ROS_DOMAIN_ID:-42}"

source /opt/ros/humble/setup.bash
source "$HOME/agx_arm_ws/install/setup.bash"

exec /usr/bin/python3 "$HOME/agx_arm_ws/tools/arm_state.py" "$@"
