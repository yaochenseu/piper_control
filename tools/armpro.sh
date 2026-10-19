#!/usr/bin/env bash
# armpro：MoveIt 规划执行（Route B）。
# 用法与 arm 一致：armpro j x x -30 x x x / armpro p x x x+10
# 需要先启动 MoveIt（moveit 命令，follow:=true）；未启动时可确认降级为 arm。

export PATH=/usr/bin:/bin:$PATH
export ROS_DOMAIN_ID="${ROS_DOMAIN_ID:-42}"
export LC_NUMERIC=en_US.UTF-8

source /opt/ros/humble/setup.bash
source "$HOME/agx_arm_ws/install/setup.bash"

exec /usr/bin/python3 "$HOME/agx_arm_ws/tools/armpro.py" "$@"
