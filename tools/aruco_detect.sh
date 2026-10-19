#!/usr/bin/env bash
# ArUco 标定板检测节点：发布 /aruco_single/pose，供手眼标定使用。
#
# 用法：
#   aruco_detect                        默认 4x4_50、id=0、50 mm
#   aruco_detect --marker-id 3
#   aruco_detect --marker-size 0.1
#
# 需要相机已启动（默认话题 /camera/camera/color/...）。

export PATH=/usr/bin:/bin:$PATH
export ROS_DOMAIN_ID="${ROS_DOMAIN_ID:-42}"

source /opt/ros/humble/setup.bash
source "$HOME/agx_arm_ws/install/setup.bash"

exec /usr/bin/python3 "$HOME/agx_arm_ws/tools/aruco_detect.py" "$@"
