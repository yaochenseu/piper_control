#!/usr/bin/env bash
# 眼在手上手眼标定：采集标定板位姿 + 末端位姿，计算 T_tcp_cam。
#
# 前置：
#   1. 驱动已运行（另一终端）
#   2. 相机已启动：ros2 launch realsense2_camera rs_launch.py
#   3. 标定板检测已运行：aruco_detect
#
# 用法：
#   calib                 默认最少 10 组
#   calib --min 15
#   calib --marker-size 0.05 --marker-id 0
#
# 采集时在另一终端用 arm 命令移动机械臂，停稳后回本终端按 Enter。

export PATH=/usr/bin:/bin:$PATH
export ROS_DOMAIN_ID="${ROS_DOMAIN_ID:-42}"

source /opt/ros/humble/setup.bash
source "$HOME/agx_arm_ws/install/setup.bash"

exec /usr/bin/python3 "$HOME/agx_arm_ws/tools/calib.py" "$@"
