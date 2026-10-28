#!/usr/bin/env bash
# Start the RealSense color stream used by the ChArUco detector.
set -e

export PATH=/usr/bin:/bin:$PATH
export ROS_DOMAIN_ID="${ROS_DOMAIN_ID:-42}"

source /opt/ros/humble/setup.bash

exec ros2 launch realsense2_camera rs_launch.py \
  enable_depth:=false \
  enable_color:=true \
  rgb_camera.color_profile:=1280x720x30
