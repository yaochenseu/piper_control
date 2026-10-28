#!/usr/bin/env bash
# Start the Piper X driver in CAN control mode, with the external control gate
# closed.  Run this in its own terminal; the calibration script opens the gate.
set -e

export PATH=/usr/bin:/bin:$PATH
export ROS_DOMAIN_ID="${ROS_DOMAIN_ID:-42}"

AGX_WS="${AGX_WS:-$HOME/agx_arm_ws}"
CAN_PORT="${CAN_PORT:-can0}"
ROBOT="${ROBOT:-piper_x}"
EFFECTOR="${EFFECTOR:-agx_gripper}"
SPEED_PERCENT="${SPEED_PERCENT:-5}"

source /opt/ros/humble/setup.bash
source "$AGX_WS/install/setup.bash"

exec ros2 launch agx_arm_ctrl start_single_agx_arm.launch.py \
  can_port:="$CAN_PORT" \
  arm_type:="$ROBOT" \
  effector_type:="$EFFECTOR" \
  speed_percent:="$SPEED_PERCENT" \
  auto_enable:=true \
  control_enabled:=false
