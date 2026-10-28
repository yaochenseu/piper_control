#!/usr/bin/env bash
# Start the local ChArUco detector.  It publishes /aruco_single/pose.
set -e

export PATH=/usr/bin:/bin:$PATH
export ROS_DOMAIN_ID="${ROS_DOMAIN_ID:-42}"

SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
AGX_WS="${AGX_WS:-$HOME/agx_arm_ws}"

SQUARES_X="${SQUARES_X:-7}"
SQUARES_Y="${SQUARES_Y:-5}"
SQUARE_LENGTH="${SQUARE_LENGTH:-0.03}"
MARKER_LENGTH="${MARKER_LENGTH:-0.022}"
DICTIONARY="${DICTIONARY:-DICT_4X4_50}"
BLUR="${BLUR:-3}"

source /opt/ros/humble/setup.bash
source "$AGX_WS/install/setup.bash"

exec /usr/bin/python3 "$SCRIPT_DIR/aruco_detect.py" \
  --board charuco \
  --dictionary "$DICTIONARY" \
  --squares-x "$SQUARES_X" \
  --squares-y "$SQUARES_Y" \
  --square-length "$SQUARE_LENGTH" \
  --marker-length "$MARKER_LENGTH" \
  --blur "$BLUR"
