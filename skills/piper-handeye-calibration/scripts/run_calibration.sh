#!/usr/bin/env bash
# Run automatic hand-eye calibration.  Pass --dry-run first.
set -e

export PATH=/usr/bin:/bin:$PATH
export ROS_DOMAIN_ID="${ROS_DOMAIN_ID:-42}"
export PYTHONUNBUFFERED=1

AGX_WS="${AGX_WS:-$HOME/agx_arm_ws}"
SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
SKILL_ROOT="$(cd "$SCRIPT_DIR/.." && pwd)"

source /opt/ros/humble/setup.bash
source "$AGX_WS/install/setup.bash"

exec /usr/bin/python3 "$SCRIPT_DIR/auto_calibrate.py" \
  --config "$SKILL_ROOT/config/default.json" "$@"
