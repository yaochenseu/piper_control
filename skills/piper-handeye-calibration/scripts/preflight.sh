#!/usr/bin/env bash
# Non-moving preflight for the hand-eye calibration skill.
set -e

export PATH=/usr/bin:/bin:$PATH
export ROS_DOMAIN_ID="${ROS_DOMAIN_ID:-42}"

AGX_WS="${AGX_WS:-$HOME/agx_arm_ws}"
CAN_PORT="${CAN_PORT:-can0}"
SKILL_ROOT="$(cd "$(dirname "${BASH_SOURCE[0]}")/.." && pwd)"

source /opt/ros/humble/setup.bash
source "$AGX_WS/install/setup.bash"

fail() {
  printf '❌ %s\n' "$*" >&2
  exit 1
}

echo "== CAN =="
ip -details link show "$CAN_PORT" 2>/dev/null \
  | grep -E "$CAN_PORT|bitrate|state" || fail "找不到或无法读取 $CAN_PORT"
ip -details link show "$CAN_PORT" 2>/dev/null \
  | grep -q "state UP" || fail "$CAN_PORT 未 UP"
ip -details link show "$CAN_PORT" 2>/dev/null \
  | grep -q "bitrate 1000000" || fail "$CAN_PORT 不是 1 Mbit/s"
echo "✅ CAN $CAN_PORT UP, 1 Mbit/s"

echo "== ROS 服务 =="
ros2 service list 2>/dev/null | grep -qx "/control_enable" \
  || fail "缺少 /control_enable（驱动未启动？）"
ros2 service list 2>/dev/null | grep -qx "/enable_agx_arm" \
  || fail "缺少 /enable_agx_arm（驱动未启动？）"

echo "== 反馈话题 =="
for topic in /feedback/joint_states /feedback/tcp_pose /feedback/arm_status; do
  ros2 topic list 2>/dev/null | grep -qx "$topic" \
    || fail "缺少 $topic"
done
timeout 5 ros2 topic echo /camera/camera/color/camera_info --once >/dev/null 2>&1 \
  || fail "没有 /camera/camera/color/camera_info（相机未启动？）"
timeout 5 ros2 topic echo /aruco_single/pose --once >/dev/null 2>&1 \
  || fail "没有 /aruco_single/pose（检测器未启动或板不可见）"

echo "== 机械臂状态 =="
STATUS="$(timeout 5 ros2 topic echo /feedback/arm_status --once 2>/dev/null)" \
  || fail "读取 /feedback/arm_status 失败"
printf '%s\n' "$STATUS"
grep -q "ctrl_mode: 1" <<<"$STATUS" \
  || fail "不是 CAN 控制模式（ctrl_mode 需要 1；2 是示教模式）"
grep -q "arm_status: 0" <<<"$STATUS" \
  || fail "arm_status 非 0"
grep -q "err_status: 0" <<<"$STATUS" \
  || fail "err_status 非 0"

if ros2 topic info /control/joint_states 2>/dev/null \
    | grep -qE "Publisher count: [1-9]"; then
  echo "⚠️ 检测到 /control/joint_states 控制流，MoveIt 可能正在运行；标定期间不要用 MoveIt"
fi

mkdir -p "$SKILL_ROOT/../../tools/calib_results"
echo "✅ 预检通过；现在可以用 run_calibration.sh --dry-run 查看候选姿态"
