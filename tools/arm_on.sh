#!/usr/bin/env bash
# 一键启动 Piper X：检查 CAN -> 启动驱动 -> 使能 -> 打开控制门 -> 打印状态。
#
# 用法：
#   arm_on                 # 默认 can0，5% 速度
#   arm_on can1            # 指定 CAN 接口
#   SPEED_PERCENT=10 arm_on
#
# 当前终端保持运行；Ctrl-C 停止驱动。

export PATH=/usr/bin:/bin:$PATH
export ROS_DOMAIN_ID="${ROS_DOMAIN_ID:-42}"

CAN_PORT="${1:-${CAN_PORT:-can0}}"
ARM_TYPE="${ARM_TYPE:-piper_x}"
EFFECTOR_TYPE="${EFFECTOR_TYPE:-agx_gripper}"
SPEED_PERCENT="${SPEED_PERCENT:-5}"

DRIVER_PID=""
LOG_FILE=""

info() { printf '%s\n' "$*"; }

fail() {
    printf '❌ %s\n' "$*" >&2
    if [[ -n "$LOG_FILE" && -f "$LOG_FILE" ]]; then
        printf '%s\n' '--- 驱动日志最后 20 行 ---' >&2
        tail -n 20 "$LOG_FILE" >&2
    fi
    exit 1
}

cleanup() {
    trap - INT TERM EXIT
    if [[ -n "$DRIVER_PID" ]] && kill -0 "$DRIVER_PID" 2>/dev/null; then
        info "停止驱动..."
        kill -INT "$DRIVER_PID" 2>/dev/null || true
        for _ in {1..25}; do
            kill -0 "$DRIVER_PID" 2>/dev/null || break
            sleep 0.2
        done
        kill -TERM "$DRIVER_PID" 2>/dev/null || true
        wait "$DRIVER_PID" 2>/dev/null || true
    fi
}
trap cleanup EXIT
trap 'exit 130' INT
trap 'exit 143' TERM

source /opt/ros/humble/setup.bash || fail "无法加载 ROS 2 Humble 环境"
source "$HOME/agx_arm_ws/install/setup.bash" || fail "无法加载 ~/agx_arm_ws/install/setup.bash"

if pgrep -f 'agx_arm_ctrl_single' >/dev/null 2>&1; then
    fail "已有 agx_arm_ctrl_single 在运行；先在旧终端 Ctrl-C 停掉。"
fi

if ! ip link show "$CAN_PORT" >/dev/null 2>&1; then
    fail "找不到 CAN 接口 $CAN_PORT（用 ip -br link show type can 查看）"
fi

if ip -details link show "$CAN_PORT" 2>/dev/null | grep -q 'can state BUS-OFF'; then
    fail "CAN $CAN_PORT 处于 BUS-OFF；请给机械臂/适配器断电重上电后再试。"
fi

if ! { ip -details link show "$CAN_PORT" 2>/dev/null | grep -q 'state UP' \
       && ip -details link show "$CAN_PORT" 2>/dev/null | grep -q 'bitrate 1000000'; }; then
    info "配置 CAN $CAN_PORT 为 1 Mbit/s（需要 sudo）..."
    sudo ip link set "$CAN_PORT" down 2>/dev/null || true
    sudo ip link set "$CAN_PORT" type can bitrate 1000000 || fail "设置 CAN bitrate 失败"
    sudo ip link set "$CAN_PORT" up || fail "拉起 CAN 失败"
    if ! ip -details link show "$CAN_PORT" | grep -q 'bitrate 1000000'; then
        fail "CAN $CAN_PORT 没有进入 1 Mbit/s"
    fi
fi
info "✅ CAN $CAN_PORT UP，1 Mbit/s"

LOG_DIR="$HOME/agx_arm_ws/log"
mkdir -p "$LOG_DIR"
LOG_FILE="$LOG_DIR/arm_on_$(date +%Y%m%d_%H%M%S).log"
info "启动驱动（日志：$LOG_FILE）..."

ros2 launch agx_arm_ctrl start_single_agx_arm.launch.py \
    can_port:="$CAN_PORT" \
    arm_type:="$ARM_TYPE" \
    effector_type:="$EFFECTOR_TYPE" \
    speed_percent:="$SPEED_PERCENT" \
    auto_enable:=true \
    control_enabled:=false \
    >"$LOG_FILE" 2>&1 &
DRIVER_PID=$!

wait_for_service() {
    local service="$1" timeout="${2:-20}" start
    start=$(date +%s)
    while true; do
        if ros2 service list 2>/dev/null | grep -qx "$service"; then
            return 0
        fi
        if ! kill -0 "$DRIVER_PID" 2>/dev/null; then
            fail "驱动已退出，服务 $service 未出现"
        fi
        if (( $(date +%s) - start >= timeout )); then
            fail "等待服务 $service 超时（${timeout}s）"
        fi
        sleep 0.5
    done
}

wait_for_topic() {
    local topic="$1" timeout="${2:-15}" start
    start=$(date +%s)
    while true; do
        if ros2 topic list 2>/dev/null | grep -qx "$topic"; then
            return 0
        fi
        if ! kill -0 "$DRIVER_PID" 2>/dev/null; then
            fail "驱动已退出，话题 $topic 未出现"
        fi
        if (( $(date +%s) - start >= timeout )); then
            fail "等待话题 $topic 超时（${timeout}s）"
        fi
        sleep 0.3
    done
}

call_service() {
    local service="$1" type="$2" request="$3" expect="$4" out
    out=$(timeout 8 ros2 service call "$service" "$type" "$request" 2>&1) || {
        printf '%s\n' "$out" >&2
        fail "调用 $service 失败"
    }
    printf '%s\n' "$out"
    if ! grep -q "$expect" <<<"$out"; then
        fail "$service 返回异常（未找到 '$expect'）"
    fi
}

wait_for_service /enable_agx_arm 20
wait_for_service /control_enable 10
wait_for_topic /feedback/tcp_pose 15
wait_for_topic /feedback/joint_states 15

info "使能机械臂..."
call_service /enable_agx_arm std_srvs/srv/SetBool "{data: true}" "success=True"

info "打开外部控制门..."
call_service /control_enable std_srvs/srv/SetBool "{data: true}" "External control gate opened"

info "当前状态："
"$HOME/agx_arm_ws/tools/arm_state.sh" || fail "arm_state 读取失败"

info ""
info "✅ 机械臂已启动、使能、控制门已打开。"
info "   驱动日志：$LOG_FILE"
info "   Ctrl-C 停止驱动。"

wait "$DRIVER_PID"
