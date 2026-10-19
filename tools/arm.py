#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""机械臂一体控制：j=关节角，p=笛卡尔位姿。

用法：
    arm.py j <j1> <j2> <j3> <j4> <j5> <j6> [夹爪比例] [选项]
    arm.py p <x> <y> <z> [rx ry rz] [夹爪比例] [选项]
    （也可写 -t <目标...>，两种等价）

单位：
    关节角、姿态：度（°）
    位置：毫米（mm）
    夹爪比例：0=闭合，0.5=半开，1=全开

写法：
    30      绝对目标
    x       保持当前
    x+10    当前值 +10
    x-10    当前值 -10

选项：
    -t  目标            -f  夹爪力(N)       -n  只预演
    -l  直线(move_l)    -c  超限自动夹紧     -w  执行后等待秒数
    --md  单次变化上限(度)   --jn  关节数   --rb  型号

默认直接执行。最后一行：✅ 成功 / ❌ 失败 / ⚠️ 误差偏大。
"""

import argparse
import math
import sys
import time

import rclpy
from rclpy.node import Node
from geometry_msgs.msg import PoseStamped
from sensor_msgs.msg import JointState
from std_srvs.srv import SetBool
from scipy.spatial.transform import Rotation as R

from pyAgxArm import create_agx_arm_config
from agx_arm_msgs.msg import AgxArmStatus

GRIPPER_MAX_WIDTH = 0.1  # 夹爪最大开口宽度（米）

ARM_STATUS_TEXT = {
    0: "正常",
    1: "急停",
    2: "无解（目标不可达）",
    3: "奇异点",
    4: "目标角度超限",
    5: "关节通信异常",
    6: "关节刹车未释放",
    7: "发生碰撞",
    8: "示教拖动超速",
    9: "关节状态异常",
    10: "其他异常",
    11: "示教记录中",
    12: "示教执行中",
    13: "示教暂停",
    14: "主控 NTC 过温",
    15: "释放电阻 NTC 过温",
}

MODE_TEXT = {
    0: "MOVE P",
    1: "MOVE J",
    2: "MOVE L",
    3: "MOVE C",
    4: "MOVE MIT",
    5: "MOVE CPV",
}


def load_joint_limits_deg(robot: str) -> dict:
    """从 pyAgxArm 配置读取关节限位，并换算成度。不连接 CAN。"""
    config = create_agx_arm_config(robot=robot, comm="can", channel="can0")
    limits = config["joint_limits"]
    return {
        name: (math.degrees(low), math.degrees(high))
        for name, (low, high) in limits.items()
    }


class Arm(Node):
    def __init__(self, joint_count: int):
        super().__init__("arm_ctrl")
        self.joint_names = [f"joint{i}" for i in range(1, joint_count + 1)]
        self.current_rad = None
        self.latest_rad = None
        self.current_pose = None
        self.gripper_current = None  # (width_m, force_n)
        self.arm_status = None

        self.create_subscription(
            JointState, "/feedback/joint_states", self._on_joint_states, 10
        )
        self.create_subscription(
            PoseStamped, "/feedback/tcp_pose", self._on_tcp_pose, 10
        )
        self.create_subscription(
            AgxArmStatus, "/feedback/arm_status", self._on_arm_status, 10
        )

        self.pub_j = self.create_publisher(JointState, "/control/move_j", 1)
        self.pub_p = self.create_publisher(PoseStamped, "/control/move_p", 1)
        self.pub_l = self.create_publisher(PoseStamped, "/control/move_l", 1)

    # ── feedback ────────────────────────────────────────────────────
    def _on_joint_states(self, msg: JointState):
        positions = dict(zip(msg.name, msg.position))
        if all(name in positions for name in self.joint_names):
            self.current_rad = [positions[name] for name in self.joint_names]
            self.latest_rad = list(self.current_rad)

        if "gripper" in positions:
            index = msg.name.index("gripper")
            force = msg.effort[index] if index < len(msg.effort) else 0.0
            self.gripper_current = (positions["gripper"], force)

    def _on_tcp_pose(self, msg: PoseStamped):
        self.current_pose = msg

    def _on_arm_status(self, msg: AgxArmStatus):
        self.arm_status = msg

    def wait_feedback(self, kind: str, timeout: float = 10.0) -> bool:
        deadline = time.time() + timeout
        while time.time() < deadline and rclpy.ok():
            rclpy.spin_once(self, timeout_sec=0.05)
            if (kind == "j" and self.current_rad is not None) or (
                kind == "p" and self.current_pose is not None
            ):
                # 再多转几圈，让夹爪等订阅也收到数据
                for _ in range(5):
                    rclpy.spin_once(self, timeout_sec=0.02)
                return True
        return False

    # ── 控制门 ──────────────────────────────────────────────────────
    def moveit_stream_active(self) -> bool:
        """MoveIt 的 joint_state_broadcaster 是否在持续下发控制流。"""
        return self.count_publishers("/control/joint_states") > 0

    def ensure_control_gate_open(self) -> bool:
        """MoveIt 未运行时，打开驱动的外部控制门。

        驱动以 control_enabled:=false 启动，或 MoveIt 退出后门控节点
        会把门留在关闭状态；此时直连指令会被驱动静默忽略。
        """
        if self.moveit_stream_active():
            print(
                "提示：检测到 /control/joint_states 控制流（MoveIt 正在运行），"
                "arm 直连指令可能被覆盖；建议改用 armplus / armpro。"
            )
            return True

        client = self.create_client(SetBool, "/control_enable")
        if not client.wait_for_service(timeout_sec=1.0):
            print("⚠️ 未找到 /control_enable 服务，无法确认控制门状态。")
            return False

        request = SetBool.Request()
        request.data = True
        future = client.call_async(request)
        rclpy.spin_until_future_complete(self, future, timeout_sec=2.0)
        response = future.result()
        if response is None or not response.success:
            print("⚠️ 打开驱动控制门失败，指令可能被驱动忽略。")
            return False
        print("已打开驱动控制门（MoveIt 未运行）。")
        return True

    @property
    def current_deg(self):
        if self.current_rad is None:
            return None
        return [math.degrees(v) for v in self.current_rad]

    @property
    def latest_deg(self):
        if self.latest_rad is None:
            return None
        return [math.degrees(v) for v in self.latest_rad]

    def current_rpy_deg(self):
        q = self.current_pose.pose.orientation
        return R.from_quat([q.x, q.y, q.z, q.w]).as_euler("xyz", degrees=True)

    # ── publish ─────────────────────────────────────────────────────
    def publish_joint(self, target_deg, gripper_width=None, gripper_force=None):
        msg = JointState()
        msg.header.stamp = self.get_clock().now().to_msg()
        msg.name = list(self.joint_names)
        msg.position = [math.radians(v) for v in target_deg]
        msg.velocity = []
        msg.effort = []

        if gripper_width is not None:
            msg.name.append("gripper")
            msg.position.append(float(gripper_width))
            msg.effort.append(float(gripper_force))

        for _ in range(5):
            self.pub_j.publish(msg)
            rclpy.spin_once(self, timeout_sec=0.05)
            time.sleep(0.05)

    def build_pose_msg(self, position, rpy_deg) -> PoseStamped:
        """position 单位：毫米；ROS 消息内部换算成米。"""
        msg = PoseStamped()
        msg.header.stamp = self.get_clock().now().to_msg()
        msg.header.frame_id = "base_link"
        msg.pose.position.x = float(position[0]) / 1000.0
        msg.pose.position.y = float(position[1]) / 1000.0
        msg.pose.position.z = float(position[2]) / 1000.0
        quat = R.from_euler("xyz", rpy_deg, degrees=True).as_quat()
        msg.pose.orientation.x = float(quat[0])
        msg.pose.orientation.y = float(quat[1])
        msg.pose.orientation.z = float(quat[2])
        msg.pose.orientation.w = float(quat[3])
        return msg

    def publish_pose(self, position, rpy_deg, linear, gripper_ratio, force):
        msg = self.build_pose_msg(position, rpy_deg)
        publisher = self.pub_l if linear else self.pub_p
        for _ in range(5):
            publisher.publish(msg)
            rclpy.spin_once(self, timeout_sec=0.02)
            time.sleep(0.02)

        if gripper_ratio is not None:
            gripper_msg = JointState()
            gripper_msg.header.stamp = self.get_clock().now().to_msg()
            gripper_msg.name = ["gripper"]
            gripper_msg.position = [GRIPPER_MAX_WIDTH * gripper_ratio]
            gripper_msg.velocity = []
            gripper_msg.effort = [float(force)]
            for _ in range(5):
                self.pub_j.publish(gripper_msg)
                rclpy.spin_once(self, timeout_sec=0.02)
                time.sleep(0.02)

    # ── monitor ─────────────────────────────────────────────────────
    def monitor_joint(self, target_deg, duration: float = 3.0):
        """等待 duration 秒，只在结束时打印一次最终实际值和误差。"""
        deadline = time.time() + duration
        while time.time() < deadline and rclpy.ok():
            rclpy.spin_once(self, timeout_sec=0.05)

        actual_deg = self.latest_deg
        if actual_deg is None:
            print("\n未收到最终关节反馈")
            return

        err = [t - a for t, a in zip(target_deg, actual_deg)]
        actual = " ".join(f"{v:+.2f}" for v in actual_deg)
        error = " ".join(f"{v:+.2f}" for v in err)
        print(f"\n最终实际（度）: [{actual}]")
        print(f"最终误差（度）: [{error}]")

        if self.arm_status is None:
            print("❌ 失败：未收到 arm_status")
            return

        arm_code = self.arm_status.arm_status
        motion = self.arm_status.motion_status
        max_err = max(abs(v) for v in err)
        if arm_code != 0:
            print(
                f"❌ 失败：arm_status={arm_code} "
                f"({ARM_STATUS_TEXT.get(arm_code, '未知')})"
            )
        elif motion != 0:
            print(f"❌ 失败：未到达目标（motion_status={motion}）")
        elif max_err > 2.0:
            print(f"⚠️ 状态成功但误差偏大：最大关节误差 {max_err:.2f}°")
            print(
                "  若机械臂完全没动：驱动可能仍处于控制门关闭状态，执行 "
                "ros2 service call /control_enable std_srvs/srv/SetBool "
                "\"{data: true}\""
            )
        else:
            print(f"✅ 成功：已到达目标（最大关节误差 {max_err:.2f}°）")

    def monitor_pose(self, target_position, target_rpy_deg, duration: float = 3.0):
        """等待 duration 秒，只在结束时打印一次最终位姿和误差。"""
        deadline = time.time() + duration
        while time.time() < deadline and rclpy.ok():
            rclpy.spin_once(self, timeout_sec=0.05)

        if self.current_pose is None:
            print("\n未收到最终 TCP 位姿反馈")
            return

        p = self.current_pose.pose.position
        actual_rpy = self.current_rpy_deg()
        actual_pos = [p.x * 1000.0, p.y * 1000.0, p.z * 1000.0]

        pos_err = [t - a for t, a in zip(target_position, actual_pos)]
        r_target = R.from_euler("xyz", target_rpy_deg, degrees=True)
        q = self.current_pose.pose.orientation
        r_actual = R.from_quat([q.x, q.y, q.z, q.w])
        rpy_err = (r_target * r_actual.inv()).as_euler("xyz", degrees=True)

        print(
            "\n最终实际位置（mm）: "
            f"[{actual_pos[0]:+.1f} {actual_pos[1]:+.1f} {actual_pos[2]:+.1f}]"
        )
        print(
            "最终实际姿态（度）: "
            f"[{actual_rpy[0]:+.2f} {actual_rpy[1]:+.2f} {actual_rpy[2]:+.2f}]"
        )
        print(
            "最终位置误差（mm）: "
            f"[{pos_err[0]:+.1f} {pos_err[1]:+.1f} {pos_err[2]:+.1f}]"
        )
        print(
            "最终姿态误差（度）: "
            f"[{rpy_err[0]:+.2f} {rpy_err[1]:+.2f} {rpy_err[2]:+.2f}]"
        )

        if self.arm_status is None:
            print("❌ 失败：未收到 arm_status")
            return

        arm_code = self.arm_status.arm_status
        motion = self.arm_status.motion_status
        mode = self.arm_status.mode_feedback
        print(
            f"状态: arm_status={arm_code} "
            f"({ARM_STATUS_TEXT.get(arm_code, '未知')}), "
            f"motion_status={motion} "
            f"({'到达目标' if motion == 0 else '未到达'}), "
            f"mode_feedback={mode} ({MODE_TEXT.get(mode, '未知')})"
        )

        max_pos_err = max(abs(v) for v in pos_err)
        max_rpy_err = max(abs(v) for v in rpy_err)
        if arm_code != 0:
            print(
                f"❌ 失败：arm_status={arm_code} "
                f"({ARM_STATUS_TEXT.get(arm_code, '未知')})"
            )
        elif motion != 0:
            print(f"❌ 失败：未到达目标（motion_status={motion}）")
        elif max_pos_err > 2.0 or max_rpy_err > 0.5:
            print(
                f"⚠️ 状态成功但误差偏大：位置 {max_pos_err:.1f} mm，"
                f"姿态 {max_rpy_err:.2f}°"
            )
            print(
                "  若机械臂完全没动：驱动可能仍处于控制门关闭状态，执行 "
                "ros2 service call /control_enable std_srvs/srv/SetBool "
                "\"{data: true}\""
            )
        else:
            print(
                f"✅ 成功：已到达目标（位置误差 {max_pos_err:.1f} mm，"
                f"姿态误差 {max_rpy_err:.2f}°）"
            )


# ── token 解析 ──────────────────────────────────────────────────────
def flatten(values):
    flat = []
    for value in values:
        flat.extend(v for v in str(value).replace(",", " ").split() if v)
    return flat


def is_valid_token(token: str) -> bool:
    """数字、x、x+10、x-10 都是合法 token。"""
    t = token.lower()
    if t == "x":
        return True
    if len(t) >= 3 and t[0] == "x" and t[1] in "+-":
        try:
            float(t[1:])
            return True
        except ValueError:
            return False
    try:
        float(token)
        return True
    except ValueError:
        return False


def resolve_token(token: str, current: float):
    """返回 (value, keep_current)。

    - 数字        -> 绝对目标
    - x           -> 保持当前
    - x+10 / x-10 -> 当前值 +10 / -10
    """
    t = token.lower()
    if t == "x":
        return current, True
    if len(t) >= 3 and t[0] == "x" and t[1] in "+-":
        return current + float(t[1:]), False
    return float(token), False


def check_tokens(tokens):
    for token in tokens:
        if not is_valid_token(token):
            raise ValueError(
                f"无效参数 '{token}'，请填数字、x（保持当前）、"
                "x+数值 或 x-数值"
            )


def parse_joint_target(values, joint_count: int):
    """返回 (臂关节 token 列表, 夹爪 token 或 None)。"""
    flat = flatten(values)
    if len(flat) not in (joint_count, joint_count + 1):
        raise ValueError(
            f"需要 {joint_count} 个臂关节角，或 {joint_count + 1} 个值"
            f"（最后一个为夹爪张开比例），实际收到 {len(flat)} 个: {flat}"
        )
    check_tokens(flat)
    return flat[:joint_count], (flat[-1] if len(flat) == joint_count + 1 else None)


def parse_pose_target(values):
    """返回 (position_tokens, rpy_tokens_or_None, gripper_token_or_None)。"""
    flat = flatten(values)
    if len(flat) not in (3, 6, 7):
        raise ValueError(
            "需要 3 个值（x y z）、6 个值（x y z rx ry rz）"
            "或 7 个值（再加夹爪比例），"
            f"实际收到 {len(flat)} 个: {flat}"
        )
    check_tokens(flat)
    position = flat[:3]
    rpy_deg = flat[3:6] if len(flat) >= 6 else None
    gripper_ratio = flat[6] if len(flat) == 7 else None
    return position, rpy_deg, gripper_ratio


def resolve_gripper(token, node):
    """夹爪 token -> 张开比例；x 或未提供返回 None。"""
    if token is None:
        return None
    t = token.lower()
    if t == "x":
        return None
    if len(t) >= 3 and t[0] == "x" and t[1] in "+-":
        if node.gripper_current is None:
            raise ValueError("无法计算夹爪相对目标：未收到夹爪反馈")
        current_ratio = node.gripper_current[0] / GRIPPER_MAX_WIDTH
        return current_ratio + float(t[1:])
    return float(token)


def check_gripper(ratio, force):
    if ratio is None:
        if force != 1.0:
            print("提示：夹爪未指定数值（未提供或为 x），-f 将被忽略")
        return
    if not (0.0 <= ratio <= 1.0):
        raise ValueError(
            f"夹爪张开比例必须在 0.0~1.0，当前 {ratio}"
            "（0=闭合，0.5=半开，1=全开）"
        )
    if not (0.5 <= force <= 3.0):
        raise ValueError(f"夹爪力必须在 0.5~3.0 N，当前 {force}")


def print_gripper(node, ratio, force, token):
    if ratio is None:
        if token is not None and token.lower() == "x":
            print("\n夹爪（第 7 轴）: 保持当前（不发送夹爪指令）")
        return
    width = GRIPPER_MAX_WIDTH * ratio
    if node.gripper_current is not None:
        cur_width, cur_force = node.gripper_current
        cur_ratio = cur_width / GRIPPER_MAX_WIDTH
        print(
            f"\n夹爪（第 7 轴）: 当前 {cur_width * 1000:.0f} mm "
            f"({cur_ratio * 100:.0f}%) / 力 {cur_force:.2f} N"
            f"  ->  目标 {width * 1000:.0f} mm "
            f"({ratio * 100:.0f}%) / 力 {force:.2f} N"
        )
    else:
        print(
            f"\n夹爪（第 7 轴）: 目标 {width * 1000:.0f} mm "
            f"({ratio * 100:.0f}%) / 力 {force:.2f} N"
            "（未收到夹爪反馈；确认驱动用 effector_type:=agx_gripper 启动）"
        )


# ── 两种模式 ────────────────────────────────────────────────────────
def run_joint(args, node: Arm):
    arm_tokens, gripper_token = parse_joint_target(args.target, args.joints)

    print("等待 /feedback/joint_states ...")
    if not node.wait_feedback("j"):
        print("超时：没有收到完整关节反馈。先确认驱动已启动且已使能。")
        sys.exit(2)

    names = node.joint_names
    current_deg = list(node.current_deg)

    target_deg = []
    kept = []
    for i, token in enumerate(arm_tokens):
        value, keep = resolve_token(token, current_deg[i])
        target_deg.append(value)
        kept.append(keep)

    gripper_ratio = resolve_gripper(gripper_token, node)
    check_gripper(gripper_ratio, args.force)

    limits_deg = load_joint_limits_deg(args.robot)
    # 只检查显式填写的关节；x 保持的关节不因零位残差被误判超限
    out_of_limits = [
        (i, name) for i, name in enumerate(names)
        if not kept[i]
        and not (limits_deg[name][0] <= target_deg[i] <= limits_deg[name][1])
    ]
    if out_of_limits:
        if not args.clamp:
            print(f"目标角超出 {args.robot} 关节限位：")
            for i, name in out_of_limits:
                low, high = limits_deg[name]
                print(
                    f"  {name}: 目标 {target_deg[i]:+.1f}°，限位 "
                    f"[{low:+.1f}°, {high:+.1f}°]"
                )
            print("确认后加 -c 自动夹紧，或修改目标角。")
            sys.exit(4)
        for i, name in out_of_limits:
            low, high = limits_deg[name]
            clamped = min(max(target_deg[i], low), high)
            print(
                f"警告: {name} 目标 {target_deg[i]:+.1f}° 超出限位 "
                f"[{low:+.1f}°, {high:+.1f}°]，已夹紧到 {clamped:+.1f}°"
            )
            target_deg[i] = clamped

    delta_deg = [t - c for t, c in zip(target_deg, current_deg)]

    print("\n当前关节角（度）:")
    for name, value in zip(names, current_deg):
        print(f"  {name}: {value:+.2f}°")
    print("\n目标关节角（度）:")
    for name, value, d, keep in zip(names, target_deg, delta_deg, kept):
        suffix = "   (保持当前)" if keep else f"   (变化 {d:+.2f}°)"
        print(f"  {name}: {value:+.2f}°{suffix}")

    gripper_width = None
    if gripper_ratio is not None:
        gripper_width = GRIPPER_MAX_WIDTH * gripper_ratio
    print_gripper(node, gripper_ratio, args.force, gripper_token)

    if args.max_delta is not None:
        too_large = [
            (name, d) for name, d in zip(names, delta_deg)
            if abs(d) > args.max_delta
        ]
        if too_large:
            print(
                f"\n拒绝执行：以下关节单次变化超过 {args.max_delta:.1f}°"
                "（你显式设置了 --md）："
            )
            for name, d in too_large:
                print(f"  {name}: {d:+.2f}°")
            print("确认安全后可调大或去掉 --md。")
            sys.exit(3)

    if args.preview:
        print("\n仅预演，未发送指令。去掉 -n 即直接执行。")
        return

    node.ensure_control_gate_open()
    node.publish_joint(target_deg, gripper_width, args.force)
    node.monitor_joint(target_deg, args.wait)
    if gripper_width is not None:
        print(
            f"夹爪目标 {gripper_width * 1000:.0f} mm "
            f"({gripper_ratio * 100:.0f}%)，力 {args.force:.2f} N；"
            "实际状态可用 ros2 topic echo /feedback/gripper_status 查看"
        )


def run_pose(args, node: Arm):
    position_tokens, rpy_tokens, gripper_token = parse_pose_target(args.target)

    print("等待 /feedback/tcp_pose ...")
    if not node.wait_feedback("p"):
        print("超时：没有收到 TCP 位姿反馈。先确认驱动已启动且已使能。")
        sys.exit(2)

    p = node.current_pose.pose.position
    current_pos = [p.x * 1000.0, p.y * 1000.0, p.z * 1000.0]
    current_rpy = node.current_rpy_deg()

    position = []
    kept_pos = []
    for i, token in enumerate(position_tokens):
        value, keep = resolve_token(token, current_pos[i])
        position.append(value)
        kept_pos.append(keep)

    if rpy_tokens is None:
        rpy_deg = list(current_rpy)
        kept_rpy = [True, True, True]
    else:
        rpy_deg = []
        kept_rpy = []
        for i, token in enumerate(rpy_tokens):
            value, keep = resolve_token(token, current_rpy[i])
            rpy_deg.append(value)
            kept_rpy.append(keep)

    gripper_ratio = resolve_gripper(gripper_token, node)
    check_gripper(gripper_ratio, args.force)

    pos_delta = [t - c for t, c in zip(position, current_pos)]
    r_target = R.from_euler("xyz", rpy_deg, degrees=True)
    q = node.current_pose.pose.orientation
    r_current = R.from_quat([q.x, q.y, q.z, q.w])
    rpy_delta = (r_target * r_current.inv()).as_euler("xyz", degrees=True)

    print("\n当前 TCP 位姿:")
    print(
        f"  x: {current_pos[0]:+.1f} mm   "
        f"y: {current_pos[1]:+.1f} mm   z: {current_pos[2]:+.1f} mm"
    )
    print(
        f"  rx: {current_rpy[0]:+.2f}°   "
        f"ry: {current_rpy[1]:+.2f}°   rz: {current_rpy[2]:+.2f}°"
    )
    print("\n目标 TCP 位姿:")
    print(
        f"  x: {position[0]:+.1f} mm   "
        f"y: {position[1]:+.1f} mm   z: {position[2]:+.1f} mm"
    )
    print(
        f"  rx: {rpy_deg[0]:+.2f}°   "
        f"ry: {rpy_deg[1]:+.2f}°   rz: {rpy_deg[2]:+.2f}°"
    )
    print(
        "  Δ位置: "
        f"[{pos_delta[0]:+.1f} {pos_delta[1]:+.1f} {pos_delta[2]:+.1f}] mm"
    )
    print(
        "  Δ姿态: "
        f"[{rpy_delta[0]:+.2f} {rpy_delta[1]:+.2f} {rpy_delta[2]:+.2f}]°"
    )
    kept_names = [
        name for name, keep in zip(
            ["x", "y", "z", "rx", "ry", "rz"],
            kept_pos + kept_rpy,
        ) if keep
    ]
    if kept_names:
        print(f"  (保持当前: {', '.join(kept_names)})")
    print(f"\n运动方式: {'MOVE L（直线）' if args.linear else 'MOVE P（点到点）'}")

    print_gripper(node, gripper_ratio, args.force, gripper_token)

    if args.preview:
        print("\n仅预演，未发送指令。去掉 -n 即直接执行。")
        return

    node.ensure_control_gate_open()
    node.publish_pose(position, rpy_deg, args.linear, gripper_ratio, args.force)
    node.monitor_pose(position, rpy_deg, args.wait)


def main():
    parser = argparse.ArgumentParser(
        prog="arm.py",
        description="机械臂控制：j=关节角（度），p=笛卡尔位姿（mm/度）",
        formatter_class=argparse.RawDescriptionHelpFormatter,
        epilog=(
            "示例：\n"
            "  arm.py j x x -30 x x x\n"
            "  arm.py j x+10 x x x x x 0.5 -f 1.0\n"
            "  arm.py p x x x+10\n"
            "  arm.py p 96.9 0 300 -85 0 -90 -l\n"
            "  arm.py j -n x x -30 x x x"
        ),
    )
    parser.add_argument(
        "mode", choices=["j", "p"], metavar="j|p",
        help="j=关节角控制，p=笛卡尔位姿控制",
    )
    parser.add_argument(
        "targets", nargs="*", metavar="目标",
        help="目标值，直接写在 j/p 后面；也可用 -t 指定",
    )
    parser.add_argument(
        "-t", "--target", dest="target_opt", nargs="+", default=None,
        help=(
            "目标值（可省略）。"
            "j 模式：6 个关节角（度）+ 可选夹爪比例；"
            "p 模式：x y z（mm）+ 可选 rx ry rz（度）+ 可选夹爪比例。"
            "支持 x / x+10 / x-10"
        ),
    )
    parser.add_argument(
        "-f", "--force", type=float, default=1.0,
        help="夹爪力（N），0.5~3.0，默认 1.0",
    )
    parser.add_argument(
        "-n", "--no", "--dry-run", dest="preview", action="store_true",
        help="只预演，不发送指令（默认直接执行）",
    )
    parser.add_argument(
        "-l", "--linear", action="store_true",
        help="p 模式使用 move_l 直线运动（默认 move_p 点到点）",
    )
    parser.add_argument(
        "-c", "--clamp", action="store_true",
        help="j 模式超限时自动夹紧到边界，否则拒绝执行",
    )
    parser.add_argument(
        "-w", "--wait", type=float, default=3.0,
        help="执行后等待/监控秒数，默认 3.0",
    )
    parser.add_argument(
        "--md", "--max-delta", dest="max_delta", type=float, default=None,
        help="j 模式可选：单次变化保护（度）",
    )
    parser.add_argument(
        "--jn", "--joints", dest="joints", type=int, default=6,
        help="关节数（Piper=6，Nero=7），默认 6",
    )
    parser.add_argument(
        "--rb", "--robot", dest="robot", default="piper_x",
        help="型号，用于读取关节限位（默认 piper_x）",
    )
    parser.add_argument("--yes", action="store_true", help=argparse.SUPPRESS)
    # 允许选项与位置参数任意穿插，如 arm j -n x x -30 x x x
    args = parser.parse_intermixed_args()

    if args.target_opt is not None and args.targets:
        parser.error("目标值不要同时使用位置参数和 -t")
    args.target = args.target_opt if args.target_opt is not None else args.targets
    if not args.target:
        parser.error("缺少目标值（直接写在 j/p 后面，或用 -t）")

    if args.mode == "j" and args.linear:
        print("提示：j 模式不使用 -l，已忽略")

    try:
        if args.mode == "j":
            parse_joint_target(args.target, args.joints)
        else:
            parse_pose_target(args.target)
    except ValueError as exc:
        print(f"参数错误：{exc}")
        sys.exit(1)

    rclpy.init()
    node = Arm(args.joints)

    try:
        if args.mode == "j":
            run_joint(args, node)
        else:
            run_pose(args, node)
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()
