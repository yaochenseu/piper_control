#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""首次安全运动工具（角度制）。

从 /feedback/joint_states 读取当前关节角，只给指定关节叠加一个角度增量，
然后把【完整】关节列表发布到 /control/move_j。

所有输入/输出都用【度（°）】，内部自动换算成弧度。

为什么要发完整关节列表：
    驱动节点在收到部分关节指令时，会用 0 补全未指定的机械臂关节
    （agx_arm_ctrl_single_node.py 中 arm_joints.get(name, 0)）。
    因此只发 joint6 会把 joint1~joint5 命令到 0，非常危险。

安全策略：
    默认只检查机械限位；超出限位则自动夹到边界并打印警告。
    --max-delta 是可选的手误保护，不设置则不做变化量检查。

用法：
    # 预演，不真正发指令
    python3 safe_joint_move.py --joint joint1 --delta 3

    # 确认后真正执行
    python3 safe_joint_move.py --joint joint1 --delta 3 --yes

    # Nero 是 7 关节
    python3 safe_joint_move.py --joint joint1 --delta 3 --joints 7 --yes
"""

import argparse
import math
import sys
import time

import rclpy
from rclpy.node import Node
from sensor_msgs.msg import JointState
from pyAgxArm import create_agx_arm_config

from agx_arm_msgs.msg import AgxArmStatus

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

# 默认不做单次变化限制，只检查机械限位。
# 如需防手误，可显式传 --max-delta（单位：度）。


def load_joint_limits_deg(robot: str) -> dict:
    """从 pyAgxArm 配置读取关节限位，并换算成度。不连接 CAN。"""
    config = create_agx_arm_config(robot=robot, comm="can", channel="can0")
    limits = config["joint_limits"]
    return {
        name: (math.degrees(low), math.degrees(high))
        for name, (low, high) in limits.items()
    }


class SafeJointMover(Node):
    def __init__(self, joint_count: int):
        super().__init__("safe_joint_mover")
        self.joint_names = [f"joint{i}" for i in range(1, joint_count + 1)]
        self.current_rad = None
        self.latest_rad = None
        self.arm_status = None

        self.create_subscription(
            JointState, "/feedback/joint_states", self._on_joint_states, 10
        )
        self.create_subscription(
            AgxArmStatus, "/feedback/arm_status", self._on_arm_status, 10
        )
        self.pub = self.create_publisher(JointState, "/control/move_j", 1)

    def _on_arm_status(self, msg: AgxArmStatus):
        self.arm_status = msg

    def _on_joint_states(self, msg: JointState):
        positions = dict(zip(msg.name, msg.position))
        if all(name in positions for name in self.joint_names):
            self.current_rad = [positions[name] for name in self.joint_names]
            self.latest_rad = list(self.current_rad)

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

    def wait_for_feedback(self, timeout: float = 10.0) -> bool:
        deadline = time.time() + timeout
        while time.time() < deadline and rclpy.ok():
            rclpy.spin_once(self, timeout_sec=0.1)
            if self.current_rad is not None:
                return True
        return False

    def publish_target_deg(self, target_deg):
        msg = JointState()
        msg.header.stamp = self.get_clock().now().to_msg()
        msg.name = self.joint_names
        # ROS 消息使用弧度
        msg.position = [math.radians(v) for v in target_deg]
        msg.velocity = []
        msg.effort = []

        # 连续发几帧，确保驱动节点收到（QoS depth=1）
        for _ in range(5):
            self.pub.publish(msg)
            rclpy.spin_once(self, timeout_sec=0.05)
            time.sleep(0.05)

    def monitor(self, target_deg, duration: float = 3.0):
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
            print("❌ 失败：未到达目标（motion_status=1）")
        elif max_err > 2.0:
            print(f"⚠️ 状态成功但误差偏大：最大关节误差 {max_err:.2f}°")
        else:
            print(f"✅ 成功：已到达目标（最大关节误差 {max_err:.2f}°）")


def main():
    parser = argparse.ArgumentParser(description="安全的小角度单关节运动（角度制）")
    parser.add_argument("--joint", default="joint1", help="要动的关节名，如 joint1")
    parser.add_argument(
        "--delta", type=float, default=2.0, help="角度增量，单位：度（默认 2.0）"
    )
    parser.add_argument(
        "--joints", type=int, default=6, help="机械臂关节数（Piper=6，Nero=7）"
    )
    parser.add_argument(
        "--robot", default="piper_x", help="型号，用于读取关节限位（默认 piper_x）"
    )
    parser.add_argument(
        "--max-delta",
        type=float,
        default=None,
        help=(
            "可选：单次角度增量保护，单位：度。"
            "不设置则不做变化量检查，只检查机械限位"
        ),
    )
    parser.add_argument(
        "--no",
        "--dry-run",
        dest="preview",
        action="store_true",
        help="只预演，不发送指令（默认直接执行）",
    )
    parser.add_argument("--yes", action="store_true", help=argparse.SUPPRESS)
    parser.add_argument("--monitor", type=float, default=3.0, help="执行后监控秒数")
    args = parser.parse_args()

    if args.max_delta is not None and abs(args.delta) > args.max_delta:
        print(
            f"拒绝执行：delta={args.delta:+.1f}° 超过你设置的 "
            f"--max-delta {args.max_delta:.1f}°"
        )
        sys.exit(1)

    if abs(args.delta) > 30.0:
        print(
            f"注意：本次单关节变化 {args.delta:+.1f}°，"
            "请确认运动路径无碰撞、急停可用。"
        )

    rclpy.init()
    node = SafeJointMover(args.joints)

    try:
        print("等待 /feedback/joint_states ...")
        if not node.wait_for_feedback():
            print("超时：没有收到完整关节反馈。先确认驱动已启动且使能。")
            sys.exit(2)

        names = node.joint_names
        if args.joint not in names:
            print(f"关节名无效：{args.joint}，可选 {names}")
            sys.exit(1)

        index = names.index(args.joint)
        current_deg = list(node.current_deg)
        target_deg = list(current_deg)
        target_deg[index] += args.delta

        limits_deg = load_joint_limits_deg(args.robot)
        for i, name in enumerate(names):
            low, high = limits_deg[name]
            if target_deg[i] < low or target_deg[i] > high:
                clamped = min(max(target_deg[i], low), high)
                print(
                    f"警告: {name} 目标 {target_deg[i]:+.1f}° 超出限位 "
                    f"[{low:+.1f}°, {high:+.1f}°]，已夹紧到 {clamped:+.1f}°"
                )
                target_deg[i] = clamped

        print("\n当前关节角（度）:")
        for name, value in zip(names, current_deg):
            print(f"  {name}: {value:+.2f}°")
        print("\n目标关节角（度）:")
        for name, value in zip(names, target_deg):
            marker = "  <-- 本次运动" if name == args.joint else ""
            print(f"  {name}: {value:+.2f}°{marker}")

        if args.preview:
            print("\n仅预演，未发送指令。去掉 --no 即直接执行。")
            return

        node.publish_target_deg(target_deg)
        node.monitor(target_deg, args.monitor)
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()
