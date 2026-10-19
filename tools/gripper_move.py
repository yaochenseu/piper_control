#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""AgileX 夹爪控制工具（第 7 轴）。

夹爪不是角度关节，而是一个直线关节：
    position = 开口宽度，单位：毫米（mm），范围 0 ~ 100
    effort   = 夹持力，单位：牛（N），范围 0.5 ~ 3.0

只发布 name=["gripper"] 到 /control/move_j，不包含机械臂关节，
因此不会影响机械臂当前姿态。

前提：
    驱动必须用 effector_type:=agx_gripper 启动。

用法：
    # 直接执行
    python3 gripper_move.py --width 50 --force 1.0

    # 只预演
    python3 gripper_move.py --width 50 --force 1.0 --no

    # 完全张开 / 闭合
    python3 gripper_move.py --width 100 --force 1.0
    python3 gripper_move.py --width 0 --force 0.5
"""

import argparse
import sys
import time

import rclpy
from rclpy.node import Node
from sensor_msgs.msg import JointState

WIDTH_MIN = 0.0    # mm
WIDTH_MAX = 100.0  # mm
FORCE_MIN = 0.5
FORCE_MAX = 3.0


class GripperMover(Node):
    def __init__(self):
        super().__init__("gripper_mover")
        self.current_width = None
        self.current_force = None

        self.create_subscription(
            JointState, "/feedback/joint_states", self._on_joint_states, 10
        )
        self.pub = self.create_publisher(JointState, "/control/move_j", 1)

    def _on_joint_states(self, msg: JointState):
        if "gripper" not in msg.name:
            return
        index = msg.name.index("gripper")
        self.current_width = msg.position[index]
        self.current_force = msg.effort[index] if index < len(msg.effort) else 0.0

    def wait_for_feedback(self, timeout: float = 5.0) -> bool:
        deadline = time.time() + timeout
        while time.time() < deadline and rclpy.ok():
            rclpy.spin_once(self, timeout_sec=0.1)
            if self.current_width is not None:
                return True
        return False

    def publish(self, width_mm: float, force: float):
        msg = JointState()
        msg.header.stamp = self.get_clock().now().to_msg()
        msg.name = ["gripper"]
        # ROS 消息使用米
        msg.position = [float(width_mm) / 1000.0]
        msg.velocity = []
        msg.effort = [float(force)]

        for _ in range(5):
            self.pub.publish(msg)
            rclpy.spin_once(self, timeout_sec=0.05)
            time.sleep(0.05)

    def monitor(self, target_width_mm: float, duration: float = 2.0):
        """等待 duration 秒，只在结束时打印一次最终夹爪状态。"""
        deadline = time.time() + duration
        while time.time() < deadline and rclpy.ok():
            rclpy.spin_once(self, timeout_sec=0.05)

        if self.current_width is None:
            print("\n❌ 失败：未收到最终夹爪反馈")
            return

        actual_mm = self.current_width * 1000.0
        diff_mm = actual_mm - target_width_mm
        print(
            f"\n最终夹爪状态：宽度 {actual_mm:.0f} mm，"
            f"力 {self.current_force:.2f} N"
        )
        if abs(diff_mm) <= 2.0:
            print(f"✅ 成功：夹爪到达目标宽度（误差 {diff_mm:+.0f} mm）")
        else:
            print(
                f"⚠️ 未到达目标宽度：误差 {diff_mm:+.0f} mm"
                "（可能被物体挡住）"
            )


def main():
    parser = argparse.ArgumentParser(description="AgileX 夹爪控制（第 7 轴）")
    parser.add_argument(
        "--width", type=float, required=True, help="目标开口宽度（mm），0 ~ 100"
    )
    parser.add_argument(
        "--force",
        type=float,
        default=1.0,
        help="夹持力（N），0.5 ~ 3.0，默认 1.0",
    )
    parser.add_argument(
        "--no",
        "--dry-run",
        dest="preview",
        action="store_true",
        help="只预演，不发送指令（默认直接执行）",
    )
    parser.add_argument("--yes", action="store_true", help=argparse.SUPPRESS)
    parser.add_argument("--monitor", type=float, default=2.0, help="执行后监控秒数")
    args = parser.parse_args()

    if not (WIDTH_MIN <= args.width <= WIDTH_MAX):
        print(
            f"宽度必须在 {WIDTH_MIN:.0f}~{WIDTH_MAX:.0f} mm，当前 {args.width}"
        )
        sys.exit(1)
    if not (FORCE_MIN <= args.force <= FORCE_MAX):
        print(
            f"夹持力必须在 {FORCE_MIN}~{FORCE_MAX} N，当前 {args.force}"
        )
        sys.exit(1)

    rclpy.init()
    node = GripperMover()

    try:
        print("等待 /feedback/joint_states 中的 gripper ...")
        if node.wait_for_feedback():
            print(
                f"当前夹爪：宽度 {node.current_width * 1000:.0f} mm，"
                f"力 {node.current_force:.2f} N"
            )
        else:
            print(
                "没有收到 gripper 反馈。确认驱动是否用 "
                "effector_type:=agx_gripper 启动。"
            )

        print(f"\n目标：宽度 {args.width:.0f} mm，力 {args.force:.2f} N")

        if args.preview:
            print("\n仅预演，未发送指令。去掉 --no 即直接执行。")
            return

        node.publish(args.width, args.force)
        node.monitor(args.width, args.monitor)
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()
