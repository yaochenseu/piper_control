#!/usr/bin/env python3
"""打印当前关节角、夹爪、法兰位姿和指尖位置（mm / 度）。"""

import math
import sys
import time

import numpy as np
import rclpy
from geometry_msgs.msg import PoseStamped
from rclpy.node import Node
from scipy.spatial.transform import Rotation
from sensor_msgs.msg import JointState

TIP_OFFSET_M = 0.1425  # 法兰 -> 指尖平面


class StateNode(Node):
    def __init__(self):
        super().__init__("arm_state")
        self.joints = None
        self.tcp = None
        self.create_subscription(JointState, "/feedback/joint_states", self._joint_cb, 10)
        self.create_subscription(PoseStamped, "/feedback/tcp_pose", self._tcp_cb, 10)

    def _joint_cb(self, msg):
        names = {n: i for i, n in enumerate(msg.name)}
        if all(f"joint{i}" in names for i in range(1, 7)):
            self.joints = msg

    def _tcp_cb(self, msg):
        self.tcp = msg


def main():
    rclpy.init()
    node = StateNode()
    deadline = time.time() + 5.0
    while time.time() < deadline and (node.joints is None or node.tcp is None):
        rclpy.spin_once(node, timeout_sec=0.1)

    if node.joints is None or node.tcp is None:
        print("❌ 超时：没有收到 /feedback/joint_states 或 /feedback/tcp_pose")
        node.destroy_node()
        rclpy.shutdown()
        return 1

    names = {n: i for i, n in enumerate(node.joints.name)}
    joints = [math.degrees(node.joints.position[names[f"joint{i}"]])
              for i in range(1, 7)]
    print("关节角 (deg): " + " ".join(f"{v:+.2f}" for v in joints))

    if "gripper" in names:
        width = node.joints.position[names["gripper"]]
        print(f"夹爪宽度 (mm): {width * 1000:.1f}  (比例 {width / 0.1:.3f})")

    p = node.tcp.pose.position
    q = node.tcp.pose.orientation
    rpy = Rotation.from_quat([q.x, q.y, q.z, q.w]).as_euler("xyz", degrees=True)
    print("法兰位置 (mm): [%+.1f %+.1f %+.1f]" % (p.x * 1000, p.y * 1000, p.z * 1000))
    print("法兰 RPY (deg): [%+.2f %+.2f %+.2f]" % (rpy[0], rpy[1], rpy[2]))
    print("法兰四元数: [%+.6f %+.6f %+.6f %+.6f]"
          % (q.x, q.y, q.z, q.w))

    R = Rotation.from_quat([q.x, q.y, q.z, q.w]).as_matrix()
    tip = np.array([p.x, p.y, p.z]) + TIP_OFFSET_M * R[:, 2]
    print("指尖位置 (mm): [%+.1f %+.1f %+.1f]"
          % (tip[0] * 1000, tip[1] * 1000, tip[2] * 1000))

    node.destroy_node()
    rclpy.shutdown()
    return 0


if __name__ == "__main__":
    sys.exit(main())
