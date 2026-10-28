#!/usr/bin/env python3
"""Automatic eye-in-hand hand-eye calibration for AgileX Piper X.

The arm must already be in CAN control mode (not teach mode), the RealSense
camera and the ChArUco detector must be running, and the board must be fully
visible at 0.25-0.75 m.  The script only moves the arm after a successful
preflight check; run it with --dry-run first to inspect the candidate poses.

Output: tools/calib_results/<timestamp>_handeye.json plus a console summary.
"""

import argparse
import json
import math
import os
import sys
import time
import types

import numpy as np
import rclpy
from agx_arm_msgs.msg import AgxArmStatus
from geometry_msgs.msg import PoseStamped
from rclpy.node import Node
from scipy.spatial.transform import Rotation
from sensor_msgs.msg import CameraInfo, JointState
from std_srvs.srv import SetBool

HERE = os.path.dirname(os.path.abspath(__file__))
SKILL_ROOT = os.path.dirname(HERE)
sys.path.insert(0, HERE)
import calib  # noqa: E402  (local copy in scripts/)

DEFAULT_CONFIG = os.path.join(SKILL_ROOT, "config", "default.json")


class AutoNode(calib.Collector):
    def __init__(self, cfg):
        self.cfg = cfg
        calib.MARKER_TOPIC = cfg["topics"]["marker"]
        calib.TCP_TOPIC = cfg["topics"]["tcp"]
        super().__init__()

        self.joint_deg = None
        self.arm_status = None
        self.K = None
        self.img_w, self.img_h = 640, 480
        self.tcp_msg = None
        self.tcp_z = None

        b = cfg["board"]
        self.board_center = np.array(
            [
                b["squares_x"] * b["square_length_m"] / 2.0,
                b["squares_y"] * b["square_length_m"] / 2.0,
                0.0,
            ]
        )

        self.create_subscription(
            JointState, cfg["topics"]["joint_states"], self._joint_cb, 10
        )
        self.create_subscription(
            AgxArmStatus, cfg["topics"]["arm_status"], self._status_cb, 10
        )
        self.create_subscription(
            CameraInfo, cfg["topics"]["camera_info"], self._info_cb, 10
        )
        self.create_subscription(
            PoseStamped, cfg["topics"]["tcp"], self._tcp_cb, 20
        )

        self.move_pub = self.create_publisher(
            JointState, cfg["topics"]["move_joint"], 1
        )
        self.move_pose_pub = self.create_publisher(
            PoseStamped, cfg["topics"]["move_pose"], 1
        )
        self.move_linear_pub = self.create_publisher(
            PoseStamped, cfg["topics"]["move_linear"], 1
        )
        self.gate_client = self.create_client(
            SetBool, cfg["topics"]["control_enable"]
        )

    # ── feedback ────────────────────────────────────────────────────
    def _joint_cb(self, msg):
        names = {n: i for i, n in enumerate(msg.name)}
        if all(f"joint{i}" in names for i in range(1, 7)):
            self.joint_deg = [
                math.degrees(msg.position[names[f"joint{i}"]])
                for i in range(1, 7)
            ]

    def _status_cb(self, msg):
        self.arm_status = msg

    def _info_cb(self, msg):
        self.img_w, self.img_h = int(msg.width), int(msg.height)
        if self.K is None:
            self.K = (msg.k[0], msg.k[4], msg.k[2], msg.k[5])

    def _tcp_cb(self, msg):
        self.tcp_msg = msg
        self.tcp_z = float(msg.pose.position.z)

    # ── preflight ───────────────────────────────────────────────────
    def status_ok(self, verbose=True):
        if self.arm_status is None:
            if verbose:
                print("❌ 未收到 /feedback/arm_status")
            return False
        mode = int(self.arm_status.ctrl_mode)
        if mode == 2:
            if verbose:
                print(
                    "❌ 当前是示教模式（ctrl_mode=2）。先手动退出示教模式，"
                    "确认 ctrl_mode=1；不要调用 /exit_teach_mode（会回零）。"
                )
            return False
        if mode != 1:
            if verbose:
                print(f"❌ 当前不是 CAN 控制模式（ctrl_mode={mode}，需要 1）")
            return False
        if int(self.arm_status.arm_status) != 0:
            if verbose:
                print(
                    f"❌ arm_status={self.arm_status.arm_status}，先排除故障"
                )
            return False
        if int(self.arm_status.err_status) != 0:
            if verbose:
                print(
                    f"❌ err_status={self.arm_status.err_status}，先排除故障"
                )
            return False
        return True

    def open_gate(self):
        if not self.gate_client.wait_for_service(timeout_sec=2.0):
            print("❌ /control_enable 服务不可用")
            return False
        req = SetBool.Request()
        req.data = True
        future = self.gate_client.call_async(req)
        rclpy.spin_until_future_complete(self, future, timeout_sec=3.0)
        response = future.result()
        if response is None or not response.success:
            print("❌ 打开外部控制门失败")
            return False
        return True

    def check_ready(self, timeout=12.0, open_gate=True):
        end = time.monotonic() + timeout
        while time.monotonic() < end:
            self.spin_for(0.1)
            if all(
                v is not None
                for v in (self.joint_deg, self.arm_status, self.K, self.tcp_msg)
            ):
                break
        missing = [
            name
            for name, value in (
                ("joint_states", self.joint_deg),
                ("arm_status", self.arm_status),
                ("camera_info", self.K),
                ("tcp_pose", self.tcp_msg),
            )
            if value is None
        ]
        if missing:
            print("❌ 缺少反馈：" + ", ".join(missing))
            return False
        if not self.status_ok():
            return False
        if open_gate and not self.open_gate():
            return False
        if self.marker_pixel(timeout=2.0) is None:
            print(
                "❌ 基准姿态看不到完整标定板，或板距离不在 0.25~0.75 m。"
                "先把末端移到板上方、让整块板进入画面再运行。"
            )
            return False
        return True

    # ── motion ──────────────────────────────────────────────────────
    def move_joints(self, target, timeout=40.0):
        if not self.status_ok():
            return False
        msg = JointState()
        msg.header.stamp = self.get_clock().now().to_msg()
        msg.name = [f"joint{i}" for i in range(1, 7)]
        msg.position = [math.radians(v) for v in target]
        for _ in range(5):
            self.move_pub.publish(msg)
            self.spin_for(0.05)

        deadline = time.monotonic() + timeout
        while time.monotonic() < deadline:
            self.spin_for(0.1)
            if self.joint_deg is None:
                continue
            if max(abs(a - b) for a, b in zip(self.joint_deg, target)) < 0.5:
                self.spin_for(0.5)
                if not self.status_ok():
                    return False
                if self.tcp_z is not None and self.tcp_z < self.cfg["calibration"]["tcp_z_min_m"]:
                    print(f"  ⚠️ TCP z={self.tcp_z:.3f} m 过低，退避")
                    return False
                return True
        print("  ❌ 运动超时")
        return False

    def move_pose(self, position_m, quat, linear=False, timeout=40.0):
        if not self.status_ok():
            return False
        msg = PoseStamped()
        msg.header.stamp = self.get_clock().now().to_msg()
        msg.header.frame_id = "base_link"
        msg.pose.position.x = float(position_m[0])
        msg.pose.position.y = float(position_m[1])
        msg.pose.position.z = float(position_m[2])
        msg.pose.orientation.x = float(quat[0])
        msg.pose.orientation.y = float(quat[1])
        msg.pose.orientation.z = float(quat[2])
        msg.pose.orientation.w = float(quat[3])
        pub = self.move_linear_pub if linear else self.move_pose_pub
        for _ in range(5):
            pub.publish(msg)
            self.spin_for(0.05)

        target_q = Rotation.from_quat(quat)
        deadline = time.monotonic() + timeout
        while time.monotonic() < deadline:
            self.spin_for(0.1)
            if self.tcp_msg is None:
                continue
            p = self.tcp_msg.pose.position
            q = self.tcp_msg.pose.orientation
            dp = math.dist([p.x, p.y, p.z], position_m)
            dq = math.degrees(
                (
                    Rotation.from_quat([q.x, q.y, q.z, q.w]).inv()
                    * target_q
                ).magnitude()
            )
            if dp < 0.003 and dq < 1.0:
                self.spin_for(0.5)
                if not self.status_ok():
                    return False
                if self.tcp_z is not None and self.tcp_z < self.cfg["calibration"]["tcp_z_min_m"]:
                    print(f"  ⚠️ TCP z={self.tcp_z:.3f} m 过低，退避")
                    return False
                return True
        print("  ❌ 位姿运动超时")
        return False

    def wait_stable(self, timeout=12.0, window=1.0):
        """Require both TCP and board pose to be stable for one window."""
        t_thr = self.cfg["calibration"]["stability_translation_mm"] / 1000.0
        r_thr = self.cfg["calibration"]["stability_rotation_deg"]
        end = time.monotonic() + timeout
        while time.monotonic() < end:
            self.marker_buf.clear()
            self.tcp_buf.clear()
            self.collecting = True
            self.spin_for(window)
            self.collecting = False
            if len(self.marker_buf) < 3 or len(self.tcp_buf) < 10:
                continue
            tcp = calib.average_pose(self.tcp_buf)
            marker = calib.average_pose(self.marker_buf)
            if (
                tcp[2] < t_thr
                and tcp[3] < r_thr
                and marker[2] < t_thr
                and marker[3] < r_thr
            ):
                return True
        return False

    # ── visibility ──────────────────────────────────────────────────
    def marker_pixel(self, timeout=1.5):
        """Return (u, v, distance_m), or None if not fully visible."""
        if self.K is None:
            return None
        end = time.monotonic() + timeout
        while time.monotonic() < end:
            self.spin_for(0.1)
            if self.marker_latest is None:
                continue
            if time.monotonic() - self.marker_time > 0.4:
                continue
            p, q = calib.pose_data(self.marker_latest)
            R = Rotation.from_quat(q).as_matrix()
            center = p + R @ self.board_center
            dist = float(np.linalg.norm(center))
            cal = self.cfg["calibration"]
            if dist < cal["min_distance_m"] or dist > cal["max_distance_m"]:
                return None
            fx, fy, cx, cy = self.K
            u = fx * center[0] / center[2] + cx
            v = fy * center[1] / center[2] + cy
            mx, my = 0.04 * self.img_w, 0.04 * self.img_h
            if not (mx < u < self.img_w - mx and my < v < self.img_h - my):
                return None
            return float(u), float(v), dist
        return None

    # ── limits ──────────────────────────────────────────────────────
    def in_limits(self, target):
        margin = self.cfg["calibration"]["joint_limit_margin_deg"]
        for value, (low, high) in zip(target, self.cfg["joint_limits_deg"]):
            if not (low + margin <= value <= high - margin):
                return False
        return True


def load_config(path):
    with open(os.path.expanduser(path), "r", encoding="utf-8") as f:
        return json.load(f)


def resolve_result_dir(path):
    path = os.path.expanduser(path)
    if not os.path.isabs(path):
        path = os.path.normpath(os.path.join(SKILL_ROOT, path))
    return path


def parse_args():
    parser = argparse.ArgumentParser(
        description="Piper X 自动手眼标定（eye-in-hand）"
    )
    parser.add_argument("--config", default=DEFAULT_CONFIG)
    parser.add_argument("--samples", type=int, default=None)
    parser.add_argument("--rot-samples", type=int, default=None)
    parser.add_argument("--avg-time", type=float, default=None)
    parser.add_argument("--result-dir", default=None)
    parser.add_argument("--marker-id", type=int, default=0)
    parser.add_argument("--marker-size", type=float, default=None)
    parser.add_argument(
        "--dry-run",
        action="store_true",
        help="只检查前置条件和打印候选目标，不发送运动指令",
    )
    return parser.parse_args()


def main():
    args = parse_args()
    cfg = load_config(args.config)
    cal = cfg["calibration"]
    if args.samples is not None:
        cal["samples"] = args.samples
    if args.rot_samples is not None:
        cal["rot_samples"] = args.rot_samples
    if args.avg_time is not None:
        cal["avg_time_s"] = args.avg_time
    result_dir = resolve_result_dir(args.result_dir or cfg["result_dir"])
    os.makedirs(result_dir, exist_ok=True)

    rclpy.init()
    node = AutoNode(cfg)
    try:
        node.spin_for(1.5)
        if not node.check_ready(open_gate=not args.dry_run):
            return 1
        base = list(node.joint_deg)
        base_pos = None
        base_quat = None
        if node.tcp_msg is not None:
            p = node.tcp_msg.pose.position
            q = node.tcp_msg.pose.orientation
            base_pos = [p.x, p.y, p.z]
            base_quat = [q.x, q.y, q.z, q.w]

        print("基准关节角（度）: [" + " ".join(f"{v:+.2f}" for v in base) + "]")
        print(
            f"目标 {cal['samples']} 组：先旋转 {cal['rot_samples']} 组，"
            f"再用平移补齐；候选姿态 {len(cfg['candidates_deg'])} 个"
        )

        if args.dry_run:
            print("\n[dry-run] 不会发送运动指令")
            for i, off in enumerate(cfg["candidates_deg"]):
                target = [b + o for b, o in zip(base, off)]
                mark = "OK" if node.in_limits(target) else "SKIP(限位)"
                print(
                    f"  [{i + 1:02d}] {mark}: "
                    + " ".join(f"{v:+.1f}" for v in target)
                )
            print("  [平移] " + ", ".join(
                "(" + ",".join(f"{v:+d}" for v in d) + ")"
                for d in cfg["translate_deltas_mm"]
            ))
            return 0

        samples = []
        last_good = base
        for i, off in enumerate(cfg["candidates_deg"]):
            target = [b + o for b, o in zip(base, off)]
            if not node.in_limits(target):
                print(
                    f"[{i + 1}/{len(cfg['candidates_deg'])}] 跳过（超限位）: "
                    + " ".join(f"{v:+.1f}" for v in target)
                )
                continue
            print(
                f"[{i + 1}/{len(cfg['candidates_deg'])}] 移动: "
                + " ".join(f"{v:+.1f}" for v in target),
                flush=True,
            )
            if not node.move_joints(target):
                node.move_joints(last_good)
                continue
            if not node.wait_stable():
                print("  机械臂未停稳，回上一个可见姿态")
                node.move_joints(last_good)
                continue
            marker = node.marker_pixel()
            if marker is None:
                print("  标定板不可见/贴边，回上一个可见姿态")
                if not node.move_joints(last_good):
                    print("❌ 回退失败，停止")
                    break
                continue
            sample, err = node.capture(cal["avg_time_s"])
            if err is not None:
                print(f"  采集失败: {err}，原地重试一次")
                sample, err = node.capture(cal["avg_time_s"])
            if err is not None:
                print(f"  重试仍失败: {err}，回上一个可见姿态")
                if not node.move_joints(last_good):
                    break
                continue
            samples.append(sample)
            last_good = target
            print(
                f"  ✅ 已采集 {len(samples)}/{cal['samples']}，"
                f"标定板像素 ({marker[0]:.0f},{marker[1]:.0f})，"
                f"距离 {marker[2]:.3f} m"
            )
            if len(samples) >= cal["rot_samples"]:
                break

        if len(samples) < cal["samples"] and base_pos is not None:
            print("\n回到基准 TCP 位姿，开始纯平移采集...")
            if node.move_pose(base_pos, base_quat):
                for j, (dx, dy, dz) in enumerate(cfg["translate_deltas_mm"]):
                    if len(samples) >= cal["samples"]:
                        break
                    target = [
                        base_pos[0] + dx / 1000.0,
                        base_pos[1] + dy / 1000.0,
                        base_pos[2] + dz / 1000.0,
                    ]
                    print(
                        f"[平移 {j + 1}/{len(cfg['translate_deltas_mm'])}] "
                        f"Δ=({dx:+d},{dy:+d},{dz:+d}) mm",
                        flush=True,
                    )
                    if not node.move_pose(target, base_quat):
                        node.move_pose(base_pos, base_quat)
                        continue
                    if not node.wait_stable():
                        print("  机械臂未停稳，回基准")
                        node.move_pose(base_pos, base_quat)
                        continue
                    marker = node.marker_pixel()
                    if marker is None:
                        print("  标定板不可见/贴边，回基准")
                        node.move_pose(base_pos, base_quat)
                        continue
                    sample, err = node.capture(cal["avg_time_s"])
                    if err is not None:
                        print(f"  采集失败: {err}，原地重试一次")
                        sample, err = node.capture(cal["avg_time_s"])
                    if err is not None:
                        print(f"  重试仍失败: {err}，回基准")
                        node.move_pose(base_pos, base_quat)
                        continue
                    samples.append(sample)
                    print(
                        f"  ✅ 已采集 {len(samples)}/{cal['samples']}，"
                        f"标定板像素 ({marker[0]:.0f},{marker[1]:.0f})，"
                        f"距离 {marker[2]:.3f} m"
                    )
            else:
                print("⚠️ 无法回到基准 TCP 位姿，跳过平移阶段")

        print("\n回到基准姿态...")
        node.move_joints(base)
        if len(samples) < 5:
            print(f"❌ 有效样本只有 {len(samples)} 组，无法标定")
            return 1

        print(f"\n开始计算（{len(samples)} 组样本，比较 5 种算法）...")
        results = calib.compute_all(samples)
        if not results:
            print("❌ 所有算法都失败（样本退化）")
            return 1
        print(f"\n{'方法':<12}{'平移RMS(mm)':>14}{'旋转RMS(deg)':>16}")
        for r in results:
            val = r["validation"]
            print(
                f"{r['name']:<12}{val['translation_rms_mm']:>14.2f}"
                f"{val['rotation_rms_deg']:>16.2f}"
            )
        best = min(results, key=lambda r: r["score"])
        marker_size = args.marker_size or cfg["board"]["square_length_m"]
        dummy = types.SimpleNamespace(
            marker_id=args.marker_id,
            marker_size=marker_size,
            result_dir=result_dir,
        )
        path, t, quat, rpy, _ = calib.print_result(best, samples, dummy)
        val = best["validation"]
        max_angle, spread = calib.rotation_diversity(samples)
        ok = (
            val["translation_rms_mm"] <= 3.0
            and val["rotation_rms_deg"] <= 1.0
            and max_angle >= 20.0
            and spread >= 80.0
            and np.linalg.norm(t) <= 0.5
        )
        print(f"结果文件: {path}")
        print(
            "✅ 标定通过验收标准"
            if ok
            else "⚠️ 标定结果未通过验收标准，请检查后重试"
        )
        return 0 if ok else 1
    except KeyboardInterrupt:
        print("\n用户中断")
        return 130
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    sys.exit(main())
