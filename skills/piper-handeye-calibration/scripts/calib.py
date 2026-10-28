#!/usr/bin/env python3
"""眼在手上（eye-in-hand）手眼标定。

订阅：
  /aruco_single/pose   标定板在相机坐标系下的位姿（aruco_detect 发布）
  /feedback/tcp_pose   末端在基座坐标系下的位姿（驱动发布，tcp_offset=0 时即法兰）

操作：
  Enter 采集一组（自动取约 0.6 s 数据并求平均）
  d     删除上一组
  q     计算并保存结果（采集满 min_num 组后可用）
  c     直接退出，不保存

结果约定：T_tcp_cam，即相机在法兰（TCP）坐标系下的位姿。
"""

import argparse
import datetime
import json
import os
import sys
import time

import cv2
import numpy as np
import rclpy
from geometry_msgs.msg import PoseStamped
from rclpy.node import Node
from scipy.spatial.transform import Rotation

MARKER_TOPIC = "/aruco_single/pose"
TCP_TOPIC = "/feedback/tcp_pose"

METHODS = [
    ("TSAI", cv2.CALIB_HAND_EYE_TSAI),
    ("PARK", cv2.CALIB_HAND_EYE_PARK),
    ("HORAUD", cv2.CALIB_HAND_EYE_HORAUD),
    ("ANDREFF", cv2.CALIB_HAND_EYE_ANDREFF),
    ("DANIILIDIS", cv2.CALIB_HAND_EYE_DANIILIDIS),
]


def pose_data(msg: PoseStamped):
    p = msg.pose.position
    q = msg.pose.orientation
    return (np.array([p.x, p.y, p.z], dtype=float),
            np.array([q.x, q.y, q.z, q.w], dtype=float))


def average_pose(samples):
    """返回 (R, t, 平移RMS_m, 旋转RMS_deg)。"""
    positions = np.array([s[0] for s in samples])
    quats = np.array([s[1] for s in samples], dtype=float)
    # q 与 -q 表示同一姿态，统一符号后再平均
    for i in range(1, len(quats)):
        if np.dot(quats[i], quats[0]) < 0:
            quats[i] = -quats[i]
    rot = Rotation.from_quat(quats).mean()
    center = positions.mean(axis=0)
    trans_rms = float(np.sqrt(np.mean(np.sum((positions - center) ** 2, axis=1))))
    angles = np.array([(rot.inv() * Rotation.from_quat(q)).magnitude() for q in quats])
    rot_rms_deg = float(np.degrees(np.sqrt(np.mean(angles ** 2))))
    return rot, center, trans_rms, rot_rms_deg


def T_from(R, t):
    T = np.eye(4)
    T[:3, :3] = R
    T[:3, 3] = np.asarray(t).reshape(3)
    return T


class Collector(Node):
    def __init__(self):
        super().__init__("handeye_calibration_tool")
        self.marker_latest = None
        self.marker_time = 0.0
        self.collecting = False
        self.marker_buf = []
        self.tcp_buf = []
        self.create_subscription(PoseStamped, MARKER_TOPIC, self.marker_cb, 10)
        self.create_subscription(PoseStamped, TCP_TOPIC, self.tcp_cb, 10)

    def marker_cb(self, msg):
        self.marker_latest = msg
        self.marker_time = time.monotonic()
        if self.collecting:
            self.marker_buf.append(pose_data(msg))

    def tcp_cb(self, msg):
        if self.collecting:
            self.tcp_buf.append(pose_data(msg))

    def spin_for(self, duration):
        end = time.monotonic() + duration
        while time.monotonic() < end:
            rclpy.spin_once(self, timeout_sec=min(0.05, end - time.monotonic()))

    def capture(self, avg_time):
        self.marker_buf.clear()
        self.tcp_buf.clear()
        self.collecting = True
        self.spin_for(avg_time)
        if len(self.marker_buf) < 2:
            # 相机帧率低或检测偶尔丢帧时，再等一个窗口
            self.spin_for(avg_time)
        self.collecting = False
        if len(self.marker_buf) < 2:
            return None, "没有收到足够的标定板位姿（标定板是否在视野内？）"
        if len(self.tcp_buf) < 10:
            return None, "没有收到足够的 /feedback/tcp_pose（驱动是否在运行？）"
        tcp = average_pose(self.tcp_buf)
        marker = average_pose(self.marker_buf)
        if tcp[2] > 0.0008 or tcp[3] > 0.25:
            return None, (f"机械臂未静止：TCP 抖动 {tcp[2] * 1000:.2f} mm / "
                          f"{tcp[3]:.2f}°，等停稳后重试")
        if marker[2] > 0.0008 or marker[3] > 0.25:
            return None, (f"标定板位姿不稳：抖动 {marker[2] * 1000:.2f} mm / "
                          f"{marker[3]:.2f}°，检查标定板是否固定、相机是否碰动")
        return {"tcp": (tcp[0], tcp[1]), "marker": (marker[0], marker[1]),
                "tcp_jitter": tcp[2:], "marker_jitter": marker[2:]}, None

    def marker_distance(self):
        if self.marker_latest is None or time.monotonic() - self.marker_time > 1.0:
            return None
        p, _ = pose_data(self.marker_latest)
        return float(np.linalg.norm(p))


def validate(T_tcp_cam, samples):
    """用标定结果把每组标定板位姿变换到基座系，残差应接近 0。"""
    Ts = []
    for s in samples:
        R_bt, t_bt = s["tcp"]
        R_cm, t_cm = s["marker"]
        Ts.append(T_from(R_bt.as_matrix(), t_bt) @ T_from(*T_tcp_cam)
                  @ T_from(R_cm.as_matrix(), t_cm))
    positions = np.array([T[:3, 3] for T in Ts])
    rotations = Rotation.from_matrix([T[:3, :3] for T in Ts])
    mean_R = rotations.mean()
    mean_p = positions.mean(axis=0)
    trans = np.linalg.norm(positions - mean_p, axis=1)
    angles = np.degrees([(mean_R.inv() * r).magnitude() for r in rotations])
    return {
        "translation_rms_mm": float(np.sqrt(np.mean(trans ** 2)) * 1000),
        "translation_max_mm": float(trans.max() * 1000),
        "rotation_rms_deg": float(np.sqrt(np.mean(angles ** 2))),
        "rotation_max_deg": float(angles.max()),
        "marker_in_base_mean_m": mean_p.tolist(),
    }, Ts


def rotation_diversity(samples):
    rotations = Rotation.from_matrix([s["tcp"][0].as_matrix() for s in samples])
    mean_R = rotations.mean()
    max_angle = max(np.degrees((mean_R.inv() * r).magnitude()) for r in rotations)
    positions = np.array([s["tcp"][1] for s in samples])
    mean_p = positions.mean(axis=0)
    spread = max(np.linalg.norm(p - mean_p) for p in positions)
    return float(max_angle), float(spread * 1000)


def compute_all(samples):
    R_g2b = [s["tcp"][0].as_matrix() for s in samples]
    t_g2b = [s["tcp"][1].reshape(3, 1) for s in samples]
    R_t2c = [s["marker"][0].as_matrix() for s in samples]
    t_t2c = [s["marker"][1].reshape(3, 1) for s in samples]

    results = []
    for name, method in METHODS:
        try:
            R_cg, t_cg = cv2.calibrateHandEye(
                R_g2b, t_g2b, R_t2c, t_t2c, method=method)
        except cv2.error as exc:
            print(f"  {name:<10} 计算失败: {exc}")
            continue
        if not (np.all(np.isfinite(R_cg)) and np.all(np.isfinite(t_cg))):
            print(f"  {name:<10} 结果出现非有限值（样本退化）")
            continue
        T_tcp_cam = (R_cg, t_cg.reshape(3))
        try:
            val, _ = validate(T_tcp_cam, samples)
        except Exception as exc:  # 退化数据可能让旋转平均失败
            print(f"  {name:<10} 验证失败: {exc}")
            continue
        # 旋转误差在 0.3 m 处折算成毫米，与平移误差一起评分
        score = val["translation_rms_mm"] + np.radians(val["rotation_rms_deg"]) * 300.0
        results.append({"name": name, "R": R_cg, "t": t_cg.reshape(3),
                        "validation": val, "score": score})
    return results


def save_result(result, samples, args):
    R, t = result["R"], result["t"]
    quat = Rotation.from_matrix(R).as_quat()
    rpy_deg = Rotation.from_matrix(R).as_euler("xyz", degrees=True)
    T = T_from(R, t)
    timestamp = datetime.datetime.now().strftime("%Y-%m-%d_%H-%M-%S")
    data = {
        "timestamp": timestamp,
        "mode": "eye_in_hand",
        "marker_id": args.marker_id,
        "marker_size_m": args.marker_size,
        "num_samples": len(samples),
        "result": {
            "method": result["name"],
            "frame": "T_tcp_cam: camera in flange/TCP frame",
            "position_m": t.tolist(),
            "position_mm": (t * 1000).tolist(),
            "orientation_xyzw": quat.tolist(),
            "rpy_deg": rpy_deg.tolist(),
            "matrix4x4": T.tolist(),
            "translation_rms_mm": result["validation"]["translation_rms_mm"],
            "translation_max_mm": result["validation"]["translation_max_mm"],
            "rotation_rms_deg": result["validation"]["rotation_rms_deg"],
            "rotation_max_deg": result["validation"]["rotation_max_deg"],
        },
        "samples": [
            {
                "tcp_position_m": s["tcp"][1].tolist(),
                "tcp_quaternion_xyzw": s["tcp"][0].as_quat().tolist(),
                "marker_position_m": s["marker"][1].tolist(),
                "marker_quaternion_xyzw": s["marker"][0].as_quat().tolist(),
                "tcp_jitter_mm": s["tcp_jitter"][0] * 1000,
                "marker_jitter_mm": s["marker_jitter"][0] * 1000,
            }
            for s in samples
        ],
    }
    out_dir = os.path.expanduser(args.result_dir)
    os.makedirs(out_dir, exist_ok=True)
    path = os.path.join(out_dir, f"{timestamp}_handeye.json")
    with open(path, "w", encoding="utf-8") as f:
        json.dump(data, f, ensure_ascii=False, indent=2)
    return path, t, quat, rpy_deg, T


def print_result(result, samples, args):
    path, t, quat, rpy_deg, T = save_result(result, samples, args)
    val = result["validation"]
    max_angle, spread = rotation_diversity(samples)

    print("\n" + "=" * 64)
    print("标定结果（相机在法兰 / TCP 坐标系下的位姿）")
    print("=" * 64)
    print(f"最佳方法   : {result['name']}（共 {len(samples)} 组样本）")
    print(f"位置 (mm)  : [{t[0] * 1000:+.2f} {t[1] * 1000:+.2f} {t[2] * 1000:+.2f}]")
    print(f"四元数     : [{quat[0]:+.6f} {quat[1]:+.6f} {quat[2]:+.6f} {quat[3]:+.6f}]")
    print(f"RPY (deg)  : [{rpy_deg[0]:+.2f} {rpy_deg[1]:+.2f} {rpy_deg[2]:+.2f}]")
    print("\n4x4 矩阵 (T_tcp_cam):")
    for row in T:
        print("  [" + " ".join(f"{v:+.6f}" for v in row) + "]")
    print(f"\n一致性残差 : 平移 {val['translation_rms_mm']:.2f} mm RMS"
          f"（最大 {val['translation_max_mm']:.2f} mm），"
          f"旋转 {val['rotation_rms_deg']:.2f}° RMS"
          f"（最大 {val['rotation_max_deg']:.2f}°）")
    print(f"姿态多样性 : 最大夹角 {max_angle:.1f}°，末端位置分散 {spread:.0f} mm")
    print("URDF 写法  :")
    print(f'  <origin xyz="{t[0]:.6f} {t[1]:.6f} {t[2]:.6f}" '
          f'rpy="{np.radians(rpy_deg[0]):.6f} {np.radians(rpy_deg[1]):.6f} '
          f'{np.radians(rpy_deg[2]):.6f}"/>')
    print(f"\n已保存     : {path}")

    if np.linalg.norm(t) > 0.5:
        print("❌ 结果位置异常（|t| > 0.5 m），样本几乎肯定退化，请不要使用。")
    if max_angle < 20:
        print("⚠️ 末端姿态变化不足 20°，标定结果不可靠，建议增加倾斜姿态。")
    if spread < 80:
        print("⚠️ 末端位置变化不足 80 mm，建议增加不同位置。")
    if max_angle < 20 or spread < 80:
        print("⚠️ 样本多样性不足，本次结果不要直接使用。")
    elif val["translation_rms_mm"] > 3.0 or val["rotation_rms_deg"] > 1.0:
        print("⚠️ 残差偏大：检查标定板是否平整固定、marker_size 是否准确、"
              "样本是否都在静止状态下采集。")
    elif np.linalg.norm(t) <= 0.5:
        print("✅ 残差和样本多样性都在合理范围内。")


def run(node, args):
    samples = []
    print("眼在手上手眼标定（标定板固定在桌面，相机固定在末端）")
    print(f"最少 {args.min} 组；建议采集 15 组以上，并让末端在多个方向倾斜。")
    print("移动机械臂请在另一个终端用 arm 命令；每次停稳后再按 Enter。")

    while True:
        node.spin_for(0.2)
        dist = node.marker_distance()
        status = f"标定板距离 {dist:.3f} m" if dist is not None else "标定板不可见"
        print(f"\n------- 第 {len(samples) + 1} 组（已有 {len(samples)} 组）-------")
        print(status)
        menu = "Enter-采集"
        if samples:
            menu += ", d-回退"
        if len(samples) >= args.min:
            menu += ", q-计算并保存"
        menu += ", c-退出: "
        try:
            cmd = input(menu).strip()
        except (EOFError, KeyboardInterrupt):
            print("\n已退出，未保存。")
            return 1

        if cmd == "":
            sample, err = node.capture(args.avg_time)
            if err is not None:
                print(f"⚠️ {err}")
                continue
            samples.append(sample)
            dist = np.linalg.norm(sample["marker"][1])
            print(f"已采集 {len(samples)} 组：标定板距相机 {dist:.3f} m，"
                  f"TCP 抖动 {sample['tcp_jitter'][0] * 1000:.2f} mm")
            if dist < 0.15 or dist > 0.8:
                print("⚠️ 距离建议保持在 0.15~0.8 m，太近/太远都会降低精度。")
        elif cmd == "d":
            if samples:
                samples.pop()
                print("已删除上一组。")
            else:
                print("没有可删除的样本。")
        elif cmd == "q":
            if len(samples) < args.min:
                print(f"至少需要 {args.min} 组，当前 {len(samples)} 组。")
                continue
            break
        elif cmd == "c":
            print("已退出，未保存。")
            return 1
        else:
            print("无效输入。")

    print("\n开始计算（比较 5 种标定算法，用一致性残差选最优）...")
    results = compute_all(samples)
    if not results:
        print("❌ 所有算法都计算失败，样本可能退化（姿态变化不足）。")
        return 1

    print(f"\n{'方法':<12}{'平移RMS(mm)':>14}{'旋转RMS(deg)':>16}")
    for r in results:
        val = r["validation"]
        print(f"{r['name']:<12}{val['translation_rms_mm']:>14.2f}"
              f"{val['rotation_rms_deg']:>16.2f}")

    best = min(results, key=lambda r: r["score"])
    print_result(best, samples, args)
    return 0


def parse_args():
    parser = argparse.ArgumentParser(description="eye-in-hand 手眼标定")
    parser.add_argument("--min", type=int, default=10, help="最少采集组数")
    parser.add_argument("--avg-time", type=float, default=0.6,
                        help="每组平均时长（秒）")
    parser.add_argument("--marker-id", type=int, default=0)
    parser.add_argument("--marker-size", type=float, default=0.05,
                        help="标定板黑框外边长，单位米")
    parser.add_argument("--result-dir", default="~/agx_arm_ws/tools/calib_results")
    return parser.parse_args()


def main():
    args = parse_args()
    if args.min < 5:
        print("--min 不能小于 5")
        return 1
    rclpy.init()
    node = Collector()
    try:
        return run(node, args)
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    sys.exit(main())
