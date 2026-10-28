#!/usr/bin/env python3
"""ArUco 标定板位姿发布节点。

替代 aruco_ros 的 single 节点：aruco_ros 5.x 内置的字典集合识别不了
OpenCV 的 DICT_4X4_50 标定板（本机标定板为 4x4_50, id=0, 50 mm）。
本节点用 OpenCV 检测，发布与 aruco_ros 相同的话题 /aruco_single/pose。
"""

import argparse
import json
import os
import time

import cv2
import numpy as np
import rclpy
from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data
from geometry_msgs.msg import PoseStamped
from sensor_msgs.msg import CameraInfo, Image
from scipy.spatial.transform import Rotation
from std_msgs.msg import Header

# OpenCV 的 ArUco 字典名 -> cv2.aruco 常量
DICT_NAMES = [
    "DICT_4X4_50", "DICT_4X4_100", "DICT_4X4_250", "DICT_4X4_1000",
    "DICT_5X5_50", "DICT_5X5_100", "DICT_5X5_250", "DICT_5X5_1000",
    "DICT_6X6_50", "DICT_6X6_100", "DICT_6X6_250", "DICT_6X6_1000",
    "DICT_7X7_50", "DICT_7X7_100", "DICT_7X7_250", "DICT_7X7_1000",
    "DICT_ARUCO_ORIGINAL",
    "DICT_APRILTAG_16h5", "DICT_APRILTAG_25h9",
    "DICT_APRILTAG_36h10", "DICT_APRILTAG_36h11",
]


class ArucoDetector(Node):
    def __init__(self, args):
        super().__init__("aruco_detector")
        self.marker_id = args.marker_id
        self.marker_size = args.marker_size
        self.frame_id = args.frame_id
        self.blur = args.blur

        if args.dictionary not in DICT_NAMES:
            raise ValueError(f"不支持的字典 {args.dictionary}，可选：{', '.join(DICT_NAMES)}")
        dictionary_id = getattr(cv2.aruco, args.dictionary)
        self.dictionary = cv2.aruco.Dictionary_get(dictionary_id)

        params = cv2.aruco.DetectorParameters_create()
        params.cornerRefinementMethod = cv2.aruco.CORNER_REFINE_SUBPIX
        params.cornerRefinementWinSize = 5
        params.cornerRefinementMaxIterations = 30
        params.cornerRefinementMinAccuracy = 0.1
        params.adaptiveThreshWinSizeMin = 3
        params.adaptiveThreshWinSizeMax = 23
        params.adaptiveThreshWinSizeStep = 10
        self.detector_params = params

        self.mode = args.board
        self.board_mode = self.mode != "single"
        self.board = None
        self.board_corners = None
        self.multi_board = None
        self.multi_ids = []
        self.multi_obj_points = {}
        # 单个码的四个角点；多码板重新初始化时也用它
        half = self.marker_size / 2.0
        self.object_points = np.array([
            [-half,  half, 0.0],
            [ half,  half, 0.0],
            [ half, -half, 0.0],
            [-half, -half, 0.0],
        ], dtype=np.float32)
        if self.mode == "charuco":
            self.board = cv2.aruco.CharucoBoard_create(
                args.squares_x, args.squares_y,
                args.square_length, args.marker_length, self.dictionary)
            self.board_corners = np.array(
                self.board.chessboardCorners, dtype=np.float64).reshape(-1, 3)
            self.min_charuco_corners = args.min_charuco_corners
        elif self.mode == "multi":
            layout_path = os.path.expanduser(args.layout)
            with open(layout_path, "r", encoding="utf-8") as f:
                layout = json.load(f)
            self.multi_ids = [int(x) for x in layout["ids"]]
            self.multi_obj_points = {
                int(k): np.array(v, dtype=np.float32)
                for k, v in layout["object_points"].items()
            }
            board_points = [self.multi_obj_points[i] for i in self.multi_ids]
            self.multi_board = cv2.aruco.Board_create(
                board_points, self.dictionary, np.array(self.multi_ids))

        self.camera_matrix = None
        self.dist_coeffs = None

        self.pose_pub = self.create_publisher(PoseStamped, args.pose_topic, 10)
        self.debug_pub = None
        if args.debug_topic:
            self.debug_pub = self.create_publisher(Image, args.debug_topic, 1)

        self.create_subscription(
            CameraInfo, args.camera_info_topic, self.camera_info_cb,
            qos_profile_sensor_data)
        self.create_subscription(
            Image, args.image_topic, self.image_cb, qos_profile_sensor_data)

        self.logged_first = False
        self.last_rot = None
        self.last_tvec = None
        self.last_pose_time = 0.0
        self.fps_t0 = time.time()
        self.fps_count = 0
        if self.mode == "charuco":
            self.get_logger().info(
                f"ChArUco 板 {args.squares_x}x{args.squares_y}，"
                f"方格 {args.square_length * 1000:.1f} mm，"
                f"码 {args.marker_length * 1000:.1f} mm，字典 {args.dictionary}")
        elif self.mode == "multi":
            self.get_logger().info(
                f"多码板 ids={self.multi_ids}，字典 {args.dictionary}，"
                f"布局 {os.path.expanduser(args.layout)}")
        else:
            self.get_logger().info(
                f"字典 {args.dictionary}，marker_id={self.marker_id}，"
                f"marker_size={self.marker_size * 1000:.1f} mm")
        self.get_logger().info(f"订阅 {args.image_topic} -> 发布 {args.pose_topic}")

    def camera_info_cb(self, msg: CameraInfo):
        if self.camera_matrix is not None:
            return
        self.camera_matrix = np.array(msg.k, dtype=np.float64).reshape(3, 3)
        self.dist_coeffs = np.array(msg.d, dtype=np.float64)
        self.get_logger().info(
            f"相机内参已就绪: fx={self.camera_matrix[0, 0]:.2f} "
            f"fy={self.camera_matrix[1, 1]:.2f} "
            f"cx={self.camera_matrix[0, 2]:.2f} cy={self.camera_matrix[1, 2]:.2f}")

    def image_cb(self, msg: Image):
        if self.camera_matrix is None:
            return
        # 没有任何订阅者时不做检测，省 CPU
        if (self.pose_pub.get_subscription_count() == 0 and
                (self.debug_pub is None or self.debug_pub.get_subscription_count() == 0)):
            return

        gray = self.to_gray(msg)
        if gray is None:
            return
        if self.blur >= 3:
            gray = cv2.GaussianBlur(gray, (self.blur, self.blur), 0)

        corners, ids, _ = cv2.aruco.detectMarkers(
            gray, self.dictionary, parameters=self.detector_params)
        if ids is None:
            return
        if self.mode == "charuco":
            self.process_board(gray, corners, ids, msg)
            return
        if self.mode == "multi":
            self.process_multi(gray, corners, ids, msg)
            return

        for corner, marker_id in zip(corners, ids.flatten()):
            if int(marker_id) != self.marker_id:
                continue
            pose = self.estimate_pose(corner.reshape(4, 2))
            if pose is None:
                return
            rvec, tvec, reproj = pose

            quat = Rotation.from_matrix(cv2.Rodrigues(rvec)[0]).as_quat()
            out = PoseStamped()
            out.header = Header()
            out.header.stamp = msg.header.stamp
            out.header.frame_id = self.frame_id
            out.pose.position.x = float(tvec[0])
            out.pose.position.y = float(tvec[1])
            out.pose.position.z = float(tvec[2])
            out.pose.orientation.x = float(quat[0])
            out.pose.orientation.y = float(quat[1])
            out.pose.orientation.z = float(quat[2])
            out.pose.orientation.w = float(quat[3])
            self.pose_pub.publish(out)

            if not self.logged_first:
                self.logged_first = True
                dist = float(np.linalg.norm(tvec))
                self.get_logger().info(
                    f"首次检测到 marker {self.marker_id}: 距离 {dist:.3f} m，"
                    f"重投影误差 {reproj:.2f} px")

            if self.debug_pub is not None and self.debug_pub.get_subscription_count() > 0:
                debug = cv2.cvtColor(gray, cv2.COLOR_GRAY2BGR)
                cv2.aruco.drawDetectedMarkers(debug, [corner.reshape(1, 4, 2)], ids)
                cv2.drawFrameAxes(debug, self.camera_matrix, self.dist_coeffs,
                                  rvec, tvec, self.marker_size * 0.5)
                self.debug_pub.publish(self.to_image(debug, msg.header))
            return

    def board_tag_pose(self, tag_id):
        """由布局中的四个角点反推 tag 在板坐标系下的位姿。"""
        pts = self.multi_obj_points[tag_id].reshape(4, 3).astype(np.float64)
        center = pts.mean(axis=0)
        x = ((pts[1] - pts[0]) + (pts[2] - pts[3])) * 0.5
        y = ((pts[0] - pts[3]) + (pts[1] - pts[2])) * 0.5
        x /= np.linalg.norm(x)
        y -= x * np.dot(x, y)
        y /= np.linalg.norm(y)
        z = np.cross(x, y)
        return np.column_stack([x, y, z]), center

    def process_multi(self, gray, corners, ids, msg):
        """多码板：用布局文件把可见的多个 tag 合成一块板求位姿。"""
        board_ids = set(self.multi_ids)
        selected = [(c, int(i)) for c, i in zip(corners, ids.flatten())
                    if int(i) in board_ids]
        # 单码退化时容易产生位姿翻转；标定只使用至少两个 tag 同时可见的帧
        if len(selected) < 2:
            return
        sel_corners = [c.reshape(1, 4, 2).astype(np.float32) for c, _ in selected]
        sel_ids = np.array([[i] for _, i in selected], dtype=np.int32)
        obj_list, img_list = [], []
        for c, i in selected:
            obj_list.append(self.multi_obj_points[i])
            img_list.append(c.reshape(4, 2))
        obj = np.vstack(obj_list).astype(np.float64)
        img = np.vstack(img_list).astype(np.float64)
        K, D = self.camera_matrix, self.dist_coeffs

        now = time.monotonic()
        initial = None
        if self.last_rot is not None and now - self.last_pose_time < 1.0:
            initial = (Rotation.from_matrix(self.last_rot.as_matrix())
                       .as_rotvec().reshape(3, 1),
                       np.asarray(self.last_tvec, float).reshape(3, 1))
        else:
            # 用任意一个可见 tag 的单码位姿 + 布局，给多码板一个初值
            for c, i in selected:
                try:
                    out = cv2.solvePnPGeneric(
                        self.object_points, c.reshape(4, 2).astype(np.float32),
                        K, D, flags=cv2.SOLVEPNP_IPPE_SQUARE)
                except cv2.error:
                    continue
                best = None
                for rv, tv in zip(out[1], out[2]):
                    rv = np.asarray(rv, float).reshape(3, 1)
                    tv = np.asarray(tv, float).reshape(3, 1)
                    R = cv2.Rodrigues(rv)[0]
                    tt = tv.reshape(3)
                    dot = float(np.dot(
                        R[:, 2], -tt / max(np.linalg.norm(tt), 1e-9)))
                    proj, _ = cv2.projectPoints(
                        self.object_points, rv, tv, K, D)
                    err = float(np.mean(np.linalg.norm(
                        proj.reshape(-1, 2) - c.reshape(4, 2), axis=1)))
                    if dot > 0.2 and (best is None or err < best[0]):
                        best = (err, rv, tv)
                if best is None:
                    continue
                R_tag = cv2.Rodrigues(best[1])[0]
                t_tag = best[2].reshape(3)
                R_bt, t_bt = self.board_tag_pose(i)
                R_board = R_tag @ R_bt
                t_board = R_tag @ t_bt + t_tag
                initial = (Rotation.from_matrix(R_board).as_rotvec().reshape(3, 1),
                           t_board.reshape(3, 1))
                break

        try:
            if initial is not None:
                ok, rvec, tvec = cv2.solvePnP(
                    obj, img, K, D, initial[0], initial[1],
                    useExtrinsicGuess=True, flags=cv2.SOLVEPNP_ITERATIVE)
            else:
                ok, rvec, tvec = cv2.solvePnP(
                    obj, img, K, D, flags=cv2.SOLVEPNP_ITERATIVE)
        except cv2.error:
            return
        if not ok:
            return

        def facing_camera(rv, tv):
            R = cv2.Rodrigues(rv)[0]
            tt = tv.reshape(3)
            return float(np.dot(R[:, 2], -tt / max(np.linalg.norm(tt), 1e-9)))

        if facing_camera(rvec, tvec) <= 0.2:
            return
        try:
            rvec, tvec = cv2.solvePnPRefineLM(obj, img, K, D, rvec, tvec)
        except cv2.error:
            pass
        if facing_camera(rvec, tvec) <= 0.2:
            return
        self.last_rot = Rotation.from_matrix(cv2.Rodrigues(rvec)[0])
        self.last_tvec = tvec.reshape(3).copy()
        self.last_pose_time = now

        projected, _ = cv2.projectPoints(
            obj, rvec, tvec, self.camera_matrix, self.dist_coeffs)
        reproj = float(np.mean(np.linalg.norm(
            projected.reshape(-1, 2) - img, axis=1)))

        quat = Rotation.from_matrix(cv2.Rodrigues(rvec)[0]).as_quat()
        out = PoseStamped()
        out.header = Header()
        out.header.stamp = msg.header.stamp
        out.header.frame_id = self.frame_id
        out.pose.position.x = float(tvec[0])
        out.pose.position.y = float(tvec[1])
        out.pose.position.z = float(tvec[2])
        out.pose.orientation.x = float(quat[0])
        out.pose.orientation.y = float(quat[1])
        out.pose.orientation.z = float(quat[2])
        out.pose.orientation.w = float(quat[3])
        self.pose_pub.publish(out)

        if not self.logged_first:
            self.logged_first = True
            self.get_logger().info(
                f"首次检测到多码板: {len(selected)} 个 tag，"
                f"距离 {float(np.linalg.norm(tvec)):.3f} m，"
                f"重投影误差 {reproj:.2f} px")

        if self.debug_pub is not None and self.debug_pub.get_subscription_count() > 0:
            debug = cv2.cvtColor(gray, cv2.COLOR_GRAY2BGR)
            cv2.aruco.drawDetectedMarkers(debug, sel_corners, sel_ids)
            cv2.drawFrameAxes(debug, self.camera_matrix, self.dist_coeffs,
                              rvec, tvec, 0.1)
            self.debug_pub.publish(self.to_image(debug, msg.header))

    def process_board(self, gray, corners, ids, msg):
        """ChArUco 板：插值棋盘角点 -> solvePnP 求整块板位姿。"""
        try:
            _, charuco_corners, charuco_ids = cv2.aruco.interpolateCornersCharuco(
                corners, ids, gray, self.board,
                self.camera_matrix, self.dist_coeffs, minMarkers=1)
        except cv2.error:
            return
        if charuco_corners is None or charuco_ids is None:
            return
        if len(charuco_ids) < self.min_charuco_corners:
            return

        obj = self.board_corners[charuco_ids.flatten()].reshape(-1, 3)
        img = charuco_corners.reshape(-1, 2).astype(np.float64)
        try:
            ok, rvec, tvec = cv2.solvePnP(
                obj, img, self.camera_matrix, self.dist_coeffs,
                flags=cv2.SOLVEPNP_ITERATIVE)
        except cv2.error:
            return
        if not ok:
            return
        try:
            rvec, tvec = cv2.solvePnPRefineLM(
                obj, img, self.camera_matrix, self.dist_coeffs, rvec, tvec)
        except cv2.error:
            pass

        projected, _ = cv2.projectPoints(
            obj, rvec, tvec, self.camera_matrix, self.dist_coeffs)
        reproj = float(np.mean(np.linalg.norm(
            projected.reshape(-1, 2) - img, axis=1)))

        quat = Rotation.from_matrix(cv2.Rodrigues(rvec)[0]).as_quat()
        out = PoseStamped()
        out.header = Header()
        out.header.stamp = msg.header.stamp
        out.header.frame_id = self.frame_id
        out.pose.position.x = float(tvec[0])
        out.pose.position.y = float(tvec[1])
        out.pose.position.z = float(tvec[2])
        out.pose.orientation.x = float(quat[0])
        out.pose.orientation.y = float(quat[1])
        out.pose.orientation.z = float(quat[2])
        out.pose.orientation.w = float(quat[3])
        self.pose_pub.publish(out)

        if not self.logged_first:
            self.logged_first = True
            self.get_logger().info(
                f"首次检测到 ChArUco 板: {len(charuco_ids)} 个角点，"
                f"距离 {float(np.linalg.norm(tvec)):.3f} m，"
                f"重投影误差 {reproj:.2f} px")

        if self.debug_pub is not None and self.debug_pub.get_subscription_count() > 0:
            debug = cv2.cvtColor(gray, cv2.COLOR_GRAY2BGR)
            cv2.aruco.drawDetectedMarkers(debug, corners, ids)
            cv2.aruco.drawDetectedCornersCharuco(
                debug, charuco_corners, charuco_ids, (0, 0, 255))
            cv2.drawFrameAxes(debug, self.camera_matrix, self.dist_coeffs,
                              rvec, tvec, 0.1)
            self.debug_pub.publish(self.to_image(debug, msg.header))

    def estimate_pose(self, image_points: np.ndarray):
        """IPPE 双解 + LM 精化 + 时间连续选解，返回 (rvec, tvec, 重投影误差px)。

        4x4 方形码存在平面位姿二义性：两个解的误差往往只差 0.01 px，
        直接取第一个解会让姿态在两组解之间跳变（实测 2~8°）。
        这里保留上一帧姿态，误差接近时选角度变化更小的解。
        """
        image_points = np.ascontiguousarray(image_points, dtype=np.float32)
        try:
            result = cv2.solvePnPGeneric(
                self.object_points, image_points, self.camera_matrix,
                self.dist_coeffs, flags=cv2.SOLVEPNP_IPPE_SQUARE)
            rvecs, tvecs = result[1], result[2]
        except cv2.error:
            return None
        if rvecs is None or len(rvecs) == 0:
            return None

        candidates = []
        for rvec, tvec in zip(rvecs, tvecs):
            rvec = np.asarray(rvec, dtype=np.float64).reshape(3, 1)
            tvec = np.asarray(tvec, dtype=np.float64).reshape(3, 1)
            try:
                rvec, tvec = cv2.solvePnPRefineLM(
                    self.object_points, image_points, self.camera_matrix,
                    self.dist_coeffs, rvec, tvec)
            except cv2.error:
                pass
            projected, _ = cv2.projectPoints(
                self.object_points, rvec, tvec,
                self.camera_matrix, self.dist_coeffs)
            reproj = float(np.mean(np.linalg.norm(
                projected.reshape(-1, 2) - image_points, axis=1)))
            rot = Rotation.from_rotvec(rvec.reshape(3))
            tv = tvec.reshape(3)
            z = rot.as_matrix()[:, 2]
            dot = float(np.dot(z, -tv / max(np.linalg.norm(tv), 1e-9)))
            candidates.append((reproj, rot, rvec, tvec, dot))
        if not candidates:
            return None
        # 标定码正面朝向相机；翻转解的法向会背离相机，直接排除
        valid = [c for c in candidates if c[4] > 0.2] or candidates

        best = min(valid, key=lambda c: c[0])
        chosen = best
        now = time.monotonic()
        if self.last_rot is not None and now - self.last_pose_time < 1.0:
            nearest = min(
                valid,
                key=lambda c: (self.last_rot.inv() * c[1]).magnitude())
            if nearest[0] <= best[0] + 0.05:
                chosen = nearest
        self.last_rot = chosen[1]
        self.last_tvec = chosen[3].reshape(3)
        self.last_pose_time = now

        reproj, _, rvec, tvec, _ = chosen
        if reproj > 5.0:
            self.get_logger().warn(f"重投影误差偏大: {reproj:.2f} px")
        return rvec, tvec, reproj

    @staticmethod
    def to_gray(msg: Image):
        channels = {"rgb8": 3, "bgr8": 3, "rgba8": 4, "bgra8": 4, "mono8": 1}
        n = channels.get(msg.encoding.lower())
        if n is None:
            return None
        buf = np.frombuffer(bytes(msg.data), dtype=np.uint8)
        img = buf.reshape(msg.height, msg.width, n) if n > 1 else buf.reshape(msg.height, msg.width)
        enc = msg.encoding.lower()
        if enc == "rgb8":
            return cv2.cvtColor(img, cv2.COLOR_RGB2GRAY)
        if enc == "bgr8":
            return cv2.cvtColor(img, cv2.COLOR_BGR2GRAY)
        if enc == "rgba8":
            return cv2.cvtColor(img, cv2.COLOR_RGBA2GRAY)
        if enc == "bgra8":
            return cv2.cvtColor(img, cv2.COLOR_BGRA2GRAY)
        return img

    @staticmethod
    def to_image(bgr: np.ndarray, header) -> Image:
        msg = Image()
        msg.header = header
        msg.height, msg.width = bgr.shape[:2]
        msg.encoding = "bgr8"
        msg.is_bigendian = 0
        msg.step = bgr.shape[1] * 3
        msg.data = bgr.tobytes()
        return msg


def parse_args():
    parser = argparse.ArgumentParser(description="ArUco 标定板位姿发布")
    parser.add_argument("--board", choices=["single", "charuco", "multi"], default="single",
                        help="single=单个码；charuco=ChArUco 板；multi=多码板")
    parser.add_argument("--layout", default="~/agx_arm_ws/tools/calib_board/multi_tag_layout.json",
                        help="多码板布局文件")
    parser.add_argument("--marker-id", type=int, default=0)
    parser.add_argument("--marker-size", type=float, default=0.05,
                        help="单个码黑框外边长，单位米")
    parser.add_argument("--squares-x", type=int, default=7)
    parser.add_argument("--squares-y", type=int, default=5)
    parser.add_argument("--square-length", type=float, default=0.030,
                        help="ChArUco 方格边长，单位米")
    parser.add_argument("--marker-length", type=float, default=0.022,
                        help="ChArUco 码边长，单位米")
    parser.add_argument("--min-charuco-corners", type=int, default=8)
    parser.add_argument("--blur", type=int, default=3,
                        help="检测前高斯模糊核大小，0 关闭；1280x720 下建议 3")
    parser.add_argument("--dictionary", default="DICT_4X4_50")
    parser.add_argument("--image-topic", default="/camera/camera/color/image_raw")
    parser.add_argument("--camera-info-topic", default="/camera/camera/color/camera_info")
    parser.add_argument("--pose-topic", default="/aruco_single/pose")
    parser.add_argument("--frame-id", default="camera_color_optical_frame")
    parser.add_argument("--debug-topic", default="/aruco_single/result",
                        help="带检测框的调试图像，置空可关闭")
    return parser.parse_args()


def main():
    args = parse_args()
    rclpy.init()
    node = ArucoDetector(args)
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()
