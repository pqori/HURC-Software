#!/usr/bin/env python3
"""
Standalone ArUco detector that consumes ROS image topics instead of direct ZED SDK.
Renders marker boxes and IDs in an OpenCV window.
"""

import argparse
import time
from typing import Optional

import cv2
import numpy as np
import rclpy
from cv_bridge import CvBridge
from cv2 import aruco
from rclpy.node import Node
from sensor_msgs.msg import CameraInfo, Image

from aruco_detector.core import ArucoDetectorCore


class ROSArucoDetector(Node):
    def __init__(self, args: argparse.Namespace):
        super().__init__("standalone_aruco_detector_ros")
        self.args = args
        self.bridge = CvBridge()

        self.detector = ArucoDetectorCore(
            dictionary=getattr(aruco, args.dictionary),
            marker_size_m=args.marker_size,
            min_side_px=20,
            adaptive_thresh_win_size_min=3,
            adaptive_thresh_win_size_max=23,
            adaptive_thresh_win_size_step=10,
            enable_async=False,
        )

        self.camera_matrix: Optional[np.ndarray] = None
        self.dist_coeffs: Optional[np.ndarray] = None
        self.latest_depth: Optional[np.ndarray] = None
        self.last_frame_time = 0.0
        self.frame_count = 0
        self.fps_time = time.time()

        self.create_subscription(CameraInfo, args.cam_info_topic, self.cam_info_cb, 10)
        self.create_subscription(Image, args.rgb_topic, self.image_cb, 5)
        if args.use_depth:
            self.create_subscription(Image, args.depth_topic, self.depth_cb, 5)

        self.get_logger().info(
            f"RGB={args.rgb_topic} camera_info={args.cam_info_topic} depth={args.depth_topic if args.use_depth else 'off'}"
        )

    def cam_info_cb(self, msg: CameraInfo) -> None:
        k = np.array(msg.k, dtype=np.float64).reshape(3, 3)
        d = np.array(msg.d, dtype=np.float64)
        self.camera_matrix = k
        self.dist_coeffs = d

    def depth_cb(self, msg: Image) -> None:
        if not self.args.use_depth:
            return
        try:
            if msg.encoding == "32FC1":
                self.latest_depth = self.bridge.imgmsg_to_cv2(msg, desired_encoding="32FC1")
            else:
                depth_mm = self.bridge.imgmsg_to_cv2(msg, desired_encoding="16UC1")
                self.latest_depth = depth_mm.astype(np.float32) / 1000.0
        except Exception:
            self.latest_depth = None

    def _depth_sampler(self, u: int, v: int) -> float:
        if self.latest_depth is None:
            return float("nan")
        h, w = self.latest_depth.shape[:2]
        if u < 0 or v < 0 or u >= w or v >= h:
            return float("nan")
        z = float(self.latest_depth[v, u])
        return z if np.isfinite(z) and z > 0.0 else float("nan")

    def image_cb(self, msg: Image) -> None:
        if self.camera_matrix is None or self.dist_coeffs is None:
            return

        frame = self.bridge.imgmsg_to_cv2(msg, desired_encoding="bgr8")
        depth_sampler = self._depth_sampler if self.args.use_depth else None
        detections = self.detector.detect(frame, self.camera_matrix, self.dist_coeffs, depth_sampler)
        rendered = self.render(frame, detections)

        cv2.imshow("ArUco Detector (ROS Topic)", rendered)
        if cv2.waitKey(1) in (27, ord("q"), ord("Q")):
            rclpy.shutdown()
            return

        self.last_frame_time = time.time()
        self.frame_count += 1
        if self.last_frame_time - self.fps_time >= 2.0:
            fps = self.frame_count / (self.last_frame_time - self.fps_time)
            self.get_logger().info(f"FPS: {fps:.2f} markers: {len(detections)}")
            self.fps_time = self.last_frame_time
            self.frame_count = 0

    def render(self, frame: np.ndarray, detections) -> np.ndarray:
        output = frame.copy()
        for det in detections:
            pts = det.corners_px.astype(np.int32)
            cv2.polylines(output, [pts], True, (0, 220, 0), 2)
            center = tuple(np.mean(pts, axis=0).astype(np.int32))
            label = f"id={det.id}"
            if self.args.show_pnp_range:
                pnp_range_m = float(np.linalg.norm(det.tvec) / 1000.0)
                if np.isfinite(pnp_range_m) and pnp_range_m > 0.0:
                    label += f" {pnp_range_m:.2f}m"
            if det.range_m is not None and np.isfinite(det.range_m):
                label += f" (depth {det.range_m:.2f}m)"
            cv2.putText(output, label, (center[0] - 40, center[1] - 10), cv2.FONT_HERSHEY_SIMPLEX, 0.6, (0, 220, 0), 2)
        return output


def main() -> int:
    parser = argparse.ArgumentParser(description="ROS-topic-based ArUco standalone detector")
    parser.add_argument("--rgb-topic", default="/zed2i/zed2i_camera/left/image_rect_color", help="ROS RGB image topic")
    parser.add_argument("--cam-info-topic", default="/zed2i/zed2i_camera/left/camera_info", help="ROS camera info topic")
    parser.add_argument("--depth-topic", default="/zed2i/zed2i_camera/depth/depth_registered", help="ROS depth image topic")
    parser.add_argument("--use-depth", action="store_true", help="Enable depth-assisted range estimate")
    parser.add_argument("--show-pnp-range", action="store_true", help="Show distance from solvePnP (no depth required)")
    parser.add_argument("--marker-size", type=float, default=0.15, help="Marker size in meters")
    parser.add_argument("--dictionary", default="DICT_4X4_50", help="OpenCV ArUco dictionary name")
    parser.add_argument("--svo", default="", help="Compatibility flag; ignored in ROS mode")
    parser.add_argument("--no-gl", action="store_true", help="Compatibility flag; ignored in ROS mode")
    args = parser.parse_args()

    if args.svo:
        print("[WARN] --svo is ignored for ROS topic mode.")
    if args.no_gl:
        print("[INFO] --no-gl set; ROS mode is 2D only.")

    rclpy.init()
    node = ROSArucoDetector(args)
    timeout_start = time.time()
    try:
        while rclpy.ok():
            rclpy.spin_once(node, timeout_sec=0.1)
            if node.last_frame_time == 0.0 and time.time() - timeout_start > 5.0:
                node.get_logger().error(f"No frames on {args.rgb_topic} after 5s.")
                break
    finally:
        node.destroy_node()
        cv2.destroyAllWindows()
        rclpy.shutdown()
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
