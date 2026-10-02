#!/usr/bin/env python3
"""
Production ROS 2 ArUco detector node (composable).

Subscribes to ZED wrapper topics, detects ArUco markers,
and publishes 3D detections with optional depth integration.
"""

import cv2
import numpy as np
from cv2 import aruco
from typing import Optional
import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, ReliabilityPolicy, HistoryPolicy, DurabilityPolicy
from rclpy.lifecycle import LifecycleNode, LifecycleState, TransitionCallbackReturn
from cv_bridge import CvBridge
from message_filters import ApproximateTimeSynchronizer, Subscriber
import tf2_ros
from tf2_ros import TransformException
from geometry_msgs.msg import TransformStamped, Pose, Point, Quaternion
from sensor_msgs.msg import Image, CameraInfo
from vision_msgs.msg import Detection3DArray, Detection3D, ObjectHypothesisWithPose, BoundingBox3D
from visualization_msgs.msg import MarkerArray, Marker
from diagnostic_updater import Updater, DiagnosticStatusWrapper
from scipy.spatial.transform import Rotation
import time

from .core import ArucoDetectorCore, rvec_tvec_to_pose_matrix
from .zed_interface import ROSBridgeAdaptor


# Sensor data QoS profile
SENSOR_QOS = QoSProfile(
    reliability=ReliabilityPolicy.BEST_EFFORT,
    history=HistoryPolicy.KEEP_LAST,
    depth=5,
    durability=DurabilityPolicy.VOLATILE
)


class ArucoDetectorNode(Node):
    """
    ArUco marker detector node (composable).

    Subscribes to image/depth/camera_info and publishes detections.
    """

    def __init__(self, node_name='aruco_detector'):
        super().__init__(node_name)

        # ==================== Parameters ==================== #
        self._declare_parameters()

        # ==================== Core Detector ==================== #
        dictionary_name = self.get_parameter('dictionary').value
        dictionary = getattr(aruco, dictionary_name)

        self.detector = ArucoDetectorCore(
            dictionary=dictionary,
            marker_size_m=self.get_parameter('marker_size_m').value,
            min_side_px=self.get_parameter('min_side_px').value,
            adaptive_thresh_win_size_min=self.get_parameter(
                'adaptive_thresh_win_size_min').value,
            adaptive_thresh_win_size_max=self.get_parameter(
                'adaptive_thresh_win_size_max').value,
            adaptive_thresh_win_size_step=self.get_parameter(
                'adaptive_thresh_win_size_step').value,
            enable_async=True
        )

        # ==================== ROS Interface ==================== #
        self.bridge = CvBridge()
        self.ros_adaptor = ROSBridgeAdaptor(self)

        # TF2 for frame transformations
        self.tf_buffer = tf2_ros.Buffer()
        self.tf_listener = tf2_ros.TransformListener(self.tf_buffer, self)

        # ==================== Subscriptions ==================== #
        rgb_topic = self.get_parameter('rgb_topic').value
        cam_info_topic = self.get_parameter('cam_info_topic').value
        depth_topic = self.get_parameter('depth_topic').value
        use_depth = self.get_parameter('use_depth').value

        # Camera info subscription
        self.cam_info_sub = self.create_subscription(
            CameraInfo,
            cam_info_topic,
            self._camera_info_cb,
            qos_profile=SENSOR_QOS
        )

        # Image + optional depth time sync
        if use_depth and depth_topic:
            self.get_logger().info("Enabling depth integration with ApproximateTimeSynchronizer")

            self.image_sub = Subscriber(
                self, Image, rgb_topic, qos_profile=SENSOR_QOS)
            self.depth_sub = Subscriber(
                self, Image, depth_topic, qos_profile=SENSOR_QOS)

            self.sync = ApproximateTimeSynchronizer(
                [self.image_sub, self.depth_sub],
                queue_size=10,
                slop=0.05  # 50ms tolerance
            )
            self.sync.registerCallback(self._synced_callback)
        else:
            self.get_logger().info("Depth integration disabled")
            self.create_subscription(
                Image,
                rgb_topic,
                self._image_only_cb,
                qos_profile=SENSOR_QOS
            )

        # ==================== Publishers ==================== #
        self.detection_pub = self.create_publisher(
            Detection3DArray,
            'aruco_detections',
            10
        )

        publish_markers = self.get_parameter('publish_markers').value
        self.marker_pub = None
        if publish_markers:
            self.marker_pub = self.create_publisher(
                MarkerArray,
                'aruco_markers',
                10
            )

        # ==================== Diagnostics ==================== #
        self.diagnostics = Updater(self)
        self.diagnostics.setHardwareID("aruco_detector")
        self.diagnostics.add("detector_status", self._diagnostic_callback)

        # Stats
        self.frame_count = 0
        self.detection_count = 0
        self.last_detection_time = 0.0
        self.start_time = time.time()

        # FPS throttling
        max_fps = self.get_parameter('max_fps').value
        if max_fps > 0:
            self.min_frame_interval = 1.0 / max_fps
        else:
            self.min_frame_interval = 0.0
        self.last_process_time = 0.0

        self.get_logger().info(
            f"ArUco detector initialized ({dictionary_name})")

    def _declare_parameters(self):
        """Declare all ROS parameters."""
        self.declare_parameter('dictionary', 'DICT_4X4_50')
        self.declare_parameter('marker_size_m', 0.15)
        self.declare_parameter('min_side_px', 20)
        self.declare_parameter('adaptive_thresh_win_size_min', 3)
        self.declare_parameter('adaptive_thresh_win_size_max', 23)
        self.declare_parameter('adaptive_thresh_win_size_step', 10)
        self.declare_parameter('rgb_topic', '/zed2i/left/image_rect_color')
        self.declare_parameter('cam_info_topic', '/zed2i/left/camera_info')
        self.declare_parameter('depth_topic', '/zed2i/depth/depth_registered')
        self.declare_parameter('use_depth', True)
        self.declare_parameter(
            'output_frame', 'zed2i_left_camera_optical_frame')
        self.declare_parameter('transform_to_base_link', False)
        self.declare_parameter('publish_markers', True)
        self.declare_parameter('max_fps', 30)

    def _camera_info_cb(self, msg: CameraInfo):
        """Cache camera calibration."""
        self.ros_adaptor.set_camera_info(msg)

    def _image_only_cb(self, img_msg: Image):
        """Process image without depth sync."""
        if not self.ros_adaptor.is_ready():
            return

        self._process_frame(img_msg, None)

    def _synced_callback(self, img_msg: Image, depth_msg: Image):
        """Process time-synchronized image + depth."""
        if not self.ros_adaptor.is_ready():
            return

        # Convert depth image
        try:
            depth_image = self.bridge.imgmsg_to_cv2(
                depth_msg, desired_encoding='32FC1')
            self.ros_adaptor.set_depth_image(depth_image)
        except Exception as e:
            self.get_logger().warn(
                f"Failed to convert depth image: {e}", throttle_duration_sec=5.0)

        self._process_frame(img_msg, depth_msg)

    def _process_frame(self, img_msg: Image, depth_msg: Optional[Image]):
        """Core frame processing logic."""
        # FPS throttling
        now = time.time()
        if self.min_frame_interval > 0:
            if (now - self.last_process_time) < self.min_frame_interval:
                return
            self.last_process_time = now

        # Convert image
        try:
            img_bgr = self.bridge.imgmsg_to_cv2(
                img_msg, desired_encoding='bgr8')
        except Exception as e:
            self.get_logger().error(f"Failed to convert image: {e}")
            return

        # Get calibration
        calib = self.ros_adaptor.get_calibration()
        depth_sampler = self.ros_adaptor.create_depth_sampler()

        # Run detection
        detections = self.detector.detect(
            img_bgr,
            calib.camera_matrix,
            calib.dist_coeffs,
            depth_sampler
        )

        self.frame_count += 1

        # Publish results
        if detections:
            self.detection_count += len(detections)
            self.last_detection_time = now

            self._publish_detections(
                detections, img_msg.header, depth_msg is not None)

            if self.marker_pub:
                self._publish_markers(detections, img_msg.header)
        else:
            # Publish empty array
            empty = Detection3DArray()
            empty.header = img_msg.header
            self.detection_pub.publish(empty)

        # Update diagnostics
        self.diagnostics.update()

    def _publish_detections(self, detections, header, has_depth: bool):
        """
        Publish Detection3DArray message.

        Args:
            detections: List of MarkerDetection objects
            header: Image header (for timestamp)
            has_depth: Whether depth was available
        """
        det_array = Detection3DArray()
        det_array.header = header

        # Optionally transform to base_link
        output_frame = self.get_parameter('output_frame').value
        transform_to_base = self.get_parameter('transform_to_base_link').value

        transform = None
        if transform_to_base:
            try:
                transform = self.tf_buffer.lookup_transform(
                    'base_link',
                    header.frame_id,
                    rclpy.time.Time(),
                    timeout=rclpy.duration.Duration(seconds=0.1)
                )
                det_array.header.frame_id = 'base_link'
            except TransformException as e:
                self.get_logger().warn(
                    f"TF lookup failed, publishing in camera frame: {e}",
                    throttle_duration_sec=5.0
                )
                det_array.header.frame_id = output_frame
        else:
            det_array.header.frame_id = output_frame

        # Convert each detection
        for det in detections:
            detection_msg = Detection3D()

            # Convert rvec/tvec to pose
            T = rvec_tvec_to_pose_matrix(det.rvec, det.tvec)

            # Apply transform if available
            if transform is not None:
                T = self._apply_transform(T, transform)

            # Extract position and orientation
            position = T[:3, 3]
            R = Rotation.from_matrix(T[:3, :3])
            quat = R.as_quat()  # [x, y, z, w]

            # Set pose
            pose = Pose()
            pose.position = Point(x=position[0], y=position[1], z=position[2])
            pose.orientation = Quaternion(
                x=quat[0], y=quat[1], z=quat[2], w=quat[3])

            # Create bounding box (small cube around marker)
            marker_size = self.get_parameter('marker_size_m').value
            bbox = BoundingBox3D()
            bbox.center = pose
            bbox.size.x = marker_size
            bbox.size.y = marker_size
            bbox.size.z = 0.001  # Thin box

            detection_msg.bbox = bbox

            # Hypothesis with marker ID
            hypo = ObjectHypothesisWithPose()
            hypo.hypothesis.class_id = str(det.id)

            # Score from reprojection error (normalized)
            score = 1.0 / (1.0 + det.reprojection_error)
            hypo.hypothesis.score = float(np.clip(score, 0.0, 1.0))

            hypo.pose.pose = pose

            detection_msg.results.append(hypo)

            # Add ID as tracking_id (optional extension)
            detection_msg.id = str(det.id)

            det_array.detections.append(detection_msg)

        self.detection_pub.publish(det_array)

    def _publish_markers(self, detections, header):
        """Publish visualization markers for RViz."""
        marker_array = MarkerArray()

        output_frame = self.get_parameter('output_frame').value

        for i, det in enumerate(detections):
            # Axis marker
            marker = Marker()
            marker.header = header
            marker.header.frame_id = output_frame
            marker.ns = "aruco_axes"
            marker.id = det.id
            marker.type = Marker.LINE_LIST
            marker.action = Marker.ADD

            # Convert pose
            T = rvec_tvec_to_pose_matrix(det.rvec, det.tvec)
            pos = T[:3, 3]
            R_mat = T[:3, :3]

            marker.pose.position = Point(x=pos[0], y=pos[1], z=pos[2])
            marker.pose.orientation = Quaternion(x=0.0, y=0.0, z=0.0, w=1.0)

            # Draw XYZ axes
            axis_length = 0.05
            origin = Point(x=0.0, y=0.0, z=0.0)

            x_axis = Point(x=axis_length, y=0.0, z=0.0)
            y_axis = Point(x=0.0, y=axis_length, z=0.0)
            z_axis = Point(x=0.0, y=0.0, z=axis_length)

            marker.points = [origin, x_axis, origin, y_axis, origin, z_axis]

            # RGB colors for XYZ
            from std_msgs.msg import ColorRGBA
            red = ColorRGBA(r=1.0, g=0.0, b=0.0, a=1.0)
            green = ColorRGBA(r=0.0, g=1.0, b=0.0, a=1.0)
            blue = ColorRGBA(r=0.0, g=0.0, b=1.0, a=1.0)
            marker.colors = [red, red, green, green, blue, blue]

            marker.scale.x = 0.01  # Line width
            marker.lifetime = rclpy.duration.Duration(seconds=0.5).to_msg()

            marker_array.markers.append(marker)

            # Text label
            text_marker = Marker()
            text_marker.header = marker.header
            text_marker.ns = "aruco_labels"
            text_marker.id = det.id + 1000
            text_marker.type = Marker.TEXT_VIEW_FACING
            text_marker.action = Marker.ADD

            text_marker.pose.position = Point(
                x=pos[0], y=pos[1], z=pos[2] + 0.05)
            text_marker.pose.orientation = Quaternion(
                x=0.0, y=0.0, z=0.0, w=1.0)

            if det.range_m is not None:
                text_marker.text = f"ID:{det.id}\n{det.range_m:.2f}m"
            else:
                text_marker.text = f"ID:{det.id}"

            text_marker.scale.z = 0.03
            text_marker.color = ColorRGBA(r=1.0, g=1.0, b=1.0, a=1.0)
            text_marker.lifetime = rclpy.duration.Duration(
                seconds=0.5).to_msg()

            marker_array.markers.append(text_marker)

        self.marker_pub.publish(marker_array)

    def _apply_transform(self, T: np.ndarray, transform: TransformStamped) -> np.ndarray:
        """Apply TF2 transform to pose matrix."""
        # Extract transform as matrix
        trans = transform.transform.translation
        rot = transform.transform.rotation

        T_tf = np.eye(4)
        T_tf[:3, 3] = [trans.x, trans.y, trans.z]

        R_tf = Rotation.from_quat([rot.x, rot.y, rot.z, rot.w])
        T_tf[:3, :3] = R_tf.as_matrix()

        # Apply transform
        return T_tf @ T

    def _diagnostic_callback(self, stat: DiagnosticStatusWrapper):
        """Update diagnostic status."""
        stats = self.detector.get_stats()

        # Compute FPS
        elapsed = time.time() - self.start_time
        fps = self.frame_count / elapsed if elapsed > 0 else 0.0

        # Detection rate
        detection_age = time.time() - self.last_detection_time

        stat.summary(DiagnosticStatusWrapper.OK, "Running")
        stat.add("Frames processed", str(self.frame_count))
        stat.add("Detections", str(self.detection_count))
        stat.add("FPS", f"{fps:.1f}")
        stat.add("Queue depth", str(stats['queue_depth']))
        stat.add("Frames dropped", str(stats['frames_dropped']))
        stat.add("Last detection age (s)", f"{detection_age:.2f}")

        # Warn if no recent detections
        if detection_age > 10.0 and self.frame_count > 50:
            stat.summary(DiagnosticStatusWrapper.WARN, "No recent detections")

        return stat

    def destroy_node(self):
        """Clean shutdown."""
        self.detector.shutdown()
        super().destroy_node()


def main(args=None):
    """Entry point."""
    rclpy.init(args=args)

    node = None
    try:
        node = ArucoDetectorNode()
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    except Exception as e:
        if node:
            node.get_logger().error(f"Unexpected error: {e}")
    finally:
        # Clean shutdown with proper error handling
        if node is not None:
            try:
                node.destroy_node()
            except Exception:
                pass

        # Safe shutdown - check if already shut down
        try:
            rclpy.try_shutdown()
        except Exception:
            pass


if __name__ == '__main__':
    main()
