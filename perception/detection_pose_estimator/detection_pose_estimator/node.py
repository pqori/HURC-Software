import rclpy
from rclpy.node import Node
from sensor_msgs.msg import PointCloud2
from vision_msgs.msg import Detection2DArray, Detection2D
from geometry_msgs.msg import PoseStamped, PoseArray, Pose
from std_msgs.msg import Header, Int32MultiArray
import sensor_msgs_py.point_cloud2 as pc2
import tf2_ros
from tf_transformations import quaternion_from_euler
import numpy as np
from typing import Dict, List, Tuple, Optional
import time
from scipy.optimize import linear_sum_assignment
from collections import deque
import math


class TrackedObject:
    """Represents a tracked object with history and confidence"""

    def __init__(self, object_id: int, detection: Detection2D, pose: Pose, timestamp: float):
        self.id = object_id
        self.class_name = detection.results[0].hypothesis.class_id if detection.results else "unknown"
        self.confidence_history = deque(maxlen=10)
        self.pose_history = deque(maxlen=20)
        self.detection_history = deque(maxlen=5)

        # Initialize with first detection
        self.confidence_history.append(
            detection.results[0].score if detection.results else 0.0)
        self.pose_history.append((pose, timestamp))
        self.detection_history.append(detection)

        self.last_seen = timestamp
        self.creation_time = timestamp
        self.lost_count = 0
        self.confirmed = False

    def update(self, detection: Detection2D, pose: Pose, timestamp: float):
        """Update object with new detection and pose"""
        self.confidence_history.append(
            detection.results[0].score if detection.results else 0.0)
        self.pose_history.append((pose, timestamp))
        self.detection_history.append(detection)
        self.last_seen = timestamp
        self.lost_count = 0

        # Confirm object after consistent detections
        if len(self.confidence_history) >= 3 and self.avg_confidence() > 0.6:
            self.confirmed = True

    def avg_confidence(self) -> float:
        """Get average confidence over history"""
        return np.mean(self.confidence_history) if self.confidence_history else 0.0

    def get_smoothed_pose(self) -> Pose:
        """Get smoothed pose using weighted average"""
        if not self.pose_history:
            return Pose()

        # Use exponential weighting (more recent poses have higher weight)
        weights = np.exp(np.linspace(-2, 0, len(self.pose_history)))
        weights /= np.sum(weights)

        smoothed_pose = Pose()
        total_x = total_y = total_z = 0.0

        for i, (pose, _) in enumerate(self.pose_history):
            total_x += pose.position.x * weights[i]
            total_y += pose.position.y * weights[i]
            total_z += pose.position.z * weights[i]

        smoothed_pose.position.x = total_x
        smoothed_pose.position.y = total_y
        smoothed_pose.position.z = total_z
        smoothed_pose.orientation = self.pose_history[-1][0].orientation

        return smoothed_pose

    def is_stale(self, current_time: float, timeout: float = 2.0) -> bool:
        """Check if object is stale (not seen recently)"""
        return (current_time - self.last_seen) > timeout


class PoseEstimator(Node):
    """Multi-object pose estimator with tracking"""

    def __init__(self):
        super().__init__('pose_estimator')

        # Parameters
        self.declare_parameter('detection_topic', '/detected_objects')
        self.declare_parameter(
            'cloud_topic', '/zed2i/point_cloud/cloud_registered')
        self.declare_parameter('poses_topic', '/tracked_object_poses')
        self.declare_parameter('pose_array_topic', '/object_pose_array')
        self.declare_parameter('tracking_ids_topic', '/object_tracking_ids')
        self.declare_parameter('frame_id', 'map')
        self.declare_parameter('min_confidence', 0.5)
        # Max distance for association
        self.declare_parameter('max_distance', 1.0)
        # Time before object is lost
        self.declare_parameter('tracking_timeout', 3.0)

        # Get parameters
        dt = self.get_parameter('detection_topic').value
        ct = self.get_parameter('cloud_topic').value
        pat = self.get_parameter('pose_array_topic').value
        tit = self.get_parameter('tracking_ids_topic').value
        self.frame_id = self.get_parameter('frame_id').value
        self.min_conf = self.get_parameter('min_confidence').value
        self.max_distance = self.get_parameter('max_distance').value
        self.tracking_timeout = self.get_parameter('tracking_timeout').value

        # TF setup
        self.tf_buffer = tf2_ros.Buffer()
        tf2_ros.TransformListener(self.tf_buffer, self)

        # Subscribers
        self.create_subscription(Detection2DArray, dt, self.cb_detections, 10)
        self.create_subscription(PointCloud2, ct, self.cb_cloud, 1)

        # Publishers
        self.poses_pub = self.create_publisher(PoseArray, pat, 10)
        self.tracking_ids_pub = self.create_publisher(Int32MultiArray, tit, 10)

        # Tracking state
        self.tracked_objects: Dict[int, TrackedObject] = {}
        self.next_object_id = 1
        self.latest_detections = None
        self.latest_cloud = None

        # Performance monitoring
        self.create_timer(1.0, self.cleanup_stale_objects)

        self.get_logger().info("🎯 Multi-Object Pose Estimator initialized")

    def cb_detections(self, msg: Detection2DArray):
        """Handle incoming detections"""
        # Filter by confidence
        self.latest_detections = [d for d in msg.detections
                                  if d.results and d.results[0].score >= self.min_conf]
        self.process_detections()


    def cb_cloud(self, msg: PointCloud2):
        """Handle incoming point cloud"""
        self.latest_cloud = msg
        self.process_detections()

    def process_detections(self):
        """Process detections and update tracking"""
        if not self.latest_detections or not self.latest_cloud:
            return

        current_time = time.time()

        # Extract 3D poses from detections
        detection_poses = []
        for detection in self.latest_detections:
            pose = self.extract_3d_pose(detection)
            if pose:
                detection_poses.append((detection, pose))

        if not detection_poses:
            return

        # Associate detections with existing tracks
        associations = self.associate_detections(detection_poses, current_time)

        # Update existing tracks
        updated_ids = set()
        for detection_idx, track_id in associations.items():
            if track_id in self.tracked_objects:
                detection, pose = detection_poses[detection_idx]
                self.tracked_objects[track_id].update(
                    detection, pose, current_time)
                updated_ids.add(track_id)

        # Create new tracks for unassociated detections
        for i, (detection, pose) in enumerate(detection_poses):
            if i not in associations:
                new_id = self.next_object_id
                self.next_object_id += 1
                self.tracked_objects[new_id] = TrackedObject(
                    new_id, detection, pose, current_time)
                self.get_logger().info(f"🆕 New object tracked: ID {new_id}")

        # Increment lost count for non-updated tracks
        for track_id, obj in self.tracked_objects.items():
            if track_id not in updated_ids:
                obj.lost_count += 1

        # Publish results
        self.publish_tracked_poses(current_time)

        # Clear processed detections
        self.latest_detections = None

    def extract_3d_pose(self, detection: Detection2D) -> Optional[Pose]:
        """Extract 3D pose from 2D detection using point cloud"""
        # Get detection center pixel
        xc = int(detection.bbox.center.position.x)
        yc = int(detection.bbox.center.position.y)

        # Sample multiple points around detection center for robustness
        sample_points = [
            (xc, yc),
            (xc - 5, yc), (xc + 5, yc),
            (xc, yc - 5), (xc, yc + 5)
        ]

        valid_points = []
        for px, py in sample_points:
            try:
                for (x, y, z, *_) in pc2.read_points(self.latest_cloud,
                                                     skip_nans=True,
                                                     uvs=[(px, py)]):
                    if z > 0 and z < 50:  # Valid depth range
                        valid_points.append([x, y, z])
                        break
            except:
                continue

        if not valid_points:
            return None

        # Use median of valid points for robustness
        median_point = np.median(valid_points, axis=0)

        # Transform to map frame
        try:
            now = rclpy.time.Time().to_msg()
            transform = self.tf_buffer.lookup_transform(
                self.frame_id,
                self.latest_cloud.header.frame_id,
                now
            )

            # Apply transform (simplified)
            pose = Pose()
            pose.position.x = median_point[0] + \
                transform.transform.translation.x
            pose.position.y = median_point[1] + \
                transform.transform.translation.y
            pose.position.z = median_point[2] + \
                transform.transform.translation.z

            # Set orientation facing the camera
            q = quaternion_from_euler(0, 0, 0)
            pose.orientation.x = q[0]
            pose.orientation.y = q[1]
            pose.orientation.z = q[2]
            pose.orientation.w = q[3]

            return pose

        except Exception as e:
            self.get_logger().warn(f"TF transform failed: {e}")
            return None

    def associate_detections(self, detection_poses: List[Tuple], current_time: float) -> Dict[int, int]:
        """Associate detections with existing tracks using Hungarian algorithm"""
        if not self.tracked_objects or not detection_poses:
            return {}

        # Create cost matrix
        track_ids = list(self.tracked_objects.keys())
        n_tracks = len(track_ids)
        n_detections = len(detection_poses)

        cost_matrix = np.full((n_detections, n_tracks), np.inf)

        for i, (detection, pose) in enumerate(detection_poses):
            for j, track_id in enumerate(track_ids):
                track = self.tracked_objects[track_id]

                # Skip if class doesn't match
                det_class = detection.results[0].hypothesis.class_id if detection.results else "unknown"
                if track.class_name != det_class:
                    continue

                # Calculate distance cost
                if track.pose_history:
                    last_pose = track.pose_history[-1][0]
                    distance = math.sqrt(
                        (pose.position.x - last_pose.position.x) ** 2 +
                        (pose.position.y - last_pose.position.y) ** 2 +
                        (pose.position.z - last_pose.position.z) ** 2
                    )

                    if distance < self.max_distance:
                        cost_matrix[i, j] = distance

        # Solve assignment problem
        associations = {}
        if n_detections > 0 and n_tracks > 0:
            row_indices, col_indices = linear_sum_assignment(cost_matrix)

            for row, col in zip(row_indices, col_indices):
                if cost_matrix[row, col] < self.max_distance:
                    associations[row] = track_ids[col]

        return associations

    def publish_tracked_poses(self, current_time: float):
        """Publish poses of all confirmed tracked objects"""
        pose_array = PoseArray()
        pose_array.header = Header()
        pose_array.header.stamp = self.get_clock().now().to_msg()
        pose_array.header.frame_id = self.frame_id

        tracking_ids = Int32MultiArray()

        confirmed_objects = [obj for obj in self.tracked_objects.values()
                             if obj.confirmed and not obj.is_stale(current_time, self.tracking_timeout)]

        for obj in confirmed_objects:
            smoothed_pose = obj.get_smoothed_pose()
            pose_array.poses.append(smoothed_pose)
            tracking_ids.data.append(obj.id)

        self.poses_pub.publish(pose_array)
        self.tracking_ids_pub.publish(tracking_ids)

        if confirmed_objects:
            self.get_logger().debug(
                f"📍 Published {len(confirmed_objects)} tracked object poses")

    def cleanup_stale_objects(self):
        """Remove stale objects from tracking"""
        current_time = time.time()
        stale_ids = []

        for obj_id, obj in self.tracked_objects.items():
            if obj.is_stale(current_time, self.tracking_timeout):
                stale_ids.append(obj_id)

        for obj_id in stale_ids:
            obj = self.tracked_objects.pop(obj_id)
            self.get_logger().info(f"🗑️ Removed stale object: ID {obj_id}")


def main(args=None):
    rclpy.init(args=args)
    node = PoseEstimator()
    rclpy.spin(node)
    rclpy.shutdown()
