#!/usr/bin/env python3
"""
Multi-Modal Sensor Fusion
=========================

Fuses detections from multiple sensors (YOLO, ArUco) with confidence-based
weighting and geometric consistency checking.

Features:
- YOLO + ArUco detection fusion
- Confidence-based sensor weighting
- Geometric consistency validation
- Cross-modal object association
- Uncertainty estimation

Author: URC Perception Team
License: Apache-2.0
"""

import rclpy
from rclpy.node import Node
from vision_msgs.msg import Detection2DArray, Detection3DArray
from geometry_msgs.msg import PoseArray, Pose, PoseWithCovarianceStamped
from std_msgs.msg import Header, Float32MultiArray
import tf2_ros
import numpy as np
from typing import Dict, List, Tuple, Optional, NamedTuple
import math
from scipy.spatial.distance import cdist
from scipy.optimize import linear_sum_assignment
import time


class FusedDetection(NamedTuple):
    """Represents a fused detection from multiple sensors"""
    pose: Pose
    class_name: str
    confidence: float
    uncertainty: np.ndarray  # 3x3 covariance matrix
    source_sensors: List[str]
    sensor_confidences: Dict[str, float]
    geometric_consistency: float


class SensorFusion(Node):
    """Multi-modal sensor fusion node with fault tolerance"""
    
    def __init__(self):
        super().__init__('sensor_fusion')
        
        # Parameters
        self.declare_parameter('yolo_detections_topic', '/detected_objects')
        self.declare_parameter('aruco_detections_topic', '/aruco_detections')
        self.declare_parameter('fused_poses_topic', '/fused_object_poses')
        self.declare_parameter('fused_confidences_topic', '/fused_confidences')
        self.declare_parameter('frame_id', 'map')
        self.declare_parameter('fusion_distance_threshold', 1.0)
        self.declare_parameter('min_confidence_threshold', 0.4)
        self.declare_parameter('geometric_consistency_threshold', 0.7)
        self.declare_parameter('yolo_weight', 0.6)
        self.declare_parameter('aruco_weight', 0.8)  # Higher weight for precise ArUco
        self.declare_parameter('temporal_window', 0.5)  # Seconds for temporal fusion
        
        # Get parameters
        yolo_topic = self.get_parameter('yolo_detections_topic').value
        aruco_topic = self.get_parameter('aruco_detections_topic').value
        poses_topic = self.get_parameter('fused_poses_topic').value
        conf_topic = self.get_parameter('fused_confidences_topic').value
        self.frame_id = self.get_parameter('frame_id').value
        self.fusion_threshold = self.get_parameter('fusion_distance_threshold').value
        self.min_confidence = self.get_parameter('min_confidence_threshold').value
        self.consistency_threshold = self.get_parameter('geometric_consistency_threshold').value
        self.yolo_weight = self.get_parameter('yolo_weight').value
        self.aruco_weight = self.get_parameter('aruco_weight').value
        self.temporal_window = self.get_parameter('temporal_window').value
        
        # TF setup
        self.tf_buffer = tf2_ros.Buffer()
        tf2_ros.TransformListener(self.tf_buffer, self)
        
        # Subscribers
        self.create_subscription(Detection2DArray, yolo_topic, self.cb_yolo_detections, 10)
        self.create_subscription(Detection3DArray, aruco_topic, self.cb_aruco_detections, 10)
        
        # Publishers
        self.fused_poses_pub = self.create_publisher(PoseArray, poses_topic, 10)
        self.fused_conf_pub = self.create_publisher(Float32MultiArray, conf_topic, 10)
        
        # Detection buffers with timestamps
        self.yolo_buffer = []
        self.aruco_buffer = []
        
        # Fault tolerance
        self.sensor_health = {
            'yolo': {'last_data': 0.0, 'failure_count': 0, 'active': True},
            'aruco': {'last_data': 0.0, 'failure_count': 0, 'active': True}
        }
        self.sensor_timeout = 5.0  # seconds
        self.max_failures = 10
        
        # Quality tracking
        self.fusion_quality_history = deque(maxlen=100)
        
        # Fusion timer
        self.create_timer(0.1, self.perform_fusion)  # 10 Hz fusion rate
        self.create_timer(1.0, self.monitor_sensor_health)  # Health monitoring
        
        self.get_logger().info("🔗 Multi-Modal Sensor Fusion with Fault Tolerance initialized")
    
    def cb_yolo_detections(self, msg: Detection2DArray):
        """Handle YOLO detections with fault tolerance"""
        timestamp = time.time()
        
        try:
            # Update sensor health
            self.sensor_health['yolo']['last_data'] = timestamp
            self.sensor_health['yolo']['active'] = True
            
            # Convert 2D detections to 3D poses (simplified - should use proper depth)
            detections_3d = []
            for det in msg.detections:
                if det.results and det.results[0].score >= self.min_confidence:
                    # For now, create a placeholder 3D pose
                    # In practice, this would use the detection_pose_estimator output
                    pose = Pose()
                    pose.position.x = det.bbox.center.position.x / 100.0  # Simplified conversion
                    pose.position.y = det.bbox.center.position.y / 100.0
                    pose.position.z = 1.0  # Default height
                    
                    detection_data = {
                        'pose': pose,
                        'class_name': det.results[0].hypothesis.class_id,
                        'confidence': det.results[0].score,
                        'sensor': 'yolo',
                        'timestamp': timestamp,
                        'quality': self.assess_detection_quality(det)
                    }
                    detections_3d.append(detection_data)
            
            # Add to buffer and clean old entries
            self.yolo_buffer.extend(detections_3d)
            self.clean_buffer(self.yolo_buffer, timestamp)
            
        except Exception as e:
            self.get_logger().error(f"YOLO detection processing failed: {e}")
            self.sensor_health['yolo']['failure_count'] += 1
    
    def cb_aruco_detections(self, msg: Detection3DArray):
        """Handle ArUco detections"""
        timestamp = time.time()
        
        detections_3d = []
        for det in msg.detections:
            if det.results and det.results[0].hypothesis.score >= self.min_confidence:
                detection_data = {
                    'pose': det.bbox.center,
                    'class_name': f"aruco_{det.results[0].hypothesis.class_id}",
                    'confidence': det.results[0].hypothesis.score,
                    'sensor': 'aruco',
                    'timestamp': timestamp
                }
                detections_3d.append(detection_data)
        
        # Add to buffer and clean old entries
        self.aruco_buffer.extend(detections_3d)
        self.clean_buffer(self.aruco_buffer, timestamp)
    
    def clean_buffer(self, buffer: List, current_time: float):
        """Remove old detections from buffer"""
        buffer[:] = [det for det in buffer 
                    if current_time - det['timestamp'] <= self.temporal_window]
    
    def perform_fusion(self):
        """Perform multi-modal sensor fusion"""
        current_time = time.time()
        
        # Clean buffers
        self.clean_buffer(self.yolo_buffer, current_time)
        self.clean_buffer(self.aruco_buffer, current_time)
        
        if not self.yolo_buffer and not self.aruco_buffer:
            return
        
        # Combine all detections
        all_detections = self.yolo_buffer + self.aruco_buffer
        
        if not all_detections:
            return
        
        # Perform fusion
        fused_detections = self.fuse_detections(all_detections)
        
        # Publish results
        self.publish_fused_results(fused_detections)
    
    def fuse_detections(self, detections: List[Dict]) -> List[FusedDetection]:
        """Fuse detections from multiple sensors"""
        if not detections:
            return []
        
        # Group detections by spatial proximity and class compatibility
        detection_groups = self.group_detections(detections)
        
        fused_results = []
        for group in detection_groups:
            fused_detection = self.fuse_detection_group(group)
            if fused_detection:
                fused_results.append(fused_detection)
        
        return fused_results
    
    def group_detections(self, detections: List[Dict]) -> List[List[Dict]]:
        """Group spatially close and class-compatible detections"""
        if not detections:
            return []
        
        # Extract positions for distance calculation
        positions = np.array([[det['pose'].position.x, 
                              det['pose'].position.y, 
                              det['pose'].position.z] for det in detections])
        
        # Calculate pairwise distances
        distances = cdist(positions, positions)
        
        # Group detections
        groups = []
        used_indices = set()
        
        for i, det in enumerate(detections):
            if i in used_indices:
                continue
            
            # Start new group
            group = [det]
            used_indices.add(i)
            
            # Find nearby detections
            for j, other_det in enumerate(detections):
                if j in used_indices or i == j:
                    continue
                
                if distances[i, j] <= self.fusion_threshold:
                    # Check class compatibility
                    if self.are_classes_compatible(det['class_name'], other_det['class_name']):
                        group.append(other_det)
                        used_indices.add(j)
            
            groups.append(group)
        
        return groups
    
    def are_classes_compatible(self, class1: str, class2: str) -> bool:
        """Check if two classes can be fused"""
        # ArUco markers can be associated with any object class
        if class1.startswith('aruco_') or class2.startswith('aruco_'):
            return True
        
        # Same class names are compatible
        if class1 == class2:
            return True
        
        # Define compatible class groups
        compatible_groups = [
            {'bottle', 'container'},
            {'rock', 'mineral_sample', 'boulder'},
            {'person', 'astronaut', 'scientist'},
            {'toolbox', 'equipment_cache'},
        ]
        
        for group in compatible_groups:
            if class1 in group and class2 in group:
                return True
        
        return False
    
    def fuse_detection_group(self, group: List[Dict]) -> Optional[FusedDetection]:
        """Fuse a group of compatible detections"""
        if not group:
            return None
        
        # Separate by sensor type
        yolo_dets = [det for det in group if det['sensor'] == 'yolo']
        aruco_dets = [det for det in group if det['sensor'] == 'aruco']
        
        # Calculate weighted average pose
        fused_pose = self.calculate_weighted_pose(group)
        
        # Determine fused class name (prefer non-ArUco classes)
        non_aruco_classes = [det['class_name'] for det in group 
                           if not det['class_name'].startswith('aruco_')]
        if non_aruco_classes:
            fused_class = max(set(non_aruco_classes), key=non_aruco_classes.count)
        else:
            fused_class = group[0]['class_name']
        
        # Calculate fused confidence
        fused_confidence = self.calculate_fused_confidence(group)
        
        # Calculate uncertainty (covariance)
        uncertainty = self.calculate_uncertainty(group, fused_pose)
        
        # Calculate geometric consistency
        consistency = self.calculate_geometric_consistency(group)
        
        # Only return if confidence and consistency are sufficient
        if (fused_confidence >= self.min_confidence and 
            consistency >= self.consistency_threshold):
            
            return FusedDetection(
                pose=fused_pose,
                class_name=fused_class,
                confidence=fused_confidence,
                uncertainty=uncertainty,
                source_sensors=[det['sensor'] for det in group],
                sensor_confidences={det['sensor']: det['confidence'] for det in group},
                geometric_consistency=consistency
            )
        
        return None
    
    def calculate_weighted_pose(self, detections: List[Dict]) -> Pose:
        """Calculate weighted average pose"""
        total_weight = 0.0
        weighted_x = weighted_y = weighted_z = 0.0
        
        for det in detections:
            # Weight by sensor type and confidence
            sensor_weight = self.aruco_weight if det['sensor'] == 'aruco' else self.yolo_weight
            confidence_weight = det['confidence']
            total_weight_det = sensor_weight * confidence_weight
            
            weighted_x += det['pose'].position.x * total_weight_det
            weighted_y += det['pose'].position.y * total_weight_det
            weighted_z += det['pose'].position.z * total_weight_det
            total_weight += total_weight_det
        
        fused_pose = Pose()
        if total_weight > 0:
            fused_pose.position.x = weighted_x / total_weight
            fused_pose.position.y = weighted_y / total_weight
            fused_pose.position.z = weighted_z / total_weight
        
        # Use orientation from highest confidence detection
        best_det = max(detections, key=lambda d: d['confidence'])
        fused_pose.orientation = best_det['pose'].orientation
        
        return fused_pose
    
    def calculate_fused_confidence(self, detections: List[Dict]) -> float:
        """Calculate fused confidence using sensor fusion principles"""
        if not detections:
            return 0.0
        
        # Multi-sensor confidence boost
        sensor_types = set(det['sensor'] for det in detections)
        multi_sensor_bonus = 0.1 if len(sensor_types) > 1 else 0.0
        
        # Weighted average confidence
        total_weight = 0.0
        weighted_confidence = 0.0
        
        for det in detections:
            sensor_weight = self.aruco_weight if det['sensor'] == 'aruco' else self.yolo_weight
            weighted_confidence += det['confidence'] * sensor_weight
            total_weight += sensor_weight
        
        base_confidence = weighted_confidence / total_weight if total_weight > 0 else 0.0
        
        return min(1.0, base_confidence + multi_sensor_bonus)
    
    def calculate_uncertainty(self, detections: List[Dict], fused_pose: Pose) -> np.ndarray:
        """Calculate 3x3 covariance matrix representing pose uncertainty"""
        if len(detections) == 1:
            # Single detection - higher uncertainty
            return np.diag([0.5, 0.5, 0.3])  # Higher uncertainty in x,y than z
        
        # Calculate spread of detections
        positions = np.array([[det['pose'].position.x, 
                              det['pose'].position.y, 
                              det['pose'].position.z] for det in detections])
        
        # Calculate covariance
        if len(positions) > 1:
            cov = np.cov(positions.T)
            # Ensure positive definite
            cov += np.eye(3) * 0.01
        else:
            cov = np.diag([0.2, 0.2, 0.1])
        
        return cov
    
    def calculate_geometric_consistency(self, detections: List[Dict]) -> float:
        """Calculate geometric consistency score for detection group"""
        if len(detections) <= 1:
            return 1.0
        
        # Calculate pairwise distances
        positions = np.array([[det['pose'].position.x, 
                              det['pose'].position.y, 
                              det['pose'].position.z] for det in detections])
        
        distances = cdist(positions, positions)
        
        # Consistency is inversely related to maximum distance
        max_distance = np.max(distances)
        consistency = max(0.0, 1.0 - (max_distance / self.fusion_threshold))
        
        return consistency
    
    def publish_fused_results(self, fused_detections: List[FusedDetection]):
        """Publish fused detection results"""
        if not fused_detections:
            return
        
        # Publish pose array
        pose_array = PoseArray()
        pose_array.header = Header()
        pose_array.header.stamp = self.get_clock().now().to_msg()
        pose_array.header.frame_id = self.frame_id
        
        confidences = Float32MultiArray()
        
        for detection in fused_detections:
            pose_array.poses.append(detection.pose)
            confidences.data.append(detection.confidence)
        
        self.fused_poses_pub.publish(pose_array)
        self.fused_conf_pub.publish(confidences)
        
        self.get_logger().debug(f"🔗 Published {len(fused_detections)} fused detections")
    
    def assess_detection_quality(self, detection) -> float:
        """Assess quality of individual detection"""
        # Base quality from confidence
        quality = detection.results[0].score if detection.results else 0.0
        
        # Adjust based on bounding box properties
        bbox_area = detection.bbox.size_x * detection.bbox.size_y
        if bbox_area < 100:  # Very small detection
            quality *= 0.7
        elif bbox_area > 50000:  # Very large detection
            quality *= 0.8
        
        return quality
    
    def monitor_sensor_health(self):
        """Monitor health of sensor inputs"""
        current_time = time.time()
        
        for sensor_name, health in self.sensor_health.items():
            # Check for timeout
            if current_time - health['last_data'] > self.sensor_timeout:
                if health['active']:
                    self.get_logger().warn(f"⚠️ Sensor {sensor_name} timeout detected")
                    health['active'] = False
                    health['failure_count'] += 1
            
            # Check for excessive failures
            if health['failure_count'] > self.max_failures:
                if health['active']:
                    self.get_logger().error(f"🚨 Sensor {sensor_name} disabled due to excessive failures")
                    health['active'] = False
        
        # Adapt fusion strategy based on available sensors
        active_sensors = [name for name, health in self.sensor_health.items() if health['active']]
        
        if len(active_sensors) == 0:
            self.get_logger().error("🚨 No active sensors for fusion!")
        elif len(active_sensors) == 1:
            self.get_logger().warn(f"⚠️ Single sensor mode: {active_sensors[0]}")
        
        # Log sensor status periodically
        if int(current_time) % 30 == 0:  # Every 30 seconds
            status = {name: "active" if health['active'] else "failed" 
                     for name, health in self.sensor_health.items()}
            self.get_logger().info(f"📊 Sensor status: {status}")


def main(args=None):
    """Main entry point"""
    rclpy.init(args=args)
    
    try:
        node = SensorFusion()
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        rclpy.shutdown()


if __name__ == '__main__':
    main()
