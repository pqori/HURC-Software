#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from vision_msgs.msg import Detection2DArray
from object_detection.msg import Detection3DArray, Detection3D
from geometry_msgs.msg import PoseStamped, PoseArray, Pose
from std_msgs.msg import String, Float32MultiArray, Int32MultiArray
from std_srvs.srv import SetBool
from visualization_msgs.msg import Marker, MarkerArray
import time
import numpy as np
from typing import Dict, List, Optional, Tuple


class ObjectSelectionService(Node):
    """Simple object selection using standard messages"""
    
    def __init__(self):
        super().__init__('object_selection_service')
        
        # Detection data (support both 2D and 3D)
        self.current_detections: Optional[Detection2DArray] = None
        self.current_detections_3d: Optional[Detection3DArray] = None
        self.current_poses: Optional[PoseArray] = None
        self.current_tracking_ids: Optional[Int32MultiArray] = None
        self.object_priorities: List[float] = []
        self.object_categories: List[str] = []
        self.selection_status: List[str] = []
        self.selected_object_idx: Optional[int] = None
        self.selected_pose: Optional[Pose] = None
        self.use_zed_3d: bool = True  # Prefer ZED 3D when available
        
        # Mission categories mapping
        self.mission_categories = {
            'bottle': 'science', 'rock': 'science', 'mineral': 'science', 'sample': 'science',
            'toolbox': 'equipment', 'container': 'equipment', 'cable': 'equipment', 'tool': 'equipment',
            'flag': 'navigation', 'post': 'navigation', 'sign': 'navigation', 'marker': 'navigation',
            'person': 'human', 'human': 'human',
            'obstacle': 'obstacle', 'barrier': 'obstacle'
        }
        
        # Subscribers (support both 2D and 3D detection pipelines)
        self.create_subscription(Detection2DArray, '/detected_objects', self.cb_detections, 10)
        self.create_subscription(Detection3DArray, '/zed_detections_3d', self.cb_detections_3d, 10)
        self.create_subscription(PoseArray, '/object_pose_array', self.cb_poses, 10)
        self.create_subscription(Int32MultiArray, '/object_tracking_ids', self.cb_tracking_ids, 10)
        
        # Publishers
        self.priorities_pub = self.create_publisher(Float32MultiArray, '/object_priorities', 10)
        self.categories_pub = self.create_publisher(String, '/object_categories', 10)
        self.selection_pub = self.create_publisher(String, '/selection_status', 10)
        self.target_pub = self.create_publisher(PoseStamped, '/target_pose', 10)
        self.approach_pub = self.create_publisher(PoseStamped, '/approach_pose', 10)
        self.markers_pub = self.create_publisher(MarkerArray, '/object_selection_markers', 10)
        
        # Services
        self.create_service(SetBool, '/select_best_science', self.select_science)
        self.create_service(SetBool, '/select_best_equipment', self.select_equipment)
        self.create_service(SetBool, '/select_best_navigation', self.select_navigation)
        self.create_service(SetBool, '/clear_selection', self.clear_selection)
        
        # Timer to publish metadata
        self.create_timer(0.2, self.publish_metadata)  # 5 Hz
        
        self.get_logger().info("🎯 Object Selection Service ready")
    
    def cb_detections(self, msg: Detection2DArray):
        """Process incoming detections and calculate metadata"""
        self.current_detections = msg
        
        # Reset metadata arrays
        self.object_priorities = []
        self.object_categories = []
        self.selection_status = []
        
        for i, detection in enumerate(msg.detections):
            if detection.results:
                class_name = detection.results[0].hypothesis.class_id.lower()
                confidence = detection.results[0].hypothesis.score
                
                # Calculate category and priority
                category = self.mission_categories.get(class_name, 'unknown')
                priority = self._calculate_priority(confidence, category)
                
                self.object_categories.append(category)
                self.object_priorities.append(priority)
                
                # Selection status
                if i == self.selected_object_idx:
                    self.selection_status.append('selected')
                else:
                    self.selection_status.append('available')
    
    def cb_poses(self, msg: PoseArray):
        """Handle 3D poses from pose estimator"""
        self.current_poses = msg
    
    def cb_detections_3d(self, msg: Detection3DArray):
        """Handle ZED 3D detections (preferred when available)"""
        self.current_detections_3d = msg
        
        # Process 3D detections for metadata
        self.object_priorities = []
        self.object_categories = []
        self.selection_status = []
        
        for i, detection_3d in enumerate(msg.detections):
            # Use built-in mission metadata from ZED detector
            category = detection_3d.mission_category
            priority = detection_3d.mission_priority
            
            self.object_categories.append(category)
            self.object_priorities.append(priority)
            
            # Selection status
            if i == self.selected_object_idx:
                self.selection_status.append('selected')
            else:
                self.selection_status.append('available')
        
        self.get_logger().debug(f"Processed {len(msg.detections)} ZED 3D detections")
    
    def cb_tracking_ids(self, msg: Int32MultiArray):
        """Handle tracking IDs from pose estimator"""
        self.current_tracking_ids = msg
    
    def _calculate_priority(self, confidence: float, category: str) -> float:
        """Calculate mission priority"""
        category_weights = {
            'science': 0.9,
            'equipment': 0.8,
            'navigation': 0.7,
            'human': 1.0,
            'obstacle': 0.6,
            'unknown': 0.3
        }
        weight = category_weights.get(category, 0.3)
        return min(1.0, confidence * weight)
    
    def select_science(self, request, response):
        """Select best science object"""
        return self._select_by_category('science', request, response)
    
    def select_equipment(self, request, response):
        """Select best equipment object"""
        return self._select_by_category('equipment', request, response)
    
    def select_navigation(self, request, response):
        """Select best navigation object"""
        return self._select_by_category('navigation', request, response)
    
    def _select_by_category(self, target_category: str, request, response):
        """Select best object of given category"""
        # Prefer ZED 3D detections when available
        if self.use_zed_3d and self.current_detections_3d and self.current_detections_3d.detections:
            return self._select_from_3d_detections(target_category, request, response)
        elif self.current_detections and self.object_categories:
            return self._select_from_2d_detections(target_category, request, response)
        else:
            response.success = False
            response.message = f"No {target_category} objects available"
            return response
    
    def _select_from_3d_detections(self, target_category: str, request, response):
        """Select from ZED 3D detections (preferred method)"""
        # Find best object in category
        best_idx = None
        best_priority = 0.0
        
        for i, detection_3d in enumerate(self.current_detections_3d.detections):
            if detection_3d.mission_category == target_category and detection_3d.mission_priority > best_priority:
                best_idx = i
                best_priority = detection_3d.mission_priority
        
        if best_idx is None:
            response.success = False
            response.message = f"No {target_category} objects found in ZED detections"
            return response
        
        # Select the object
        self.selected_object_idx = best_idx
        detection_3d = self.current_detections_3d.detections[best_idx]
        class_name = detection_3d.detection_2d.results[0].hypothesis.class_id
        confidence = detection_3d.confidence_3d
        
        # Use native 3D pose from ZED
        pose_3d = Pose()
        pose_3d.position = detection_3d.position
        pose_3d.orientation = detection_3d.orientation
        
        self.selected_pose = pose_3d
        self._publish_target_poses(pose_3d, target_category)
        
        # Publish visual markers
        self._publish_selection_markers()
        
        response.success = True
        response.message = f"Selected {class_name} ({target_category}) with 3D confidence {confidence:.2f} at distance {detection_3d.distance:.2f}m"
        
        self.get_logger().info(f"🎯 ZED 3D: {response.message}")
        return response
    
    def _select_from_2d_detections(self, target_category: str, request, response):
        """Fallback to 2D detections with pose estimation"""
        
        # Find best object in category
        best_idx = None
        best_priority = 0.0
        
        for i, (category, priority) in enumerate(zip(self.object_categories, self.object_priorities)):
            if category == target_category and priority > best_priority:
                best_idx = i
                best_priority = priority
        
        if best_idx is None:
            response.success = False
            response.message = f"No {target_category} objects found in 2D detections"
            return response
        
        # Select the object
        self.selected_object_idx = best_idx
        detection = self.current_detections.detections[best_idx]
        class_name = detection.results[0].hypothesis.class_id
        confidence = detection.results[0].hypothesis.score
        
        # Get 3D pose if available from pose estimator
        pose_3d = self._get_3d_pose_for_detection(best_idx)
        if pose_3d:
            self.selected_pose = pose_3d
            self._publish_target_poses(pose_3d, target_category)
            response.success = True
            response.message = f"Selected {class_name} ({target_category}) with 2D→3D confidence {confidence:.2f}"
        else:
            # Fallback to 2D estimation
            self._publish_target_pose_2d(detection)
            response.success = True
            response.message = f"Selected {class_name} ({target_category}) with 2D confidence {confidence:.2f} (2D fallback)"
        
        # Publish visual markers
        self._publish_selection_markers()
        
        self.get_logger().info(f"🎯 2D Fallback: {response.message}")
        return response
    
    def clear_selection(self, request, response):
        """Clear current selection"""
        self.selected_object_idx = None
        self.selected_pose = None
        
        # Clear visual markers
        self._publish_selection_markers()
        
        response.success = True
        response.message = "Selection cleared"
        self.get_logger().info("🔄 Selection cleared")
        return response
    
    def _publish_target_pose(self, detection):
        """Publish simplified target pose"""
        pose_msg = PoseStamped()
        pose_msg.header.stamp = self.get_clock().now().to_msg()
        pose_msg.header.frame_id = 'base_link'  # Will be transformed by pose estimator
        
        # Use detection center as target (simplified)
        pose_msg.pose.position.x = detection.bbox.center.position.x / 1000.0  # Convert to meters
        pose_msg.pose.position.y = detection.bbox.center.position.y / 1000.0
        pose_msg.pose.position.z = 1.0  # Default distance
        
        # Default orientation
        pose_msg.pose.orientation.w = 1.0
        
        self.target_pub.publish(pose_msg)
    
    def _get_3d_pose_for_detection(self, detection_idx: int) -> Optional[Pose]:
        """Get 3D pose for detection from pose estimator"""
        if not self.current_poses or not self.current_poses.poses:
            return None
        
        # Simple matching - assumes detection order matches pose order
        # In a more robust system, would use tracking IDs for proper association
        if detection_idx < len(self.current_poses.poses):
            return self.current_poses.poses[detection_idx]
        
        return None
    
    def _publish_target_poses(self, pose_3d: Pose, category: str):
        """Publish both target pose and approach pose for navigation"""
        current_time = self.get_clock().now().to_msg()
        
        # Direct target pose (object location)
        target_msg = PoseStamped()
        target_msg.header.stamp = current_time
        target_msg.header.frame_id = 'map'
        target_msg.pose = pose_3d
        self.target_pub.publish(target_msg)
        
        # Approach pose (safe distance from object)
        approach_msg = PoseStamped()
        approach_msg.header.stamp = current_time
        approach_msg.header.frame_id = 'map'
        approach_msg.pose = self._calculate_approach_pose(pose_3d, category)
        self.approach_pub.publish(approach_msg)
        
        self.get_logger().info(f"📍 Published target at ({pose_3d.position.x:.2f}, {pose_3d.position.y:.2f}, {pose_3d.position.z:.2f})")
    
    def _publish_target_pose_2d(self, detection):
        """Fallback 2D pose publishing"""
        pose_msg = PoseStamped()
        pose_msg.header.stamp = self.get_clock().now().to_msg()
        pose_msg.header.frame_id = 'base_link'
        
        # Convert pixel coordinates to rough 3D estimate
        pose_msg.pose.position.x = detection.bbox.center.position.x / 1000.0
        pose_msg.pose.position.y = detection.bbox.center.position.y / 1000.0
        pose_msg.pose.position.z = 1.0  # Default distance
        pose_msg.pose.orientation.w = 1.0
        
        self.target_pub.publish(pose_msg)
    
    def _calculate_approach_pose(self, target_pose: Pose, category: str) -> Pose:
        """Calculate safe approach pose for navigation"""
        approach_pose = Pose()
        
        # Get category-specific approach distance
        approach_distances = {
            'science': 0.8,      # Close for sample collection
            'equipment': 1.0,    # Moderate for manipulation
            'navigation': 2.0,   # Can stay at distance
            'human': 3.0,        # Safe distance for humans
            'obstacle': 2.0,     # Avoidance distance
            'unknown': 1.5       # Default safe distance
        }
        
        approach_distance = approach_distances.get(category, 1.5)
        
        # Calculate approach position (offset back along X-axis for simplicity)
        approach_pose.position.x = target_pose.position.x - approach_distance
        approach_pose.position.y = target_pose.position.y
        approach_pose.position.z = target_pose.position.z
        
        # Orient towards the target
        approach_pose.orientation.w = 1.0  # Simplified - should calculate proper orientation
        
        return approach_pose
    
    def _publish_selection_markers(self):
        """Publish RViz markers for visual feedback"""
        marker_array = MarkerArray()
        
        if self.selected_object_idx is not None and self.selected_pose:
            # Target marker
            target_marker = Marker()
            target_marker.header.frame_id = 'map'
            target_marker.header.stamp = self.get_clock().now().to_msg()
            target_marker.ns = 'selected_objects'
            target_marker.id = 0
            target_marker.type = Marker.SPHERE
            target_marker.action = Marker.ADD
            
            target_marker.pose = self.selected_pose
            target_marker.scale.x = 0.3
            target_marker.scale.y = 0.3
            target_marker.scale.z = 0.3
            
            # Color based on category
            if self.selected_object_idx < len(self.object_categories):
                category = self.object_categories[self.selected_object_idx]
                if category == 'science':
                    target_marker.color.r = 0.0
                    target_marker.color.g = 1.0
                    target_marker.color.b = 0.0
                elif category == 'equipment':
                    target_marker.color.r = 1.0
                    target_marker.color.g = 0.5
                    target_marker.color.b = 0.0
                elif category == 'navigation':
                    target_marker.color.r = 0.0
                    target_marker.color.g = 0.0
                    target_marker.color.b = 1.0
                else:
                    target_marker.color.r = 1.0
                    target_marker.color.g = 0.0
                    target_marker.color.b = 1.0
            else:
                target_marker.color.r = 1.0
                target_marker.color.g = 1.0
                target_marker.color.b = 0.0
            
            target_marker.color.a = 0.8
            target_marker.lifetime.sec = 0  # Persistent
            
            marker_array.markers.append(target_marker)
            
            # Text label
            text_marker = Marker()
            text_marker.header = target_marker.header
            text_marker.ns = 'selected_objects'
            text_marker.id = 1
            text_marker.type = Marker.TEXT_VIEW_FACING
            text_marker.action = Marker.ADD
            
            text_marker.pose = self.selected_pose
            text_marker.pose.position.z += 0.5  # Offset above object
            
            text_marker.scale.z = 0.3
            text_marker.color.r = 1.0
            text_marker.color.g = 1.0
            text_marker.color.b = 1.0
            text_marker.color.a = 1.0
            
            if (self.current_detections and 
                self.selected_object_idx < len(self.current_detections.detections)):
                detection = self.current_detections.detections[self.selected_object_idx]
                class_name = detection.results[0].hypothesis.class_id
                confidence = detection.results[0].hypothesis.score
                text_marker.text = f"SELECTED: {class_name} ({confidence:.2f})"
            else:
                text_marker.text = "SELECTED OBJECT"
            
            marker_array.markers.append(text_marker)
        
        # Always publish (empty array clears markers when nothing selected)
        self.markers_pub.publish(marker_array)
    
    def publish_metadata(self):
        """Publish object metadata using standard messages"""
        if not self.current_detections:
            return
        
        # Publish priorities
        priorities_msg = Float32MultiArray()
        priorities_msg.data = self.object_priorities
        self.priorities_pub.publish(priorities_msg)
        
        # Publish categories (as comma-separated string)
        categories_msg = String()
        categories_msg.data = ','.join(self.object_categories)
        self.categories_pub.publish(categories_msg)
        
        # Publish selection status
        status_msg = String()
        status_msg.data = ','.join(self.selection_status)
        self.selection_pub.publish(status_msg)


def main(args=None):
    """Main entry point"""
    rclpy.init(args=args)
    
    try:
        service = ObjectSelectionService()
        rclpy.spin(service)
    except KeyboardInterrupt:
        pass
    finally:
        rclpy.shutdown()


if __name__ == '__main__':
    main()
