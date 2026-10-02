#!/usr/bin/env python3
"""
Semantic Mapping Integration
===========================

Integrates object detections with SLAM map to create semantic maps.
Maintains persistent object locations and provides semantic queries.

Features:
- Object persistence in map coordinates
- Semantic map publishing
- Object location queries
- Integration with RTAB-Map
- Mission-specific object categorization

Author: URC Perception Team
License: Apache-2.0
"""

import rclpy
from rclpy.node import Node
from geometry_msgs.msg import PoseArray, Pose, Point
from std_msgs.msg import Int32MultiArray, Header, String
from nav_msgs.msg import OccupancyGrid
from visualization_msgs.msg import MarkerArray, Marker
from vision_msgs.msg import Detection2DArray
import tf2_ros
import numpy as np
from typing import Dict, List, Tuple, Optional
import json
import time
import math
from collections import defaultdict


class SemanticObject:
    """Represents a semantic object in the map"""
    
    def __init__(self, obj_id: int, class_name: str, pose: Pose, confidence: float, timestamp: float):
        self.id = obj_id
        self.class_name = class_name
        self.pose = pose
        self.confidence = confidence
        self.first_seen = timestamp
        self.last_updated = timestamp
        self.observation_count = 1
        self.confidence_history = [confidence]
        
        # Mission-specific categorization
        self.mission_category = self._categorize_for_mission(class_name)
        
    def update(self, new_pose: Pose, confidence: float, timestamp: float):
        """Update object with new observation"""
        # Weighted average of poses (more recent observations have higher weight)
        alpha = 0.3  # Learning rate
        
        self.pose.position.x = (1 - alpha) * self.pose.position.x + alpha * new_pose.position.x
        self.pose.position.y = (1 - alpha) * self.pose.position.y + alpha * new_pose.position.y
        self.pose.position.z = (1 - alpha) * self.pose.position.z + alpha * new_pose.position.z
        
        # Update confidence (weighted average)
        self.confidence = (1 - alpha) * self.confidence + alpha * confidence
        self.confidence_history.append(confidence)
        if len(self.confidence_history) > 20:
            self.confidence_history.pop(0)
        
        self.last_updated = timestamp
        self.observation_count += 1
    
    def _categorize_for_mission(self, class_name: str) -> str:
        """Categorize object for URC mission tasks"""
        science_objects = {'rock', 'mineral_sample', 'biological_sample', 'soil_sample', 'plant', 'lichen', 'moss'}
        equipment_objects = {'bottle', 'toolbox', 'container', 'drill', 'hammer', 'wrench', 'cable', 'rope', 'battery', 'solar_panel'}
        navigation_objects = {'flag', 'post', 'sign', 'marker', 'beacon', 'gate', 'checkpoint'}
        obstacle_objects = {'large_rock', 'boulder', 'equipment_cache', 'habitat_module', 'rover', 'antenna', 'obstacle'}
        human_objects = {'person', 'astronaut', 'scientist', 'backpack', 'helmet'}
        
        if class_name in science_objects:
            return 'science'
        elif class_name in equipment_objects:
            return 'equipment'
        elif class_name in navigation_objects:
            return 'navigation'
        elif class_name in obstacle_objects:
            return 'obstacle'
        elif class_name in human_objects:
            return 'human'
        else:
            return 'unknown'
    
    def get_avg_confidence(self) -> float:
        """Get average confidence over all observations"""
        return np.mean(self.confidence_history)
    
    def is_reliable(self, min_observations: int = 3, min_confidence: float = 0.6) -> bool:
        """Check if object is reliable enough for mission use"""
        return (self.observation_count >= min_observations and 
                self.get_avg_confidence() >= min_confidence)
    
    def to_dict(self) -> Dict:
        """Convert to dictionary for serialization"""
        return {
            'id': self.id,
            'class_name': self.class_name,
            'mission_category': self.mission_category,
            'pose': {
                'x': self.pose.position.x,
                'y': self.pose.position.y,
                'z': self.pose.position.z
            },
            'confidence': self.confidence,
            'avg_confidence': self.get_avg_confidence(),
            'observation_count': self.observation_count,
            'first_seen': self.first_seen,
            'last_updated': self.last_updated,
            'reliable': self.is_reliable()
        }


class SemanticMapper(Node):
    """Semantic mapping node that integrates objects with SLAM map"""
    
    def __init__(self):
        super().__init__('semantic_mapper')
        
        # Parameters
        self.declare_parameter('poses_topic', '/object_pose_array')
        self.declare_parameter('tracking_ids_topic', '/object_tracking_ids')
        self.declare_parameter('detections_topic', '/detected_objects')
        self.declare_parameter('map_topic', '/map')
        self.declare_parameter('semantic_map_topic', '/semantic_map')
        self.declare_parameter('semantic_markers_topic', '/semantic_markers')
        self.declare_parameter('semantic_query_topic', '/semantic_query')
        self.declare_parameter('frame_id', 'map')
        self.declare_parameter('merge_distance_threshold', 0.5)  # Merge objects within this distance
        self.declare_parameter('min_confidence_for_mapping', 0.6)
        self.declare_parameter('map_update_rate', 2.0)
        
        # Get parameters
        poses_topic = self.get_parameter('poses_topic').value
        ids_topic = self.get_parameter('tracking_ids_topic').value
        det_topic = self.get_parameter('detections_topic').value
        map_topic = self.get_parameter('map_topic').value
        sem_map_topic = self.get_parameter('semantic_map_topic').value
        markers_topic = self.get_parameter('semantic_markers_topic').value
        query_topic = self.get_parameter('semantic_query_topic').value
        self.frame_id = self.get_parameter('frame_id').value
        self.merge_threshold = self.get_parameter('merge_distance_threshold').value
        self.min_confidence = self.get_parameter('min_confidence_for_mapping').value
        update_rate = self.get_parameter('map_update_rate').value
        
        # Subscribers
        self.create_subscription(PoseArray, poses_topic, self.cb_poses, 10)
        self.create_subscription(Int32MultiArray, ids_topic, self.cb_tracking_ids, 10)
        self.create_subscription(Detection2DArray, det_topic, self.cb_detections, 10)
        self.create_subscription(OccupancyGrid, map_topic, self.cb_map, 1)
        
        # Publishers
        self.semantic_map_pub = self.create_publisher(String, sem_map_topic, 1)
        self.markers_pub = self.create_publisher(MarkerArray, markers_topic, 1)
        
        # State
        self.semantic_objects: Dict[int, SemanticObject] = {}
        self.latest_poses = None
        self.latest_tracking_ids = None
        self.latest_detections = None
        self.occupancy_map = None
        
        # Timers
        self.create_timer(1.0 / update_rate, self.update_semantic_map)
        
        self.get_logger().info("🗺️ Semantic Mapper initialized")
    
    def cb_poses(self, msg: PoseArray):
        """Handle tracked object poses"""
        self.latest_poses = msg.poses
        self.process_semantic_update()
    
    def cb_tracking_ids(self, msg: Int32MultiArray):
        """Handle tracking IDs"""
        self.latest_tracking_ids = msg.data
        self.process_semantic_update()
    
    def cb_detections(self, msg: Detection2DArray):
        """Handle raw detections for confidence information"""
        self.latest_detections = msg.detections
    
    def cb_map(self, msg: OccupancyGrid):
        """Handle SLAM occupancy grid"""
        self.occupancy_map = msg
    
    def process_semantic_update(self):
        """Process new tracking data and update semantic map"""
        if not self.latest_poses or not self.latest_tracking_ids:
            return
        
        if len(self.latest_poses) != len(self.latest_tracking_ids):
            self.get_logger().warn("Pose and ID arrays length mismatch")
            return
        
        current_time = time.time()
        
        # Process each tracked object
        for i, (pose, track_id) in enumerate(zip(self.latest_poses, self.latest_tracking_ids)):
            # Get detection confidence if available
            confidence = 0.8  # Default confidence
            class_name = "unknown"
            
            if self.latest_detections and i < len(self.latest_detections):
                det = self.latest_detections[i]
                if det.results:
                    confidence = det.results[0].score
                    class_name = det.results[0].hypothesis.class_id
            
            # Skip low-confidence detections
            if confidence < self.min_confidence:
                continue
            
            # Check if we should merge with existing object
            merged_id = self.find_merge_candidate(pose, class_name)
            
            if merged_id is not None:
                # Update existing object
                self.semantic_objects[merged_id].update(pose, confidence, current_time)
            else:
                # Create new semantic object or update existing one
                if track_id in self.semantic_objects:
                    self.semantic_objects[track_id].update(pose, confidence, current_time)
                else:
                    self.semantic_objects[track_id] = SemanticObject(
                        track_id, class_name, pose, confidence, current_time
                    )
                    self.get_logger().info(f"🏷️ New semantic object: {class_name} at ({pose.position.x:.2f}, {pose.position.y:.2f})")
    
    def find_merge_candidate(self, pose: Pose, class_name: str) -> Optional[int]:
        """Find existing object that should be merged with new detection"""
        for obj_id, obj in self.semantic_objects.items():
            if obj.class_name == class_name:
                distance = math.sqrt(
                    (pose.position.x - obj.pose.position.x) ** 2 +
                    (pose.position.y - obj.pose.position.y) ** 2 +
                    (pose.position.z - obj.pose.position.z) ** 2
                )
                
                if distance < self.merge_threshold:
                    return obj_id
        
        return None
    
    def update_semantic_map(self):
        """Publish updated semantic map and visualizations"""
        # Create semantic map data
        semantic_data = {
            'timestamp': time.time(),
            'frame_id': self.frame_id,
            'objects': [obj.to_dict() for obj in self.semantic_objects.values()],
            'statistics': self.get_map_statistics()
        }
        
        # Publish semantic map
        map_msg = String()
        map_msg.data = json.dumps(semantic_data, indent=2)
        self.semantic_map_pub.publish(map_msg)
        
        # Publish visualization markers
        self.publish_visualization_markers()
    
    def get_map_statistics(self) -> Dict:
        """Get statistics about the semantic map"""
        stats = {
            'total_objects': len(self.semantic_objects),
            'reliable_objects': sum(1 for obj in self.semantic_objects.values() if obj.is_reliable()),
            'by_category': defaultdict(int),
            'by_class': defaultdict(int)
        }
        
        for obj in self.semantic_objects.values():
            stats['by_category'][obj.mission_category] += 1
            stats['by_class'][obj.class_name] += 1
        
        return dict(stats)
    
    def publish_visualization_markers(self):
        """Publish RViz markers for semantic objects"""
        marker_array = MarkerArray()
        
        for i, obj in enumerate(self.semantic_objects.values()):
            if not obj.is_reliable():
                continue
            
            # Object marker
            marker = Marker()
            marker.header.frame_id = self.frame_id
            marker.header.stamp = self.get_clock().now().to_msg()
            marker.ns = "semantic_objects"
            marker.id = obj.id
            marker.type = Marker.SPHERE
            marker.action = Marker.ADD
            
            marker.pose = obj.pose
            marker.scale.x = 0.3
            marker.scale.y = 0.3
            marker.scale.z = 0.3
            
            # Color by mission category
            if obj.mission_category == 'science':
                marker.color.r, marker.color.g, marker.color.b = 0.0, 1.0, 0.0  # Green
            elif obj.mission_category == 'equipment':
                marker.color.r, marker.color.g, marker.color.b = 0.0, 0.0, 1.0  # Blue
            elif obj.mission_category == 'navigation':
                marker.color.r, marker.color.g, marker.color.b = 1.0, 1.0, 0.0  # Yellow
            elif obj.mission_category == 'obstacle':
                marker.color.r, marker.color.g, marker.color.b = 1.0, 0.0, 0.0  # Red
            elif obj.mission_category == 'human':
                marker.color.r, marker.color.g, marker.color.b = 1.0, 0.0, 1.0  # Magenta
            else:
                marker.color.r, marker.color.g, marker.color.b = 0.5, 0.5, 0.5  # Gray
            
            marker.color.a = min(1.0, obj.get_avg_confidence())
            marker.lifetime.sec = 10
            
            marker_array.markers.append(marker)
            
            # Text label
            text_marker = Marker()
            text_marker.header = marker.header
            text_marker.ns = "semantic_labels"
            text_marker.id = obj.id + 10000
            text_marker.type = Marker.TEXT_VIEW_FACING
            text_marker.action = Marker.ADD
            
            text_marker.pose = obj.pose
            text_marker.pose.position.z += 0.5  # Offset above object
            
            text_marker.scale.z = 0.3
            text_marker.color.r = text_marker.color.g = text_marker.color.b = 1.0
            text_marker.color.a = 1.0
            text_marker.text = f"{obj.class_name}\nID:{obj.id}\nConf:{obj.get_avg_confidence():.2f}"
            text_marker.lifetime.sec = 10
            
            marker_array.markers.append(text_marker)
        
        self.markers_pub.publish(marker_array)
        
        if self.semantic_objects:
            reliable_count = sum(1 for obj in self.semantic_objects.values() if obj.is_reliable())
            self.get_logger().debug(f"🗺️ Published {len(self.semantic_objects)} semantic objects ({reliable_count} reliable)")


def main(args=None):
    """Main entry point"""
    rclpy.init(args=args)
    
    try:
        node = SemanticMapper()
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        rclpy.shutdown()


if __name__ == '__main__':
    main()
