#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
from object_detection.msg import Detection3DArray, Detection3D
from vision_msgs.msg import Detection2D, ObjectHypothesisWithPose
from geometry_msgs.msg import Point, Vector3, Quaternion
from std_msgs.msg import Header
import numpy as np
import time
from typing import List, Dict, Optional

# Try to import ZED messages - fallback if not available
try:
    from zed_msgs.msg import ObjectsStamped, Object
    ZED_MSGS_AVAILABLE = True
except ImportError:
    ZED_MSGS_AVAILABLE = False
    print("⚠️ zed_msgs not available. Install zed-ros2-wrapper for ZED integration.")


class ZEDObjectBridge(Node):
    """Bridge between ZED ROS2 wrapper and our object selection system"""
    
    def __init__(self):
        super().__init__('zed_object_bridge')
        
        if not ZED_MSGS_AVAILABLE:
            self.get_logger().error("zed_msgs not available! Install zed-ros2-wrapper package.")
            return
        
        # Parameters
        self.declare_parameter('zed_objects_topic', '/zed2i/obj_det/objects')
        self.declare_parameter('output_topic', '/zed_detections_3d')
        self.declare_parameter('min_confidence', 0.5)
        
        zed_topic = self.get_parameter('zed_objects_topic').value
        output_topic = self.get_parameter('output_topic').value
        self.min_confidence = self.get_parameter('min_confidence').value
        
        # Mission categories mapping (same as before)
        self.mission_categories = {
            'person': 'human', 'human': 'human',
            'bottle': 'science', 'rock': 'science', 'mineral': 'science',
            'toolbox': 'equipment', 'container': 'equipment', 'backpack': 'equipment',
            'bag': 'equipment', 'electronics': 'equipment',
            'flag': 'navigation', 'sign': 'navigation', 'marker': 'navigation',
            'vehicle': 'obstacle', 'obstacle': 'obstacle', 'barrier': 'obstacle',
            'animal': 'obstacle', 'fruit_vegetable': 'science', 'sport': 'equipment'
        }
        
        # ROS2 setup
        self.subscriber = self.create_subscription(
            ObjectsStamped, 
            zed_topic, 
            self.zed_objects_callback, 
            10
        )
        
        self.publisher = self.create_publisher(Detection3DArray, output_topic, 10)
        
        # Performance monitoring
        self.frame_count = 0
        self.last_fps_time = time.time()
        self.create_timer(5.0, self.log_performance)
        
        self.get_logger().info(f"🌉 ZED Object Bridge initialized")
        self.get_logger().info(f"   Subscribing: {zed_topic}")
        self.get_logger().info(f"   Publishing: {output_topic}")
        self.get_logger().info(f"   Min confidence: {self.min_confidence}")
    
    def zed_objects_callback(self, msg: ObjectsStamped):
        """Convert ZED ObjectsStamped to our Detection3DArray format"""
        try:
            detection_msg = self.convert_zed_objects(msg)
            if detection_msg.detections:  # Only publish if we have detections
                self.publisher.publish(detection_msg)
                self.get_logger().debug(f"Published {len(detection_msg.detections)} ZED detections")
            
            self.frame_count += 1
            
        except Exception as e:
            self.get_logger().error(f"ZED object conversion failed: {e}")
    
    def convert_zed_objects(self, zed_msg: ObjectsStamped) -> Detection3DArray:
        """Convert ZED ObjectsStamped to Detection3DArray"""
        msg = Detection3DArray()
        msg.header = zed_msg.header
        
        for zed_obj in zed_msg.objects:
            # Filter by confidence
            if zed_obj.confidence < self.min_confidence:
                continue
            
            detection_3d = self.convert_zed_object(zed_obj)
            if detection_3d:
                msg.detections.append(detection_3d)
        
        return msg
    
    def convert_zed_object(self, zed_obj) -> Optional[Detection3D]:
        """Convert single ZED Object to Detection3D"""
        try:
            detection_3d = Detection3D()
            
            # 2D Detection info
            detection_2d = Detection2D()
            
            # Bounding box from ZED object
            if hasattr(zed_obj, 'bounding_box_2d') and len(zed_obj.bounding_box_2d) >= 4:
                # ZED provides 4 corners of 2D bounding box
                corners = zed_obj.bounding_box_2d
                
                # Calculate center and size
                x_coords = [corner.kp[0] for corner in corners]
                y_coords = [corner.kp[1] for corner in corners]
                
                x_min, x_max = min(x_coords), max(x_coords)
                y_min, y_max = min(y_coords), max(y_coords)
                
                detection_2d.bbox.center.position.x = (x_min + x_max) / 2.0
                detection_2d.bbox.center.position.y = (y_min + y_max) / 2.0
                detection_2d.bbox.size_x = x_max - x_min
                detection_2d.bbox.size_y = y_max - y_min
            
            # Object hypothesis
            hypothesis = ObjectHypothesisWithPose()
            hypothesis.hypothesis.class_id = zed_obj.label
            hypothesis.hypothesis.score = zed_obj.confidence / 100.0  # ZED gives 0-100
            detection_2d.results.append(hypothesis)
            
            detection_3d.detection_2d = detection_2d
            
            # 3D Position
            if hasattr(zed_obj, 'position'):
                detection_3d.position = Point(
                    x=zed_obj.position[0],
                    y=zed_obj.position[1], 
                    z=zed_obj.position[2]
                )
            
            # 3D Dimensions
            if hasattr(zed_obj, 'dimensions'):
                detection_3d.dimensions = Vector3(
                    x=zed_obj.dimensions[0],
                    y=zed_obj.dimensions[1],
                    z=zed_obj.dimensions[2]
                )
            
            # Orientation (ZED may provide this)
            detection_3d.orientation = Quaternion(x=0.0, y=0.0, z=0.0, w=1.0)
            
            # Tracking info
            detection_3d.tracking_id = zed_obj.label_id if hasattr(zed_obj, 'label_id') else -1
            detection_3d.tracking_state = self._get_tracking_state(zed_obj)
            detection_3d.confidence_3d = zed_obj.confidence / 100.0
            
            # Distance
            if hasattr(zed_obj, 'position'):
                detection_3d.distance = np.linalg.norm(zed_obj.position)
            
            # Velocity
            if hasattr(zed_obj, 'velocity'):
                detection_3d.velocity = Vector3(
                    x=zed_obj.velocity[0],
                    y=zed_obj.velocity[1],
                    z=zed_obj.velocity[2]
                )
            else:
                detection_3d.velocity = Vector3(x=0.0, y=0.0, z=0.0)
            
            # Mission metadata
            class_name = zed_obj.label.lower()
            detection_3d.mission_category = self.mission_categories.get(class_name, 'unknown')
            detection_3d.mission_priority = self._calculate_mission_priority(
                detection_3d.confidence_3d, detection_3d.mission_category
            )
            detection_3d.interaction_distance = self._get_interaction_distance(detection_3d.mission_category)
            
            return detection_3d
            
        except Exception as e:
            self.get_logger().error(f"Failed to convert ZED object: {e}")
            return None
    
    def _get_tracking_state(self, zed_obj) -> str:
        """Get tracking state from ZED object"""
        if hasattr(zed_obj, 'tracking_state'):
            # ZED tracking states: OK, SEARCHING, TERMINATE, etc.
            return str(zed_obj.tracking_state)
        return 'UNKNOWN'
    
    def _calculate_mission_priority(self, confidence: float, category: str) -> float:
        """Calculate mission-specific priority"""
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
    
    def _get_interaction_distance(self, category: str) -> float:
        """Get required interaction distance for category"""
        distances = {
            'science': 0.5,      # Close for sample collection
            'equipment': 0.8,    # Moderate for manipulation
            'navigation': 2.0,   # Can navigate from distance
            'human': 3.0,        # Safe distance for humans
            'obstacle': 1.5,     # Avoidance distance
            'unknown': 1.0       # Default safe distance
        }
        return distances.get(category, 1.0)
    
    def log_performance(self):
        """Log bridge performance"""
        current_time = time.time()
        elapsed = current_time - self.last_fps_time
        fps = self.frame_count / elapsed if elapsed > 0 else 0
        
        self.get_logger().info(f"🌉 ZED Bridge FPS: {fps:.2f}")
        
        self.frame_count = 0
        self.last_fps_time = current_time


def main(args=None):
    """Main entry point"""
    rclpy.init(args=args)
    
    try:
        bridge = ZEDObjectBridge()
        rclpy.spin(bridge)
    except KeyboardInterrupt:
        pass
    finally:
        rclpy.shutdown()


if __name__ == '__main__':
    main()
