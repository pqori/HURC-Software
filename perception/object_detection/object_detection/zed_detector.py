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

# ZED SDK imports
try:
    import pyzed.sl as sl
    ZED_AVAILABLE = True
except ImportError:
    ZED_AVAILABLE = False
    print("⚠️ ZED SDK not available. Install pyzed for native detection.")


class ZEDObjectDetector(Node):
    """ZED native object detection with 3D tracking"""
    
    def __init__(self):
        super().__init__('zed_object_detector')
        
        if not ZED_AVAILABLE:
            self.get_logger().error("ZED SDK not available! Install pyzed package.")
            return
        
        # Parameters
        self.declare_parameter('detection_model', 'MULTI_CLASS_BOX_ACCURATE')
        self.declare_parameter('enable_tracking', True)
        self.declare_parameter('enable_segmentation', False)
        self.declare_parameter('detection_confidence', 0.5)
        self.declare_parameter('max_range', 20.0)
        self.declare_parameter('output_topic', '/zed_detections_3d')
        self.declare_parameter('publish_rate', 10.0)  # Hz
        
        # Get parameters
        model_name = self.get_parameter('detection_model').value
        self.enable_tracking = self.get_parameter('enable_tracking').value
        self.enable_segmentation = self.get_parameter('enable_segmentation').value
        self.detection_confidence = self.get_parameter('detection_confidence').value
        self.max_range = self.get_parameter('max_range').value
        output_topic = self.get_parameter('output_topic').value
        publish_rate = self.get_parameter('publish_rate').value
        
        # Mission categories mapping
        self.mission_categories = {
            'person': 'human', 'human': 'human',
            'bottle': 'science', 'rock': 'science', 'mineral': 'science',
            'toolbox': 'equipment', 'container': 'equipment', 'backpack': 'equipment',
            'flag': 'navigation', 'sign': 'navigation', 'marker': 'navigation',
            'obstacle': 'obstacle', 'barrier': 'obstacle', 'vehicle': 'obstacle'
        }
        
        # Initialize ZED Camera
        self.zed = sl.Camera()
        
        # Camera configuration
        init_params = sl.InitParameters()
        init_params.camera_resolution = sl.RESOLUTION.HD720  # Balance speed/quality
        init_params.camera_fps = 30
        init_params.depth_mode = sl.DEPTH_MODE.NEURAL  # AI-enhanced depth
        init_params.coordinate_units = sl.UNIT.METER
        init_params.coordinate_system = sl.COORDINATE_SYSTEM.RIGHT_HANDED_Y_UP
        init_params.depth_maximum_distance = self.max_range
        
        # Open camera
        status = self.zed.open(init_params)
        if status != sl.ERROR_CODE.SUCCESS:
            self.get_logger().error(f"Failed to open ZED camera: {status}")
            return
        
        # Object detection configuration
        obj_param = sl.ObjectDetectionParameters()
        
        # Set detection model
        model_map = {
            'MULTI_CLASS_BOX_ACCURATE': sl.DETECTION_MODEL.MULTI_CLASS_BOX_ACCURATE,
            'MULTI_CLASS_BOX_FAST': sl.DETECTION_MODEL.MULTI_CLASS_BOX_FAST,
            'PERSON_HEAD_BOX_ACCURATE': sl.DETECTION_MODEL.PERSON_HEAD_BOX_ACCURATE,
            'PERSON_HEAD_BOX_FAST': sl.DETECTION_MODEL.PERSON_HEAD_BOX_FAST,
            'CUSTOM_BOX_OBJECTS': sl.DETECTION_MODEL.CUSTOM_BOX_OBJECTS
        }
        obj_param.detection_model = model_map.get(model_name, sl.DETECTION_MODEL.MULTI_CLASS_BOX_ACCURATE)
        
        obj_param.enable_tracking = self.enable_tracking
        obj_param.enable_segmentation = self.enable_segmentation
        obj_param.detection_confidence_threshold = self.detection_confidence
        obj_param.max_range = self.max_range
        
        # Enable object detection
        status = self.zed.enable_object_detection(obj_param)
        if status != sl.ERROR_CODE.SUCCESS:
            self.get_logger().error(f"Failed to enable object detection: {status}")
            return
        
        # ROS2 setup
        self.publisher = self.create_publisher(Detection3DArray, output_topic, 10)
        
        # Detection loop timer
        self.create_timer(1.0 / publish_rate, self.detection_callback)
        
        # Performance monitoring
        self.frame_count = 0
        self.last_fps_time = time.time()
        self.create_timer(5.0, self.log_performance)
        
        self.get_logger().info(f"🎯 ZED Object Detector initialized")
        self.get_logger().info(f"   Model: {model_name}")
        self.get_logger().info(f"   Tracking: {self.enable_tracking}")
        self.get_logger().info(f"   Confidence: {self.detection_confidence}")
        self.get_logger().info(f"   Max Range: {self.max_range}m")
    
    def detection_callback(self):
        """Main detection loop"""
        if not ZED_AVAILABLE:
            return
        
        try:
            # Grab frame
            runtime_params = sl.RuntimeParameters()
            if self.zed.grab(runtime_params) == sl.ERROR_CODE.SUCCESS:
                
                # Retrieve objects
                objects = sl.Objects()
                obj_runtime_param = sl.ObjectDetectionRuntimeParameters()
                obj_runtime_param.detection_confidence_threshold = self.detection_confidence
                
                self.zed.retrieve_objects(objects, obj_runtime_param)
                
                # Convert to ROS message
                if objects.object_list:
                    detection_msg = self.create_detection_message(objects)
                    self.publisher.publish(detection_msg)
                    
                    self.get_logger().debug(f"Published {len(objects.object_list)} 3D detections")
                
                self.frame_count += 1
                
        except Exception as e:
            self.get_logger().error(f"Detection failed: {e}")
    
    def create_detection_message(self, objects: sl.Objects) -> Detection3DArray:
        """Convert ZED objects to ROS Detection3DArray"""
        msg = Detection3DArray()
        msg.header = Header()
        msg.header.stamp = self.get_clock().now().to_msg()
        msg.header.frame_id = 'zed2i_left_camera_frame'  # ZED camera frame
        
        for obj in objects.object_list:
            if obj.tracking_state == sl.OBJECT_TRACKING_STATE.OK or not self.enable_tracking:
                detection_3d = self.convert_zed_object(obj)
                if detection_3d:
                    msg.detections.append(detection_3d)
        
        return msg
    
    def convert_zed_object(self, obj) -> Optional[Detection3D]:
        """Convert single ZED object to Detection3D"""
        try:
            detection_3d = Detection3D()
            
            # 2D Detection info
            detection_2d = Detection2D()
            
            # Bounding box in image coordinates
            bbox_2d = obj.bounding_box_2d
            if len(bbox_2d) >= 2:
                # Calculate center and size from corners
                x_coords = [pt[0] for pt in bbox_2d]
                y_coords = [pt[1] for pt in bbox_2d]
                
                x_min, x_max = min(x_coords), max(x_coords)
                y_min, y_max = min(y_coords), max(y_coords)
                
                detection_2d.bbox.center.position.x = (x_min + x_max) / 2.0
                detection_2d.bbox.center.position.y = (y_min + y_max) / 2.0
                detection_2d.bbox.size_x = x_max - x_min
                detection_2d.bbox.size_y = y_max - y_min
            
            # Object hypothesis
            hypothesis = ObjectHypothesisWithPose()
            hypothesis.hypothesis.class_id = self._get_class_name(obj.label)
            hypothesis.hypothesis.score = obj.confidence / 100.0  # ZED gives 0-100
            detection_2d.results.append(hypothesis)
            
            detection_3d.detection_2d = detection_2d
            
            # 3D Position (center of bounding box)
            position = obj.position
            detection_3d.position = Point(x=position[0], y=position[1], z=position[2])
            
            # 3D Dimensions
            dimensions = obj.dimensions
            detection_3d.dimensions = Vector3(x=dimensions[0], y=dimensions[1], z=dimensions[2])
            
            # Orientation (simplified - ZED doesn't provide full orientation for all objects)
            detection_3d.orientation = Quaternion(x=0.0, y=0.0, z=0.0, w=1.0)
            
            # Tracking info
            detection_3d.tracking_id = obj.id if self.enable_tracking else -1
            detection_3d.tracking_state = self._get_tracking_state(obj.tracking_state)
            detection_3d.confidence_3d = obj.confidence / 100.0
            
            # Distance
            detection_3d.distance = np.linalg.norm(position)
            
            # Velocity (if tracking enabled)
            if self.enable_tracking and hasattr(obj, 'velocity'):
                velocity = obj.velocity
                detection_3d.velocity = Vector3(x=velocity[0], y=velocity[1], z=velocity[2])
            else:
                detection_3d.velocity = Vector3(x=0.0, y=0.0, z=0.0)
            
            # Mission metadata
            class_name = hypothesis.hypothesis.class_id.lower()
            detection_3d.mission_category = self.mission_categories.get(class_name, 'unknown')
            detection_3d.mission_priority = self._calculate_mission_priority(
                detection_3d.confidence_3d, detection_3d.mission_category
            )
            detection_3d.interaction_distance = self._get_interaction_distance(detection_3d.mission_category)
            
            return detection_3d
            
        except Exception as e:
            self.get_logger().error(f"Failed to convert ZED object: {e}")
            return None
    
    def _get_class_name(self, label: sl.OBJECT_CLASS) -> str:
        """Convert ZED object class to string"""
        class_map = {
            sl.OBJECT_CLASS.PERSON: 'person',
            sl.OBJECT_CLASS.VEHICLE: 'vehicle',
            sl.OBJECT_CLASS.BAG: 'backpack',
            sl.OBJECT_CLASS.ANIMAL: 'animal',
            sl.OBJECT_CLASS.ELECTRONICS: 'electronics',
            sl.OBJECT_CLASS.FRUIT_VEGETABLE: 'fruit',
            sl.OBJECT_CLASS.SPORT: 'sports_equipment'
        }
        return class_map.get(label, 'unknown')
    
    def _get_tracking_state(self, state) -> str:
        """Convert ZED tracking state to string"""
        if not self.enable_tracking:
            return 'DISABLED'
        
        state_map = {
            sl.OBJECT_TRACKING_STATE.OFF: 'OFF',
            sl.OBJECT_TRACKING_STATE.OK: 'OK',
            sl.OBJECT_TRACKING_STATE.SEARCHING: 'SEARCHING',
            sl.OBJECT_TRACKING_STATE.TERMINATE: 'TERMINATE'
        }
        return state_map.get(state, 'UNKNOWN')
    
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
        """Log detection performance"""
        current_time = time.time()
        elapsed = current_time - self.last_fps_time
        fps = self.frame_count / elapsed if elapsed > 0 else 0
        
        self.get_logger().info(f"🎯 ZED Detection FPS: {fps:.2f}")
        
        self.frame_count = 0
        self.last_fps_time = current_time
    
    def __del__(self):
        """Cleanup ZED resources"""
        if hasattr(self, 'zed'):
            self.zed.close()


def main(args=None):
    """Main entry point"""
    rclpy.init(args=args)
    
    try:
        detector = ZEDObjectDetector()
        rclpy.spin(detector)
    except KeyboardInterrupt:
        pass
    finally:
        rclpy.shutdown()


if __name__ == '__main__':
    main()
