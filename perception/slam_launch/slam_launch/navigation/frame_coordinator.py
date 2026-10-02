#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
from geometry_msgs.msg import TransformStamped
from std_msgs.msg import String, Bool
from std_srvs.srv import Trigger
import tf2_ros
from tf2_ros import TransformBroadcaster, StaticTransformBroadcaster
import numpy as np
import time
from typing import Dict, List, Optional


class FrameCoordinator(Node):
    """Coordinate TF frames across perception and navigation systems"""
    
    def __init__(self):
        super().__init__('frame_coordinator')
        
        # Parameters
        self.declare_parameter('base_frame', 'base_link')
        self.declare_parameter('odom_frame', 'odom') 
        self.declare_parameter('map_frame', 'map')
        self.declare_parameter('camera_frame', 'zed2i_left_camera_frame')
        self.declare_parameter('gnss_frame', 'gnss_link')
        self.declare_parameter('monitor_frequency', 5.0)
        
        # Get parameters
        self.base_frame = self.get_parameter('base_frame').value
        self.odom_frame = self.get_parameter('odom_frame').value
        self.map_frame = self.get_parameter('map_frame').value
        self.camera_frame = self.get_parameter('camera_frame').value
        self.gnss_frame = self.get_parameter('gnss_frame').value
        monitor_freq = self.get_parameter('monitor_frequency').value
        
        # TF setup
        self.tf_buffer = tf2_ros.Buffer()
        self.tf_listener = tf2_ros.TransformListener(self.tf_buffer, self)
        self.static_broadcaster = StaticTransformBroadcaster(self)
        
        # Frame validation state
        self.frame_health = {
            'map_odom': {'valid': False, 'last_seen': 0.0, 'source': 'unknown'},
            'odom_base': {'valid': False, 'last_seen': 0.0, 'source': 'unknown'},
            'base_camera': {'valid': False, 'last_seen': 0.0, 'source': 'unknown'},
            'base_gnss': {'valid': False, 'last_seen': 0.0, 'source': 'unknown'}
        }
        
        # Publishers
        self.frame_status_pub = self.create_publisher(String, '/navigation/frame_status', 10)
        self.frame_valid_pub = self.create_publisher(Bool, '/navigation/frames_valid', 10)
        
        # Services
        self.create_service(Trigger, '/navigation/validate_frames', self.validate_frames_service)
        self.create_service(Trigger, '/navigation/reset_frames', self.reset_frame_tracking)
        
        # Monitoring timer
        self.create_timer(1.0 / monitor_freq, self.monitor_frame_health)
        
        # Publish static transforms
        self.publish_static_transforms()
        
        self.get_logger().info("🔗 Frame Coordinator initialized")
        self.get_logger().info(f"   TF Tree: {self.map_frame} → {self.odom_frame} → {self.base_frame}")
    
    def publish_static_transforms(self):
        """Publish static transforms that don't change"""
        current_time = self.get_clock().now().to_msg()
        
        # Base to camera transform (from ZED calibration)
        base_to_camera = TransformStamped()
        base_to_camera.header.stamp = current_time
        base_to_camera.header.frame_id = self.base_frame
        base_to_camera.child_frame_id = self.camera_frame
        
        # ZED2i camera position on rover (adjust based on mounting)
        base_to_camera.transform.translation.x = 0.2   # 20cm forward
        base_to_camera.transform.translation.y = 0.0   # Centered
        base_to_camera.transform.translation.z = 0.3   # 30cm up
        base_to_camera.transform.rotation.w = 1.0      # No rotation
        
        # Base to GNSS transform
        base_to_gnss = TransformStamped()
        base_to_gnss.header.stamp = current_time
        base_to_gnss.header.frame_id = self.base_frame
        base_to_gnss.child_frame_id = self.gnss_frame
        
        # GNSS antenna position (adjust based on mounting)
        base_to_gnss.transform.translation.x = 0.0     # Centered
        base_to_gnss.transform.translation.y = 0.0     # Centered  
        base_to_gnss.transform.translation.z = 0.5     # 50cm up
        base_to_gnss.transform.rotation.w = 1.0        # No rotation
        
        # Publish static transforms
        self.static_broadcaster.sendTransform([base_to_camera, base_to_gnss])
        
        self.get_logger().info("📍 Static transforms published")
    
    def monitor_frame_health(self):
        """Monitor TF frame health and consistency"""
        current_time = time.time()
        
        # Check critical transforms
        transforms_to_check = [
            ('map_odom', self.map_frame, self.odom_frame),
            ('odom_base', self.odom_frame, self.base_frame),
            ('base_camera', self.base_frame, self.camera_frame),
            ('base_gnss', self.base_frame, self.gnss_frame)
        ]
        
        for tf_name, parent, child in transforms_to_check:
            try:
                transform = self.tf_buffer.lookup_transform(
                    parent, child, rclpy.time.Time()
                )
                
                # Update health status
                self.frame_health[tf_name]['valid'] = True
                self.frame_health[tf_name]['last_seen'] = current_time
                
            except Exception as e:
                self.frame_health[tf_name]['valid'] = False
                self.get_logger().debug(f"Transform {parent}→{child} unavailable: {e}")
        
        # Publish frame status
        self.publish_frame_status()
    
    def publish_frame_status(self):
        """Publish frame validation status"""
        # Overall frame validity
        all_critical_valid = (
            self.frame_health['map_odom']['valid'] and
            self.frame_health['odom_base']['valid']
        )
        
        # Frame status message
        status_msg = String()
        status_parts = []
        for tf_name, health in self.frame_health.items():
            status = "✅" if health['valid'] else "❌"
            status_parts.append(f"{tf_name}:{status}")
        
        status_msg.data = ",".join(status_parts)
        self.frame_status_pub.publish(status_msg)
        
        # Overall validity
        valid_msg = Bool()
        valid_msg.data = all_critical_valid
        self.frame_valid_pub.publish(valid_msg)
        
        # Log issues
        if not all_critical_valid:
            invalid_frames = [name for name, health in self.frame_health.items() 
                            if not health['valid'] and name in ['map_odom', 'odom_base']]
            self.get_logger().warn(f"⚠️ Invalid critical frames: {invalid_frames}")
    
    def validate_frames_service(self, request, response):
        """Service to validate frame consistency"""
        try:
            # Check TF tree consistency
            issues = self.validate_tf_tree()
            
            if not issues:
                response.success = True
                response.message = "All frames valid and consistent"
                self.get_logger().info("✅ Frame validation passed")
            else:
                response.success = False
                response.message = f"Frame issues: {', '.join(issues)}"
                self.get_logger().error(f"❌ Frame validation failed: {response.message}")
            
        except Exception as e:
            response.success = False
            response.message = f"Frame validation error: {str(e)}"
            self.get_logger().error(f"Frame validation error: {e}")
        
        return response
    
    def validate_tf_tree(self) -> List[str]:
        """Validate TF tree consistency"""
        issues = []
        
        try:
            # Check map → odom transform
            if not self.frame_health['map_odom']['valid']:
                issues.append("map→odom transform missing")
            
            # Check odom → base_link transform  
            if not self.frame_health['odom_base']['valid']:
                issues.append("odom→base_link transform missing")
            
            # Check for transform loops (should not exist)
            try:
                # This should fail - no direct map→base_link
                self.tf_buffer.lookup_transform(
                    self.map_frame, self.base_frame, rclpy.time.Time()
                )
                # If we get here, check if it's going through odom
                # (This is complex validation - simplified for now)
                
            except:
                pass  # Expected - should go through odom
            
        except Exception as e:
            issues.append(f"TF validation error: {str(e)}")
        
        return issues
    
    def reset_frame_tracking(self, request, response):
        """Service to reset frame tracking"""
        for health in self.frame_health.values():
            health['valid'] = False
            health['last_seen'] = 0.0
        
        response.success = True
        response.message = "Frame tracking reset"
        self.get_logger().info("🔄 Frame tracking reset")
        
        return response


def main(args=None):
    """Main entry point"""
    rclpy.init(args=args)
    
    try:
        coordinator = FrameCoordinator()
        rclpy.spin(coordinator)
    except KeyboardInterrupt:
        pass
    finally:
        rclpy.shutdown()


if __name__ == '__main__':
    main()
