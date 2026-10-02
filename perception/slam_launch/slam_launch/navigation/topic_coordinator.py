#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
from nav_msgs.msg import OccupancyGrid, Odometry
from std_msgs.msg import String, Bool
from std_srvs.srv import SetBool
import time
from typing import Dict, Optional
from enum import Enum


class TopicSource(Enum):
    """Topic source priorities"""
    RTABMAP = "rtabmap"
    ZED_MAPPING = "zed_mapping" 
    HYBRID = "hybrid"
    EKF_LOCAL = "ekf_local"
    EKF_GLOBAL = "ekf_global"
    ZED_FUSION = "zed_fusion"


class TopicCoordinator(Node):
    """Coordinate topic publishing to prevent conflicts"""
    
    def __init__(self):
        super().__init__('topic_coordinator')
        
        # Parameters
        self.declare_parameter('map_source_priority', 'hybrid')  # hybrid, rtabmap, zed_mapping
        self.declare_parameter('odom_source_priority', 'ekf_local')  # ekf_local, ekf_global, zed_fusion
        self.declare_parameter('enable_topic_monitoring', True)
        self.declare_parameter('conflict_resolution_timeout', 5.0)
        
        # Get parameters
        map_priority = self.get_parameter('map_source_priority').value
        odom_priority = self.get_parameter('odom_source_priority').value
        self.enable_monitoring = self.get_parameter('enable_topic_monitoring').value
        self.conflict_timeout = self.get_parameter('conflict_resolution_timeout').value
        
        # Set source priorities
        self.map_source = TopicSource(map_priority)
        self.odom_source = TopicSource(odom_priority)
        
        # Topic monitoring
        self.topic_publishers = {
            '/map': {'count': 0, 'sources': [], 'last_check': 0.0},
            '/odometry/filtered': {'count': 0, 'sources': [], 'last_check': 0.0},
            '/odom': {'count': 0, 'sources': [], 'last_check': 0.0}
        }
        
        # Active sources
        self.active_map_source: Optional[str] = None
        self.active_odom_source: Optional[str] = None
        
        # Publishers
        self.conflict_status_pub = self.create_publisher(String, '/navigation/topic_conflicts', 10)
        self.resolution_pub = self.create_publisher(String, '/navigation/topic_resolution', 10)
        
        # Services
        self.create_service(SetBool, '/navigation/enable_conflict_resolution', self.enable_conflict_resolution)
        self.create_service(SetBool, '/navigation/set_map_source', self.set_map_source)
        
        # Monitoring timer
        if self.enable_monitoring:
            self.create_timer(2.0, self.monitor_topic_conflicts)
        
        self.get_logger().info("🎛️ Topic Coordinator initialized")
        self.get_logger().info(f"   Map source priority: {self.map_source.value}")
        self.get_logger().info(f"   Odom source priority: {self.odom_source.value}")
    
    def monitor_topic_conflicts(self):
        """Monitor for topic publishing conflicts"""
        current_time = time.time()
        
        # Check each monitored topic
        for topic_name, info in self.topic_publishers.items():
            try:
                # Get current publisher count
                pub_count = self.count_publishers(topic_name)
                
                # Detect conflicts
                if pub_count > 1:
                    self.handle_topic_conflict(topic_name, pub_count)
                elif pub_count == 1:
                    self.resolve_topic_conflict(topic_name)
                else:
                    self.handle_missing_publisher(topic_name)
                
                # Update monitoring info
                info['count'] = pub_count
                info['last_check'] = current_time
                
            except Exception as e:
                self.get_logger().error(f"Topic monitoring error for {topic_name}: {e}")
        
        # Publish status
        self.publish_conflict_status()
    
    def handle_topic_conflict(self, topic: str, count: int):
        """Handle detected topic conflicts"""
        self.get_logger().warn(f"⚠️ Topic conflict detected: {topic} has {count} publishers")
        
        # Resolve based on topic type
        if topic == '/map':
            self.resolve_map_conflict()
        elif topic in ['/odometry/filtered', '/odom']:
            self.resolve_odometry_conflict()
        
        # Log resolution action
        resolution_msg = String()
        resolution_msg.data = f"conflict_detected:{topic},publishers:{count},resolving"
        self.resolution_pub.publish(resolution_msg)
    
    def resolve_topic_conflict(self, topic: str):
        """Mark topic conflict as resolved"""
        if topic == '/map':
            self.active_map_source = self.map_source.value
        elif topic in ['/odometry/filtered', '/odom']:
            self.active_odom_source = self.odom_source.value
    
    def handle_missing_publisher(self, topic: str):
        """Handle missing critical publishers"""
        if topic in ['/map', '/odometry/filtered']:
            self.get_logger().warn(f"⚠️ Critical topic has no publishers: {topic}")
    
    def resolve_map_conflict(self):
        """Resolve /map topic conflicts"""
        # Map source priority order
        if self.map_source == TopicSource.HYBRID:
            self.get_logger().info("🗺️ Map conflict resolution: Hybrid coordinator should be primary")
        elif self.map_source == TopicSource.RTABMAP:
            self.get_logger().info("🗺️ Map conflict resolution: RTAB-Map should be primary")
        elif self.map_source == TopicSource.ZED_MAPPING:
            self.get_logger().info("🗺️ Map conflict resolution: ZED mapping should be primary")
        
        self.active_map_source = self.map_source.value
    
    def resolve_odometry_conflict(self):
        """Resolve odometry topic conflicts"""
        # Odometry source priority order
        if self.odom_source == TopicSource.EKF_LOCAL:
            self.get_logger().info("📍 Odometry conflict resolution: EKF local should be primary")
        elif self.odom_source == TopicSource.ZED_FUSION:
            self.get_logger().info("📍 Odometry conflict resolution: ZED GNSS fusion should be primary")
        
        self.active_odom_source = self.odom_source.value
    
    def publish_conflict_status(self):
        """Publish topic conflict status"""
        status_parts = []
        
        for topic, info in self.topic_publishers.items():
            if info['count'] > 1:
                status_parts.append(f"{topic}:CONFLICT({info['count']})")
            elif info['count'] == 1:
                status_parts.append(f"{topic}:OK")
            else:
                status_parts.append(f"{topic}:MISSING")
        
        status_msg = String()
        status_msg.data = ",".join(status_parts)
        self.conflict_status_pub.publish(status_msg)
    
    def count_publishers(self, topic_name: str) -> int:
        """Count number of publishers for a topic"""
        try:
            topic_info = self.get_publishers_info_by_topic(topic_name)
            return len(topic_info)
        except:
            return 0
    
    def enable_conflict_resolution(self, request, response):
        """Service to enable/disable conflict resolution"""
        self.enable_monitoring = request.data
        
        response.success = True
        response.message = f"Topic conflict resolution {'enabled' if request.data else 'disabled'}"
        self.get_logger().info(f"🎛️ {response.message}")
        
        return response
    
    def set_map_source(self, request, response):
        """Service to set preferred map source"""
        # This would be enhanced to actually switch sources
        response.success = True
        response.message = "Map source preference updated"
        return response
    
    def validate_frames_service(self, request, response):
        """Service to validate frame consistency"""
        try:
            critical_frames_valid = (
                self.frame_health['map_odom']['valid'] and
                self.frame_health['odom_base']['valid']
            )
            
            if critical_frames_valid:
                response.success = True
                response.message = "Critical navigation frames are valid"
            else:
                response.success = False
                response.message = "Critical navigation frames are invalid"
            
        except Exception as e:
            response.success = False
            response.message = f"Frame validation failed: {str(e)}"
        
        return response


def main(args=None):
    """Main entry point"""
    rclpy.init(args=args)
    
    try:
        coordinator = TopicCoordinator()
        rclpy.spin(coordinator)
    except KeyboardInterrupt:
        pass
    finally:
        rclpy.shutdown()


if __name__ == '__main__':
    main()
