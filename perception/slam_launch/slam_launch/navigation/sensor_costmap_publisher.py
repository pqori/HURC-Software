#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
from sensor_msgs.msg import PointCloud2, LaserScan
from nav_msgs.msg import OccupancyGrid
from geometry_msgs.msg import PointStamped
from std_msgs.msg import String, Float32, Header
import sensor_msgs_py.point_cloud2 as pc2
import numpy as np
import math
from typing import List, Optional


class SensorCostmapPublisher(Node):
    """Publish high-rate sensor data for Nav2 costmap integration"""
    
    def __init__(self):
        super().__init__('sensor_costmap_publisher')
        
        # Parameters
        self.declare_parameter('zed_cloud_topic', '/zed2i/point_cloud/cloud_registered')
        self.declare_parameter('output_scan_topic', '/scan')
        self.declare_parameter('output_cloud_topic', '/costmap_cloud')
        self.declare_parameter('scan_height_min', 0.1)
        self.declare_parameter('scan_height_max', 2.0)
        self.declare_parameter('scan_range_max', 10.0)
        self.declare_parameter('scan_angle_min', -3.14159)
        self.declare_parameter('scan_angle_max', 3.14159)
        self.declare_parameter('scan_resolution', 0.5)  # degrees
        self.declare_parameter('publish_rate', 15.0)
        
        # Get parameters
        zed_topic = self.get_parameter('zed_cloud_topic').value
        self.scan_topic = self.get_parameter('output_scan_topic').value
        self.cloud_topic = self.get_parameter('output_cloud_topic').value
        self.height_min = self.get_parameter('scan_height_min').value
        self.height_max = self.get_parameter('scan_height_max').value
        self.range_max = self.get_parameter('scan_range_max').value
        self.angle_min = self.get_parameter('scan_angle_min').value
        self.angle_max = self.get_parameter('scan_angle_max').value
        self.angle_res = math.radians(self.get_parameter('scan_resolution').value)
        self.publish_rate = self.get_parameter('publish_rate').value
        
        # Calculate scan parameters
        self.angle_count = int((self.angle_max - self.angle_min) / self.angle_res)
        
        # State
        self.latest_cloud: Optional[PointCloud2] = None
        
        # Subscribers
        self.create_subscription(PointCloud2, zed_topic, self.cb_point_cloud, 5)
        
        # Publishers
        self.scan_pub = self.create_publisher(LaserScan, self.scan_topic, 10)
        self.filtered_cloud_pub = self.create_publisher(PointCloud2, self.cloud_topic, 5)
        self.performance_pub = self.create_publisher(String, '/sensor_integration/performance', 10)
        
        # Processing timer
        self.create_timer(1.0 / self.publish_rate, self.process_and_publish)
        
        # Performance monitoring
        self.frame_count = 0
        self.create_timer(5.0, self.log_performance)
        
        self.get_logger().info("📡 Sensor Costmap Publisher initialized")
        self.get_logger().info(f"   Input: {zed_topic}")
        self.get_logger().info(f"   Scan output: {self.scan_topic} ({self.publish_rate} Hz)")
        self.get_logger().info(f"   Cloud output: {self.cloud_topic}")
    
    def cb_point_cloud(self, msg: PointCloud2):
        """Store latest point cloud"""
        self.latest_cloud = msg
    
    def process_and_publish(self):
        """Convert point cloud to laser scan and filtered cloud"""
        if not self.latest_cloud:
            return
        
        try:
            # Convert to laser scan for Nav2
            scan_msg = self.cloud_to_scan(self.latest_cloud)
            if scan_msg:
                self.scan_pub.publish(scan_msg)
            
            # Publish filtered cloud for costmap
            filtered_cloud = self.filter_cloud_for_costmap(self.latest_cloud)
            if filtered_cloud:
                self.filtered_cloud_pub.publish(filtered_cloud)
            
            self.frame_count += 1
            
        except Exception as e:
            self.get_logger().error(f"Sensor processing failed: {e}")
    
    def cloud_to_scan(self, cloud_msg: PointCloud2) -> Optional[LaserScan]:
        """Convert 3D point cloud to 2D laser scan"""
        try:
            scan = LaserScan()
            scan.header = cloud_msg.header
            scan.header.frame_id = 'base_link'  # Nav2 expects base_link frame
            
            # Scan parameters
            scan.angle_min = self.angle_min
            scan.angle_max = self.angle_max
            scan.angle_increment = self.angle_res
            scan.range_min = 0.1
            scan.range_max = self.range_max
            
            # Initialize ranges
            ranges = [float('inf')] * self.angle_count
            
            # Process points
            for point in pc2.read_points(cloud_msg, skip_nans=True, field_names=("x", "y", "z")):
                x, y, z = point[:3]
                
                # Filter by height
                if not (self.height_min <= z <= self.height_max):
                    continue
                
                # Calculate range and angle
                range_val = math.sqrt(x*x + y*y)
                if range_val > self.range_max:
                    continue
                
                angle = math.atan2(y, x)
                if not (self.angle_min <= angle <= self.angle_max):
                    continue
                
                # Find corresponding angle bin
                angle_idx = int((angle - self.angle_min) / self.angle_res)
                if 0 <= angle_idx < self.angle_count:
                    # Keep closest point in each angular bin
                    if range_val < ranges[angle_idx]:
                        ranges[angle_idx] = range_val
            
            # Convert inf to max range
            scan.ranges = [self.range_max if r == float('inf') else r for r in ranges]
            
            return scan
            
        except Exception as e:
            self.get_logger().error(f"Cloud to scan conversion failed: {e}")
            return None
    
    def filter_cloud_for_costmap(self, cloud_msg: PointCloud2) -> Optional[PointCloud2]:
        """Filter point cloud for costmap integration"""
        try:
            # Create filtered cloud with same header
            filtered_cloud = PointCloud2()
            filtered_cloud.header = cloud_msg.header
            filtered_cloud.header.frame_id = 'base_link'
            
            # Filter points for costmap use
            filtered_points = []
            for point in pc2.read_points(cloud_msg, skip_nans=True, field_names=("x", "y", "z")):
                x, y, z = point[:3]
                
                # Filter by height and range
                if (self.height_min <= z <= self.height_max and 
                    math.sqrt(x*x + y*y) <= self.range_max):
                    filtered_points.append([x, y, z])
            
            # Create filtered point cloud
            if filtered_points:
                filtered_cloud = pc2.create_cloud_xyz32(filtered_cloud.header, filtered_points)
                return filtered_cloud
            
            return None
            
        except Exception as e:
            self.get_logger().error(f"Cloud filtering failed: {e}")
            return None
    
    def log_performance(self):
        """Log sensor integration performance"""
        fps = self.frame_count / 5.0
        
        # Publish performance metrics
        perf_msg = String()
        perf_msg.data = f"fps:{fps:.1f},scan_rate:{fps:.1f},cloud_rate:{fps:.1f}"
        self.performance_pub.publish(perf_msg)
        
        self.get_logger().info(f"📡 Sensor Integration FPS: {fps:.1f}")
        self.frame_count = 0


def main(args=None):
    """Main entry point"""
    rclpy.init(args=args)
    
    try:
        publisher = SensorCostmapPublisher()
        rclpy.spin(publisher)
    except KeyboardInterrupt:
        pass
    finally:
        rclpy.shutdown()


if __name__ == '__main__':
    main()
