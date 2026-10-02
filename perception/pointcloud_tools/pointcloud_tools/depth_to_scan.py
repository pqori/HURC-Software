#!/usr/bin/env python3
"""
Convert ZED point cloud to 2D LaserScan for Nav2
Takes 3D point cloud and creates a virtual 2D laser scan by slicing a horizontal plane
"""

import rclpy
from rclpy.node import Node
from sensor_msgs.msg import PointCloud2, LaserScan
from sensor_msgs_py import point_cloud2
import numpy as np
import math


class DepthToScan(Node):
    def __init__(self):
        super().__init__('depth_to_scan')

        # Parameters
        self.declare_parameter('frame_id', 'base_link')
        self.declare_parameter('range_max', 12.0)
        self.declare_parameter('fov_deg', 120.0)
        self.declare_parameter('scan_height', 0.30)
        self.declare_parameter('min_z', -0.1)
        self.declare_parameter('max_z', 0.30)
        self.declare_parameter('angular_res_deg', 1.0)
        self.declare_parameter(
            'pointcloud_topic', '/zed2i/zed2i_camera/point_cloud/cloud_registered')
        self.declare_parameter('scan_topic', '/scan')

        self.frame_id = self.get_parameter('frame_id').value
        self.range_max = self.get_parameter('range_max').value
        self.fov_deg = self.get_parameter('fov_deg').value
        self.scan_height = self.get_parameter('scan_height').value
        self.min_z = self.get_parameter('min_z').value
        self.max_z = self.get_parameter('max_z').value
        self.angular_res_deg = self.get_parameter('angular_res_deg').value
        pc_topic = self.get_parameter('pointcloud_topic').value
        scan_topic = self.get_parameter('scan_topic').value

        # Calculate scan parameters
        self.fov_rad = math.radians(self.fov_deg)
        self.angular_res_rad = math.radians(self.angular_res_deg)
        self.num_beams = int(self.fov_deg / self.angular_res_deg)
        self.angle_min = -self.fov_rad / 2
        self.angle_max = self.fov_rad / 2

        # Create subscriber and publisher
        self.sub = self.create_subscription(
            PointCloud2,
            pc_topic,
            self.pointcloud_callback,
            10
        )
        self.pub = self.create_publisher(LaserScan, scan_topic, 10)

        self.get_logger().info(f'Depth to Scan converter initialized')
        self.get_logger().info(f'  Input:  {pc_topic}')
        self.get_logger().info(f'  Output: {scan_topic}')
        self.get_logger().info(
            f'  FOV:    {self.fov_deg}° ({self.num_beams} beams)')
        self.get_logger().info(f'  Range:  {self.range_max}m')
        self.get_logger().info(f'  Height: {self.min_z}m to {self.max_z}m')

    def pointcloud_callback(self, msg):
        # Convert PointCloud2 to numpy array
        points = []
        for point in point_cloud2.read_points(msg, field_names=("x", "y", "z"), skip_nans=True):
            x, y, z = point
            # Filter by height
            if self.min_z <= z <= self.max_z:
                points.append([x, y, z])

        if len(points) == 0:
            self.get_logger().warn('No points in height range', throttle_duration_sec=5.0)
            return

        points = np.array(points)

        # Create LaserScan message
        scan = LaserScan()
        scan.header = msg.header
        scan.header.frame_id = self.frame_id
        scan.angle_min = self.angle_min
        scan.angle_max = self.angle_max
        scan.angle_increment = self.angular_res_rad
        scan.time_increment = 0.0
        scan.scan_time = 0.1
        scan.range_min = 0.2
        scan.range_max = self.range_max

        # Initialize ranges to max
        ranges = np.full(self.num_beams, self.range_max, dtype=np.float32)

        # Calculate range and angle for each point
        for point in points:
            x, y, z = point

            # Calculate distance and angle
            distance = math.sqrt(x*x + y*y)
            if distance > self.range_max or distance < scan.range_min:
                continue

            angle = math.atan2(y, x)

            # Check if angle is within FOV
            if angle < self.angle_min or angle > self.angle_max:
                continue

            # Calculate beam index
            beam_idx = int((angle - self.angle_min) / self.angular_res_rad)
            if 0 <= beam_idx < self.num_beams:
                # Keep minimum distance (closest obstacle)
                ranges[beam_idx] = min(ranges[beam_idx], distance)

        scan.ranges = ranges.tolist()
        scan.intensities = []

        self.pub.publish(scan)


def main(args=None):
    rclpy.init(args=args)
    node = DepthToScan()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
