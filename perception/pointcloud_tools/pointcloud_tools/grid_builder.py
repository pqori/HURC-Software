#!/usr/bin/env python3
"""
Build 2D occupancy grid from point cloud for Nav2
Creates a local costmap-like grid from 3D point cloud data
"""

import rclpy
from rclpy.node import Node
from sensor_msgs.msg import PointCloud2
from sensor_msgs_py import point_cloud2
from nav_msgs.msg import OccupancyGrid
import numpy as np


class GridBuilder(Node):
    def __init__(self):
        super().__init__('grid_builder')
        
        # Parameters
        self.declare_parameter('frame_id', 'map')
        self.declare_parameter('resolution', 0.05)
        self.declare_parameter('size_xy', 20.0)
        self.declare_parameter('z_thresh', 0.15)
        self.declare_parameter('range_max', 15.0)
        self.declare_parameter('cloud_topic', '/rtabmap/cloud_map')
        self.declare_parameter('grid_topic', '/local_map')
        
        self.frame_id = self.get_parameter('frame_id').value
        self.resolution = self.get_parameter('resolution').value
        self.size_xy = self.get_parameter('size_xy').value
        self.z_thresh = self.get_parameter('z_thresh').value
        self.range_max = self.get_parameter('range_max').value
        cloud_topic = self.get_parameter('cloud_topic').value
        grid_topic = self.get_parameter('grid_topic').value
        
        # Calculate grid dimensions
        self.width = int(self.size_xy / self.resolution)
        self.height = int(self.size_xy / self.resolution)
        self.origin_x = -self.size_xy / 2.0
        self.origin_y = -self.size_xy / 2.0
        
        # Create subscriber and publisher
        self.sub = self.create_subscription(
            PointCloud2,
            cloud_topic,
            self.pointcloud_callback,
            10
        )
        self.pub = self.create_publisher(OccupancyGrid, grid_topic, 10)
        
        self.get_logger().info(f'Grid Builder initialized')
        self.get_logger().info(f'  Input:      {cloud_topic}')
        self.get_logger().info(f'  Output:     {grid_topic}')
        self.get_logger().info(f'  Size:       {self.size_xy}m x {self.size_xy}m')
        self.get_logger().info(f'  Resolution: {self.resolution}m ({self.width}x{self.height} cells)')
        self.get_logger().info(f'  Z thresh:   {self.z_thresh}m')

    def pointcloud_callback(self, msg):
        # Initialize grid
        grid = np.zeros((self.height, self.width), dtype=np.int8)
        
        # Process points
        obstacle_count = 0
        for point in point_cloud2.read_points(msg, field_names=("x", "y", "z"), skip_nans=True):
            x, y, z = point
            
            # Check if point is an obstacle (above threshold)
            if z < self.z_thresh:
                continue
            
            # Check range
            if abs(x) > self.range_max or abs(y) > self.range_max:
                continue
            
            # Convert to grid coordinates
            grid_x = int((x - self.origin_x) / self.resolution)
            grid_y = int((y - self.origin_y) / self.resolution)
            
            # Check bounds
            if 0 <= grid_x < self.width and 0 <= grid_y < self.height:
                grid[grid_y, grid_x] = 100  # Mark as occupied
                obstacle_count += 1
        
        # Create OccupancyGrid message
        occupancy_grid = OccupancyGrid()
        occupancy_grid.header = msg.header
        occupancy_grid.header.frame_id = self.frame_id
        occupancy_grid.info.resolution = self.resolution
        occupancy_grid.info.width = self.width
        occupancy_grid.info.height = self.height
        occupancy_grid.info.origin.position.x = self.origin_x
        occupancy_grid.info.origin.position.y = self.origin_y
        occupancy_grid.info.origin.position.z = 0.0
        occupancy_grid.info.origin.orientation.w = 1.0
        
        # Flatten grid and convert to list
        occupancy_grid.data = grid.flatten().tolist()
        
        self.pub.publish(occupancy_grid)
        
        if obstacle_count > 0:
            self.get_logger().debug(f'Published grid with {obstacle_count} obstacle cells')


def main(args=None):
    rclpy.init(args=args)
    node = GridBuilder()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()

