#!/usr/bin/env python3
"""
Map Server Bridge

Bridges ROS topics to file-based format for web map visualization.
Writes position data that JavaScript can poll for live updates.
"""

import rclpy
from rclpy.node import Node
from geographic_msgs.msg import GeoPoseStamped
from sensor_msgs.msg import NavSatFix
from pathlib import Path
import os


class MapServer(Node):
    """Bridge ROS topics to web-accessible files"""

    def __init__(self):
        super().__init__('map_server')

        # Parameters
        self.declare_parameter('map_data_dir', str(
            Path.home() / 'zed_map_data'))
        self.declare_parameter('fused_topic', '/zed_gnss_fusion/geo_pose')
        self.declare_parameter('raw_gnss_topic', '/gps/fix')

        # Get parameters
        self.map_data_dir = Path(self.get_parameter('map_data_dir').value)
        fused_topic = self.get_parameter('fused_topic').value
        raw_topic = self.get_parameter('raw_gnss_topic').value

        # Create data directory
        self.map_data_dir.mkdir(parents=True, exist_ok=True)

        # Subscribers
        self.create_subscription(GeoPoseStamped, fused_topic,
                                 self.fused_callback, 10)
        self.create_subscription(NavSatFix, raw_topic,
                                 self.raw_gnss_callback, 10)

        self.get_logger().info('🗺️  Map Server Bridge started')
        self.get_logger().info(f'   Data directory: {self.map_data_dir}')
        self.get_logger().info(f'   Fused topic: {fused_topic}')
        self.get_logger().info(f'   Raw GNSS topic: {raw_topic}')
        self.get_logger().info('')
        self.get_logger().info('📍 To view map:')
        self.get_logger().info(f'   cd {self.map_data_dir}')
        self.get_logger().info('   python3 -m http.server 8000')
        self.get_logger().info('   Open: http://localhost:8000/')

    def fused_callback(self, msg: GeoPoseStamped):
        """Write fused VIO+GNSS position"""
        try:
            lat = msg.pose.position.latitude
            lon = msg.pose.position.longitude
            timestamp = self.get_clock().now().nanoseconds // 1_000_000

            # Write to file (overwrite mode for polling)
            data_file = self.map_data_dir / 'data.txt'
            with open(data_file, 'w') as f:
                f.write(f"{lat:.17f},{lon:.17f},{timestamp}")

        except Exception as e:
            self.get_logger().error(f'Error writing fused data: {e}')

    def raw_gnss_callback(self, msg: NavSatFix):
        """Write raw GNSS position"""
        try:
            lat = msg.latitude
            lon = msg.longitude
            timestamp = self.get_clock().now().nanoseconds // 1_000_000

            # Extract covariance
            if len(msg.position_covariance) >= 9:
                lon_std = msg.position_covariance[0] ** 0.5
                lat_std = msg.position_covariance[4] ** 0.5
                alt_std = msg.position_covariance[8] ** 0.5
            else:
                lon_std = lat_std = alt_std = 2.0

            # Map status to string
            status_map = {
                -1: "NO_FIX",
                0: "SINGLE",
                1: "DGPS",
                2: "RTK_FIX"
            }
            status = status_map.get(msg.status.status, "UNKNOWN")

            # Write to file
            raw_file = self.map_data_dir / 'raw_data.txt'
            with open(raw_file, 'w') as f:
                f.write(f"{lat:.17f},{lon:.17f},{timestamp},")
                f.write(f"{lon_std:.6f},{lat_std:.6f},{alt_std:.6f},{status}")

        except Exception as e:
            self.get_logger().error(f'Error writing raw GNSS: {e}')


def main(args=None):
    rclpy.init(args=args)

    try:
        node = MapServer()
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == '__main__':
    main()
