#!/usr/bin/env python3
"""
Monitor ArUco detections from ROS 2 topics.

Subscribes to /aruco/detections and displays detected markers
with their IDs, positions, and distances.
"""

import rclpy
from rclpy.node import Node
from vision_msgs.msg import Detection3DArray
import numpy as np
from datetime import datetime


class ArucoMonitor(Node):
    """Monitor and display ArUco marker detections."""
    
    def __init__(self):
        super().__init__('aruco_monitor')
        
        # Subscribe to detections
        self.subscription = self.create_subscription(
            Detection3DArray,
            '/aruco/detections',
            self.detection_callback,
            10
        )
        
        self.frame_count = 0
        self.detection_count = 0
        self.markers_seen = set()
        
        print("\n" + "="*70)
        print("ArUco Detection Monitor")
        print("="*70)
        print("Listening to /aruco/detections...")
        print("Press Ctrl+C to stop\n")
    
    def detection_callback(self, msg: Detection3DArray):
        """Process incoming detection messages."""
        self.frame_count += 1
        
        if len(msg.detections) > 0:
            self.detection_count += len(msg.detections)
            
            timestamp = datetime.now().strftime("%H:%M:%S.%f")[:-3]
            print(f"\n[{timestamp}] Frame {self.frame_count}: {len(msg.detections)} marker(s)")
            print("-" * 70)
            
            for i, det in enumerate(msg.detections):
                # Get marker ID
                if det.results:
                    marker_id = int(det.results[0].hypothesis.class_id)
                    confidence = det.results[0].hypothesis.score
                else:
                    marker_id = int(det.id) if det.id else -1
                    confidence = 1.0
                
                # Get position
                pos = det.bbox.center.position
                distance = np.sqrt(pos.x**2 + pos.y**2 + pos.z**2)
                
                # Track unique markers
                self.markers_seen.add(marker_id)
                
                # Display
                print(f"  [{i+1}] Marker ID: {marker_id:2d}")
                print(f"      Position:   X={pos.x:+.3f}m  Y={pos.y:+.3f}m  Z={pos.z:+.3f}m")
                print(f"      Distance:   {distance:.3f}m")
                print(f"      Confidence: {confidence*100:.1f}%")
                print(f"      Frame:      {msg.header.frame_id}")
        
        # Status update every 30 frames
        if self.frame_count % 30 == 0 and self.frame_count > 0:
            print(f"\n{'='*70}")
            print(f"Status: {self.frame_count} frames processed, {self.detection_count} total detections")
            print(f"Unique markers seen: {sorted(self.markers_seen)}")
            print(f"{'='*70}\n")


def main(args=None):
    """Main entry point."""
    rclpy.init(args=args)
    
    monitor = None
    try:
        monitor = ArucoMonitor()
        rclpy.spin(monitor)
    except KeyboardInterrupt:
        print("\n\n" + "="*70)
        print("Monitoring stopped")
        print("="*70)
        if monitor is not None:
            print(f"Total frames:      {monitor.frame_count}")
            print(f"Total detections:  {monitor.detection_count}")
            print(f"Unique markers:    {sorted(monitor.markers_seen) if monitor.markers_seen else 'None'}")
        print("="*70 + "\n")
    finally:
        # Clean shutdown with proper error handling
        if monitor is not None:
            try:
                monitor.destroy_node()
            except Exception:
                pass
        
        # Safe shutdown - check if already shut down
        try:
            rclpy.try_shutdown()
        except Exception:
            pass


if __name__ == '__main__':
    main()

