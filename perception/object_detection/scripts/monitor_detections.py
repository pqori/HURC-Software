#!/usr/bin/env python3
"""
Monitor Object Detection from ROS 2 topics.

Subscribes to /detected_objects and displays detected objects
with their classes and confidence scores.
"""

import rclpy
from rclpy.node import Node
from vision_msgs.msg import Detection2DArray
from datetime import datetime
from collections import Counter


class ObjectDetectionMonitor(Node):
    """Monitor and display object detections."""
    
    def __init__(self):
        super().__init__('object_detection_monitor')
        
        # Subscribe to detections
        self.subscription = self.create_subscription(
            Detection2DArray,
            '/detected_objects',
            self.detection_callback,
            10
        )
        
        self.frame_count = 0
        self.detection_count = 0
        self.objects_seen = Counter()
        self.last_detection_time = None
        
        print("\n" + "="*70)
        print("🎯 Object Detection Monitor")
        print("="*70)
        print("Listening to /detected_objects...")
        print("Press Ctrl+C to stop\n")
    
    def detection_callback(self, msg: Detection2DArray):
        """Process incoming detection messages."""
        self.frame_count += 1
        
        if len(msg.detections) > 0:
            self.detection_count += len(msg.detections)
            self.last_detection_time = datetime.now()
            
            timestamp = datetime.now().strftime("%H:%M:%S")
            
            # Extract detected objects (with names from results)
            detected_objects = []
            for det in msg.detections:
                if det.results:
                    result = det.results[0]
                    class_id = result.hypothesis.class_id
                    confidence = result.hypothesis.score
                    
                    # Try to get class name from hypothesis if available
                    class_name = result.hypothesis.class_id if hasattr(result.hypothesis, 'class_id') else str(class_id)
                    
                    detected_objects.append((class_id, class_name, confidence))
                    self.objects_seen[str(class_id)] += 1
            
            # Group by class for summary
            class_summary = Counter([name for _, name, _ in detected_objects])
            summary_str = ', '.join([f"{name}×{count}" for name, count in class_summary.most_common()])
            
            # Display
            print(f"[{timestamp}] ✅ {len(msg.detections)} object(s): {summary_str}")
            
            # Show details
            for i, (class_id, class_name, conf) in enumerate(detected_objects[:8], 1):  # Limit to 8
                print(f"    [{i}] {class_name} ({class_id}): {conf*100:.1f}%")
        
        # Status update every 50 frames
        if self.frame_count % 50 == 0 and self.frame_count > 0:
            print(f"\n{'='*70}")
            print(f"📊 Status: {self.frame_count} frames | {self.detection_count} total detections")
            if self.objects_seen:
                print(f"Most detected: {self.objects_seen.most_common(3)}")
            print(f"{'='*70}\n")


def main(args=None):
    """Main entry point."""
    rclpy.init(args=args)
    
    monitor = None
    try:
        monitor = ObjectDetectionMonitor()
        rclpy.spin(monitor)
    except KeyboardInterrupt:
        print("\n\n" + "="*70)
        print("🛑 Monitoring stopped")
        print("="*70)
        if monitor is not None:
            print(f"Total frames:      {monitor.frame_count}")
            print(f"Total detections:  {monitor.detection_count}")
            if monitor.objects_seen:
                print(f"Detected classes:  {dict(monitor.objects_seen)}")
        print("="*70 + "\n")
    finally:
        if monitor is not None:
            try:
                monitor.destroy_node()
            except Exception:
                pass
        
        try:
            rclpy.try_shutdown()
        except Exception:
            pass


if __name__ == '__main__':
    main()

