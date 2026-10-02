#!/usr/bin/env python3
"""
URC Perception System Status Publisher
=====================================

Publishes high-level system status and component information.
Provides a simple interface for mission control to monitor perception health.

Author: URC Perception Team  
License: Apache-2.0
"""

import rclpy
from rclpy.node import Node
from std_msgs.msg import String
from diagnostic_msgs.msg import DiagnosticArray
import json
import time
from typing import Dict, List


class SystemStatusPublisher(Node):
    """Publishes system status information"""
    
    def __init__(self):
        super().__init__('perception_status')
        
        # Parameters
        self.declare_parameter('system_name', 'URC_Perception_Stack')
        self.declare_parameter('publish_rate', 2.0)
        self.declare_parameter('monitor_components', [])
        
        self.system_name = self.get_parameter('system_name').value
        publish_rate = self.get_parameter('publish_rate').value
        self.components = self.get_parameter('monitor_components').value
        
        # Publishers
        self.status_pub = self.create_publisher(String, '/perception/status', 10)
        
        # Subscribers
        self.diagnostics_sub = self.create_subscription(
            DiagnosticArray,
            '/perception/diagnostics', 
            self._diagnostics_callback,
            10
        )
        
        # State
        self.last_diagnostics = None
        self.start_time = time.time()
        
        # Status publishing timer
        self.status_timer = self.create_timer(1.0 / publish_rate, self._publish_status)
        
        self.get_logger().info(f"📊 System Status Publisher started for {self.system_name}")
    
    def _diagnostics_callback(self, msg: DiagnosticArray):
        """Handle incoming diagnostics messages"""
        self.last_diagnostics = msg
    
    def _publish_status(self):
        """Publish current system status"""
        status_data = {
            'system_name': self.system_name,
            'timestamp': time.time(),
            'uptime': time.time() - self.start_time,
            'status': self._get_overall_status(),
            'components': self._get_component_status(),
            'summary': self._get_status_summary()
        }
        
        status_msg = String()
        status_msg.data = json.dumps(status_data, indent=2)
        self.status_pub.publish(status_msg)
    
    def _get_overall_status(self) -> str:
        """Get overall system status"""
        if self.last_diagnostics is None:
            return "INITIALIZING"
        
        # Check system-level diagnostic (first in array)
        if len(self.last_diagnostics.status) > 0:
            system_diag = self.last_diagnostics.status[0]
            if system_diag.level == 0:  # DiagnosticStatus.OK
                return "OPERATIONAL"
            elif system_diag.level == 1:  # DiagnosticStatus.WARN
                return "WARNING"
            elif system_diag.level == 2:  # DiagnosticStatus.ERROR
                return "ERROR"
            else:
                return "STALE"
        
        return "UNKNOWN"
    
    def _get_component_status(self) -> Dict:
        """Get status of individual components"""
        components = {}
        
        if self.last_diagnostics is None:
            for comp in self.components:
                components[comp] = {"status": "INITIALIZING", "message": "Starting up..."}
            return components
        
        # Parse diagnostics for component status
        for diag in self.last_diagnostics.status[1:]:  # Skip system-level diagnostic
            if diag.name.startswith("Topic: "):
                topic = diag.name.replace("Topic: ", "")
                
                # Map topics to components
                component = self._topic_to_component(topic)
                if component:
                    status = "OK" if diag.level == 0 else "WARN" if diag.level == 1 else "ERROR"
                    components[component] = {
                        "status": status,
                        "message": diag.message,
                        "topic": topic
                    }
        
        # Add any missing components
        for comp in self.components:
            if comp not in components:
                components[comp] = {"status": "UNKNOWN", "message": "No data available"}
        
        return components
    
    def _topic_to_component(self, topic: str) -> str:
        """Map topic names to component names"""
        topic_map = {
            '/zed2i/left/image_rect_color': 'zed2i_camera',
            '/zed2i/imu/data': 'zed2i_camera', 
            '/odometry/filtered': 'localization',
            '/map': 'rtabmap',
            '/detected_objects': 'object_detection',
            '/aruco_detections': 'aruco_detector',
            '/target_pose': 'pose_estimator'
        }
        return topic_map.get(topic)
    
    def _get_status_summary(self) -> str:
        """Get human-readable status summary"""
        overall = self._get_overall_status()
        components = self._get_component_status()
        
        if overall == "OPERATIONAL":
            active_count = sum(1 for c in components.values() if c['status'] == 'OK')
            return f"System operational - {active_count}/{len(components)} components active"
        elif overall == "WARNING":
            warning_comps = [name for name, info in components.items() if info['status'] == 'WARN']
            return f"System warnings in: {', '.join(warning_comps)}"
        elif overall == "ERROR":
            error_comps = [name for name, info in components.items() if info['status'] == 'ERROR']
            return f"System errors in: {', '.join(error_comps)}"
        elif overall == "INITIALIZING":
            return "System starting up..."
        else:
            return "System status unknown"


def main(args=None):
    """Main entry point"""
    rclpy.init(args=args)
    
    try:
        publisher = SystemStatusPublisher()
        rclpy.spin(publisher)
    except KeyboardInterrupt:
        pass
    finally:
        rclpy.shutdown()


if __name__ == '__main__':
    main()
