#!/usr/bin/env python3
"""
URC Perception Health Monitor
============================

Monitors the health of all perception system components.
Publishes diagnostics and alerts for system failures.

Author: URC Perception Team
License: Apache-2.0
"""

import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, ReliabilityPolicy
from diagnostic_msgs.msg import DiagnosticArray, DiagnosticStatus, KeyValue
from std_msgs.msg import Header
import time
from typing import Dict, List, Any
import yaml


class PerceptionHealthMonitor(Node):
    """Monitors health of perception system components"""
    
    def __init__(self):
        super().__init__('perception_health_monitor')
        
        # Load configuration
        self.declare_parameter('config_file', '')
        config_file = self.get_parameter('config_file').value
        
        if config_file:
            with open(config_file, 'r') as f:
                self.config = yaml.safe_load(f)['perception_health_monitor']
        else:
            self.config = self._default_config()
        
        # Topic monitoring state
        self.topic_states: Dict[str, Dict[str, Any]] = {}
        self.subscribers: Dict[str, Any] = {}
        
        # Initialize monitoring for each topic
        self._setup_topic_monitoring()
        
        # Diagnostics publisher
        self.diagnostics_pub = self.create_publisher(
            DiagnosticArray, 
            self.config.get('diagnostics_topic', '/perception/diagnostics'),
            10
        )
        
        # Health check timer
        check_freq = self.config.get('check_frequency', 2.0)
        self.health_timer = self.create_timer(1.0 / check_freq, self._check_system_health)
        
        self.get_logger().info("🏥 Perception Health Monitor started")
    
    def _default_config(self) -> Dict:
        """Default configuration if no config file provided"""
        return {
            'critical_topics': [
                {'topic': '/zed2i/left/image_rect_color', 'timeout': 2.0, 'description': 'ZED2i camera'},
                {'topic': '/odometry/filtered', 'timeout': 2.0, 'description': 'Filtered odometry'},
                {'topic': '/map', 'timeout': 10.0, 'description': 'SLAM map'}
            ],
            'detection_topics': [
                {'topic': '/detected_objects', 'timeout': 5.0, 'description': 'Object detections'}
            ],
            'check_frequency': 2.0,
            'warning_threshold': 3,
            'error_threshold': 10
        }
    
    def _setup_topic_monitoring(self):
        """Setup subscribers for all monitored topics"""
        qos = QoSProfile(depth=1, reliability=ReliabilityPolicy.BEST_EFFORT)
        
        # Monitor critical topics
        for topic_info in self.config.get('critical_topics', []):
            topic = topic_info['topic']
            self._init_topic_monitoring(topic, topic_info, qos, critical=True)
        
        # Monitor detection topics
        for topic_info in self.config.get('detection_topics', []):
            topic = topic_info['topic']
            self._init_topic_monitoring(topic, topic_info, qos, critical=False)
    
    def _init_topic_monitoring(self, topic: str, topic_info: Dict, qos: QoSProfile, critical: bool):
        """Initialize monitoring for a specific topic"""
        self.topic_states[topic] = {
            'last_received': None,
            'timeout': topic_info.get('timeout', 5.0),
            'description': topic_info.get('description', topic),
            'critical': critical,
            'failure_count': 0,
            'status': 'UNKNOWN'
        }
        
        # Create generic subscriber (we just need to know when messages arrive)
        self.subscribers[topic] = self.create_subscription(
            rclpy.node.AnyMsg,
            topic,
            lambda msg, t=topic: self._topic_callback(t, msg),
            qos
        )
        
        self.get_logger().info(f"📡 Monitoring {topic} ({'critical' if critical else 'optional'})")
    
    def _topic_callback(self, topic: str, msg):
        """Callback when a monitored topic receives a message"""
        if topic in self.topic_states:
            self.topic_states[topic]['last_received'] = time.time()
            self.topic_states[topic]['failure_count'] = 0
            self.topic_states[topic]['status'] = 'OK'
    
    def _check_system_health(self):
        """Check health of all monitored topics and publish diagnostics"""
        current_time = time.time()
        diagnostics = DiagnosticArray()
        diagnostics.header = Header()
        diagnostics.header.stamp = self.get_clock().now().to_msg()
        
        system_status = DiagnosticStatus.OK
        critical_failures = []
        warnings = []
        
        for topic, state in self.topic_states.items():
            status = self._check_topic_health(topic, state, current_time)
            diagnostics.status.append(status)
            
            # Track system-level status
            if status.level == DiagnosticStatus.ERROR:
                if state['critical']:
                    system_status = DiagnosticStatus.ERROR
                    critical_failures.append(state['description'])
                else:
                    warnings.append(state['description'])
            elif status.level == DiagnosticStatus.WARN and system_status == DiagnosticStatus.OK:
                system_status = DiagnosticStatus.WARN
        
        # Add system-level diagnostic
        system_diag = DiagnosticStatus()
        system_diag.name = "URC Perception System"
        system_diag.level = system_status
        
        if system_status == DiagnosticStatus.OK:
            system_diag.message = "All perception components operational"
        elif system_status == DiagnosticStatus.WARN:
            system_diag.message = f"Warnings: {', '.join(warnings)}"
        else:
            system_diag.message = f"Critical failures: {', '.join(critical_failures)}"
        
        diagnostics.status.insert(0, system_diag)
        
        # Publish diagnostics
        self.diagnostics_pub.publish(diagnostics)
        
        # Log critical issues
        if critical_failures:
            self.get_logger().error(f"🚨 Critical perception failures: {critical_failures}")
        elif warnings:
            self.get_logger().warn(f"⚠️ Perception warnings: {warnings}")
    
    def _check_topic_health(self, topic: str, state: Dict, current_time: float) -> DiagnosticStatus:
        """Check health of a specific topic"""
        status = DiagnosticStatus()
        status.name = f"Topic: {topic}"
        
        # Check if we've ever received a message
        if state['last_received'] is None:
            state['failure_count'] += 1
            status.level = DiagnosticStatus.WARN if state['failure_count'] < self.config.get('error_threshold', 10) else DiagnosticStatus.ERROR
            status.message = f"No messages received from {state['description']}"
            state['status'] = 'NO_DATA'
        else:
            # Check if topic is stale
            time_since_last = current_time - state['last_received']
            if time_since_last > state['timeout']:
                state['failure_count'] += 1
                if state['failure_count'] >= self.config.get('error_threshold', 10):
                    status.level = DiagnosticStatus.ERROR
                    status.message = f"{state['description']} stale for {time_since_last:.1f}s"
                    state['status'] = 'STALE'
                elif state['failure_count'] >= self.config.get('warning_threshold', 3):
                    status.level = DiagnosticStatus.WARN
                    status.message = f"{state['description']} potentially stale ({time_since_last:.1f}s)"
                    state['status'] = 'WARNING'
                else:
                    status.level = DiagnosticStatus.OK
                    status.message = f"{state['description']} active"
                    state['status'] = 'OK'
            else:
                status.level = DiagnosticStatus.OK
                status.message = f"{state['description']} active ({time_since_last:.1f}s ago)"
                state['status'] = 'OK'
                state['failure_count'] = 0
        
        # Add diagnostic details
        status.values = [
            KeyValue(key="timeout", value=str(state['timeout'])),
            KeyValue(key="failure_count", value=str(state['failure_count'])),
            KeyValue(key="critical", value=str(state['critical'])),
            KeyValue(key="status", value=state['status'])
        ]
        
        return status


def main(args=None):
    """Main entry point"""
    rclpy.init(args=args)
    
    try:
        monitor = PerceptionHealthMonitor()
        rclpy.spin(monitor)
    except KeyboardInterrupt:
        pass
    finally:
        rclpy.shutdown()


if __name__ == '__main__':
    main()
