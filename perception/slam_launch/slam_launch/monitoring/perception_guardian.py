#!/usr/bin/env python3
"""
Perception Guardian - Robustness & Error Handling
=================================================

Provides graceful degradation, automatic recovery, and data quality validation
for the perception system. Monitors sensor health and implements fallback strategies.

Features:
- Sensor failure detection and graceful degradation
- Automatic recovery mechanisms
- Data quality validation (motion blur, lighting)
- Fault-tolerant sensor fusion
- Performance monitoring and adaptive control

Author: URC Perception Team
License: Apache-2.0
"""

import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Image, Imu, PointCloud2
from nav_msgs.msg import Odometry
from diagnostic_msgs.msg import DiagnosticArray, DiagnosticStatus
from std_msgs.msg import String, Bool, Float32
from geometry_msgs.msg import Twist
import cv2
from cv_bridge import CvBridge
import numpy as np
import time
from typing import Dict, List, Optional, Tuple
from collections import deque
from enum import Enum
import threading
import json


class SensorState(Enum):
    """Sensor operational states"""
    HEALTHY = "healthy"
    DEGRADED = "degraded"
    FAILED = "failed"
    RECOVERING = "recovering"
    UNKNOWN = "unknown"


class DataQualityMetrics:
    """Data quality assessment metrics"""
    
    def __init__(self):
        self.motion_blur_score = 0.0
        self.lighting_score = 0.0
        self.noise_level = 0.0
        self.sharpness_score = 0.0
        self.exposure_quality = 0.0
        self.timestamp = 0.0
        
    def overall_quality(self) -> float:
        """Calculate overall quality score (0-1)"""
        weights = {
            'motion_blur': 0.3,
            'lighting': 0.25,
            'sharpness': 0.25,
            'exposure': 0.15,
            'noise': 0.05
        }
        
        return (
            weights['motion_blur'] * (1.0 - self.motion_blur_score) +
            weights['lighting'] * self.lighting_score +
            weights['sharpness'] * self.sharpness_score +
            weights['exposure'] * self.exposure_quality +
            weights['noise'] * (1.0 - self.noise_level)
        )


class SensorMonitor:
    """Monitors individual sensor health and performance"""
    
    def __init__(self, sensor_name: str, timeout: float = 2.0):
        self.name = sensor_name
        self.timeout = timeout
        self.state = SensorState.UNKNOWN
        self.last_data_time = 0.0
        self.failure_count = 0
        self.recovery_attempts = 0
        self.quality_history = deque(maxlen=50)
        self.performance_history = deque(maxlen=100)
        
    def update_data(self, quality_score: float = 1.0):
        """Update sensor with new data"""
        self.last_data_time = time.time()
        self.quality_history.append(quality_score)
        
        # Update state based on data quality and timing
        if quality_score > 0.7:
            if self.state == SensorState.FAILED:
                self.state = SensorState.RECOVERING
            elif self.state == SensorState.RECOVERING and len(self.quality_history) > 10:
                avg_quality = np.mean(list(self.quality_history)[-10:])
                if avg_quality > 0.8:
                    self.state = SensorState.HEALTHY
                    self.failure_count = 0
                    self.recovery_attempts = 0
            else:
                self.state = SensorState.HEALTHY
        elif quality_score > 0.4:
            self.state = SensorState.DEGRADED
        
    def check_timeout(self) -> bool:
        """Check if sensor has timed out"""
        if time.time() - self.last_data_time > self.timeout:
            self.failure_count += 1
            if self.failure_count > 3:
                self.state = SensorState.FAILED
            return True
        return False
    
    def get_avg_quality(self) -> float:
        """Get average quality over recent history"""
        if not self.quality_history:
            return 0.0
        return np.mean(self.quality_history)
    
    def is_operational(self) -> bool:
        """Check if sensor is operational (not failed)"""
        return self.state not in [SensorState.FAILED]


class PerceptionGuardian(Node):
    """Main guardian node for perception system robustness"""
    
    def __init__(self):
        super().__init__('perception_guardian')
        
        # Parameters
        self.declare_parameter('enable_auto_recovery', True)
        self.declare_parameter('quality_check_rate', 5.0)
        self.declare_parameter('min_operational_sensors', 2)
        self.declare_parameter('degraded_mode_threshold', 0.6)
        self.declare_parameter('recovery_timeout', 30.0)
        
        self.auto_recovery = self.get_parameter('enable_auto_recovery').value
        check_rate = self.get_parameter('quality_check_rate').value
        self.min_sensors = self.get_parameter('min_operational_sensors').value
        self.degraded_threshold = self.get_parameter('degraded_mode_threshold').value
        self.recovery_timeout = self.get_parameter('recovery_timeout').value
        
        # Initialize sensor monitors
        self.sensors = {
            'zed2i_camera': SensorMonitor('zed2i_camera', timeout=2.0),
            'zed2i_imu': SensorMonitor('zed2i_imu', timeout=1.0),
            'gnss': SensorMonitor('gnss', timeout=5.0),
            'odometry': SensorMonitor('odometry', timeout=2.0)
        }
        
        # Data quality assessment
        self.bridge = CvBridge()
        self.current_image_quality = DataQualityMetrics()
        
        # System state
        self.system_mode = "normal"  # normal, degraded, emergency
        self.active_fallbacks = set()
        
        # Subscribers for sensor monitoring
        self.create_subscription(Image, '/zed2i/left/image_rect_color', self.cb_camera_image, 10)
        self.create_subscription(Imu, '/zed2i/imu/data', self.cb_imu_data, 10)
        self.create_subscription(Odometry, '/odometry/filtered', self.cb_odometry, 10)
        
        # Publishers for system control
        self.system_mode_pub = self.create_publisher(String, '/perception/system_mode', 10)
        self.quality_pub = self.create_publisher(Float32, '/perception/data_quality', 10)
        self.recovery_pub = self.create_publisher(String, '/perception/recovery_actions', 10)
        self.emergency_stop_pub = self.create_publisher(Bool, '/perception/emergency_stop', 10)
        
        # Timers
        self.create_timer(1.0 / check_rate, self.monitor_system_health)
        self.create_timer(0.5, self.check_sensor_timeouts)
        
        # Recovery thread
        self.recovery_thread = None
        self.recovery_lock = threading.Lock()
        
        self.get_logger().info("🛡️ Perception Guardian initialized - System protection active")
    
    def cb_camera_image(self, msg: Image):
        """Process camera image for quality assessment"""
        try:
            cv_image = self.bridge.imgmsg_to_cv2(msg, 'bgr8')
            quality = self.assess_image_quality(cv_image)
            
            self.current_image_quality = quality
            self.sensors['zed2i_camera'].update_data(quality.overall_quality())
            
            # Publish quality score
            quality_msg = Float32()
            quality_msg.data = quality.overall_quality()
            self.quality_pub.publish(quality_msg)
            
        except Exception as e:
            self.get_logger().error(f"Image quality assessment failed: {e}")
            self.sensors['zed2i_camera'].update_data(0.0)
    
    def cb_imu_data(self, msg: Imu):
        """Process IMU data for quality assessment"""
        try:
            # Check IMU data quality (acceleration and angular velocity ranges)
            acc_norm = np.sqrt(msg.linear_acceleration.x**2 + 
                              msg.linear_acceleration.y**2 + 
                              msg.linear_acceleration.z**2)
            
            gyro_norm = np.sqrt(msg.angular_velocity.x**2 + 
                               msg.angular_velocity.y**2 + 
                               msg.angular_velocity.z**2)
            
            # Quality based on reasonable ranges and covariance
            quality = 1.0
            if acc_norm > 50.0 or acc_norm < 5.0:  # Unreasonable acceleration
                quality *= 0.5
            if gyro_norm > 10.0:  # Excessive rotation
                quality *= 0.7
            
            self.sensors['zed2i_imu'].update_data(quality)
            
        except Exception as e:
            self.get_logger().error(f"IMU quality assessment failed: {e}")
            self.sensors['zed2i_imu'].update_data(0.0)
    
    def cb_odometry(self, msg: Odometry):
        """Process odometry for quality assessment"""
        try:
            # Check odometry quality based on covariance and reasonable values
            pos_cov = msg.pose.covariance[0] + msg.pose.covariance[7] + msg.pose.covariance[14]
            vel_cov = msg.twist.covariance[0] + msg.twist.covariance[7] + msg.twist.covariance[14]
            
            # Quality based on covariance (lower is better)
            quality = 1.0 / (1.0 + pos_cov + vel_cov)
            quality = min(1.0, max(0.0, quality))
            
            self.sensors['odometry'].update_data(quality)
            
        except Exception as e:
            self.get_logger().error(f"Odometry quality assessment failed: {e}")
            self.sensors['odometry'].update_data(0.0)
    
    def assess_image_quality(self, image: np.ndarray) -> DataQualityMetrics:
        """Comprehensive image quality assessment"""
        quality = DataQualityMetrics()
        quality.timestamp = time.time()
        
        try:
            gray = cv2.cvtColor(image, cv2.COLOR_BGR2GRAY)
            
            # 1. Motion blur detection using Laplacian variance
            laplacian_var = cv2.Laplacian(gray, cv2.CV_64F).var()
            quality.motion_blur_score = max(0.0, min(1.0, (200.0 - laplacian_var) / 200.0))
            
            # 2. Sharpness assessment
            quality.sharpness_score = min(1.0, laplacian_var / 500.0)
            
            # 3. Lighting assessment
            mean_brightness = np.mean(gray)
            # Optimal brightness around 127, penalty for too dark/bright
            brightness_penalty = abs(mean_brightness - 127) / 127.0
            quality.lighting_score = max(0.0, 1.0 - brightness_penalty)
            
            # 4. Exposure quality (histogram analysis)
            hist = cv2.calcHist([gray], [0], None, [256], [0, 256])
            # Check for clipping (too many pixels at extremes)
            clipped_pixels = hist[0] + hist[255]
            total_pixels = gray.shape[0] * gray.shape[1]
            clipping_ratio = clipped_pixels / total_pixels
            quality.exposure_quality = max(0.0, 1.0 - clipping_ratio * 10.0)
            
            # 5. Noise level estimation
            noise = np.std(cv2.GaussianBlur(gray, (5, 5), 0) - gray)
            quality.noise_level = min(1.0, noise / 50.0)
            
        except Exception as e:
            self.get_logger().error(f"Image quality assessment error: {e}")
            # Return poor quality on error
            quality.motion_blur_score = 1.0
            quality.lighting_score = 0.0
            quality.sharpness_score = 0.0
            quality.exposure_quality = 0.0
            quality.noise_level = 1.0
        
        return quality
    
    def check_sensor_timeouts(self):
        """Check for sensor timeouts and update states"""
        for sensor in self.sensors.values():
            if sensor.check_timeout():
                self.get_logger().warn(f"⚠️ Sensor timeout: {sensor.name}")
    
    def monitor_system_health(self):
        """Main system health monitoring and decision making"""
        operational_sensors = sum(1 for s in self.sensors.values() if s.is_operational())
        avg_quality = np.mean([s.get_avg_quality() for s in self.sensors.values() if s.is_operational()])
        
        # Determine system mode
        previous_mode = self.system_mode
        
        if operational_sensors < self.min_sensors:
            self.system_mode = "emergency"
        elif avg_quality < self.degraded_threshold or operational_sensors < len(self.sensors):
            self.system_mode = "degraded"
        else:
            self.system_mode = "normal"
        
        # Handle mode transitions
        if self.system_mode != previous_mode:
            self.handle_mode_transition(previous_mode, self.system_mode)
        
        # Publish system mode
        mode_msg = String()
        mode_msg.data = self.system_mode
        self.system_mode_pub.publish(mode_msg)
        
        # Trigger recovery if needed
        if self.auto_recovery and self.system_mode in ["degraded", "emergency"]:
            self.trigger_recovery()
    
    def handle_mode_transition(self, old_mode: str, new_mode: str):
        """Handle transitions between system modes"""
        self.get_logger().info(f"🔄 System mode transition: {old_mode} → {new_mode}")
        
        if new_mode == "emergency":
            self.activate_emergency_mode()
        elif new_mode == "degraded":
            self.activate_degraded_mode()
        elif new_mode == "normal":
            self.activate_normal_mode()
    
    def activate_emergency_mode(self):
        """Activate emergency mode with minimal functionality"""
        self.get_logger().error("🚨 EMERGENCY MODE ACTIVATED - Critical sensor failures detected")
        
        # Publish emergency stop
        emergency_msg = Bool()
        emergency_msg.data = True
        self.emergency_stop_pub.publish(emergency_msg)
        
        # Activate fallback strategies
        self.active_fallbacks.add("emergency_stop")
        self.active_fallbacks.add("minimal_sensors")
        
        recovery_msg = String()
        recovery_msg.data = json.dumps({
            "action": "emergency_mode",
            "fallbacks": list(self.active_fallbacks),
            "failed_sensors": [name for name, sensor in self.sensors.items() 
                             if sensor.state == SensorState.FAILED]
        })
        self.recovery_pub.publish(recovery_msg)
    
    def activate_degraded_mode(self):
        """Activate degraded mode with reduced functionality"""
        self.get_logger().warn("⚠️ DEGRADED MODE ACTIVATED - Reduced perception capability")
        
        # Implement graceful degradation strategies
        failed_sensors = [name for name, sensor in self.sensors.items() 
                         if sensor.state == SensorState.FAILED]
        
        if 'zed2i_camera' in failed_sensors:
            self.active_fallbacks.add("no_vision")
        if 'zed2i_imu' in failed_sensors:
            self.active_fallbacks.add("no_imu")
        if 'gnss' in failed_sensors:
            self.active_fallbacks.add("no_gps")
        
        recovery_msg = String()
        recovery_msg.data = json.dumps({
            "action": "degraded_mode",
            "fallbacks": list(self.active_fallbacks),
            "failed_sensors": failed_sensors
        })
        self.recovery_pub.publish(recovery_msg)
    
    def activate_normal_mode(self):
        """Return to normal operation"""
        self.get_logger().info("✅ NORMAL MODE RESTORED - Full perception capability")
        
        # Clear fallbacks
        self.active_fallbacks.clear()
        
        # Cancel emergency stop
        emergency_msg = Bool()
        emergency_msg.data = False
        self.emergency_stop_pub.publish(emergency_msg)
        
        recovery_msg = String()
        recovery_msg.data = json.dumps({
            "action": "normal_mode",
            "message": "All systems operational"
        })
        self.recovery_pub.publish(recovery_msg)
    
    def trigger_recovery(self):
        """Trigger automatic recovery procedures"""
        with self.recovery_lock:
            if self.recovery_thread is None or not self.recovery_thread.is_alive():
                self.recovery_thread = threading.Thread(target=self.recovery_procedure)
                self.recovery_thread.daemon = True
                self.recovery_thread.start()
    
    def recovery_procedure(self):
        """Automatic recovery procedure"""
        self.get_logger().info("🔧 Starting automatic recovery procedure...")
        
        recovery_actions = []
        
        # Attempt to recover failed sensors
        for name, sensor in self.sensors.items():
            if sensor.state == SensorState.FAILED:
                recovery_actions.append(f"restart_{name}")
                sensor.recovery_attempts += 1
                
                # Simulate recovery attempt (in real system, this would restart nodes/drivers)
                time.sleep(2.0)
                
                # Reset sensor state to allow new data
                sensor.state = SensorState.RECOVERING
                sensor.failure_count = 0
        
        # Publish recovery actions
        if recovery_actions:
            recovery_msg = String()
            recovery_msg.data = json.dumps({
                "action": "recovery_attempt",
                "actions": recovery_actions,
                "timestamp": time.time()
            })
            self.recovery_pub.publish(recovery_msg)
            
            self.get_logger().info(f"🔧 Recovery actions executed: {recovery_actions}")
        
        # Wait for recovery to take effect
        time.sleep(self.recovery_timeout)


def main(args=None):
    """Main entry point"""
    rclpy.init(args=args)
    
    try:
        guardian = PerceptionGuardian()
        rclpy.spin(guardian)
    except KeyboardInterrupt:
        pass
    finally:
        rclpy.shutdown()


if __name__ == '__main__':
    main()
