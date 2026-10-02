#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
from nav_msgs.msg import OccupancyGrid, Odometry
from geometry_msgs.msg import PoseStamped, Twist
from std_msgs.msg import String, Bool
from std_srvs.srv import SetBool, Trigger
import time
from typing import Optional, List
from enum import Enum


class RecoveryMode(Enum):
    """Navigation recovery modes"""
    NORMAL = "normal"
    SENSOR_DEGRADED = "sensor_degraded"
    LOCALIZATION_LOST = "localization_lost"
    MAPPING_FAILED = "mapping_failed"
    EMERGENCY_STOP = "emergency_stop"


class NavigationRecovery(Node):
    """Handle navigation failures and implement recovery strategies"""
    
    def __init__(self):
        super().__init__('navigation_recovery')
        
        # Parameters
        self.declare_parameter('localization_timeout', 5.0)
        self.declare_parameter('mapping_timeout', 10.0)
        self.declare_parameter('sensor_timeout', 3.0)
        self.declare_parameter('recovery_attempt_limit', 3)
        self.declare_parameter('enable_auto_recovery', True)
        
        # Get parameters
        self.loc_timeout = self.get_parameter('localization_timeout').value
        self.map_timeout = self.get_parameter('mapping_timeout').value
        self.sensor_timeout = self.get_parameter('sensor_timeout').value
        self.recovery_limit = self.get_parameter('recovery_attempt_limit').value
        self.auto_recovery = self.get_parameter('enable_auto_recovery').value
        
        # State tracking
        self.current_mode = RecoveryMode.NORMAL
        self.recovery_attempts = 0
        self.last_localization_time = 0.0
        self.last_map_time = 0.0
        self.last_sensor_time = 0.0
        
        # Subscribers for health monitoring
        self.create_subscription(Odometry, '/odometry/filtered', self.cb_localization, 10)
        self.create_subscription(OccupancyGrid, '/map', self.cb_map, 5)
        self.create_subscription(String, '/perception/system_mode', self.cb_perception_health, 10)
        self.create_subscription(Bool, '/navigation/frames_valid', self.cb_frame_health, 10)
        
        # Publishers
        self.recovery_mode_pub = self.create_publisher(String, '/navigation/recovery_mode', 10)
        self.recovery_action_pub = self.create_publisher(String, '/navigation/recovery_action', 10)
        self.emergency_stop_pub = self.create_publisher(Bool, '/navigation/emergency_stop', 10)
        self.fallback_cmd_pub = self.create_publisher(Twist, '/cmd_vel_fallback', 10)
        
        # Services
        self.create_service(Trigger, '/navigation/trigger_recovery', self.trigger_recovery)
        self.create_service(SetBool, '/navigation/enable_auto_recovery', self.enable_auto_recovery)
        self.create_service(Trigger, '/navigation/emergency_stop', self.emergency_stop)
        
        # Recovery monitoring
        self.create_timer(1.0, self.monitor_navigation_health)
        
        self.get_logger().info("🛡️ Navigation Recovery System initialized")
        self.get_logger().info(f"   Auto recovery: {self.auto_recovery}")
        self.get_logger().info(f"   Recovery attempt limit: {self.recovery_limit}")
    
    def cb_localization(self, msg: Odometry):
        """Monitor localization health"""
        self.last_localization_time = time.time()
        
        # Check localization quality from covariance
        pos_cov = msg.pose.covariance[0] + msg.pose.covariance[7] + msg.pose.covariance[14]
        if pos_cov > 10.0:  # High uncertainty
            self.get_logger().warn(f"⚠️ High localization uncertainty: {pos_cov:.2f}")
    
    def cb_map(self, msg: OccupancyGrid):
        """Monitor mapping health"""
        self.last_map_time = time.time()
    
    def cb_perception_health(self, msg: String):
        """Monitor perception system health"""
        if msg.data in ['degraded', 'emergency']:
            self.last_sensor_time = 0.0  # Force sensor timeout
        else:
            self.last_sensor_time = time.time()
    
    def cb_frame_health(self, msg: Bool):
        """Monitor frame health"""
        if not msg.data:
            self.get_logger().warn("⚠️ TF frames invalid - potential navigation issues")
    
    def monitor_navigation_health(self):
        """Monitor overall navigation health"""
        current_time = time.time()
        
        # Check for timeouts
        loc_age = current_time - self.last_localization_time
        map_age = current_time - self.last_map_time
        sensor_age = current_time - self.last_sensor_time
        
        # Determine recovery mode
        previous_mode = self.current_mode
        
        if sensor_age > self.sensor_timeout:
            self.current_mode = RecoveryMode.SENSOR_DEGRADED
        elif loc_age > self.loc_timeout:
            self.current_mode = RecoveryMode.LOCALIZATION_LOST
        elif map_age > self.map_timeout:
            self.current_mode = RecoveryMode.MAPPING_FAILED
        else:
            self.current_mode = RecoveryMode.NORMAL
            self.recovery_attempts = 0  # Reset on successful operation
        
        # Handle mode transitions
        if self.current_mode != previous_mode:
            self.handle_recovery_transition(previous_mode, self.current_mode)
        
        # Publish status
        self.publish_recovery_status()
        
        # Trigger recovery if needed
        if (self.auto_recovery and 
            self.current_mode != RecoveryMode.NORMAL and 
            self.recovery_attempts < self.recovery_limit):
            self.execute_recovery_strategy()
    
    def handle_recovery_transition(self, old_mode: RecoveryMode, new_mode: RecoveryMode):
        """Handle recovery mode transitions"""
        self.get_logger().info(f"🔄 Recovery mode: {old_mode.value} → {new_mode.value}")
        
        if new_mode == RecoveryMode.SENSOR_DEGRADED:
            self.get_logger().warn("⚠️ Sensor degraded - reducing navigation performance")
        elif new_mode == RecoveryMode.LOCALIZATION_LOST:
            self.get_logger().error("🚨 Localization lost - attempting recovery")
        elif new_mode == RecoveryMode.MAPPING_FAILED:
            self.get_logger().error("🚨 Mapping failed - switching to sensor-only navigation")
        elif new_mode == RecoveryMode.NORMAL:
            self.get_logger().info("✅ Navigation health restored")
    
    def execute_recovery_strategy(self):
        """Execute appropriate recovery strategy"""
        self.recovery_attempts += 1
        
        recovery_action = ""
        
        if self.current_mode == RecoveryMode.SENSOR_DEGRADED:
            recovery_action = "sensor_fallback"
            self.implement_sensor_fallback()
        elif self.current_mode == RecoveryMode.LOCALIZATION_LOST:
            recovery_action = "localization_reset"
            self.implement_localization_recovery()
        elif self.current_mode == RecoveryMode.MAPPING_FAILED:
            recovery_action = "mapping_fallback"
            self.implement_mapping_fallback()
        
        # Publish recovery action
        action_msg = String()
        action_msg.data = f"mode:{self.current_mode.value},action:{recovery_action},attempt:{self.recovery_attempts}"
        self.recovery_action_pub.publish(action_msg)
        
        self.get_logger().info(f"🔧 Recovery attempt {self.recovery_attempts}: {recovery_action}")
        
        # Emergency stop if too many failures
        if self.recovery_attempts >= self.recovery_limit:
            self.trigger_emergency_stop()
    
    def implement_sensor_fallback(self):
        """Implement sensor degradation fallback"""
        # Reduce navigation aggressiveness
        # In a full implementation, this would:
        # 1. Reduce velocity limits
        # 2. Increase safety margins
        # 3. Switch to more conservative navigation
        pass
    
    def implement_localization_recovery(self):
        """Implement localization recovery"""
        # In a full implementation, this would:
        # 1. Request localization reset
        # 2. Switch to alternative localization source
        # 3. Use visual landmarks for re-localization
        self.get_logger().info("🔧 Attempting localization recovery")
    
    def implement_mapping_fallback(self):
        """Implement mapping system fallback"""
        # In a full implementation, this would:
        # 1. Switch from ZED to RTAB-Map mapping
        # 2. Use cached map for navigation
        # 3. Enable sensor-only obstacle avoidance
        self.get_logger().info("🔧 Switching to mapping fallback mode")
    
    def trigger_emergency_stop(self):
        """Trigger emergency stop after repeated failures"""
        self.current_mode = RecoveryMode.EMERGENCY_STOP
        
        # Publish emergency stop
        stop_msg = Bool()
        stop_msg.data = True
        self.emergency_stop_pub.publish(stop_msg)
        
        # Stop rover movement
        stop_cmd = Twist()  # All zeros
        self.fallback_cmd_pub.publish(stop_cmd)
        
        self.get_logger().error("🚨 EMERGENCY STOP - Navigation recovery failed")
    
    def publish_recovery_status(self):
        """Publish recovery status"""
        mode_msg = String()
        mode_msg.data = self.current_mode.value
        self.recovery_mode_pub.publish(mode_msg)
    
    def trigger_recovery(self, request, response):
        """Service to manually trigger recovery"""
        if self.current_mode == RecoveryMode.NORMAL:
            response.success = False
            response.message = "Navigation is healthy - no recovery needed"
        else:
            self.execute_recovery_strategy()
            response.success = True
            response.message = f"Recovery triggered for mode: {self.current_mode.value}"
        
        return response
    
    def enable_auto_recovery(self, request, response):
        """Service to enable/disable auto recovery"""
        self.auto_recovery = request.data
        
        response.success = True
        response.message = f"Auto recovery {'enabled' if request.data else 'disabled'}"
        self.get_logger().info(f"🛡️ {response.message}")
        
        return response
    
    def emergency_stop(self, request, response):
        """Service to trigger emergency stop"""
        self.trigger_emergency_stop()
        
        response.success = True
        response.message = "Emergency stop activated"
        return response
    
    def log_performance(self):
        """Log navigation recovery performance"""
        fps = self.frame_count / 5.0
        
        self.get_logger().info(
            f"🛡️ Recovery System: Mode={self.current_mode.value}, "
            f"Attempts={self.recovery_attempts}, "
            f"Sensor_FPS={fps:.1f}"
        )
        
        self.frame_count = 0


def main(args=None):
    """Main entry point"""
    rclpy.init(args=args)
    
    try:
        recovery = NavigationRecovery()
        rclpy.spin(recovery)
    except KeyboardInterrupt:
        pass
    finally:
        rclpy.shutdown()


if __name__ == '__main__':
    main()
