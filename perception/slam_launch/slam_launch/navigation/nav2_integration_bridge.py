#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
from rclpy.action import ActionClient
from geometry_msgs.msg import PoseStamped
from nav2_msgs.action import NavigateToPose
from std_msgs.msg import String, Bool
from std_srvs.srv import Trigger
import time
from typing import Optional
from enum import Enum


class NavigationState(Enum):
    """Navigation system states"""
    IDLE = "idle"
    NAVIGATING_TO_OBJECT = "navigating_to_object"
    NAVIGATING_TO_GPS = "navigating_to_gps"
    APPROACHING_TARGET = "approaching_target"
    NAVIGATION_FAILED = "navigation_failed"


class Nav2IntegrationBridge(Node):
    """Bridge enhanced perception with Nav2 navigation"""
    
    def __init__(self):
        super().__init__('nav2_integration_bridge')
        
        # Parameters
        self.declare_parameter('nav2_timeout', 30.0)
        self.declare_parameter('approach_distance_tolerance', 0.2)
        self.declare_parameter('enable_auto_navigation', True)
        
        # Get parameters
        self.nav2_timeout = self.get_parameter('nav2_timeout').value
        self.approach_tolerance = self.get_parameter('approach_distance_tolerance').value
        self.auto_navigation = self.get_parameter('enable_auto_navigation').value
        
        # State tracking
        self.current_state = NavigationState.IDLE
        self.current_goal: Optional[PoseStamped] = None
        self.navigation_start_time = 0.0
        
        # Nav2 action client
        self.nav2_client = ActionClient(self, NavigateToPose, 'navigate_to_pose')
        
        # Subscribers for enhanced perception
        self.create_subscription(PoseStamped, '/target_pose', self.cb_target_pose, 10)
        self.create_subscription(PoseStamped, '/approach_pose', self.cb_approach_pose, 10)
        self.create_subscription(PoseStamped, '/navigation/waypoint_target', self.cb_gps_waypoint, 10)
        
        # Publishers
        self.nav_state_pub = self.create_publisher(String, '/navigation/state', 10)
        self.nav_status_pub = self.create_publisher(String, '/navigation/status', 10)
        self.goal_reached_pub = self.create_publisher(Bool, '/navigation/goal_reached', 10)
        
        # Services
        self.create_service(Trigger, '/navigation/cancel_navigation', self.cancel_navigation)
        self.create_service(SetBool, '/navigation/enable_auto_nav', self.enable_auto_navigation)
        
        # Status monitoring
        self.create_timer(1.0, self.monitor_navigation_state)
        
        self.get_logger().info("🌉 Nav2 Integration Bridge initialized")
        self.get_logger().info(f"   Auto navigation: {self.auto_navigation}")
        self.get_logger().info(f"   Nav2 timeout: {self.nav2_timeout}s")
    
    def cb_target_pose(self, msg: PoseStamped):
        """Handle object target poses from object selection"""
        if self.auto_navigation and self.current_state == NavigationState.IDLE:
            self.navigate_to_pose(msg, NavigationState.NAVIGATING_TO_OBJECT)
            self.get_logger().info(f"🎯 Auto-navigating to selected object")
    
    def cb_approach_pose(self, msg: PoseStamped):
        """Handle safe approach poses for object interaction"""
        if self.auto_navigation:
            self.navigate_to_pose(msg, NavigationState.APPROACHING_TARGET)
            self.get_logger().info(f"🎯 Navigating to safe approach position")
    
    def cb_gps_waypoint(self, msg: PoseStamped):
        """Handle GPS waypoint navigation"""
        if self.auto_navigation:
            self.navigate_to_pose(msg, NavigationState.NAVIGATING_TO_GPS)
            self.get_logger().info(f"🌍 Navigating to GPS waypoint")
    
    def navigate_to_pose(self, pose: PoseStamped, nav_state: NavigationState):
        """Send navigation goal to Nav2"""
        try:
            # Wait for Nav2 action server
            if not self.nav2_client.wait_for_server(timeout_sec=5.0):
                self.get_logger().error("❌ Nav2 action server not available")
                return
            
            # Create navigation goal
            goal_msg = NavigateToPose.Goal()
            goal_msg.pose = pose
            
            # Send goal
            future = self.nav2_client.send_goal_async(goal_msg)
            future.add_done_callback(self.nav2_goal_response_callback)
            
            # Update state
            self.current_state = nav_state
            self.current_goal = pose
            self.navigation_start_time = time.time()
            
            self.get_logger().info(f"📍 Navigation goal sent: ({pose.pose.position.x:.2f}, {pose.pose.position.y:.2f})")
            
        except Exception as e:
            self.get_logger().error(f"Navigation goal failed: {e}")
            self.current_state = NavigationState.NAVIGATION_FAILED
    
    def nav2_goal_response_callback(self, future):
        """Handle Nav2 goal response"""
        try:
            goal_handle = future.result()
            if not goal_handle.accepted:
                self.get_logger().error("❌ Nav2 goal rejected")
                self.current_state = NavigationState.NAVIGATION_FAILED
                return
            
            self.get_logger().info("✅ Nav2 goal accepted")
            
            # Get result
            result_future = goal_handle.get_result_async()
            result_future.add_done_callback(self.nav2_result_callback)
            
        except Exception as e:
            self.get_logger().error(f"Nav2 goal response error: {e}")
            self.current_state = NavigationState.NAVIGATION_FAILED
    
    def nav2_result_callback(self, future):
        """Handle Nav2 navigation result"""
        try:
            result = future.result().result
            
            if result:  # Success
                self.get_logger().info("🎉 Navigation goal reached")
                self.current_state = NavigationState.IDLE
                
                # Publish goal reached
                reached_msg = Bool()
                reached_msg.data = True
                self.goal_reached_pub.publish(reached_msg)
                
            else:  # Failed
                self.get_logger().error("❌ Navigation goal failed")
                self.current_state = NavigationState.NAVIGATION_FAILED
                
        except Exception as e:
            self.get_logger().error(f"Nav2 result error: {e}")
            self.current_state = NavigationState.NAVIGATION_FAILED
    
    def monitor_navigation_state(self):
        """Monitor navigation state and handle timeouts"""
        current_time = time.time()
        
        # Check for navigation timeout
        if (self.current_state != NavigationState.IDLE and 
            current_time - self.navigation_start_time > self.nav2_timeout):
            
            self.get_logger().error(f"⏰ Navigation timeout after {self.nav2_timeout}s")
            self.cancel_current_navigation()
            self.current_state = NavigationState.NAVIGATION_FAILED
        
        # Publish current state
        self.publish_navigation_status()
    
    def publish_navigation_status(self):
        """Publish navigation status"""
        # Current state
        state_msg = String()
        state_msg.data = self.current_state.value
        self.nav_state_pub.publish(state_msg)
        
        # Detailed status
        status_msg = String()
        if self.current_goal:
            elapsed = time.time() - self.navigation_start_time
            status_msg.data = f"state:{self.current_state.value},elapsed:{elapsed:.1f}s,goal:({self.current_goal.pose.position.x:.2f},{self.current_goal.pose.position.y:.2f})"
        else:
            status_msg.data = f"state:{self.current_state.value},goal:none"
        
        self.nav_status_pub.publish(status_msg)
    
    def cancel_current_navigation(self):
        """Cancel current navigation goal"""
        try:
            if self.nav2_client.server_is_ready():
                self.nav2_client.cancel_all_goals()
                self.get_logger().info("🛑 Navigation cancelled")
        except Exception as e:
            self.get_logger().error(f"Navigation cancellation failed: {e}")
    
    def cancel_navigation(self, request, response):
        """Service to cancel current navigation"""
        self.cancel_current_navigation()
        self.current_state = NavigationState.IDLE
        self.current_goal = None
        
        response.success = True
        response.message = "Navigation cancelled"
        return response
    
    def enable_auto_navigation(self, request, response):
        """Service to enable/disable automatic navigation"""
        self.auto_navigation = request.data
        
        response.success = True
        response.message = f"Auto navigation {'enabled' if request.data else 'disabled'}"
        self.get_logger().info(f"🤖 {response.message}")
        
        return response
    
    def enable_conflict_resolution(self, request, response):
        """Service to enable/disable topic conflict resolution"""
        self.enable_monitoring = request.data
        
        response.success = True
        response.message = f"Topic conflict resolution {'enabled' if request.data else 'disabled'}"
        return response


def main(args=None):
    """Main entry point"""
    rclpy.init(args=args)
    
    try:
        bridge = Nav2IntegrationBridge()
        rclpy.spin(bridge)
    except KeyboardInterrupt:
        pass
    finally:
        rclpy.shutdown()


if __name__ == '__main__':
    main()
