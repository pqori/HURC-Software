#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
from nav_msgs.msg import Odometry
from geometry_msgs.msg import PoseWithCovarianceStamped, PoseStamped
from std_msgs.msg import String, Bool
from std_srvs.srv import SetBool
import time
from typing import Optional
from enum import Enum


class LocalizationSource(Enum):
    """Available localization sources"""
    EKF_LOCAL = "ekf_local"           # /odometry/filtered (local VIO + IMU)
    ZED_GNSS_FUSION = "zed_gnss"      # /camera/gnss/pose (global GNSS + VIO)
    FALLBACK = "fallback"             # Emergency fallback mode


class LocalizationSwitcher(Node):
    """Switch Nav2 localization source between local and global modes"""
    
    def __init__(self):
        super().__init__('localization_switcher')
        
        # Parameters
        self.declare_parameter('default_source', 'ekf_local')
        self.declare_parameter('auto_switch_enabled', True)
        self.declare_parameter('global_quality_threshold', 0.8)
        self.declare_parameter('switch_hysteresis', 3.0)
        self.declare_parameter('localization_timeout', 5.0)
        
        # Get parameters
        default_source = self.get_parameter('default_source').value
        self.auto_switch = self.get_parameter('auto_switch_enabled').value
        self.quality_threshold = self.get_parameter('global_quality_threshold').value
        self.switch_hysteresis = self.get_parameter('switch_hysteresis').value
        self.timeout = self.get_parameter('localization_timeout').value
        
        # State tracking
        self.current_source = LocalizationSource(default_source)
        self.last_switch_time = 0.0
        self.global_localization_quality = 0.0
        self.local_localization_available = False
        self.global_localization_available = False
        
        # Latest poses
        self.latest_local_odom: Optional[Odometry] = None
        self.latest_global_pose: Optional[PoseStamped] = None
        
        # Subscribers (monitor both sources)
        self.create_subscription(Odometry, '/odometry/filtered', self.cb_local_odom, 10)
        self.create_subscription(PoseStamped, '/camera/gnss/pose', self.cb_global_pose, 10)
        self.create_subscription(Bool, '/localization/global_ready', self.cb_global_ready, 10)
        self.create_subscription(String, '/localization/mode', self.cb_localization_mode, 10)
        
        # Publishers (Nav2 localization input)
        self.nav2_pose_pub = self.create_publisher(PoseWithCovarianceStamped, '/initialpose', 10)
        self.nav2_odom_pub = self.create_publisher(Odometry, '/nav2/localization/odom', 10)
        
        # Status publishers
        self.source_pub = self.create_publisher(String, '/navigation/localization_source', 10)
        self.switch_status_pub = self.create_publisher(String, '/navigation/localization_switch_status', 10)
        
        # Services
        self.create_service(SetBool, '/navigation/use_global_localization', self.use_global_localization)
        self.create_service(SetBool, '/navigation/use_local_localization', self.use_local_localization)
        
        # Monitoring and switching timer
        self.create_timer(1.0, self.monitor_and_switch_localization)
        
        self.get_logger().info("🧭 Nav2 Localization Switcher initialized")
        self.get_logger().info(f"   Default source: {self.current_source.value}")
        self.get_logger().info(f"   Auto-switching: {self.auto_switch}")
    
    def cb_local_odom(self, msg: Odometry):
        """Monitor local EKF odometry"""
        self.latest_local_odom = msg
        self.local_localization_available = True
        
        # If this is current source, republish for Nav2
        if self.current_source == LocalizationSource.EKF_LOCAL:
            self.publish_nav2_localization(msg)
    
    def cb_global_pose(self, msg: PoseStamped):
        """Monitor global ZED-GNSS pose"""
        self.latest_global_pose = msg
        self.global_localization_available = True
        
        # If this is current source, convert and republish for Nav2
        if self.current_source == LocalizationSource.ZED_GNSS_FUSION:
            self.publish_nav2_localization_from_pose(msg)
    
    def cb_global_ready(self, msg: Bool):
        """Monitor global localization readiness"""
        if msg.data:
            self.global_localization_quality = 0.9  # High quality when ready
        else:
            self.global_localization_quality = 0.0
    
    def cb_localization_mode(self, msg: String):
        """Monitor localization mode from global localization manager"""
        if msg.data == 'global_ready':
            self.global_localization_quality = 1.0
        elif msg.data == 'gnss_fusion':
            self.global_localization_quality = 0.7
        elif msg.data == 'gnss_lost':
            self.global_localization_quality = 0.3
        else:
            self.global_localization_quality = 0.0
    
    def monitor_and_switch_localization(self):
        """Monitor localization sources and switch if needed"""
        current_time = time.time()
        
        # Auto-switching logic
        if (self.auto_switch and 
            current_time - self.last_switch_time > self.switch_hysteresis):
            
            should_switch = False
            new_source = self.current_source
            
            # Switch to global if available and high quality
            if (self.current_source == LocalizationSource.EKF_LOCAL and
                self.global_localization_available and
                self.global_localization_quality > self.quality_threshold):
                
                new_source = LocalizationSource.ZED_GNSS_FUSION
                should_switch = True
                self.get_logger().info("🌍 Switching to global localization (GNSS fusion available)")
            
            # Switch back to local if global degrades
            elif (self.current_source == LocalizationSource.ZED_GNSS_FUSION and
                  self.global_localization_quality < self.quality_threshold - 0.2):  # Hysteresis
                
                new_source = LocalizationSource.EKF_LOCAL
                should_switch = True
                self.get_logger().info("📍 Switching to local localization (GNSS quality degraded)")
            
            # Execute switch
            if should_switch:
                self.switch_localization_source(new_source)
        
        # Publish status
        self.publish_localization_status()
    
    def switch_localization_source(self, new_source: LocalizationSource):
        """Switch active localization source"""
        previous_source = self.current_source
        self.current_source = new_source
        self.last_switch_time = time.time()
        
        # Publish switch status
        switch_msg = String()
        switch_msg.data = f"switched:{previous_source.value}→{new_source.value},time:{self.last_switch_time}"
        self.switch_status_pub.publish(switch_msg)
        
        self.get_logger().info(f"🔄 Localization source: {previous_source.value} → {new_source.value}")
    
    def publish_nav2_localization(self, odom_msg: Odometry):
        """Publish local odometry for Nav2"""
        # Convert Odometry to PoseWithCovarianceStamped for Nav2
        pose_msg = PoseWithCovarianceStamped()
        pose_msg.header = odom_msg.header
        pose_msg.header.frame_id = 'map'  # Nav2 expects map frame
        pose_msg.pose = odom_msg.pose
        
        # Republish odometry for Nav2
        nav2_odom = Odometry()
        nav2_odom.header = odom_msg.header
        nav2_odom.child_frame_id = 'base_link'
        nav2_odom.pose = odom_msg.pose
        nav2_odom.twist = odom_msg.twist
        
        self.nav2_odom_pub.publish(nav2_odom)
    
    def publish_nav2_localization_from_pose(self, pose_msg: PoseStamped):
        """Convert global pose to Nav2 localization format"""
        # Convert PoseStamped to PoseWithCovarianceStamped
        pose_with_cov = PoseWithCovarianceStamped()
        pose_with_cov.header = pose_msg.header
        pose_with_cov.header.frame_id = 'map'
        pose_with_cov.pose.pose = pose_msg.pose
        
        # Set reasonable covariance for global localization
        # Lower covariance = higher confidence
        pose_with_cov.pose.covariance[0] = 0.1   # x variance
        pose_with_cov.pose.covariance[7] = 0.1   # y variance
        pose_with_cov.pose.covariance[14] = 0.1  # z variance
        pose_with_cov.pose.covariance[21] = 0.05 # roll variance
        pose_with_cov.pose.covariance[28] = 0.05 # pitch variance
        pose_with_cov.pose.covariance[35] = 0.1  # yaw variance
        
        # Create odometry message for Nav2
        odom_msg = Odometry()
        odom_msg.header = pose_msg.header
        odom_msg.header.frame_id = 'map'
        odom_msg.child_frame_id = 'base_link'
        odom_msg.pose.pose = pose_msg.pose
        odom_msg.pose.covariance = pose_with_cov.pose.covariance
        
        self.nav2_odom_pub.publish(odom_msg)
    
    def publish_localization_status(self):
        """Publish current localization status"""
        source_msg = String()
        source_msg.data = f"source:{self.current_source.value},global_quality:{self.global_localization_quality:.2f},local_available:{self.local_localization_available},global_available:{self.global_localization_available}"
        self.source_pub.publish(source_msg)
    
    def use_global_localization(self, request, response):
        """Service to force global localization"""
        if request.data and self.global_localization_available:
            self.switch_localization_source(LocalizationSource.ZED_GNSS_FUSION)
            response.success = True
            response.message = "Switched to global localization"
        elif not request.data:
            self.switch_localization_source(LocalizationSource.EKF_LOCAL)
            response.success = True
            response.message = "Switched to local localization"
        else:
            response.success = False
            response.message = "Global localization not available"
        
        return response
    
    def use_local_localization(self, request, response):
        """Service to force local localization"""
        self.switch_localization_source(LocalizationSource.EKF_LOCAL)
        response.success = True
        response.message = "Switched to local localization"
        return response


def main(args=None):
    """Main entry point"""
    rclpy.init(args=args)
    
    try:
        switcher = LocalizationSwitcher()
        rclpy.spin(switcher)
    except KeyboardInterrupt:
        pass
    finally:
        rclpy.shutdown()


if __name__ == '__main__':
    main()
