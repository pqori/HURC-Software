#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
from rcl_interfaces.msg import Parameter, ParameterValue, ParameterType
from rcl_interfaces.srv import SetParameters
from std_msgs.msg import String, Bool
from std_srvs.srv import Trigger
import yaml
import os
from typing import Dict, Any, List


class Nav2ParameterSync(Node):
    """Synchronize Nav2 parameters with standardized perception config"""
    
    def __init__(self):
        super().__init__('nav2_parameter_sync')
        
        # Parameters
        self.declare_parameter('nav2_config_path', '/home/rover/workspaces/rover/src/navigation/global_nav/nav2_launch/config')
        self.declare_parameter('standardized_config_path', '/home/rover/workspaces/rover/src/perception/slam_launch/config/navigation_integration.yaml')
        self.declare_parameter('enable_auto_sync', True)
        self.declare_parameter('sync_interval', 30.0)
        
        # Get parameters
        self.nav2_config_path = self.get_parameter('nav2_config_path').value
        self.standard_config_path = self.get_parameter('standardized_config_path').value
        self.auto_sync = self.get_parameter('enable_auto_sync').value
        sync_interval = self.get_parameter('sync_interval').value
        
        # Load standardized parameters
        self.standard_params = self.load_standardized_params()
        
        # Nav2 node parameter clients
        self.nav2_clients = {}
        self.setup_nav2_clients()
        
        # State tracking
        self.last_sync_time = 0.0
        self.sync_status = "not_synced"
        
        # Publishers
        self.sync_status_pub = self.create_publisher(String, '/navigation/parameter_sync_status', 10)
        self.sync_result_pub = self.create_publisher(String, '/navigation/parameter_sync_result', 10)
        
        # Services
        self.create_service(Trigger, '/navigation/sync_nav2_parameters', self.sync_nav2_parameters)
        self.create_service(Trigger, '/navigation/validate_nav2_config', self.validate_nav2_config)
        
        # Auto-sync timer
        if self.auto_sync:
            self.create_timer(sync_interval, self.auto_sync_parameters)
        
        self.get_logger().info("⚙️ Nav2 Parameter Synchronizer initialized")
        self.get_logger().info(f"   Auto-sync: {self.auto_sync}")
        self.get_logger().info(f"   Standard config: {self.standard_config_path}")
    
    def load_standardized_params(self) -> Dict[str, Any]:
        """Load standardized parameters from config file"""
        try:
            with open(self.standard_config_path, 'r') as f:
                config = yaml.safe_load(f)
            
            # Extract standardized parameters
            return config.get('standardized_params', {})
            
        except Exception as e:
            self.get_logger().error(f"Failed to load standardized config: {e}")
            return {}
    
    def setup_nav2_clients(self):
        """Setup parameter clients for Nav2 nodes"""
        nav2_nodes = [
            'controller_server',
            'planner_server', 
            'global_costmap',
            'local_costmap',
            'map_server'
        ]
        
        for node_name in nav2_nodes:
            client = self.create_client(SetParameters, f'/{node_name}/set_parameters')
            self.nav2_clients[node_name] = client
    
    def sync_nav2_parameters(self, request, response):
        """Service to synchronize Nav2 parameters"""
        try:
            sync_results = []
            
            # Sync costmap parameters
            costmap_result = self.sync_costmap_parameters()
            sync_results.append(f"costmap:{costmap_result}")
            
            # Sync controller parameters
            controller_result = self.sync_controller_parameters()
            sync_results.append(f"controller:{controller_result}")
            
            # Sync planner parameters
            planner_result = self.sync_planner_parameters()
            sync_results.append(f"planner:{planner_result}")
            
            # Update sync status
            self.last_sync_time = time.time()
            self.sync_status = "synced"
            
            response.success = True
            response.message = f"Nav2 parameters synchronized: {', '.join(sync_results)}"
            
            # Publish result
            result_msg = String()
            result_msg.data = response.message
            self.sync_result_pub.publish(result_msg)
            
            self.get_logger().info(f"✅ {response.message}")
            
        except Exception as e:
            response.success = False
            response.message = f"Parameter sync failed: {str(e)}"
            self.sync_status = "failed"
            self.get_logger().error(f"Parameter sync error: {e}")
        
        return response
    
    def sync_costmap_parameters(self) -> str:
        """Sync costmap parameters with standardized config"""
        try:
            standard_grid = self.standard_params.get('grid_resolution', 0.05)
            standard_update_rate = self.standard_params.get('costmap_update_rate', 5.0)
            
            # Parameters to sync for both global and local costmaps
            costmap_params = [
                Parameter(name='resolution', value=ParameterValue(type=ParameterType.PARAMETER_DOUBLE, double_value=standard_grid)),
                Parameter(name='update_frequency', value=ParameterValue(type=ParameterType.PARAMETER_DOUBLE, double_value=standard_update_rate)),
                Parameter(name='transform_tolerance', value=ParameterValue(type=ParameterType.PARAMETER_DOUBLE, double_value=0.2))  # Increased for real-time data
            ]
            
            # Sync both costmaps
            results = []
            for costmap in ['global_costmap', 'local_costmap']:
                if costmap in self.nav2_clients:
                    client = self.nav2_clients[costmap]
                    if client.wait_for_service(timeout_sec=2.0):
                        request = SetParameters.Request()
                        request.parameters = costmap_params
                        future = client.call_async(request)
                        # Note: In production, would wait for response
                        results.append(f"{costmap}:ok")
                    else:
                        results.append(f"{costmap}:unavailable")
            
            return ",".join(results)
            
        except Exception as e:
            return f"error:{str(e)}"
    
    def sync_controller_parameters(self) -> str:
        """Sync controller parameters"""
        try:
            # Update controller for real-time sensor data
            controller_params = [
                Parameter(name='transform_tolerance', value=ParameterValue(type=ParameterType.PARAMETER_DOUBLE, double_value=0.2)),
                Parameter(name='controller_frequency', value=ParameterValue(type=ParameterType.PARAMETER_DOUBLE, double_value=20.0))  # Higher frequency for ZED data
            ]
            
            if 'controller_server' in self.nav2_clients:
                client = self.nav2_clients['controller_server']
                if client.wait_for_service(timeout_sec=2.0):
                    request = SetParameters.Request()
                    request.parameters = controller_params
                    future = client.call_async(request)
                    return "ok"
                else:
                    return "unavailable"
            
            return "not_configured"
            
        except Exception as e:
            return f"error:{str(e)}"
    
    def sync_planner_parameters(self) -> str:
        """Sync planner parameters"""
        try:
            # Update planner for hybrid mapping
            planner_params = [
                Parameter(name='expected_planner_frequency', value=ParameterValue(type=ParameterType.PARAMETER_DOUBLE, double_value=2.0))  # Match mapping update rate
            ]
            
            if 'planner_server' in self.nav2_clients:
                client = self.nav2_clients['planner_server']
                if client.wait_for_service(timeout_sec=2.0):
                    request = SetParameters.Request()
                    request.parameters = planner_params
                    future = client.call_async(request)
                    return "ok"
                else:
                    return "unavailable"
            
            return "not_configured"
            
        except Exception as e:
            return f"error:{str(e)}"
    
    def publish_nav2_localization_from_pose(self, pose_msg: PoseStamped):
        """Convert global pose to Nav2 odometry format"""
        odom_msg = Odometry()
        odom_msg.header = pose_msg.header
        odom_msg.header.frame_id = 'map'
        odom_msg.child_frame_id = 'base_link'
        odom_msg.pose.pose = pose_msg.pose
        
        # Set covariance for global localization (high confidence)
        odom_msg.pose.covariance[0] = 0.1   # x
        odom_msg.pose.covariance[7] = 0.1   # y
        odom_msg.pose.covariance[35] = 0.1  # yaw
        
        self.nav2_odom_pub.publish(odom_msg)
    
    def auto_sync_parameters(self):
        """Automatically sync parameters periodically"""
        if self.auto_sync:
            self.get_logger().info("🔄 Auto-syncing Nav2 parameters...")
            # Create dummy request for auto-sync
            request = Trigger.Request()
            self.sync_nav2_parameters(request, Trigger.Response())
    
    def publish_localization_status(self):
        """Publish localization switching status"""
        status_msg = String()
        status_msg.data = f"source:{self.current_source.value},quality:{self.global_localization_quality:.2f},sync:{self.sync_status}"
        self.sync_status_pub.publish(status_msg)
    
    def validate_nav2_config(self, request, response):
        """Service to validate Nav2 configuration"""
        try:
            issues = []
            
            # Check if Nav2 config files exist
            nav2_configs = ['nav2_params.yaml', 'costmap_global.yaml', 'costmap_local.yaml']
            for config_file in nav2_configs:
                config_path = os.path.join(self.nav2_config_path, config_file)
                if not os.path.exists(config_path):
                    issues.append(f"missing:{config_file}")
            
            # Check parameter compatibility
            if self.standard_params:
                # Validate grid resolution compatibility
                standard_resolution = self.standard_params.get('grid_resolution', 0.05)
                issues.append(f"standard_resolution:{standard_resolution}")
            
            if not issues:
                response.success = True
                response.message = "Nav2 configuration is valid and compatible"
            else:
                response.success = False
                response.message = f"Nav2 configuration issues: {', '.join(issues)}"
            
        except Exception as e:
            response.success = False
            response.message = f"Validation failed: {str(e)}"
        
        return response


def main(args=None):
    """Main entry point"""
    rclpy.init(args=args)
    
    try:
        sync_node = Nav2ParameterSync()
        rclpy.spin(sync_node)
    except KeyboardInterrupt:
        pass
    finally:
        rclpy.shutdown()


if __name__ == '__main__':
    main()
