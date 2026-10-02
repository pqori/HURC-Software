#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
from sensor_msgs.msg import PointCloud2
from nav_msgs.msg import OccupancyGrid
from std_msgs.msg import String, Float32, Bool
from std_srvs.srv import SetBool
import sensor_msgs_py.point_cloud2 as pc2
import numpy as np
import time
from typing import Optional
from enum import Enum


class MappingMode(Enum):
    """Mapping system modes"""
    RTABMAP_ONLY = "rtabmap_only"
    ZED_PRIMARY = "zed_primary" 
    HYBRID_FUSION = "hybrid_fusion"
    AUTO_SELECT = "auto_select"


class MappingCoordinator(Node):
    """Coordinate ZED GPU mapping with RTAB-Map for optimal performance"""
    
    def __init__(self):
        super().__init__('mapping_coordinator')
        
        # Parameters
        self.declare_parameter('mapping_mode', 'hybrid_fusion')
        self.declare_parameter('zed_weight', 0.7)
        self.declare_parameter('rtabmap_weight', 0.3)
        self.declare_parameter('performance_threshold', 0.8)
        self.declare_parameter('switch_hysteresis', 2.0)
        self.declare_parameter('grid_resolution', 0.05)
        self.declare_parameter('grid_size', 2048)
        
        # Get parameters
        mode_str = self.get_parameter('mapping_mode').value
        self.current_mode = MappingMode(mode_str)
        self.zed_weight = self.get_parameter('zed_weight').value
        self.rtabmap_weight = self.get_parameter('rtabmap_weight').value
        self.perf_threshold = self.get_parameter('performance_threshold').value
        self.switch_hysteresis = self.get_parameter('switch_hysteresis').value
        self.resolution = self.get_parameter('grid_resolution').value
        self.grid_size = self.get_parameter('grid_size').value
        
        # State tracking
        self.zed_cloud: Optional[PointCloud2] = None
        self.rtabmap_grid: Optional[OccupancyGrid] = None
        self.zed_performance = 0.0
        self.rtabmap_performance = 0.0
        self.last_switch_time = 0.0
        
        # Subscribers
        self.create_subscription(PointCloud2, '/camera/mapping/fused_cloud', self.cb_zed_cloud, 5)
        self.create_subscription(OccupancyGrid, '/rtabmap/grid_map', self.cb_rtabmap_grid, 5)
        
        # Publishers
        self.unified_map_pub = self.create_publisher(OccupancyGrid, '/map', 1)
        self.mode_pub = self.create_publisher(String, '/mapping/active_mode', 10)
        self.performance_pub = self.create_publisher(String, '/mapping/performance', 10)
        
        # Services
        self.create_service(SetBool, '/mapping/enable_hybrid', self.enable_hybrid_mapping)
        
        # Processing timer
        self.create_timer(0.5, self.process_and_publish_map)  # 2 Hz unified map
        self.create_timer(2.0, self.monitor_performance)      # Performance monitoring
        
        self.get_logger().info(f"🗺️ Hybrid Mapping Coordinator initialized")
        self.get_logger().info(f"   Mode: {self.current_mode.value}")
        self.get_logger().info(f"   ZED weight: {self.zed_weight}")
        self.get_logger().info(f"   RTAB-Map weight: {self.rtabmap_weight}")
    
    def cb_zed_cloud(self, msg: PointCloud2):
        """Handle ZED fused point cloud"""
        self.zed_cloud = msg
        self.zed_performance = self.assess_zed_performance(msg)
    
    def cb_rtabmap_grid(self, msg: OccupancyGrid):
        """Handle RTAB-Map occupancy grid"""
        self.rtabmap_grid = msg
        self.rtabmap_performance = self.assess_rtabmap_performance(msg)
    
    def process_and_publish_map(self):
        """Process maps and publish unified result"""
        try:
            unified_map = self.create_unified_map()
            if unified_map:
                self.unified_map_pub.publish(unified_map)
                self.get_logger().debug(f"Published unified map ({self.current_mode.value})")
        except Exception as e:
            self.get_logger().error(f"Map processing failed: {e}")
    
    def create_unified_map(self) -> Optional[OccupancyGrid]:
        """Create unified occupancy grid based on current mode"""
        if self.current_mode == MappingMode.RTABMAP_ONLY:
            return self.rtabmap_grid
        
        elif self.current_mode == MappingMode.ZED_PRIMARY:
            if self.zed_cloud:
                return self.convert_cloud_to_grid(self.zed_cloud)
            else:
                return self.rtabmap_grid  # Fallback
        
        elif self.current_mode == MappingMode.HYBRID_FUSION:
            return self.fuse_maps()
        
        elif self.current_mode == MappingMode.AUTO_SELECT:
            return self.auto_select_best_map()
        
        return self.rtabmap_grid  # Safe fallback
    
    def fuse_maps(self) -> Optional[OccupancyGrid]:
        """Intelligently fuse ZED and RTAB-Map data"""
        if not self.zed_cloud or not self.rtabmap_grid:
            # Use whichever is available
            if self.zed_cloud:
                return self.convert_cloud_to_grid(self.zed_cloud)
            else:
                return self.rtabmap_grid
        
        try:
            # Convert ZED cloud to grid
            zed_grid = self.convert_cloud_to_grid(self.zed_cloud)
            if not zed_grid:
                return self.rtabmap_grid
            
            # Fuse grids with weighted combination
            fused_grid = self.blend_occupancy_grids(zed_grid, self.rtabmap_grid)
            return fused_grid
            
        except Exception as e:
            self.get_logger().error(f"Map fusion failed: {e}")
            return self.rtabmap_grid  # Safe fallback
    
    def convert_cloud_to_grid(self, cloud: PointCloud2) -> Optional[OccupancyGrid]:
        """Convert ZED point cloud to occupancy grid"""
        try:
            grid = OccupancyGrid()
            grid.header = cloud.header
            grid.header.frame_id = 'map'
            
            # Grid metadata
            grid.info.resolution = self.resolution
            grid.info.width = self.grid_size
            grid.info.height = self.grid_size
            grid.info.origin.position.x = -self.grid_size * self.resolution / 2.0
            grid.info.origin.position.y = -self.grid_size * self.resolution / 2.0
            grid.info.origin.orientation.w = 1.0
            
            # Initialize grid (unknown = -1)
            grid_data = np.full(self.grid_size * self.grid_size, -1, dtype=np.int8)
            
            # Process point cloud
            points = list(pc2.read_points(cloud, skip_nans=True, field_names=("x", "y", "z")))
            
            for x, y, z in points:
                grid_x = int((x - grid.info.origin.position.x) / self.resolution)
                grid_y = int((y - grid.info.origin.position.y) / self.resolution)
                
                if 0 <= grid_x < self.grid_size and 0 <= grid_y < self.grid_size:
                    grid_idx = grid_y * self.grid_size + grid_x
                    
                    if z > 0.3:  # Obstacle height threshold
                        grid_data[grid_idx] = 100  # Occupied
                    elif grid_data[grid_idx] == -1:
                        grid_data[grid_idx] = 0   # Free
            
            grid.data = grid_data.tolist()
            return grid
            
        except Exception as e:
            self.get_logger().error(f"Cloud to grid conversion failed: {e}")
            return None
    
    def blend_occupancy_grids(self, zed_grid: OccupancyGrid, rtabmap_grid: OccupancyGrid) -> OccupancyGrid:
        """Blend two occupancy grids with confidence weighting"""
        # Use ZED grid as base (higher resolution, real-time)
        fused_grid = zed_grid
        
        # Blend data where both have information
        zed_data = np.array(zed_grid.data)
        rtabmap_data = np.array(rtabmap_grid.data)
        
        # Simple fusion: prefer occupied cells from either source
        for i in range(len(zed_data)):
            if rtabmap_data[i] == 100:  # RTAB-Map sees obstacle
                fused_grid.data[i] = 100
            elif zed_data[i] == 100:    # ZED sees obstacle
                fused_grid.data[i] = 100
            elif rtabmap_data[i] == 0 and zed_data[i] == 0:  # Both see free
                fused_grid.data[i] = 0
            # Keep unknown (-1) if no consensus
        
        return fused_grid
    
    def auto_select_best_map(self) -> Optional[OccupancyGrid]:
        """Automatically select best mapping source"""
        if self.zed_performance > self.rtabmap_performance + 0.2:
            return self.convert_cloud_to_grid(self.zed_cloud) if self.zed_cloud else self.rtabmap_grid
        elif self.rtabmap_performance > self.zed_performance + 0.2:
            return self.rtabmap_grid
        else:
            return self.fuse_maps()  # Similar performance, fuse them
    
    def assess_zed_performance(self, cloud: PointCloud2) -> float:
        """Assess ZED mapping performance"""
        try:
            # Performance based on point density and update rate
            point_count = len(cloud.data) // (cloud.point_step or 16)
            
            # Normalize performance (more points = better)
            density_score = min(1.0, point_count / 50000.0)
            
            # Check update freshness
            current_time = time.time()
            if hasattr(self, '_last_zed_time'):
                time_diff = current_time - self._last_zed_time
                update_score = min(1.0, 2.0 / max(0.1, time_diff))  # Target 2 Hz
            else:
                update_score = 0.5
            
            self._last_zed_time = current_time
            
            return (density_score + update_score) / 2.0
            
        except Exception:
            return 0.0
    
    def assess_rtabmap_performance(self, grid: OccupancyGrid) -> float:
        """Assess RTAB-Map performance"""
        try:
            # Performance based on map coverage and consistency
            occupied_cells = sum(1 for cell in grid.data if cell == 100)
            free_cells = sum(1 for cell in grid.data if cell == 0)
            total_known = occupied_cells + free_cells
            
            coverage_score = min(1.0, total_known / (len(grid.data) * 0.3))  # 30% coverage target
            
            # Check update freshness
            current_time = time.time()
            if hasattr(self, '_last_rtabmap_time'):
                time_diff = current_time - self._last_rtabmap_time
                update_score = min(1.0, 5.0 / max(0.1, time_diff))  # Target 0.2 Hz
            else:
                update_score = 0.5
            
            self._last_rtabmap_time = current_time
            
            return (coverage_score + update_score) / 2.0
            
        except Exception:
            return 0.0
    
    def monitor_performance(self):
        """Monitor and potentially switch mapping systems"""
        current_time = time.time()
        
        # Auto-switch logic for AUTO_SELECT mode
        if (self.current_mode == MappingMode.AUTO_SELECT and 
            current_time - self.last_switch_time > self.switch_hysteresis):
            
            if self.zed_performance > self.perf_threshold and self.zed_performance > self.rtabmap_performance + 0.2:
                self.current_mode = MappingMode.ZED_PRIMARY
                self.last_switch_time = current_time
                self.get_logger().info("🚀 Switched to ZED primary mapping")
            
            elif self.rtabmap_performance > self.perf_threshold:
                self.current_mode = MappingMode.RTABMAP_ONLY
                self.last_switch_time = current_time
                self.get_logger().info("🗺️ Switched to RTAB-Map mapping")
        
        # Publish status
        self.publish_status()
    
    def publish_status(self):
        """Publish mapping status and performance"""
        # Current mode
        mode_msg = String()
        mode_msg.data = self.current_mode.value
        self.mode_pub.publish(mode_msg)
        
        # Performance metrics
        perf_msg = String()
        perf_msg.data = f"zed:{self.zed_performance:.2f},rtabmap:{self.rtabmap_performance:.2f},mode:{self.current_mode.value}"
        self.performance_pub.publish(perf_msg)
    
    def enable_hybrid_mapping(self, request, response):
        """Service to enable/disable hybrid mapping"""
        if request.data:
            self.current_mode = MappingMode.HYBRID_FUSION
            response.success = True
            response.message = "Hybrid mapping enabled - fusing ZED + RTAB-Map"
            self.get_logger().info("🔗 Hybrid mapping enabled")
        else:
            self.current_mode = MappingMode.RTABMAP_ONLY
            response.success = True
            response.message = "Hybrid mapping disabled - RTAB-Map only"
            self.get_logger().info("🗺️ Switched to RTAB-Map only")
        
        return response


def main(args=None):
    """Main entry point"""
    rclpy.init(args=args)
    
    try:
        coordinator = MappingCoordinator()
        rclpy.spin(coordinator)
    except KeyboardInterrupt:
        pass
    finally:
        rclpy.shutdown()


if __name__ == '__main__':
    main()
