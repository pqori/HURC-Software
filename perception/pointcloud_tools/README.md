# Point Cloud Tools

Converts ZED camera 3D point cloud into 2D representations for Nav2 navigation.

## What's Inside

**depth_to_scan** - Converts 3D point cloud → 2D LaserScan  
**grid_builder** - Converts 3D point cloud → 2D occupancy grid  

## Published Topics

- `/scan` (sensor_msgs/LaserScan) - Virtual 2D laser scan for Nav2 obstacle avoidance
- `/local_map` (nav_msgs/OccupancyGrid) - 2D occupancy grid for Nav2 costmaps

## What It Does

Takes the ZED camera's 3D point cloud (`/zed2i/zed2i_camera/point_cloud/cloud_registered`) and creates 2D slices that Nav2 can understand:

1. **depth_to_scan**: Slices a horizontal plane from the point cloud at a specific height (default 0.3m) and converts it to a 360° laser scan. This mimics a traditional 2D LIDAR.

2. **grid_builder**: Projects points above a threshold (default 0.15m) onto a 2D grid to mark obstacles. Creates an occupancy grid for Nav2 costmaps.

## Why We Need This

Nav2 expects 2D sensor data (LaserScan or OccupancyGrid), but the ZED camera provides 3D point clouds. These tools bridge that gap, allowing Nav2 to use ZED depth data for:
- Obstacle avoidance (local planner)
- Costmap updates
- Dynamic obstacle detection

## Usage

Launch as part of perception stack:
```bash
~/workspaces/rover/src/perception/run_full_stack.sh
```

Or launch individually:
```bash
ros2 run pointcloud_tools depth_to_scan
ros2 run pointcloud_tools grid_builder
```

## Configuration

Edit `config/pointcloud_params.yaml` to adjust:
- Scan height and range
- Grid resolution
- FOV and angular resolution
- Z-axis thresholds for obstacles

## For Nav2

Both topics feed directly into Nav2:
- `/scan` → local_costmap → obstacle_layer (for real-time obstacle avoidance)
- `/local_map` → global_costmap or additional layers (for path planning)

Nav2 can use both representations:
- LaserScan is great for dynamic obstacles and local navigation
- OccupancyGrid is useful for longer-range planning and static obstacles

## Quick Test

```bash
# Check topics exist
ros2 topic list | grep -E '(scan|local_map)'

# View scan data
ros2 topic echo /scan --once

# Visualize in RViz
rviz2
# Add: LaserScan → /scan
# Add: Map → /local_map
```

## Parameters

**depth_to_scan:**
- `frame_id`: base_link
- `range_max`: 12.0m (max detection range)
- `fov_deg`: 120° (field of view)
- `scan_height`: 0.30m (height slice)
- `min_z/max_z`: -0.1m to 0.3m (vertical filter range)
- `angular_res_deg`: 1.0° (beam spacing)

**grid_builder:**
- `resolution`: 0.05m (5cm cells)
- `size_xy`: 20.0m (20x20m grid)
- `z_thresh`: 0.15m (obstacle height threshold)
- `range_max`: 15.0m (max mapping range)

