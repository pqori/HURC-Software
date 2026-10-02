# Localization Fusion (loc_fusion)

Multi-sensor localization system using robot_localization EKF for GPS + ZED camera fusion.

## What's Inside

**ekf_local** - Fuses ZED VIO + IMU for local odometry (`odom` → `base_link`)  
**navsat_transform** - Converts GPS lat/lon to local map coordinates  
**ekf_global** - Fuses local odometry + GPS for drift-free global pose (`map` → `odom`)  

## Published Topics

- `/odometry/filtered` - Local odometry from ZED VIO + IMU (for Nav2 local planning)
- `/odom/gps` - GPS position in odometry frame (intermediate)
- `/odometry/global` - Global odometry from GPS + local odom (for Nav2 global planning)

## What It Does

Combines multiple sensors to provide accurate, drift-free localization:

1. **ekf_local**: Fuses ZED camera visual-inertial odometry with ZED IMU data
   - Provides short-term accurate odometry
   - Publishes `odom` → `base_link` transform
   - Fast update rate (50Hz)

2. **navsat_transform**: Converts GPS coordinates to local map frame
   - Takes `/gps/fix` (latitude/longitude)
   - Outputs `/odom/gps` (x, y in meters)
   - Uses initial GPS reading as origin

3. **ekf_global**: Fuses local odometry with GPS for global consistency
   - Prevents long-term drift by anchoring to GPS
   - Publishes `map` → `odom` transform
   - Slower update rate (30Hz)

## Why We Need This

**Without Fusion:**
- ZED odometry drifts over time (accumulates error)
- GPS alone is too noisy and low-rate for navigation
- No consistent global reference frame

**With Fusion:**
- Best of both: ZED accuracy + GPS drift correction
- Smooth, high-rate odometry for Nav2
- Globally consistent for long missions
- Automatic recovery from GPS dropouts

## Usage

### Full Localization Stack
```bash
# Launch GPS + ZED + Fusion
ros2 launch loc_fusion loc_fusion.launch.py
```

This launches:
- GNSS driver (from gnss_launch)
- ZED 2i camera (from zed2i_launch)
- Both EKF nodes + navsat_transform

### Or via Full Perception Stack
```bash
~/workspaces/rover/src/perception/run_full_stack.sh
```

## Integration

**Subscribes to:**
- `/zed2i/odom` - ZED visual-inertial odometry
- `/zed2i/imu/data` - ZED IMU data
- `/gps/fix` - GPS position (from gnss_launch)

**Publishes:**
- `/odometry/filtered` - Used by Nav2 local costmap
- `/odometry/global` - Used by Nav2 global planner
- TF transforms: `map` → `odom` → `base_link`

**Used by:**
- Nav2 navigation stack
- Path planning algorithms
- Waypoint following
- Mission control

## Configuration Files

### `config/ekf.yaml`
Configures both EKF filters:
- **ekf_local**: Which axes to fuse from ZED odom and IMU
- **ekf_global**: How to combine local odom with GPS

Key parameters:
- `frequency`: Update rate
- `sensor_timeout`: How long before sensor is considered dead
- `two_d_mode`: false (we use 3D)
- `odom0_config`: Which state variables to use from each sensor

### `config/navsat_transform.yaml`
GPS coordinate conversion:
- `frequency`: 10Hz
- `zero_altitude`: true (ignore altitude, ground robot)
- `use_odometry_yaw`: true (use local odom heading, not magnetic)
- `wait_for_datum`: false (start immediately)

## Testing

```bash
# Check all topics are publishing
ros2 topic list | grep -E '(odometry|gps)'

# Monitor local odometry
ros2 topic hz /odometry/filtered
ros2 topic echo /odometry/filtered --once

# Monitor global odometry
ros2 topic hz /odometry/global

# Verify GPS is being fused
ros2 topic echo /odom/gps --once

# Check TF tree
ros2 run tf2_tools view_frames
# Opens: frames.pdf showing map→odom→base_link
```

## Verification Script

```bash
# Test localization stack
~/workspaces/rover/src/perception/loc_fusion/verify_localization.sh
```

Checks:
- All nodes are running
- Topics are publishing at expected rates
- TF transforms are being broadcast
- GPS fix quality

## Requirements

- **GPS**: gnss_launch must be running with valid GPS fix
- **ZED Camera**: zed2i_launch must be running
- **robot_localization**: ROS2 package (installed via apt)
- **Clear sky view** for GPS (outdoor testing)

## Troubleshooting

**No /odometry/filtered:**
- Check ZED camera is running: `ros2 topic hz /zed2i/odom`
- Check IMU data: `ros2 topic hz /zed2i/imu/data`
- Verify ekf_local is running: `ros2 node list | grep ekf_local`

**No /odometry/global:**
- Check GPS fix: `ros2 topic echo /gps/fix --once`
- Verify navsat_transform: `ros2 node list | grep navsat`
- Check if GPS has valid fix (status >= 1)

**Odometry drifting:**
- GPS may not be available (indoor/underground)
- Check GPS health: `ros2 topic echo /gnss/health_status`
- This is expected without GPS - use ekf_local only

**TF errors:**
- Both EKFs publish transforms - check for conflicts
- Verify `publish_tf: true` in both ekf configs
- Check TF tree: `ros2 run tf2_tools view_frames`

## Frame Hierarchy

```
map (global, GPS-anchored)
  └─ odom (local odometry frame)
      └─ base_link (robot body)
          └─ sensors (gps_link, zed2i_camera_center, etc.)
```

## Files

```
loc_fusion/
├── launch/
│   └── loc_fusion.launch.py      # Main launch file
├── config/
│   ├── ekf.yaml                  # EKF configurations
│   └── navsat_transform.yaml     # GPS transform config
├── verify_localization.sh        # Testing script
├── CMakeLists.txt
└── package.xml
```

## For Nav2

This package provides the **localization foundation** for Nav2:
- `/odometry/filtered` feeds into local costmap for precise local planning
- `/odometry/global` used by global planner for waypoint navigation
- TF tree allows Nav2 to transform between coordinate frames
- GPS integration enables lat/lon waypoint navigation

