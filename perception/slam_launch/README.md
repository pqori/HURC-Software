# URC Complete Perception System

## 🚀 Quick Start

### Production Mode (Complete System)
```bash
# Launch everything for mission operations
ros2 launch slam_launch perception_complete.launch.py

# With data recording
ros2 launch slam_launch perception_complete.launch.py enable_recording:=true

# Selective components
ros2 launch slam_launch perception_complete.launch.py \
    enable_object_detection:=true \
    enable_aruco_detection:=false \
    enable_pose_estimation:=true
```

### Development Mode (Testing)
```bash
# Basic SLAM only
ros2 launch slam_launch perception_dev.launch.py

# Test object detection
ros2 launch slam_launch perception_dev.launch.py enable_object_detection:=true

# Debug mode with verbose logging
ros2 launch slam_launch perception_dev.launch.py debug_mode:=true
```

## 📋 System Components

### Core Stack (Always Active)
- **ZED2i Camera**: Stereo vision, depth, IMU
- **GNSS**: GPS positioning
- **Localization**: Dual EKF sensor fusion
- **SLAM**: RTAB-Map RGB-D mapping
- **Pointcloud Tools**: 2D scan + occupancy grid

### Detection Components (Optional)
- **Object Detection**: YOLO-based URC object detection with multi-object tracking
- **ArUco Detection**: Fiducial marker detection
- **Multi-Object Tracking**: Persistent object IDs and pose smoothing
- **Semantic Mapping**: Object persistence in map coordinates
- **Sensor Fusion**: Multi-modal detection fusion (YOLO + ArUco)

### System Utilities
- **Health Monitor**: Real-time component monitoring
- **Status Publisher**: System status for mission control
- **Data Recorder**: Automatic rosbag recording
- **Perception Guardian**: Robustness and error handling with graceful degradation
- **Calibration Validator**: Automated sensor calibration validation and quality assessment

## 📊 Monitoring & Diagnostics

### System Status
```bash
# View system status
ros2 topic echo /perception/status

# View detailed diagnostics
ros2 topic echo /perception/diagnostics

# Monitor specific topics
ros2 topic hz /detected_objects
ros2 topic hz /map
```

### Health Dashboard
The system publishes comprehensive health information:
- Component status (OK/WARN/ERROR)
- Topic monitoring with timeouts
- System uptime and performance metrics
- Automatic failure detection and alerts

## 🎛️ Configuration

### Launch Arguments

| Argument | Default | Description |
|----------|---------|-------------|
| `enable_object_detection` | `true` | Enable YOLO object detection |
| `enable_aruco_detection` | `true` | Enable ArUco marker detection |
| `enable_pose_estimation` | `true` | Enable multi-object tracking |
| `enable_semantic_mapping` | `true` | Enable semantic mapping |
| `enable_sensor_fusion` | `true` | Enable sensor fusion |
| `enable_perception_guardian` | `true` | Enable robustness and error handling |
| `enable_calibration_validation` | `true` | Enable calibration validation |
| `enable_recording` | `false` | Enable data recording |
| `enable_monitoring` | `true` | Enable health monitoring |
| `debug_mode` | `false` | Enable debug logging |

### Configuration Files

- `config/monitoring_config.yaml` - Health monitoring settings
- `config/recording_config.yaml` - Data recording configuration
- `config/robustness_config.yaml` - Robustness and calibration settings
- `config/rtabmap_params.yaml` - SLAM parameters

## 💾 Data Recording

### Automatic Recording
```bash
# Record all perception data
ros2 launch slam_launch perception_complete.launch.py enable_recording:=true

# Data saved to: /home/rover/perception_data/
# Format: urc_perception_YYYYMMDD_HHMMSS.bag
```

### Manual Recording
```bash
# Record specific topics
ros2 bag record /zed2i/left/image_rect_color /detected_objects /map

# Playback recorded data
ros2 bag play urc_perception_20241225_143022.bag
```

## 🔧 Development & Testing

### Component Testing
```bash
# Test individual components
ros2 launch slam_launch perception_dev.launch.py enable_object_detection:=true
ros2 launch slam_launch perception_dev.launch.py enable_aruco_detection:=true

# Test with recorded data
ros2 bag play test_data.bag &
ros2 launch slam_launch perception_dev.launch.py
```

### Performance Monitoring
```bash
# Check system performance
ros2 run slam_launch system_status.py

# Monitor topic rates
ros2 topic hz /zed2i/left/image_rect_color
ros2 topic hz /detected_objects
```

## 🚨 Troubleshooting

### Common Issues

**No detections appearing:**
```bash
# Check object detection is enabled
ros2 param get /urc_object_detection enable_object_detection

# Verify camera feed
ros2 topic hz /zed2i/left/image_rect_color

# Check detection confidence
ros2 param set /urc_object_detection confidence_threshold 0.3
```

**SLAM not working:**
```bash
# Check ZED2i camera
ros2 topic list | grep zed2i

# Verify IMU data
ros2 topic hz /zed2i/imu/data

# Check odometry
ros2 topic echo /odometry/filtered --once
```

**System health warnings:**
```bash
# View diagnostics
ros2 topic echo /perception/diagnostics

# Check component status
ros2 topic echo /perception/status
```

### Debug Mode
```bash
# Enable verbose logging
ros2 launch slam_launch perception_dev.launch.py debug_mode:=true

# View logs
ros2 log view
```

## 📁 File Structure

```
slam_launch/
├── launch/
│   ├── slam.launch.py                    # Basic SLAM (original)
│   ├── perception_complete.launch.py     # Complete system
│   └── perception_dev.launch.py          # Development mode
├── config/
│   ├── rtabmap_params.yaml              # SLAM configuration
│   ├── monitoring_config.yaml           # Health monitoring
│   └── recording_config.yaml            # Data recording
├── scripts/
│   ├── perception_health_monitor.py     # Health monitoring
│   └── system_status.py                 # Status publishing
└── slam_launch/                         # Python package
    ├── __init__.py
    ├── perception_health_monitor.py
    └── system_status.py
```

## 🎯 Mission Integration

This perception system provides everything needed for URC autonomous missions:

- **Science Task**: Object detection for sample identification
- **Equipment Retrieval**: Detection and pose estimation for tools/containers
- **Autonomous Navigation**: SLAM mapping and obstacle detection
- **System Monitoring**: Real-time health monitoring for reliability

The system is designed to be robust, modular, and mission-ready with comprehensive monitoring and data collection capabilities.
