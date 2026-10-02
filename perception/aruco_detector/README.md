# ArUco Detector – Production ROS 2 Package

**Version 1.0** – Elegant "Shared Core + Dual Interface" architecture for Jetson-based rovers

## Quick Start

```bash
# Full system test (ZED camera + ArUco detector + monitor)
cd /home/rover/workspaces/rover/src/perception/aruco_detector/scripts
./test_full_system.sh

# Or monitor detections separately
cd /home/rover/workspaces/rover
# Clean up old processes first
pkill -f "zed2i_driver\|aruco_detector\|standalone_detector" 2>/dev/null
unset CYCLONEDDS_URI && export ROS_LOCALHOST_ONLY=1
python3 src/perception/aruco_detector/scripts/monitor_detections.py
```

---

## Overview

Production-grade ArUco marker detector designed for rover navigation with:

- **Pure detection engine** (testable, reusable)
- **Dual interfaces**: ROS 2 node (mission) + Standalone script (development)
- **Concurrent processing**: Non-blocking async detection
- **3D tracking**: ZED SDK depth integration for accurate distance
- **Composable node**: Optimized for resource sharing on Jetson
- **Resilient design**: Handles missing depth, TF failures gracefully

---

## Architecture: Separation of Concerns

```
┌─────────────────────────────────────────────────┐
│         Core Detection Engine (core.py)         │
│  • Pure ArUco detection (no ROS/ZED deps)       │
│  • Sync + async modes with thread-safe queue    │
│  • Deterministic, unit-testable                 │
└──────────────┬──────────────────┬────────────────┘
               │                  │
       ┌───────▼────────┐  ┌──────▼──────────┐
       │   ROS Node     │  │   Standalone    │
       │  (Production)  │  │   (Testing)     │
       ├────────────────┤  ├─────────────────┤
       │ • Composable   │  │ • Direct ZED    │
       │ • QoS tuned    │  │   SDK access    │
       │ • Time sync    │  │ • 2D + 3D viz   │
       │ • TF2 support  │  │ • FPS monitor   │
       │ • Diagnostics  │  │ • Snapshots     │
       └────────────────┘  └─────────────────┘
```

### Components

1. **`core.py`**: Pure detection logic
   - Accepts: image, camera_matrix, dist_coeffs, depth_sampler
   - Returns: list of detections with ID, pose, range, error
   - Modes: `detect()` (sync) or `detect_async()` (non-blocking)

2. **`zed_interface.py`**: Camera adaptors
   - `StandaloneZEDAdaptor`: Direct ZED SDK for testing
   - `ROSBridgeAdaptor`: Adapts ROS topics to core interface
   - Ensures **same calibration** for both modes

3. **`node.py`**: Production ROS 2 node
   - Composable, QoS-optimized
   - Time-synchronized image+depth
   - TF2 transforms to base_link (optional)
   - Diagnostic reporting

4. **`standalone_detector.py`**: Test script
   - Live visualization with overlays
   - FPS and statistics
   - Save snapshots for validation

---

## Quick Start

### 1. Build & Install

```bash
cd ~/workspaces/rover
colcon build --packages-select aruco_detector
source install/setup.bash
```

### 2. Test with Standalone Script

**Recommended first** – verify detections visually:

```bash
# Live camera
python3 src/perception/aruco_detector/scripts/standalone_detector.py

# Or with SVO file
python3 src/perception/aruco_detector/scripts/standalone_detector.py \
    --svo /path/to/recording.svo \
    --marker-size 0.20 \
    --save-snapshots

# Controls: Q/ESC = quit, S = save snapshot
```

**What you'll see:**
- Marker IDs with bounding boxes
- XYZ axes (RGB = Red/Green/Blue)
- Distance in meters (from depth)
- Real-time FPS and stats

### 3. Deploy ROS Node (Mission Mode)

```bash
# Standalone mode
ros2 launch aruco_detector aruco_detector.launch.py

# Or composable (recommended)
ros2 launch aruco_detector aruco_detector.launch.py \
    use_composable:=true \
    container_name:=perception_container
```

**Published topics:**
- `/aruco_detections` – `vision_msgs/Detection3DArray`
- `/aruco_markers` – `visualization_msgs/MarkerArray` (RViz)

**Subscribed topics:**
- `/zed2i/left/image_rect_color` – RGB image
- `/zed2i/left/camera_info` – Camera calibration
- `/zed2i/depth/depth_registered` – Depth map (optional)

---

## Configuration

Edit `config/detector_params.yaml`:

```yaml
dictionary: "DICT_4X4_50"        # ArUco dictionary
marker_size_m: 0.20              # Physical marker size (meters)

# Detection tuning
min_side_px: 20                  # Min marker side (pixels)
adaptive_thresh_win_size_min: 3
adaptive_thresh_win_size_max: 23
adaptive_thresh_win_size_step: 10

# Topics
rgb_topic: "/zed2i/left/image_rect_color"
cam_info_topic: "/zed2i/left/camera_info"
depth_topic: "/zed2i/depth/depth_registered"

# Features
use_depth: true                  # Enable depth sampling
output_frame: "zed2i_left_camera_optical_frame"
transform_to_base_link: false    # TF2 transform to base_link
publish_markers: true            # RViz visualization
max_fps: 30                      # Processing rate limit (0 = unlimited)
```

---

## Workflow: Development → Deployment

### Phase 1: Development (Standalone)

```bash
# 1. Generate test markers
ros2 run aruco_detector generate_markers
# → Creates ./markers/marker_0.png ... marker_9.png

# 2. Print markers and place in environment

# 3. Test detection
python3 scripts/standalone_detector.py --save-snapshots

# 4. Verify:
#    - IDs detected correctly
#    - Distances match ground truth (±2cm)
#    - Pose axes aligned with markers
#    - FPS acceptable (>20 on Jetson)
```

### Phase 2: ROS Integration

```bash
# 1. Start ZED driver
ros2 launch zed_wrapper zed2i.launch.py

# 2. Start detector
ros2 launch aruco_detector aruco_detector.launch.py

# 3. Monitor detections
ros2 topic echo /aruco_detections

# 4. Visualize in RViz
rviz2
# Add: Detection3DArray → /aruco_detections
#      MarkerArray → /aruco_markers
#      TF frames
```

### Phase 3: Mission Deployment

Add to `rover.launch.py`:

```python
from launch.actions import IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch_ros.substitutions import FindPackageShare

aruco = IncludeLaunchDescription(
    PythonLaunchDescriptionSource([
        FindPackageShare('aruco_detector'),
        '/launch/aruco_detector.launch.py'
    ]),
    launch_arguments={
        'use_composable': 'true',
        'container_name': 'perception_container'
    }.items()
)
ld.add_action(aruco)
```

---

## Message Format

### Detection3DArray

Each detection contains:

```yaml
header:
  frame_id: "zed2i_left_camera_optical_frame"  # or "base_link"
  stamp: <image_timestamp>

detections:
  - id: "5"  # Marker ID as string
    results:
      - hypothesis:
          class_id: "5"
          score: 0.98  # Confidence (from reprojection error)
        pose:
          pose:
            position: {x: 1.23, y: -0.45, z: 0.02}  # meters
            orientation: {x, y, z, w}  # quaternion
    bbox:
      center: <same as pose>
      size: {x: 0.20, y: 0.20, z: 0.001}  # marker_size
```

**Coordinate frames:**
- **Camera optical**: +X right, +Y down, +Z forward
- **base_link** (if enabled): REP-103 convention

---

## Performance Tuning (Jetson)

### CPU Core Pinning

Pin detection thread to little core (saves power):

```bash
# In systemd service or launch script
taskset -c 0-1 ros2 launch aruco_detector ...
```

### Memory Optimization

- Use `intra_process_comms` for zero-copy with ZED wrapper
- Set `max_fps` to match navigation update rate
- Reduce `min_side_px` if detecting distant markers

### QoS Profiles

- Image/depth subs: `BEST_EFFORT` (avoids buffering)
- Detection pub: `RELIABLE` (critical for navigation)

---

## Failure Modes & Resilience

### No depth available
- Publishes 2D detections with `range_m = None`
- Logs WARNING (throttled)
- Does NOT stall pipeline

### TF lookup fails
- Publishes in camera frame
- Logs WARN (not ERROR)
- Navigation can still use relative poses

### No detections for >10s
- Diagnostic status → WARN
- Check: markers visible, lighting OK, calibration correct

---

## Diagnostics

Monitor health:

```bash
ros2 topic echo /diagnostics
```

Reports:
- Frames processed / FPS
- Detection count
- Queue depth / frames dropped
- Last detection age

---

## Advanced: Custom Dictionary or Size

```yaml
# config/detector_params.yaml
dictionary: "DICT_ARUCO_ORIGINAL"  # or DICT_5X5_100, etc.
marker_size_m: 0.15                # Measure printed size accurately
```

Generate custom markers:

```python
# scripts/generate_markers.py (modify)
dict_custom = aruco.getPredefinedDictionary(aruco.DICT_6X6_250)
```

---

## Troubleshooting

| Issue | Solution |
|-------|----------|
| No detections | Check: markers in view, lighting, `min_side_px` too high |
| Wrong distances | Verify `marker_size_m` matches physical size |
| Low FPS | Reduce `max_fps`, check CPU load, disable `publish_markers` |
| TF errors | Set `transform_to_base_link: false` or fix TF tree |
| Jittery poses | Enable ZED object tracking (already on), increase marker size |

---

## Testing

### Unit tests (core detection logic)

```bash
# TODO: Add pytest tests
colcon test --packages-select aruco_detector
```

### Integration test (with rosbag)

```bash
# 1. Record test sequence
ros2 bag record /zed2i/left/image_rect_color /zed2i/left/camera_info

# 2. Play back and verify
ros2 bag play <bag_file>
ros2 launch aruco_detector aruco_detector.launch.py
ros2 topic echo /aruco_detections
```

---

## Dependencies

**System:**
- ROS 2 Humble
- ZED SDK 4.x
- OpenCV 4.5+
- Python 3.8+

**Python:**
- numpy >= 1.19
- scipy >= 1.5 (for Rotation utilities)
- cv_bridge
- pyzed (for standalone mode)

---

## License

Apache 2.0

---

## Maintainer

Djordje Ivanovic (dorde_ivanovic@college.harvard.edu)

---

## References

- ArUco markers: https://docs.opencv.org/4.x/d5/dae/tutorial_aruco_detection.html
- ZED SDK: https://www.stereolabs.com/docs/
- ROS 2 composable nodes: https://docs.ros.org/en/humble/Concepts/About-Composition.html
