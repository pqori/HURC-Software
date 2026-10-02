# Detection Runbook (ArUco + Object Detection)

This document is the operational guide for running:
- ArUco tag detection
- Object detection

using the current ROS-topic-based scripts in this repo.

## Prerequisites

1. Build and source workspace.
2. ZED camera connected and working on the rover.
3. ROS 2 environment sourced in each terminal:

```bash
source install/local_setup.bash
```

## 1) Start ZED ROS Publisher

Run first, in its own terminal:

```bash
ros2 launch zed2i_launch zed2i_driver.launch.py
```

Expected image topic:
- `/zed2i/zed2i_camera/left/image_rect_color`

## 2) Run ArUco Detector

New terminal:

```bash
cd /src/perception/aruco_detector/scripts
./run_detector_ros.sh --source ros
```
If want solvePnP (allowing for slightly larger range depth perception)

```bash
./run_detector_ros.sh --source ros --show-pnp-range
```

Notes:
- `--source ros` uses ROS image topics (not direct ZED SDK capture).
- `--use-depth` enables distance estimate from depth topic.

## 3) Run Object Detector

New terminal:

```bash
cd /src/perception/object_detection/scripts
./test_continuous_ros.sh --source ros
```

This uses:
- model: `perception/object_detection/model/model.onnx`
- classes: `perception/object_detection/model/classes_coco.txt`
- image topic: `/zed2i/zed2i_camera/left/image_rect_color`

