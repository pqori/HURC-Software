# URC Object Detection Module

**High-Performance Object Detection with Dual Pipeline Support**

This module provides **two detection systems** optimized for University Rover Challenge missions:

1. **🚀 ZED Native Detection** (Recommended) - GPU-accelerated with native 3D bounding boxes  
2. **🔄 YOLO Detection** (Fallback) - CPU-based with 2D→3D conversion

## 🎯 Detection Pipeline Comparison

| Feature | ZED Native | YOLO Pipeline |
|---------|------------|---------------|
| **Speed** | 30-50ms (GPU) | 150-250ms (CPU) |
| **3D Accuracy** | Native 3D boxes | 2D→3D conversion |
| **Tracking** | Built-in persistent IDs | Custom tracker |
| **Hardware** | Requires ZED SDK | Any camera |
| **Custom Models** | ZED training tools | ONNX models |

## Quick Start Guide

### 🚀 Option 1: ZED Integrated Detection (Recommended - 2 minutes)

**Uses official ZED ROS2 wrapper with GPU-accelerated object detection**

```bash
# Build the packages
cd /home/rover/workspaces/rover
colcon build --packages-select zed_wrapper zed_components object_detection

# Launch ZED integrated detection (uses official ZED ROS2 wrapper)
ros2 launch object_detection zed_integrated_detection.launch.py

# Check 3D detections
ros2 topic echo /zed_detections_3d

# Test object selection
ros2 service call /select_best_science std_srvs/srv/SetBool "{data: true}"
```

**Architecture:** `ZED Wrapper → zed_msgs/ObjectsStamped → Bridge → Detection3DArray → Selection`

### 🔄 Option 2: YOLO Detection (Fallback - 5 minutes)

**CPU-based detection with pre-trained YOLOv8 model**

```bash
# Launch YOLO detection (works without ZED SDK)
ros2 launch object_detection object_detection.launch.py

# Check 2D detections
ros2 topic echo /detected_objects
```

**What it detects:** 80 COCO objects including `person`, `bottle`, `backpack`, `chair`, `laptop` - some overlap with URC mission objects.

## 🔄 Migration Plan: YOLO → ZED Native

### **Phase 1: Parallel Testing (Current)**
- ✅ Both systems available
- ✅ ZED detector implemented with 3D messages
- ✅ Selection service supports both pipelines
- 🔄 **Test both systems side-by-side**

### **Phase 2: Switch to ZED (Recommended)**
```bash
# Use ZED integrated detection in complete perception system
ros2 launch slam_launch perception_complete.launch.py use_zed_detection:=true

# Or disable ZED and use YOLO fallback  
ros2 launch slam_launch perception_complete.launch.py use_zed_detection:=false
```

**Note:** ZED detection uses the official ZED ROS2 wrapper, not direct SDK access. This avoids camera resource conflicts and leverages the existing ZED ecosystem.

### **Phase 3: Custom URC Model Training**
1. **Collect URC dataset** (bottles, rocks, equipment, flags)
2. **Use ZED training tools** (Docker-based pipeline)
3. **Deploy custom model** to Jetson
4. **Fine-tune for mission scenarios**

## 🛠️ ZED Integration Setup

### **Architecture: Official ZED ROS2 Wrapper (Recommended)**

Our implementation uses the **official ZED ROS2 wrapper** instead of direct SDK access:

```
ZED Camera → ZED ROS2 Wrapper → zed_msgs/ObjectsStamped → Bridge → Detection3DArray → Selection
```

**Benefits:**
- ✅ **No camera conflicts** - uses existing ZED integration
- ✅ **Leverages ZED configuration** - uses official config files
- ✅ **Better performance** - optimized ZED ROS2 implementation
- ✅ **Custom model support** - via ZED wrapper's ONNX integration

### **Dependencies (Already in workspace)**
```bash
# ZED packages already available:
# - zed_wrapper (official ZED ROS2 wrapper)
# - zed_components (ZED camera components)
# - zed_ros2 (ZED ROS2 utilities)

# Verify ZED packages
ros2 pkg list | grep zed
```

### **Custom URC Model Training**
```bash
# Use ZED's official training pipeline
# 1. Collect URC dataset
# 2. Train ONNX model using ZED tools
# 3. Configure via custom_object_detection.yaml
# 4. Load in ZED wrapper: custom_onnx_file: '/path/to/urc_model.onnx'
```

### 🎯 Option 3: Train Custom URC Model (2-3 days)

For competition-ready detection of URC-specific objects like rocks, toolboxes, and flags:

1. **Collect Images** (1-2 days): 500-2000 images of URC objects
2. **Annotate Dataset** (1-2 days): Use LabelImg or Roboflow  
3. **Train Model** (2-4 hours): YOLOv8 training
4. **Deploy** (5 minutes): Replace model files

See detailed instructions below.

---

## Overview

This module provides real-time object detection for University Rover Challenge (URC) missions using YOLO models with ONNX runtime. It detects mission-critical objects and publishes them as ROS2 `Detection2DArray` messages for downstream processing.

## Features

- **Dual Mode**: Pre-trained COCO model for testing + Custom URC training pipeline
- **YOLO Model Support**: Compatible with YOLOv5/YOLOv8 ONNX models
- **Jetson Optimized**: CPU-only inference for ARM64 compatibility
- **ROS2 Integration**: Publishes standardized vision messages
- **Performance Monitoring**: Built-in FPS tracking and logging

## Object Classes

### Science Task Objects
- `rock`, `mineral_sample`, `biological_sample`, `soil_sample`
- `plant`, `lichen`, `moss`

### Equipment Retrieval Objects  
- `bottle`, `toolbox`, `container`, `drill`, `hammer`, `wrench`
- `cable`, `rope`, `battery`, `solar_panel`

### Navigation Markers
- `flag`, `post`, `sign`, `marker`, `beacon`, `gate`, `checkpoint`

### Obstacles & Structures
- `large_rock`, `boulder`, `equipment_cache`, `habitat_module`
- `rover`, `antenna`, `obstacle`

### Autonomous Navigation
- `person`, `astronaut`, `scientist`, `backpack`, `helmet`

## Dependencies

### System Requirements
```bash
# Install ONNX Runtime with GPU support
pip install onnxruntime-gpu  # or onnxruntime for CPU-only

# Install additional Python dependencies
pip install opencv-python numpy
```

### ROS2 Dependencies
- `rclpy`
- `sensor_msgs`
- `vision_msgs` 
- `geometry_msgs`
- `cv_bridge`

## Configuration

### Parameters

| Parameter | Type | Default | Description |
|-----------|------|---------|-------------|
| `model_path` | string | - | Path to ONNX model file |
| `classes_file` | string | - | Path to classes.txt file |
| `confidence_threshold` | float | 0.5 | Minimum detection confidence |
| `iou_threshold` | float | 0.45 | NMS IoU threshold |
| `input_size` | int | 640 | Model input image size |
| `image_topic` | string | `/zed2i/left/image_rect_color` | Input image topic |
| `output_topic` | string | `/detected_objects` | Output detections topic |
| `max_detections` | int | 100 | Maximum detections per frame |

### Model Requirements

The ONNX model should:
- Accept input shape `[1, 3, 640, 640]` (batch, channels, height, width)
- Output YOLO format: `[1, N, 85]` where N is number of detections
- Use standard YOLO output format: `[x_center, y_center, width, height, confidence, class_scores...]`

## Usage

### Launch Object Detection
```bash
ros2 launch object_detection object_detection.launch.py
```

### Monitor Detections
```bash
# View detection messages
ros2 topic echo /detected_objects

# Monitor performance
ros2 topic echo /rosout | grep "Detection FPS"
```

### Integration with Pose Estimator
The detection output is compatible with `detection_pose_estimator`:
```bash
# Launch complete detection + pose estimation pipeline
ros2 launch object_detection object_detection.launch.py &
ros2 launch detection_pose_estimator pose_estimator.launch.py
```

## Custom URC Model Training (Option 2)

### Step 1: Data Collection (1-2 days)

Collect 500-2000 images of URC objects in various conditions:

```bash
# Create dataset structure
mkdir -p ~/urc_dataset/{train,val}/{images,labels}

# Collect images of:
# - Rocks, minerals, biological samples
# - Bottles, toolboxes, containers, equipment
# - Flags, posts, signs, markers
# - People, backpacks, rovers
# - Various lighting conditions, angles, distances
```

**Tips:**
- Take photos in Mars-like environments (desert, rocky terrain)
- Include objects at different distances (1m to 50m)
- Vary lighting conditions (bright sun, shadows, overcast)
- Include partial occlusions and multiple objects per image

### Step 2: Data Annotation (1-2 days)

**Option A: LabelImg (Local)**
```bash
pip install labelImg
labelImg ~/urc_dataset/train/images
```

**Option B: Roboflow (Online - Recommended)**
1. Upload images to [roboflow.com](https://roboflow.com)
2. Draw bounding boxes around objects
3. Label with URC class names
4. Export in YOLOv8 format

### Step 3: Training Setup

```bash
# Install training dependencies
pip install ultralytics wandb  # wandb for training visualization

# Create dataset configuration
cat > ~/urc_dataset/dataset.yaml << EOF
train: /home/rover/urc_dataset/train/images
val: /home/rover/urc_dataset/val/images
nc: 35  # number of classes
names: 
  0: rock
  1: mineral_sample
  2: biological_sample
  3: soil_sample
  4: plant
  5: lichen
  6: moss
  7: bottle
  8: toolbox
  9: container
  10: drill
  11: hammer
  12: wrench
  13: cable
  14: rope
  15: battery
  16: solar_panel
  17: flag
  18: post
  19: sign
  20: marker
  21: beacon
  22: gate
  23: checkpoint
  24: large_rock
  25: boulder
  26: equipment_cache
  27: habitat_module
  28: rover
  29: antenna
  30: obstacle
  31: person
  32: astronaut
  33: scientist
  34: backpack
  35: helmet
EOF
```

### Step 4: Train Model (2-4 hours)

```python
# Create training script: train_urc_model.py
from ultralytics import YOLO
import torch

def train_urc_model():
    # Load pretrained model
    model = YOLO('yolov8n.pt')  # or yolov8s.pt for better accuracy
    
    # Train on URC dataset
    results = model.train(
        data='/home/rover/urc_dataset/dataset.yaml',
        epochs=100,
        imgsz=640,
        batch=8,  # Adjust based on available memory
        device='cpu',  # Use 'cuda' if GPU available
        project='urc_training',
        name='urc_yolo_v1',
        save_period=10,  # Save checkpoint every 10 epochs
        patience=20,  # Early stopping
        plots=True
    )
    
    # Export best model to ONNX
    best_model = YOLO('urc_training/urc_yolo_v1/weights/best.pt')
    best_model.export(format='onnx')
    
    print("Training complete! Model saved as best.onnx")
    return results

if __name__ == "__main__":
    train_urc_model()
```

```bash
# Run training
python3 train_urc_model.py

# Monitor training (if using wandb)
# Visit https://wandb.ai to see training progress
```

### Step 5: Deploy Custom Model

```bash
# Copy trained model
cp urc_training/urc_yolo_v1/weights/best.onnx /home/rover/workspaces/rover/src/perception/object_detection/model/model.onnx

# Update launch file to use URC classes
sed -i 's/classes_coco.txt/classes.txt/' /home/rover/workspaces/rover/src/perception/object_detection/launch/object_detection.launch.py

# Rebuild and test
cd /home/rover/workspaces/rover
colcon build --packages-select object_detection
ros2 launch object_detection object_detection.launch.py
```

## Performance Optimization

### GPU Acceleration
Ensure CUDA is available:
```python
import onnxruntime as ort
print(ort.get_available_providers())  # Should include 'CUDAExecutionProvider'
```

### Model Optimization
- Use smaller models (YOLOv8n/s) for real-time performance
- Reduce input size (416x416) if speed is critical
- Use TensorRT for maximum GPU performance

### Memory Management
- Adjust `max_detections` based on scene complexity
- Monitor memory usage during long missions
- Consider frame skipping for very high frame rates

## Troubleshooting

### Common Issues

**Model Loading Errors**
```bash
# Check model path and permissions
ls -la /path/to/model.onnx
```

**Low Detection Performance**
- Verify model is trained on similar objects
- Adjust confidence threshold
- Check lighting conditions and image quality

**GPU Not Used**
```bash
# Install CUDA-compatible ONNX Runtime
pip uninstall onnxruntime
pip install onnxruntime-gpu
```

**Topic Connection Issues**
```bash
# Verify ZED camera is publishing
ros2 topic list | grep zed2i
ros2 topic hz /zed2i/left/image_rect_color
```

## Integration with Rover Pipeline

This module integrates with:
- **ZED2i Camera**: Receives rectified color images
- **Detection Pose Estimator**: Provides 2D detections for 3D pose estimation
- **Mission Control**: Object detections inform autonomous decision making
- **Navigation**: Obstacle detection for path planning

## Testing Your Setup

### Test Pre-trained Model (Option 1)
```bash
# Check if model files exist
ls -la /home/rover/workspaces/rover/src/perception/object_detection/model/

# Test node without camera (will show initialization logs)
cd /home/rover/workspaces/rover
source install/setup.bash
ros2 run object_detection object_detection

# Test with ZED2i camera
ros2 launch object_detection object_detection.launch.py

# Monitor detections
ros2 topic echo /detected_objects --once
```

### Validate Custom Model (Option 2)
```bash
# Test model accuracy on validation set
python3 -c "
from ultralytics import YOLO
model = YOLO('best.onnx')
results = model.val(data='dataset.yaml')
print(f'mAP50: {results.box.map50:.3f}')
print(f'mAP50-95: {results.box.map:.3f}')
"

# Test inference speed
python3 -c "
import time
from ultralytics import YOLO
model = YOLO('best.onnx')
start = time.time()
for i in range(10):
    model.predict('test_image.jpg', verbose=False)
avg_time = (time.time() - start) / 10
print(f'Average inference time: {avg_time:.3f}s ({1/avg_time:.1f} FPS)')
"
```

## Performance Expectations

- **Pre-trained Model**: 5-15 FPS on Jetson (CPU), detects bottles, people, backpacks
- **Custom URC Model**: 3-12 FPS depending on model size and object complexity
- **Memory Usage**: ~500MB-1GB depending on model size
- **Accuracy**: Custom models typically achieve 80-95% mAP50 with good training data

## Troubleshooting

### Common Issues
- **No detections**: Check confidence threshold, lighting, object size
- **Low FPS**: Use smaller model (yolov8n vs yolov8m), reduce input size
- **High memory usage**: Reduce batch size, use model quantization
- **Training fails**: Check dataset format, reduce batch size, verify labels

### Debug Commands
```bash
# Check model info
python3 -c "
from ultralytics import YOLO
model = YOLO('model.onnx')
model.info()
"

# Test single image
ros2 run object_detection object_detection --ros-args -p image_topic:=/test_image
```

## Development

### Adding New Object Classes
1. Update `model/classes.txt` with new class names
2. Collect and annotate training data for new classes  
3. Retrain YOLO model including new classes
4. Test detection accuracy on validation set
5. Deploy updated model

### Extending Functionality
- **Object Tracking**: Add tracking IDs across frames
- **Multi-Scale Detection**: Process multiple image resolutions
- **Semantic Segmentation**: Upgrade to YOLO segmentation models
- **Mission Integration**: Add mission-specific detection logic

## License

Apache-2.0 License
