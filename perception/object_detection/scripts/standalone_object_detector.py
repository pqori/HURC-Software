#!/usr/bin/env python3
"""
Standalone Object Detector with ZED 3D Visualization

Similar to ZED custom detector samples - shows:
- 3D point cloud (OpenGL)
- 3D object bounding boxes with tracking
- 2D overlay with classes and confidence
- Birds-eye tracking view
- FPS and performance stats

Supports both PyTorch and ONNX models for maximum flexibility.

Usage:
    # With PyTorch model (requires ultralytics)
    python3 standalone_object_detector.py --weights yolov8m.pt
    
    # With ONNX model (recommended for deployment)
    python3 standalone_object_detector.py --onnx model/model.onnx --classes model/classes_coco.txt
    
    # With SVO file for testing
    python3 standalone_object_detector.py --svo test.svo --onnx model/model.onnx
    
    # Performance tuning
    python3 standalone_object_detector.py --conf-thres 0.6 --img-size 416

Controls:
    Q/ESC: Quit
    C: Toggle class filters
    I: Zoom in (birds-eye view)
    O: Zoom out (birds-eye view)
"""

import sys
import os
import numpy as np
import argparse
import cv2
import pyzed.sl as sl
import time
from threading import Thread, Lock
from typing import List, Tuple, Optional

# Add current directory to path for viewer imports
sys.path.insert(0, os.path.dirname(__file__))

# Import viewer utilities
try:
    import ogl_viewer.viewer as gl
    import cv_viewer.tracking_viewer as cv_viewer
    HAS_GL_VIEWER = True
except ImportError:
    print("⚠️ OpenGL viewer not available. Install PyOpenGL for 3D visualization.")
    HAS_GL_VIEWER = False

# Try to import PyTorch YOLO (optional)
try:
    from ultralytics import YOLO
    HAS_PYTORCH = True
except ImportError:
    print("⚠️ Ultralytics not available. PyTorch models (.pt) will not work.")
    print("   Install: pip install ultralytics")
    HAS_PYTORCH = False

# Import ONNX runtime (required)
try:
    import onnxruntime as ort
    HAS_ONNX = True
except ImportError:
    print("❌ ONNX Runtime not available. Install: pip install onnxruntime")
    HAS_ONNX = False
    sys.exit(1)


# Global state for threading
lock = Lock()
run_signal = False
exit_signal = False
image_net = None
detections_data = []


# ============================================================================
#  DETECTION FUNCTIONS
# ============================================================================

def xywh2abcd(xywh, im_shape):
    """
    Convert YOLO format (center_x, center_y, width, height) to 
    ZED format (4 corners: A, B, C, D)

    A ------ B
    |        |
    D ------ C
    """
    output = np.zeros((4, 2))

    # Center / Width / Height -> BBox corners coordinates
    x_min = xywh[0] - 0.5 * xywh[2]
    x_max = xywh[0] + 0.5 * xywh[2]
    y_min = xywh[1] - 0.5 * xywh[3]
    y_max = xywh[1] + 0.5 * xywh[3]

    # Top-left (A)
    output[0][0] = x_min
    output[0][1] = y_min

    # Top-right (B)
    output[1][0] = x_max
    output[1][1] = y_min

    # Bottom-right (C)
    output[2][0] = x_max
    output[2][1] = y_max

    # Bottom-left (D)
    output[3][0] = x_min
    output[3][1] = y_max

    return output


def preprocess_image_onnx(image: np.ndarray, input_size: int = 640) -> Tuple[np.ndarray, float, Tuple[int, int]]:
    """
    Preprocess image for ONNX YOLO inference
    Returns: (preprocessed_tensor, scale_factor, (pad_x, pad_y))
    """
    original_shape = image.shape[:2]  # (height, width)

    # Resize with aspect ratio preservation (letterboxing)
    scale = min(input_size / original_shape[0], input_size / original_shape[1])
    new_shape = (int(original_shape[1] * scale),
                 int(original_shape[0] * scale))

    # Resize image
    resized = cv2.resize(image, new_shape, interpolation=cv2.INTER_LINEAR)

    # Create padded image
    pad_x = (input_size - new_shape[0]) // 2
    pad_y = (input_size - new_shape[1]) // 2

    padded = np.full((input_size, input_size, 3), 114, dtype=np.uint8)
    padded[pad_y:pad_y + new_shape[1], pad_x:pad_x + new_shape[0]] = resized

    # Convert to tensor format: HWC -> CHW, BGR -> RGB, normalize
    tensor = padded.transpose(2, 0, 1).astype(np.float32) / 255.0
    tensor = np.expand_dims(tensor, axis=0)  # Add batch dimension

    return tensor, scale, (pad_x, pad_y)


def postprocess_onnx_detections(outputs: np.ndarray, scale: float, pads: Tuple[int, int],
                                input_size: int, conf_thresh: float,
                                iou_thresh: float, classes: List[str],
                                urc_filter: dict = None,
                                img_wh: Tuple[int, int] = None) -> List[sl.CustomBoxObjectData]:
    """
    Robust postprocess for YOLOv8 ONNX → ZED custom boxes.
    Accepts output shapes (1,84,8400) or (1,8400,84).
    Handles normalized vs pixels. Clips corners. Returns [] if nothing valid.
    """
    W, H = img_wh if img_wh else (input_size, input_size)

    # ---- 1) Normalize output shape to (N, 84)
    preds = outputs
    print(f"[DEBUG] Raw ONNX output shape: {preds.shape}")

    if preds.ndim == 3 and preds.shape[0] == 1:
        preds = preds[0]  # (84,8400) or (8400,84)

    if preds.shape[0] < preds.shape[1]:   # (84, 8400) → (8400, 84)
        preds = preds.T
        print(f"[DEBUG] Transposed to: {preds.shape}")

    if preds.shape[1] != 84:
        # Unexpected head shape; bail safely
        print(f"[WARN] Unexpected head width: {preds.shape[1]} (expected 84)")
        return []

    boxes = preds[:, :4].astype(np.float32)           # cx, cy, w, h
    cls_scores = preds[:, 4:].astype(np.float32)      # 80 classes
    class_ids = np.argmax(cls_scores, axis=1)
    confidences = np.max(cls_scores, axis=1)

    if confidences.size:
        print(
            f"[DEBUG] Confidence range: {confidences.min():.4f} - {confidences.max():.4f}")
        print(f"[DEBUG] Mean confidence: {confidences.mean():.4f}")
        print(
            f"[DEBUG] Num above {conf_thresh}: {(confidences >= conf_thresh).sum()}")

    # ---- 2) Confidence filter (pre-NMS)
    keep = confidences >= conf_thresh
    if not np.any(keep):
        return []
    boxes = boxes[keep]
    class_ids = class_ids[keep]
    confidences = confidences[keep]

    # ---- 3) Detect whether coords are normalized (0..1) or in input pixels
    # Heuristic: if the 95th percentile of w/h <= 1.5 → assume normalized
    wh = boxes[:, 2:4]
    pct95 = np.percentile(wh, 95)
    normalized = pct95 <= 1.5
    if normalized:
        # scale to network input pixels
        boxes[:, :4] *= float(input_size)

    # ---- 4) Undo letterbox (remove pad, then divide by scale)
    pad_x, pad_y = pads
    boxes[:, 0] = (boxes[:, 0] - pad_x) / max(scale, 1e-6)  # cx
    boxes[:, 1] = (boxes[:, 1] - pad_y) / max(scale, 1e-6)  # cy
    boxes[:, 2] = boxes[:, 2] / max(scale, 1e-6)            # w
    boxes[:, 3] = boxes[:, 3] / max(scale, 1e-6)            # h

    # ---- 5) Convert to corners and clip to image bounds
    x1 = boxes[:, 0] - boxes[:, 2] / 2
    y1 = boxes[:, 1] - boxes[:, 3] / 2
    x2 = boxes[:, 0] + boxes[:, 2] / 2
    y2 = boxes[:, 1] + boxes[:, 3] / 2

    x1 = np.clip(x1, 0, W - 1)
    y1 = np.clip(y1, 0, H - 1)
    x2 = np.clip(x2, 0, W - 1)
    y2 = np.clip(y2, 0, H - 1)

    # Remove degenerate/tiny boxes (avoid ZED unsigned int conversion issues)
    w_px = (x2 - x1)
    h_px = (y2 - y1)
    good = (w_px >= 2.0) & (h_px >= 2.0)
    if not np.any(good):
        return []
    x1, y1, x2, y2 = x1[good], y1[good], x2[good], y2[good]
    class_ids = class_ids[good]
    confidences = confidences[good]

    # ---- 6) NMS on corners
    nms_boxes = np.column_stack([x1, y1, x2, y2]).astype(np.float32)
    keep_idx = nms(nms_boxes, confidences, iou_thresh)

    # ---- 7) Build ZED boxes from *clipped corners* (not center format)
    custom = []
    for i in keep_idx:
        cid = int(class_ids[i])
        if urc_filter and cid not in urc_filter:
            continue

        # Build bounding box as numpy array (4 corners: A, B, C, D)
        # ZED expects shape (4, 2) with corners in order: top-left, top-right, bottom-right, bottom-left
        bbox_corners = np.array([
            [x1[i], y1[i]],  # A: top-left
            [x2[i], y1[i]],  # B: top-right
            [x2[i], y2[i]],  # C: bottom-right
            [x1[i], y2[i]]   # D: bottom-left
        ], dtype=np.float32)

        obj = sl.CustomBoxObjectData()
        obj.bounding_box_2d = bbox_corners
        obj.label = cid                      # store raw COCO id
        obj.probability = float(confidences[i])
        obj.is_grounded = False              # unknown by default
        obj.unique_object_id = sl.generate_unique_id()

        custom.append(obj)

    return custom


def nms(boxes: np.ndarray, scores: np.ndarray, iou_threshold: float) -> List[int]:
    """Non-Maximum Suppression implementation"""
    if len(boxes) == 0:
        return []

    # Sort by confidence scores
    indices = np.argsort(scores)[::-1]
    keep = []

    while len(indices) > 0:
        current = indices[0]
        keep.append(current)

        if len(indices) == 1:
            break

        # Calculate IoU with remaining boxes
        current_box = boxes[current]
        remaining_boxes = boxes[indices[1:]]

        # Calculate intersection
        x1 = np.maximum(current_box[0], remaining_boxes[:, 0])
        y1 = np.maximum(current_box[1], remaining_boxes[:, 1])
        x2 = np.minimum(current_box[2], remaining_boxes[:, 2])
        y2 = np.minimum(current_box[3], remaining_boxes[:, 3])

        intersection = np.maximum(0, x2 - x1) * np.maximum(0, y2 - y1)

        # Calculate union
        current_area = (current_box[2] - current_box[0]) * \
            (current_box[3] - current_box[1])
        remaining_areas = (remaining_boxes[:, 2] - remaining_boxes[:, 0]) * \
            (remaining_boxes[:, 3] - remaining_boxes[:, 1])
        union = current_area + remaining_areas - intersection

        # Calculate IoU
        iou = intersection / (union + 1e-6)

        # Keep boxes with IoU below threshold
        indices = indices[1:][iou <= iou_threshold]

    return keep


# ============================================================================
#  DETECTION THREAD
# ============================================================================

def detection_thread_onnx(session, input_name, output_names, classes,
                          input_size, conf_thres, iou_thres, urc_filter=None):
    """Worker thread for ONNX inference"""
    global image_net, exit_signal, run_signal, detections_data

    print("🔄 ONNX detection thread started...")
    frame_count = 0

    while not exit_signal:
        if run_signal:
            lock.acquire()
            img = image_net.copy()
            lock.release()

            # Preprocess
            input_tensor, scale, pads = preprocess_image_onnx(img, input_size)

            # Run inference
            outputs = session.run(output_names, {input_name: input_tensor})

            # Postprocess to ZED format
            detections = postprocess_onnx_detections(
                outputs[0], scale, pads, input_size,
                conf_thres, iou_thres, classes, urc_filter,
                img_wh=(img.shape[1], img.shape[0])  # (W,H)
            )

            # Debug output every 30 frames
            if frame_count % 30 == 0:
                print(
                    f"[DEBUG] Frame {frame_count}: Found {len(detections)} objects")
                for det in detections[:5]:  # Show first 5
                    # det.label is a plain int here (set in postprocess)
                    label_id = det.label
                    class_name = classes[label_id] if label_id < len(
                        classes) else "unknown"
                    print(f"        {class_name}: {det.probability:.2f}")

            frame_count += 1

            lock.acquire()
            detections_data = detections
            lock.release()

            run_signal = False

        time.sleep(0.001)


def detection_thread_pytorch(model, classes, conf_thres, img_size):
    """Worker thread for PyTorch YOLO inference"""
    global image_net, exit_signal, run_signal, detections_data

    print("🔄 PyTorch detection thread started...")
    frame_count = 0

    while not exit_signal:
        if run_signal:
            lock.acquire()
            img = cv2.cvtColor(image_net, cv2.COLOR_BGRA2RGB)
            lock.release()

            # Run YOLO inference
            results = model.predict(img, save=False, imgsz=img_size,
                                    conf=conf_thres, verbose=False)[0]

            # Convert to ZED format
            detections = []
            for det in results.boxes:
                xywh = det.xywh[0].cpu().numpy()

                obj = sl.CustomBoxObjectData()
                obj.bounding_box_2d = xywh2abcd(xywh, img.shape)
                obj.label = int(det.cls[0].cpu().numpy())
                obj.probability = float(det.conf[0].cpu().numpy())
                obj.is_grounded = True
                obj.unique_object_id = sl.generate_unique_id()

                detections.append(obj)

            # Debug output
            if frame_count % 30 == 0:
                print(
                    f"[DEBUG] Frame {frame_count}: Found {len(detections)} objects")

            frame_count += 1

            lock.acquire()
            detections_data = detections
            lock.release()

            run_signal = False

        time.sleep(0.001)


# ============================================================================
#  VISUALIZATION
# ============================================================================

def draw_stats(image, fps, num_objects, frame_count, classes_dict):
    """Draw FPS and detection statistics"""
    stats = [
        f"FPS: {fps:.1f}",
        f"Objects: {num_objects}",
        f"Frame: {frame_count}"
    ]

    # Add per-class counts
    if classes_dict:
        stats.append("---")
        for class_name, count in sorted(classes_dict.items()):
            stats.append(f"{class_name}: {count}")

    font = cv2.FONT_HERSHEY_SIMPLEX
    y = 30
    for stat in stats:
        cv2.putText(image, stat, (10, y), font, 0.6, (0, 255, 0), 2)
        y += 25

    return image


def count_classes(objects, classes):
    """Count detections per class"""
    counts = {}
    for obj in objects.object_list:
        # obj.label can be an enum; use .value if present
        raw = obj.label.value if hasattr(obj.label, 'value') else obj.label
        cid = int(raw)
        if 0 <= cid < len(classes):
            name = classes[cid]
            counts[name] = counts.get(name, 0) + 1
    return counts


# ============================================================================
#  MAIN
# ============================================================================

def main():
    global image_net, exit_signal, run_signal, detections_data

    parser = argparse.ArgumentParser(
        description="Standalone object detector with ZED 3D visualization",
        formatter_class=argparse.RawDescriptionHelpFormatter,
        epilog=__doc__
    )

    # Model options
    parser.add_argument('--weights', type=str, help='PyTorch model (.pt file)')
    parser.add_argument('--onnx', type=str, help='ONNX model (.onnx file)')
    parser.add_argument('--classes', type=str, default='model/classes_coco.txt',
                        help='Classes file (one class per line)')
    parser.add_argument('--urc-filter', type=str, default='config/urc_classes.txt',
                        help='URC mission object filter (class_id:name format)')

    # Detection parameters
    parser.add_argument('--conf-thres', type=float, default=0.5,
                        help='Confidence threshold (0-1)')
    parser.add_argument('--iou-thres', type=float, default=0.45,
                        help='NMS IoU threshold (0-1)')
    parser.add_argument('--img-size', type=int, default=640,
                        help='Input image size for inference')

    # Camera options
    parser.add_argument('--svo', type=str, help='SVO file for playback')
    parser.add_argument('--resolution', type=str, default='HD720',
                        choices=['HD2K', 'HD1080', 'HD720', 'VGA'],
                        help='Camera resolution')

    # Visualization options
    parser.add_argument('--no-gl', action='store_true',
                        help='Disable 3D OpenGL viewer (2D only)')

    args = parser.parse_args()

    # Validate model selection
    if not args.weights and not args.onnx:
        print("❌ Error: Must specify either --weights or --onnx")
        sys.exit(1)

    if args.weights and not HAS_PYTORCH:
        print("❌ Error: PyTorch model specified but ultralytics not installed")
        print("   Install: pip install ultralytics")
        sys.exit(1)

    use_gl = HAS_GL_VIEWER and not args.no_gl

    print("=" * 70)
    print("🎯 Standalone Object Detector with 3D Visualization")
    print("=" * 70)

    # Load classes
    print(f"\n[1/5] Loading classes from {args.classes}...")
    try:
        with open(args.classes, 'r') as f:
            classes = [line.strip() for line in f if line.strip()]
        print(f"✓ Loaded {len(classes)} COCO classes")
    except FileNotFoundError:
        print(f"❌ Classes file not found: {args.classes}")
        sys.exit(1)

    # Load URC filter
    urc_filter = {}
    if args.urc_filter and os.path.exists(args.urc_filter):
        print(f"\n[1.5/5] Loading URC filter from {args.urc_filter}...")
        try:
            with open(args.urc_filter, 'r') as f:
                for line in f:
                    line = line.strip()
                    if line and not line.startswith('#'):
                        if ':' in line:
                            class_id, name = line.split(':', 1)
                            urc_filter[int(class_id.strip())] = name.strip()
            print(
                f"✓ URC Filter: Detecting {len(urc_filter)} mission objects: {list(urc_filter.values())}")
        except Exception as e:
            print(f"⚠️ Failed to load URC filter: {e}")
            urc_filter = {}
    else:
        print(
            f"\n[1.5/5] No URC filter - detecting all {len(classes)} classes")

    # Initialize model
    print("\n[2/5] Initializing detection model...")
    detection_thread_func = None

    if args.onnx:
        print(f"   Loading ONNX model: {args.onnx}")
        try:
            providers = ['CUDAExecutionProvider', 'CPUExecutionProvider']
            session = ort.InferenceSession(args.onnx, providers=providers)
            input_name = session.get_inputs()[0].name
            output_names = [output.name for output in session.get_outputs()]
            print(f"✓ ONNX model loaded")
            print(f"   Input: {input_name}")
            print(f"   Outputs: {output_names}")
            print(f"   Provider: {session.get_providers()[0]}")

            def detection_thread_func(): return detection_thread_onnx(
                session, input_name, output_names, classes,
                args.img_size, args.conf_thres, args.iou_thres, urc_filter
            )
        except Exception as e:
            print(f"❌ Failed to load ONNX model: {e}")
            sys.exit(1)
    else:
        print(f"   Loading PyTorch model: {args.weights}")
        try:
            model = YOLO(args.weights)
            print(f"✓ PyTorch model loaded")

            def detection_thread_func(): return detection_thread_pytorch(
                model, classes, args.conf_thres, args.img_size
            )
        except Exception as e:
            print(f"❌ Failed to load PyTorch model: {e}")
            sys.exit(1)

    # Initialize ZED camera
    print("\n[3/5] Initializing ZED camera...")
    zed = sl.Camera()

    init_params = sl.InitParameters()
    if args.svo:
        init_params.set_from_svo_file(args.svo)
        init_params.svo_real_time_mode = True
        print(f"   Using SVO: {args.svo}")

    # Set resolution
    res_map = {
        'HD2K': sl.RESOLUTION.HD2K,
        'HD1080': sl.RESOLUTION.HD1080,
        'HD720': sl.RESOLUTION.HD720,
        'VGA': sl.RESOLUTION.VGA
    }
    init_params.camera_resolution = res_map[args.resolution]
    init_params.depth_mode = sl.DEPTH_MODE.NEURAL
    init_params.coordinate_units = sl.UNIT.METER
    init_params.coordinate_system = sl.COORDINATE_SYSTEM.RIGHT_HANDED_Y_UP
    init_params.depth_maximum_distance = 20.0

    status = zed.open(init_params)
    if status != sl.ERROR_CODE.SUCCESS:
        print(f"❌ Failed to open ZED camera: {status}")
        sys.exit(1)

    print(f"✓ ZED camera opened ({args.resolution})")

    # Enable positional tracking
    print("\n[4/5] Enabling positional tracking...")
    tracking_params = sl.PositionalTrackingParameters()
    zed.enable_positional_tracking(tracking_params)
    print("✓ Tracking enabled")

    # Enable custom object detection
    print("\n[5/5] Configuring object detection...")
    obj_params = sl.ObjectDetectionParameters()
    obj_params.detection_model = sl.OBJECT_DETECTION_MODEL.CUSTOM_BOX_OBJECTS
    obj_params.enable_tracking = True
    obj_params.enable_segmentation = False

    status = zed.enable_object_detection(obj_params)
    if status != sl.ERROR_CODE.SUCCESS:
        print(f"❌ Failed to enable object detection: {status}")
        zed.close()
        sys.exit(1)

    print("✓ Object detection enabled")
    print(f"   Model: {'ONNX' if args.onnx else 'PyTorch'}")
    print(f"   Confidence threshold: {args.conf_thres}")
    print(f"   Input size: {args.img_size}")

    # Start detection thread
    print("\n[6/6] Starting detection thread...")
    detection_thread = Thread(target=detection_thread_func)
    detection_thread.start()
    print("✓ Detection thread running")

    # Get camera info
    cam_info = zed.get_camera_information()
    camera_res = cam_info.camera_configuration.resolution
    display_res = sl.Resolution(min(camera_res.width, 1280),
                                min(camera_res.height, 720))

    # Initialize OpenGL 3D viewer
    viewer = None
    point_cloud_res = None
    cam_w_pose = sl.Pose()

    if use_gl:
        print("\n[7/7] Initializing 3D OpenGL viewer...")
        viewer = gl.GLViewer()
        point_cloud_res = sl.Resolution(min(camera_res.width, 720),
                                        min(camera_res.height, 404))
        viewer.init(cam_info.camera_model, point_cloud_res,
                    obj_params.enable_tracking)
        print("✓ 3D viewer ready")

    # Image scale for 2D rendering
    image_scale = [display_res.width / camera_res.width,
                   display_res.height / camera_res.height]

    # Tracking viewer (birds-eye view)
    track_view_generator = None
    image_track_ocv = None

    if use_gl:
        camera_config = cam_info.camera_configuration
        tracks_resolution = sl.Resolution(400, display_res.height)
        track_view_generator = cv_viewer.TrackingViewer(
            tracks_resolution,
            camera_config.fps,
            init_params.depth_maximum_distance
        )
        track_view_generator.set_camera_calibration(
            camera_config.calibration_parameters)
        # Match the background format (3 channels RGB, not 4 RGBA)
        image_track_ocv = np.full(
            (tracks_resolution.height, tracks_resolution.width, 3), [245, 239, 239], np.uint8)

    # Allocate buffers
    image_left = sl.Mat()
    image_left_cpu = sl.Mat()
    image_left_display = sl.Mat()
    point_cloud = sl.Mat()
    objects = sl.Objects()

    runtime_params = sl.RuntimeParameters()
    obj_runtime_params = sl.ObjectDetectionRuntimeParameters()

    # Stats
    frame_count = 0
    start_time = time.time()
    last_fps_time = start_time
    fps_counter = 0
    current_fps = 0.0

    print("\n" + "=" * 70)
    print("🚀 Detection loop started (Press 'Q' or ESC to quit)")
    print("=" * 70)
    print("Controls:")
    print("  Q/ESC: Quit")
    print("  I: Zoom in (birds-eye view)")
    print("  O: Zoom out (birds-eye view)")
    print("=" * 70 + "\n")

    try:
        while (not use_gl) or (viewer and viewer.is_available()):
            # Grab frame
            if zed.grab(runtime_params) == sl.ERROR_CODE.SUCCESS:
                # Get image for detection (GPU memory - faster)
                lock.acquire()
                zed.retrieve_image(image_left, sl.VIEW.LEFT,
                                   sl.MEM.GPU, display_res)

                # Convert and transfer to CPU (proper GPU workflow, remove alpha channel)
                sl.Mat.convert_color(image_left, image_left_cpu,
                                     swap_RB_channels=False, remove_alpha_channels=True,
                                     memory_type=sl.MEM.GPU)
                image_left_cpu.update_cpu_from_gpu()
                image_net = image_left_cpu.get_data()
                lock.release()
                run_signal = True

                # Get display image
                zed.retrieve_image(image_left_display, sl.VIEW.LEFT,
                                   sl.MEM.CPU, display_res)

                # Wait for detection to complete
                while run_signal:
                    time.sleep(0.001)

                # Ingest custom boxes
                lock.acquire()
                boxes = detections_data
                lock.release()

                if boxes:
                    try:
                        zed.ingest_custom_box_objects(boxes)
                    except Exception as e:
                        # Last-ditch protection: if anything sneaks past validation
                        print(f"[WARN] Failed to ingest boxes: {e}. Dropping this batch.")

                # Retrieve tracked objects with 3D data
                zed.retrieve_objects(objects, obj_runtime_params)

                # Get 3D data for OpenGL viewer
                if use_gl:
                    zed.retrieve_measure(point_cloud, sl.MEASURE.XYZRGBA,
                                         sl.MEM.CPU, point_cloud_res)
                    zed.get_position(cam_w_pose, sl.REFERENCE_FRAME.WORLD)

                # Update FPS
                frame_count += 1
                fps_counter += 1
                now = time.time()
                if (now - last_fps_time) >= 1.0:
                    current_fps = fps_counter / (now - last_fps_time)
                    fps_counter = 0
                    last_fps_time = now

                # 3D rendering (OpenGL viewer)
                if use_gl and viewer:
                    viewer.updateData(point_cloud, objects)

                # 2D rendering with overlay
                display_img = image_left_display.get_data()[:, :, :3].copy()
                if HAS_GL_VIEWER:
                    cv_viewer.render_2D(display_img, image_scale, objects,
                                        obj_params.enable_tracking)

                # Draw stats
                classes_dict = count_classes(objects, classes)
                display_img = draw_stats(display_img, current_fps,
                                         len(objects.object_list), frame_count,
                                         classes_dict)

                # Tracking view (birds-eye)
                if use_gl and track_view_generator:
                    track_view_generator.generate_view(objects, cam_w_pose,
                                                       image_track_ocv,
                                                       objects.is_tracked)
                    # Combine views
                    global_image = cv2.hconcat([display_img, image_track_ocv])
                    cv2.imshow("Object Detector - 2D View and Birds View",
                               global_image)
                else:
                    cv2.imshow("Object Detector - 2D View", display_img)

                # Handle key press
                key = cv2.waitKey(1) & 0xFF
                if key == ord('q') or key == 27:  # Q or ESC
                    break
                elif key == ord('i'):  # Zoom in
                    if track_view_generator:
                        track_view_generator.zoomIn()
                elif key == ord('o'):  # Zoom out
                    if track_view_generator:
                        track_view_generator.zoomOut()
            else:
                if args.svo:
                    print("End of SVO file")
                break

    except KeyboardInterrupt:
        print("\n⚠️ Interrupted by user")

    finally:
        # Cleanup
        print("\n" + "=" * 70)
        print("🛑 Shutting down...")
        print("=" * 70)

        exit_signal = True
        detection_thread.join(timeout=2.0)

        # Close viewers
        if viewer:
            viewer.exit()
        cv2.destroyAllWindows()

        # Calculate statistics
        elapsed = time.time() - start_time
        avg_fps = frame_count / elapsed if elapsed > 0 else 0.0

        print(f"\n📊 Session Statistics:")
        print(f"  Total frames: {frame_count}")
        print(f"  Average FPS: {avg_fps:.1f}")
        print(f"  Runtime: {elapsed:.1f}s")
        print(f"  Model: {'ONNX' if args.onnx else 'PyTorch'}")
        print(f"  Resolution: {args.resolution}")

        zed.close()
        print("\n✓ Shutdown complete")

    return 0


if __name__ == '__main__':
    sys.exit(main())
