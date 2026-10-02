#!/usr/bin/env python3
"""
Standalone ArUco detector with full ZED SDK 3D visualization.

Similar to ZED's custom object detection samples - shows:
- 3D point cloud (OpenGL)
- 3D marker bounding boxes with tracking
- 2D overlay with distances
- Birds-eye tracking view
"""

import pyzed.sl as sl
from aruco_detector.core import ArucoDetectorCore
import sys
import argparse
import cv2
import numpy as np
from cv2 import aruco
import time
from threading import Thread, Lock

# Add parent directory to path
import os
sys.path.insert(0, os.path.join(os.path.dirname(__file__), '..'))


# Import viewer utilities (copied from ZED samples)
try:
    import ogl_viewer.viewer as gl
    import cv_viewer.tracking_viewer as cv_viewer
    HAS_GL_VIEWER = True
except ImportError:
    print("Warning: OpenGL viewer not available. Install PyOpenGL for 3D visualization.")
    HAS_GL_VIEWER = False


# Global state for threading
lock = Lock()
run_signal = False
exit_signal = False
image_net = None
detections_data = []
last_detections = []
persist_misses = 0
PERSIST_FRAMES = 10


def aruco_thread(detector, camera_matrix, dist_coeffs, marker_size_m):
    """Worker thread for ArUco detection."""
    global image_net, exit_signal, run_signal, detections_data

    print("ArUco detection thread started...")
    frame_count = 0

    while not exit_signal:
        if run_signal:
            lock.acquire()

            # Run detection on current image
            img_copy = image_net.copy()

            lock.release()

            # Debug: Check image format on first frame
            if frame_count == 0:
                print(
                    f"[DEBUG] Image shape: {img_copy.shape}, dtype: {img_copy.dtype}")
                print(
                    f"[DEBUG] Image range: [{img_copy.min()}, {img_copy.max()}]")
                # Save first frame for debugging
                cv2.imwrite('/tmp/aruco_test_frame.jpg', img_copy)
                print(f"[DEBUG] Saved test frame to /tmp/aruco_test_frame.jpg")

            # Detect markers
            detections = detector.detect(
                img_copy, camera_matrix, dist_coeffs, None)

            # Debug output every 30 frames
            if frame_count % 30 == 0:
                print(
                    f"[DEBUG] Frame {frame_count}: Found {len(detections)} markers")
                for det in detections:
                    print(
                        f"        Marker ID {det.id}, error: {det.reprojection_error:.1f}px")

            frame_count += 1

            # Convert to ZED CustomBoxObjectData format
            custom_boxes = []
            for det in detections:
                obj = sl.CustomBoxObjectData()

                # 2D bounding box from corners
                corners = det.corners_px.astype(np.float32)
                x_min = corners[:, 0].min()
                x_max = corners[:, 0].max()
                y_min = corners[:, 1].min()
                y_max = corners[:, 1].max()

                # Create 2D box in ABCD format (top-left, top-right, bottom-right, bottom-left)
                obj.bounding_box_2d = np.array([
                    [x_min, y_min],
                    [x_max, y_min],
                    [x_max, y_max],
                    [x_min, y_max]
                ], dtype=np.float32)

                obj.label = det.id
                obj.probability = 1.0 / \
                    (1.0 + det.reprojection_error / 10.0)  # Confidence score
                obj.is_grounded = False  # Markers float in 3D space
                obj.unique_object_id = sl.generate_unique_id()

                custom_boxes.append(obj)

            lock.acquire()
            detections_data = custom_boxes
            lock.release()

            run_signal = False

        time.sleep(0.001)


def render_2d_view(image, image_scale, objects, enable_tracking):
    """
    Render 2D overlay using ZED's CV viewer utilities.

    Args:
        image: Input image
        image_scale: Scale factors [x, y]
        objects: sl.Objects
        enable_tracking: Whether tracking is enabled
    """
    overlay = image.copy()

    if HAS_GL_VIEWER:
        # Use ZED's professional rendering
        cv_viewer.render_2D(overlay, image_scale, objects, enable_tracking)
    else:
        # Fallback: simple rendering
        for obj in objects.object_list:
            box_2d = obj.bounding_box_2d
            if len(box_2d) >= 4:
                pts = box_2d[:4].astype(np.int32)
                cv2.polylines(overlay, [pts], True, (0, 255, 0), 2)

                marker_id = int(obj.label)
                position = obj.position
                distance = np.sqrt(
                    position[0]**2 + position[1]**2 + position[2]**2)

                text = f"ID:{marker_id} {distance:.2f}m"
                center_x = int(pts[:, 0].mean())
                center_y = int(pts[:, 1].min()) - 10

                cv2.putText(overlay, text, (center_x, center_y),
                            cv2.FONT_HERSHEY_SIMPLEX, 0.6, (0, 255, 255), 2)

    return overlay


def draw_stats(image, fps, num_markers, frame_count):
    """Draw FPS and stats."""
    stats = [
        f"FPS: {fps:.1f}",
        f"Markers: {num_markers}",
        f"Frames: {frame_count}"
    ]

    font = cv2.FONT_HERSHEY_SIMPLEX
    y = 30
    for stat in stats:
        cv2.putText(image, stat, (10, y), font, 0.7, (0, 255, 0), 2)
        y += 35

    return image


def main():
    """Main entry point."""
    global image_net, exit_signal, run_signal, detections_data

    parser = argparse.ArgumentParser(
        description="ArUco detector with 3D ZED visualization")
    parser.add_argument('--svo', type=str, default=None,
                        help='Optional SVO file')
    parser.add_argument('--marker-size', type=float,
                        default=0.15, help='Marker size in meters')
    parser.add_argument('--dictionary', type=str,
                        default='DICT_4X4_50', help='ArUco dictionary')
    parser.add_argument('--no-gl', action='store_true',
                        help='Disable 3D OpenGL viewer (2D only)')
    args = parser.parse_args()

    use_gl = HAS_GL_VIEWER and not args.no_gl

    print("=" * 70)
    print("ArUco Detector with 3D Visualization (ZED SDK Style)")
    print("=" * 70)
    print(f"Dictionary: {args.dictionary}")
    print(f"Marker size: {args.marker_size}m")
    print(f"SVO: {args.svo or 'Live camera'}")
    print("=" * 70)

    # Initialize ZED camera
    print("\n[1/4] Initializing ZED camera...")
    zed = sl.Camera()

    init_params = sl.InitParameters()
    if args.svo:
        init_params.set_from_svo_file(args.svo)
        init_params.svo_real_time_mode = True

    init_params.camera_resolution = sl.RESOLUTION.HD720
    init_params.depth_mode = sl.DEPTH_MODE.NEURAL
    init_params.coordinate_units = sl.UNIT.METER
    init_params.coordinate_system = sl.COORDINATE_SYSTEM.RIGHT_HANDED_Y_UP
    init_params.depth_maximum_distance = 20.0

    status = zed.open(init_params)
    if status != sl.ERROR_CODE.SUCCESS:
        print(f"✗ Failed to open ZED camera: {status}")
        return 1

    print("✓ ZED camera opened")

    # Enable positional tracking
    print("\n[2/4] Enabling positional tracking...")
    tracking_params = sl.PositionalTrackingParameters()
    zed.enable_positional_tracking(tracking_params)
    print("✓ Tracking enabled")

    # Enable custom object detection
    print("\n[3/4] Configuring object detection...")
    obj_params = sl.ObjectDetectionParameters()
    obj_params.detection_model = sl.OBJECT_DETECTION_MODEL.CUSTOM_BOX_OBJECTS
    obj_params.enable_tracking = True
    obj_params.enable_segmentation = False

    status = zed.enable_object_detection(obj_params)
    if status != sl.ERROR_CODE.SUCCESS:
        print(f"✗ Failed to enable object detection: {status}")
        zed.close()
        return 1

    print("✓ Object detection enabled")

    # Get camera calibration
    cam_info = zed.get_camera_information()
    calib = cam_info.camera_configuration.calibration_parameters.left_cam

    camera_matrix = np.array([
        [calib.fx, 0, calib.cx],
        [0, calib.fy, calib.cy],
        [0, 0, 1]
    ], dtype=np.float64)

    dist_coeffs = np.array([calib.disto[0], calib.disto[1], calib.disto[2],
                            calib.disto[3], calib.disto[4]], dtype=np.float64)

    # Initialize ArUco detector
    print("\n[4/4] Initializing ArUco detector...")
    dictionary = getattr(aruco, args.dictionary)
    detector = ArucoDetectorCore(
        dictionary=dictionary,
        marker_size_m=args.marker_size,
        min_side_px=10,  # Lower threshold for distant/small markers
        enable_async=False  # We'll handle threading ourselves
    )
    print("✓ Detector ready")
    print(f"   Dictionary: {args.dictionary}")
    print(f"   Marker size: {args.marker_size}m")

    # Start detection thread
    detection_thread = Thread(
        target=aruco_thread,
        args=(detector, camera_matrix, dist_coeffs, args.marker_size)
    )
    detection_thread.start()

    # Allocate buffers
    image_left = sl.Mat()
    image_left_cpu = sl.Mat()  # For GPU->CPU transfer
    image_left_display = sl.Mat()
    depth_map = sl.Mat()
    point_cloud = sl.Mat()
    objects = sl.Objects()

    runtime_params = sl.RuntimeParameters()
    obj_runtime_params = sl.ObjectDetectionRuntimeParameters()

    # Display resolution
    camera_res = cam_info.camera_configuration.resolution
    display_res = sl.Resolution(min(camera_res.width, 1280),
                                min(camera_res.height, 720))

    # Initialize OpenGL 3D viewer
    viewer = None
    point_cloud_res = None
    cam_w_pose = sl.Pose()

    if use_gl:
        print("\n[5/5] Initializing 3D OpenGL viewer...")
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
    tracks_resolution = None

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
        image_track_ocv = np.zeros(
            (tracks_resolution.height, tracks_resolution.width, 3), np.uint8)

    # Stats
    frame_count = 0
    start_time = time.time()
    last_fps_time = start_time
    fps_counter = 0
    current_fps = 0.0

    print("\n" + "=" * 70)
    print("Starting detection loop... (Press 'Q' to quit)")
    print("=" * 70 + "\n")

    try:
        while (not use_gl) or (viewer and viewer.is_available()):
            # Fast image capture (like ZED async samples)
            if zed.read() <= sl.ERROR_CODE.SUCCESS:
                # Get image for detection (GPU memory - faster)
                lock.acquire()
                zed.retrieve_image(image_left, sl.VIEW.LEFT,
                                   sl.MEM.GPU, display_res)

                # Convert and transfer to CPU (proper GPU workflow like YOLO sample)
                sl.Mat.convert_color(image_left, image_left_cpu,
                                     swap_RB_channels=False, remove_alpha_channels=True,
                                     memory_type=sl.MEM.GPU)
                image_left_cpu.update_cpu_from_gpu()
                image_net = image_left_cpu.get_data()
                lock.release()
                run_signal = True

                # Compute depth/tracking in parallel while detection runs
                zed.grab(runtime_params)

                # Get display image
                zed.retrieve_image(image_left_display,
                                   sl.VIEW.LEFT, sl.MEM.CPU, display_res)

                # Wait for detection to complete
                while run_signal:
                    time.sleep(0.001)

                # Ingest custom boxes
                lock.acquire()
                try:
                    global last_detections, persist_misses

                    if detections_data and len(detections_data) > 0:
                        # fresh detections this frame
                        last_detections = detections_data
                        persist_misses = 0
                        zed.ingest_custom_box_objects(detections_data)
                    else:
                        # no detections this frame -> reuse last for a few frames, then clear
                        if last_detections and persist_misses < PERSIST_FRAMES:
                            persist_misses += 1
                            zed.ingest_custom_box_objects(last_detections)
                        else:
                            persist_misses = 0
                            last_detections = []
                            zed.ingest_custom_box_objects([])  # force-clear after grace window
                finally:
                    lock.release()

                # Retrieve tracked objects
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

                # 2D rendering with professional overlay
                display_img = image_left_display.get_data()[:, :, :3].copy()
                display_img = render_2d_view(display_img, image_scale, objects,
                                             obj_params.enable_tracking)

                # Draw stats
                display_img = draw_stats(display_img, current_fps,
                                         len(objects.object_list), frame_count)

                # Tracking view (birds-eye)
                if use_gl and track_view_generator:
                    track_view_generator.generate_view(objects, cam_w_pose,
                                                       image_track_ocv, objects.is_tracked)
                    # Combine 2D view and tracking view
                    global_image = cv2.hconcat([display_img, image_track_ocv])
                    cv2.imshow(
                        "ArUco Detector - 2D View and Birds View", global_image)
                else:
                    cv2.imshow("ArUco Detector - 2D View", display_img)

                # Handle key press
                key = cv2.waitKey(1) & 0xFF
                if key == ord('q') or key == 27:
                    break
            else:
                print("End of stream")
                break

    except KeyboardInterrupt:
        print("\nInterrupted by user")

    finally:
        # Cleanup
        print("\n" + "=" * 70)
        print("Shutting down...")
        print("=" * 70)

        exit_signal = True
        detection_thread.join(timeout=2.0)

        # Close OpenGL viewer
        if viewer:
            viewer.exit()

        elapsed = time.time() - start_time
        avg_fps = frame_count / elapsed if elapsed > 0 else 0.0

        print(f"\nStatistics:")
        print(f"  Total frames: {frame_count}")
        print(f"  Average FPS: {avg_fps:.1f}")
        print(f"  Runtime: {elapsed:.1f}s")

        zed.close()
        cv2.destroyAllWindows()

        print("\n✓ Shutdown complete")

    return 0


if __name__ == '__main__':
    sys.exit(main())
