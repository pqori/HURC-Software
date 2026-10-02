#!/usr/bin/env python3
"""
Pure ArUco detection engine - deterministic, no ROS/ZED dependencies.

Provides both synchronous and asynchronous detection with configurable threading.
"""

import cv2
import numpy as np
from cv2 import aruco
from typing import List, Optional, Callable, Tuple
from dataclasses import dataclass
from threading import Thread, Lock, Event
from collections import deque
import time


def estimate_pose_single_markers(corners, marker_size, camera_matrix, dist_coeffs):
    """
    Estimate pose for ArUco markers (compatible with OpenCV 4.7+).
    
    Args:
        corners: Detected marker corners
        marker_size: Marker size in same units as camera matrix
        camera_matrix: 3x3 camera intrinsic matrix
        dist_coeffs: Distortion coefficients
        
    Returns:
        rvecs, tvecs: Rotation and translation vectors
    """
    rvecs = []
    tvecs = []
    
    # Define 3D points for a square marker
    half_size = marker_size / 2.0
    obj_points = np.array([
        [-half_size, half_size, 0],
        [half_size, half_size, 0],
        [half_size, -half_size, 0],
        [-half_size, -half_size, 0]
    ], dtype=np.float32)
    
    for corner in corners:
        # Solve PnP for each marker
        success, rvec, tvec = cv2.solvePnP(
            obj_points,
            corner,
            camera_matrix,
            dist_coeffs,
            flags=cv2.SOLVEPNP_IPPE_SQUARE
        )
        if success:
            rvecs.append(rvec)
            tvecs.append(tvec)
        else:
            rvecs.append(np.zeros((3, 1)))
            tvecs.append(np.zeros((3, 1)))
    
    return np.array(rvecs), np.array(tvecs), obj_points


@dataclass
class MarkerDetection:
    """Single marker detection result."""
    id: int
    rvec: np.ndarray  # 3x1 rotation vector
    tvec: np.ndarray  # 3x1 translation vector (mm)
    corners_px: np.ndarray  # 4x2 pixel coordinates
    reprojection_error: float  # RMS pixel error
    range_m: Optional[float] = None  # Distance in meters (if depth available)
    pose_cov: Optional[np.ndarray] = None  # 6x6 pose covariance (future)


class ArucoDetectorCore:
    """
    Core ArUco marker detector with sync and async modes.
    
    Thread-safe, deterministic, pure OpenCV implementation.
    """
    
    def __init__(
        self,
        dictionary: int = aruco.DICT_4X4_50,
        marker_size_m: float = 0.20,
        min_side_px: int = 20,
        adaptive_thresh_win_size_min: int = 3,
        adaptive_thresh_win_size_max: int = 23,
        adaptive_thresh_win_size_step: int = 10,
        max_queue_size: int = 2,
        enable_async: bool = True
    ):
        """
        Initialize detector with ArUco parameters.
        
        Args:
            dictionary: ArUco dictionary (e.g., aruco.DICT_4X4_50)
            marker_size_m: Physical marker size in meters (edge length)
            min_side_px: Minimum marker side in pixels to accept
            adaptive_thresh_win_size_*: ArUco adaptive threshold params
            max_queue_size: Max frames to queue before dropping (async mode)
            enable_async: Enable async detection thread
        """
        self.marker_size_mm = marker_size_m * 1000.0  # OpenCV uses mm
        
        # ArUco dictionary and parameters
        self.dictionary = aruco.getPredefinedDictionary(dictionary)
        self.params = aruco.DetectorParameters()
        self.params.minMarkerPerimeterRate = min_side_px / 1000.0
        self.params.adaptiveThreshWinSizeMin = adaptive_thresh_win_size_min
        self.params.adaptiveThreshWinSizeMax = adaptive_thresh_win_size_max
        self.params.adaptiveThreshWinSizeStep = adaptive_thresh_win_size_step
        
        # IMPROVEMENTS: Better detection
        self.params.cornerRefinementMethod = aruco.CORNER_REFINE_SUBPIX  # Sub-pixel accuracy
        self.params.cornerRefinementWinSize = 5  # Refinement window
        self.params.cornerRefinementMaxIterations = 30  # Max iterations
        self.params.cornerRefinementMinAccuracy = 0.01  # Min accuracy
        self.params.polygonalApproxAccuracyRate = 0.03  # More lenient polygon fitting
        
        # Create ArUco detector (OpenCV 4.7+ API)
        self.aruco_detector = aruco.ArucoDetector(self.dictionary, self.params)
        
        # Async processing
        self.enable_async = enable_async
        self.max_queue_size = max_queue_size
        self.lock = Lock()
        self.frame_queue = deque(maxlen=max_queue_size)
        self.result_queue = deque(maxlen=1)  # Keep only latest result
        self.shutdown_event = Event()
        
        # Statistics
        self.frames_processed = 0
        self.frames_dropped = 0
        self.last_detection_time = 0.0
        
        # Worker thread
        self.worker_thread = None
        if enable_async:
            self.worker_thread = Thread(target=self._detection_loop, daemon=True)
            self.worker_thread.start()
    
    def detect(
        self,
        image: np.ndarray,
        camera_matrix: np.ndarray,
        dist_coeffs: np.ndarray,
        depth_sampler: Optional[Callable[[int, int], float]] = None
    ) -> List[MarkerDetection]:
        """
        Synchronous detection - blocks until complete.
        
        Args:
            image: Input image (BGR or grayscale)
            camera_matrix: 3x3 camera intrinsic matrix
            dist_coeffs: Distortion coefficients (4, 5, 8, 12, or 14 elements)
            depth_sampler: Optional callback(u, v) -> depth_m
            
        Returns:
            List of MarkerDetection objects
        """
        return self._process_frame(image, camera_matrix, dist_coeffs, depth_sampler)
    
    def detect_async(
        self,
        image: np.ndarray,
        camera_matrix: np.ndarray,
        dist_coeffs: np.ndarray,
        depth_sampler: Optional[Callable[[int, int], float]] = None
    ):
        """
        Non-blocking detection - queues frame for processing.
        
        Drops frames if queue is full (backpressure handling).
        Use get_detections() to retrieve results.
        """
        if not self.enable_async:
            raise RuntimeError("Async mode not enabled. Set enable_async=True in constructor.")
        
        with self.lock:
            # Drop frame if queue full
            if len(self.frame_queue) >= self.max_queue_size:
                self.frames_dropped += 1
                return
            
            # Queue frame for processing
            self.frame_queue.append({
                'image': image.copy(),  # Copy to avoid race conditions
                'camera_matrix': camera_matrix,
                'dist_coeffs': dist_coeffs,
                'depth_sampler': depth_sampler,
                'timestamp': time.time()
            })
    
    def get_detections(self, timeout: float = 0.0) -> Optional[List[MarkerDetection]]:
        """
        Retrieve latest detection results (async mode).
        
        Args:
            timeout: Max time to wait for results (seconds). 0 = non-blocking.
            
        Returns:
            List of detections or None if no results available
        """
        if not self.enable_async:
            raise RuntimeError("Async mode not enabled.")
        
        start_time = time.time()
        while True:
            with self.lock:
                if self.result_queue:
                    return self.result_queue.popleft()
            
            if timeout == 0 or (time.time() - start_time) > timeout:
                return None
            
            time.sleep(0.001)
    
    def _detection_loop(self):
        """Worker thread for async detection."""
        while not self.shutdown_event.is_set():
            # Get next frame
            frame_data = None
            with self.lock:
                if self.frame_queue:
                    frame_data = self.frame_queue.popleft()
            
            if frame_data is None:
                time.sleep(0.001)
                continue
            
            # Process frame
            detections = self._process_frame(
                frame_data['image'],
                frame_data['camera_matrix'],
                frame_data['dist_coeffs'],
                frame_data['depth_sampler']
            )
            
            # Store result
            with self.lock:
                self.result_queue.append(detections)
                self.last_detection_time = time.time()
    
    def _process_frame(
        self,
        image: np.ndarray,
        camera_matrix: np.ndarray,
        dist_coeffs: np.ndarray,
        depth_sampler: Optional[Callable[[int, int], float]]
    ) -> List[MarkerDetection]:
        """Core detection logic."""
        # Convert to grayscale if needed
        if len(image.shape) == 3:
            gray = cv2.cvtColor(image, cv2.COLOR_BGR2GRAY)
        else:
            gray = image
        
        # Detect markers (OpenCV 4.7+ API)
        corners, ids, rejected = self.aruco_detector.detectMarkers(gray)
        
        detections = []
        
        if ids is not None and len(ids) > 0:
            # Estimate pose for each marker
            rvecs, tvecs, obj_points = estimate_pose_single_markers(
                corners, self.marker_size_mm, camera_matrix, dist_coeffs
            )
            
            for i, marker_id in enumerate(ids.flatten()):
                # Compute reprojection error
                projected, _ = cv2.projectPoints(
                    obj_points, rvecs[i], tvecs[i], camera_matrix, dist_coeffs
                )
                error = np.sqrt(np.mean((corners[i] - projected) ** 2))
                
                # Get marker center for depth sampling
                range_m = None
                if depth_sampler is not None:
                    center_px = corners[i][0].mean(axis=0)
                    u, v = int(center_px[0]), int(center_px[1])
                    range_m = depth_sampler(u, v)
                
                detections.append(MarkerDetection(
                    id=int(marker_id),
                    rvec=rvecs[i],
                    tvec=tvecs[i],
                    corners_px=corners[i][0],
                    reprojection_error=float(error),
                    range_m=range_m
                ))
        
        self.frames_processed += 1
        return detections
    
    def get_stats(self) -> dict:
        """Get processing statistics."""
        return {
            'frames_processed': self.frames_processed,
            'frames_dropped': self.frames_dropped,
            'queue_depth': len(self.frame_queue),
            'last_detection_age': time.time() - self.last_detection_time if self.last_detection_time > 0 else None
        }
    
    def shutdown(self):
        """Gracefully shutdown async worker."""
        if self.worker_thread:
            self.shutdown_event.set()
            self.worker_thread.join(timeout=2.0)


def rvec_tvec_to_pose_matrix(rvec: np.ndarray, tvec: np.ndarray) -> np.ndarray:
    """
    Convert OpenCV rvec/tvec to 4x4 pose matrix.
    
    Returns:
        4x4 homogeneous transformation matrix
    """
    R, _ = cv2.Rodrigues(rvec)
    T = np.eye(4)
    T[:3, :3] = R
    T[:3, 3] = tvec.flatten() / 1000.0  # Convert mm to meters
    return T


def pose_matrix_to_ros_pose(T: np.ndarray):
    """
    Convert 4x4 pose matrix to ROS geometry_msgs/Pose representation.
    
    Returns:
        (position, orientation) as numpy arrays for easy conversion
        position: [x, y, z] in meters
        orientation: [x, y, z, w] quaternion
    """
    from scipy.spatial.transform import Rotation
    
    position = T[:3, 3]
    rotation = Rotation.from_matrix(T[:3, :3])
    quaternion = rotation.as_quat()  # [x, y, z, w]
    
    return position, quaternion

