#!/usr/bin/env python3
"""
ZED camera interface adaptors for ArUco detection.

Provides two thin adaptors:
1. Standalone: Direct ZED SDK access
2. ROS Bridge: Adapts ROS topics to core interface
"""

import numpy as np
import cv2
from typing import Optional, Callable, Tuple
from dataclasses import dataclass


@dataclass
class CameraCalibration:
    """Camera calibration data."""
    camera_matrix: np.ndarray  # 3x3 K matrix
    dist_coeffs: np.ndarray    # Distortion coefficients
    width: int
    height: int
    frame_id: str


class StandaloneZEDAdaptor:
    """
    Direct ZED SDK adaptor for standalone testing.
    
    Provides images, calibration, and depth sampling from ZED SDK.
    """
    
    def __init__(self, svo_path: Optional[str] = None):
        """
        Initialize ZED camera.
        
        Args:
            svo_path: Optional SVO file path (None = live camera)
        """
        try:
            import pyzed.sl as sl
        except ImportError:
            raise ImportError("pyzed not available. Install ZED SDK Python API.")
        
        self.sl = sl
        self.zed = sl.Camera()
        
        # Configure camera
        init_params = sl.InitParameters()
        init_params.camera_resolution = sl.RESOLUTION.HD720
        init_params.depth_mode = sl.DEPTH_MODE.NEURAL
        init_params.coordinate_units = sl.UNIT.METER
        init_params.coordinate_system = sl.COORDINATE_SYSTEM.RIGHT_HANDED_Y_UP
        init_params.depth_maximum_distance = 20.0  # meters
        
        if svo_path:
            init_params.set_from_svo_file(svo_path)
            init_params.svo_real_time_mode = True
        
        # Open camera
        status = self.zed.open(init_params)
        if status != sl.ERROR_CODE.SUCCESS:
            raise RuntimeError(f"ZED camera failed to open: {status}")
        
        # Enable positional tracking (needed for object detection)
        tracking_params = sl.PositionalTrackingParameters()
        self.zed.enable_positional_tracking(tracking_params)
        
        # Allocate buffers
        self.image_left = sl.Mat()
        self.depth_map = sl.Mat()
        self.point_cloud = sl.Mat()
        
        # Cache calibration
        self.calibration = self._get_calibration()
    
    def _get_calibration(self) -> CameraCalibration:
        """Extract camera calibration from ZED."""
        cam_info = self.zed.get_camera_information()
        calib = cam_info.camera_configuration.calibration_parameters.left_cam
        
        # Build camera matrix
        K = np.array([
            [calib.fx, 0, calib.cx],
            [0, calib.fy, calib.cy],
            [0, 0, 1]
        ], dtype=np.float64)
        
        # Distortion coefficients (k1, k2, p1, p2, k3)
        dist = np.array([
            calib.disto[0], calib.disto[1],  # k1, k2
            calib.disto[2], calib.disto[3],  # p1, p2
            calib.disto[4]                    # k3
        ], dtype=np.float64)
        
        res = cam_info.camera_configuration.resolution
        
        return CameraCalibration(
            camera_matrix=K,
            dist_coeffs=dist,
            width=res.width,
            height=res.height,
            frame_id="zed2i_left_camera_optical_frame"
        )
    
    def grab_frame(self) -> Tuple[bool, np.ndarray, Callable[[int, int], float]]:
        """
        Grab a new frame from ZED.
        
        Returns:
            (success, image_bgr, depth_sampler_callback)
        """
        # Grab frame
        if self.zed.grab() != self.sl.ERROR_CODE.SUCCESS:
            return False, None, None
        
        # Retrieve left image
        self.zed.retrieve_image(self.image_left, self.sl.VIEW.LEFT)
        image_bgr = self.image_left.get_data()[:, :, :3]  # Drop alpha
        
        # Retrieve depth
        self.zed.retrieve_measure(self.depth_map, self.sl.MEASURE.DEPTH)
        
        # Create depth sampler
        def depth_sampler(u: int, v: int) -> Optional[float]:
            """Sample depth at pixel (u, v) in meters."""
            if 0 <= u < self.depth_map.get_width() and 0 <= v < self.depth_map.get_height():
                err, depth = self.depth_map.get_value(u, v)
                if err == self.sl.ERROR_CODE.SUCCESS and np.isfinite(depth):
                    return float(depth)
            return None
        
        return True, image_bgr, depth_sampler
    
    def get_calibration(self) -> CameraCalibration:
        """Get camera calibration."""
        return self.calibration
    
    def get_point_cloud(self) -> np.ndarray:
        """
        Retrieve point cloud for 3D visualization.
        
        Returns:
            Nx4 array (XYZRGBA)
        """
        self.zed.retrieve_measure(self.point_cloud, self.sl.MEASURE.XYZRGBA)
        return self.point_cloud.get_data()
    
    def close(self):
        """Close camera."""
        self.zed.close()


class ROSBridgeAdaptor:
    """
    ROS topics bridge adaptor.
    
    Subscribes to ZED wrapper topics and rebuilds the same inputs
    the core detector expects. Ensures calibration paths match standalone.
    """
    
    def __init__(self, node):
        """
        Initialize ROS bridge.
        
        Args:
            node: rclpy Node instance for subscriptions
        """
        self.node = node
        self.logger = node.get_logger()
        
        # Cached data
        self.calibration: Optional[CameraCalibration] = None
        self.latest_depth: Optional[np.ndarray] = None
        self.image_width = 0
        self.image_height = 0
        
        # Parameters
        self.use_depth = node.get_parameter('use_depth').value
        
    def set_camera_info(self, msg):
        """
        Process CameraInfo message to extract calibration.
        
        Args:
            msg: sensor_msgs/CameraInfo
        """
        if self.calibration is not None:
            return  # Already initialized
        
        # Build camera matrix from CameraInfo
        K = np.array(msg.k).reshape(3, 3)
        dist = np.array(msg.d)
        
        self.calibration = CameraCalibration(
            camera_matrix=K,
            dist_coeffs=dist,
            width=msg.width,
            height=msg.height,
            frame_id=msg.header.frame_id
        )
        
        self.image_width = msg.width
        self.image_height = msg.height
        
        self.logger.info(f"Camera calibration loaded: {msg.width}x{msg.height}")
    
    def set_depth_image(self, depth_image: np.ndarray):
        """
        Store latest depth image (from /depth/depth_registered).
        
        Args:
            depth_image: HxW float32 array (meters)
        """
        self.latest_depth = depth_image
    
    def get_calibration(self) -> Optional[CameraCalibration]:
        """Get camera calibration."""
        return self.calibration
    
    def create_depth_sampler(self) -> Optional[Callable[[int, int], float]]:
        """
        Create a depth sampler callback from latest depth image.
        
        Returns:
            Callback function or None if depth unavailable/disabled
        """
        if not self.use_depth or self.latest_depth is None:
            return None
        
        depth_map = self.latest_depth  # Capture in closure
        
        def sampler(u: int, v: int) -> Optional[float]:
            """Sample depth at pixel (u, v)."""
            h, w = depth_map.shape[:2]
            if 0 <= u < w and 0 <= v < h:
                depth = depth_map[v, u]
                if np.isfinite(depth) and depth > 0:
                    return float(depth)
            return None
        
        return sampler
    
    def is_ready(self) -> bool:
        """Check if calibration is available."""
        return self.calibration is not None


def depth_image_from_point_cloud(point_cloud_msg) -> Optional[np.ndarray]:
    """
    Extract depth map from PointCloud2 message.
    
    Args:
        point_cloud_msg: sensor_msgs/PointCloud2
        
    Returns:
        HxW float32 depth map (meters) or None
    """
    try:
        import sensor_msgs_py.point_cloud2 as pc2
        
        # Read points
        points = pc2.read_points(
            point_cloud_msg,
            field_names=("x", "y", "z"),
            skip_nans=False
        )
        
        # Reshape to image
        h = point_cloud_msg.height
        w = point_cloud_msg.width
        depth = np.zeros((h, w), dtype=np.float32)
        
        for i, (x, y, z) in enumerate(points):
            row = i // w
            col = i % w
            # Depth is Z coordinate (forward)
            if np.isfinite(z):
                depth[row, col] = z
        
        return depth
    
    except Exception as e:
        return None

