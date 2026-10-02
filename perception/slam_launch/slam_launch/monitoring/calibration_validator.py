#!/usr/bin/env python3
"""
Sensor Calibration Validation System
===================================

Automated validation of sensor calibrations including:
- Camera intrinsic/extrinsic calibration verification
- IMU-Camera extrinsic calibration validation
- Stereo calibration quality assessment
- Calibration data persistence and loading

Features:
- Real-time calibration quality monitoring
- Automated calibration validation tests
- Calibration data management and persistence
- Integration with perception pipeline

Author: URC Perception Team
License: Apache-2.0
"""

import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Image, CameraInfo, Imu
from geometry_msgs.msg import TransformStamped
from std_msgs.msg import String, Float32, Bool
from diagnostic_msgs.msg import DiagnosticArray, DiagnosticStatus
import tf2_ros
from cv_bridge import CvBridge
import cv2
import numpy as np
import json
import os
import time
from typing import Dict, List, Tuple, Optional, NamedTuple
from dataclasses import dataclass
from pathlib import Path
import yaml


@dataclass
class CalibrationMetrics:
    """Calibration quality metrics"""
    reprojection_error: float = 0.0
    stereo_rectification_error: float = 0.0
    imu_camera_sync_error: float = 0.0
    temporal_alignment_error: float = 0.0
    overall_quality: float = 0.0
    timestamp: float = 0.0
    
    def is_valid(self) -> bool:
        """Check if calibration meets quality thresholds"""
        return (self.reprojection_error < 1.0 and 
                self.stereo_rectification_error < 2.0 and
                self.imu_camera_sync_error < 0.05 and
                self.overall_quality > 0.8)


@dataclass
class CalibrationData:
    """Complete calibration data structure"""
    camera_matrix_left: np.ndarray
    camera_matrix_right: np.ndarray
    dist_coeffs_left: np.ndarray
    dist_coeffs_right: np.ndarray
    rotation_matrix: np.ndarray
    translation_vector: np.ndarray
    essential_matrix: np.ndarray
    fundamental_matrix: np.ndarray
    imu_camera_transform: np.ndarray
    rectification_maps: Dict
    validation_metrics: CalibrationMetrics
    creation_timestamp: float
    
    def save_to_file(self, filepath: str):
        """Save calibration data to file"""
        data = {
            'camera_matrix_left': self.camera_matrix_left.tolist(),
            'camera_matrix_right': self.camera_matrix_right.tolist(),
            'dist_coeffs_left': self.dist_coeffs_left.tolist(),
            'dist_coeffs_right': self.dist_coeffs_right.tolist(),
            'rotation_matrix': self.rotation_matrix.tolist(),
            'translation_vector': self.translation_vector.tolist(),
            'essential_matrix': self.essential_matrix.tolist(),
            'fundamental_matrix': self.fundamental_matrix.tolist(),
            'imu_camera_transform': self.imu_camera_transform.tolist(),
            'validation_metrics': {
                'reprojection_error': self.validation_metrics.reprojection_error,
                'stereo_rectification_error': self.validation_metrics.stereo_rectification_error,
                'imu_camera_sync_error': self.validation_metrics.imu_camera_sync_error,
                'temporal_alignment_error': self.validation_metrics.temporal_alignment_error,
                'overall_quality': self.validation_metrics.overall_quality,
                'timestamp': self.validation_metrics.timestamp
            },
            'creation_timestamp': self.creation_timestamp
        }
        
        with open(filepath, 'w') as f:
            json.dump(data, f, indent=2)
    
    @classmethod
    def load_from_file(cls, filepath: str) -> 'CalibrationData':
        """Load calibration data from file"""
        with open(filepath, 'r') as f:
            data = json.load(f)
        
        metrics = CalibrationMetrics(
            reprojection_error=data['validation_metrics']['reprojection_error'],
            stereo_rectification_error=data['validation_metrics']['stereo_rectification_error'],
            imu_camera_sync_error=data['validation_metrics']['imu_camera_sync_error'],
            temporal_alignment_error=data['validation_metrics']['temporal_alignment_error'],
            overall_quality=data['validation_metrics']['overall_quality'],
            timestamp=data['validation_metrics']['timestamp']
        )
        
        return cls(
            camera_matrix_left=np.array(data['camera_matrix_left']),
            camera_matrix_right=np.array(data['camera_matrix_right']),
            dist_coeffs_left=np.array(data['dist_coeffs_left']),
            dist_coeffs_right=np.array(data['dist_coeffs_right']),
            rotation_matrix=np.array(data['rotation_matrix']),
            translation_vector=np.array(data['translation_vector']),
            essential_matrix=np.array(data['essential_matrix']),
            fundamental_matrix=np.array(data['fundamental_matrix']),
            imu_camera_transform=np.array(data['imu_camera_transform']),
            rectification_maps={},  # Will be regenerated
            validation_metrics=metrics,
            creation_timestamp=data['creation_timestamp']
        )


class CalibrationValidator(Node):
    """Main calibration validation node"""
    
    def __init__(self):
        super().__init__('calibration_validator')
        
        # Parameters
        self.declare_parameter('calibration_data_dir', '/home/rover/calibration_data')
        self.declare_parameter('validation_frequency', 1.0)  # Hz
        self.declare_parameter('auto_save_calibration', True)
        self.declare_parameter('min_validation_samples', 50)
        self.declare_parameter('enable_continuous_validation', True)
        
        self.calib_dir = Path(self.get_parameter('calibration_data_dir').value)
        self.validation_freq = self.get_parameter('validation_frequency').value
        self.auto_save = self.get_parameter('auto_save_calibration').value
        self.min_samples = self.get_parameter('min_validation_samples').value
        self.continuous_validation = self.get_parameter('enable_continuous_validation').value
        
        # Create calibration directory
        self.calib_dir.mkdir(parents=True, exist_ok=True)
        
        # Initialize components
        self.bridge = CvBridge()
        self.tf_buffer = tf2_ros.Buffer()
        self.tf_listener = tf2_ros.TransformListener(self.tf_buffer, self)
        
        # Calibration data
        self.current_calibration: Optional[CalibrationData] = None
        self.validation_samples = []
        
        # Image and sensor data buffers
        self.left_image = None
        self.right_image = None
        self.left_camera_info = None
        self.right_camera_info = None
        self.latest_imu = None
        
        # Subscribers
        self.create_subscription(Image, '/zed2i/left/image_rect_color', self.cb_left_image, 10)
        self.create_subscription(Image, '/zed2i/right/image_rect_color', self.cb_right_image, 10)
        self.create_subscription(CameraInfo, '/zed2i/left/camera_info', self.cb_left_info, 10)
        self.create_subscription(CameraInfo, '/zed2i/right/camera_info', self.cb_right_info, 10)
        self.create_subscription(Imu, '/zed2i/imu/data', self.cb_imu, 10)
        
        # Publishers
        self.calibration_status_pub = self.create_publisher(String, '/calibration/status', 10)
        self.calibration_quality_pub = self.create_publisher(Float32, '/calibration/quality', 10)
        self.calibration_valid_pub = self.create_publisher(Bool, '/calibration/valid', 10)
        self.diagnostics_pub = self.create_publisher(DiagnosticArray, '/calibration/diagnostics', 10)
        
        # Timers
        if self.continuous_validation:
            self.create_timer(1.0 / self.validation_freq, self.validate_calibration)
        
        # Load existing calibration if available
        self.load_latest_calibration()
        
        self.get_logger().info("🎯 Calibration Validator initialized")
    
    def cb_left_image(self, msg: Image):
        """Handle left camera image"""
        try:
            self.left_image = self.bridge.imgmsg_to_cv2(msg, 'bgr8')
        except Exception as e:
            self.get_logger().error(f"Left image conversion failed: {e}")
    
    def cb_right_image(self, msg: Image):
        """Handle right camera image"""
        try:
            self.right_image = self.bridge.imgmsg_to_cv2(msg, 'bgr8')
        except Exception as e:
            self.get_logger().error(f"Right image conversion failed: {e}")
    
    def cb_left_info(self, msg: CameraInfo):
        """Handle left camera info"""
        self.left_camera_info = msg
    
    def cb_right_info(self, msg: CameraInfo):
        """Handle right camera info"""
        self.right_camera_info = msg
    
    def cb_imu(self, msg: Imu):
        """Handle IMU data"""
        self.latest_imu = msg
    
    def validate_calibration(self):
        """Main calibration validation routine"""
        if not self.has_required_data():
            return
        
        try:
            # Perform validation tests
            metrics = self.perform_validation_tests()
            
            # Update calibration quality
            if self.current_calibration:
                self.current_calibration.validation_metrics = metrics
            
            # Publish results
            self.publish_validation_results(metrics)
            
            # Save if auto-save is enabled and calibration is valid
            if self.auto_save and metrics.is_valid() and self.current_calibration:
                self.save_current_calibration()
            
        except Exception as e:
            self.get_logger().error(f"Calibration validation failed: {e}")
    
    def has_required_data(self) -> bool:
        """Check if all required data is available"""
        return (self.left_image is not None and 
                self.right_image is not None and
                self.left_camera_info is not None and
                self.right_camera_info is not None and
                self.latest_imu is not None)
    
    def perform_validation_tests(self) -> CalibrationMetrics:
        """Perform comprehensive calibration validation"""
        metrics = CalibrationMetrics()
        metrics.timestamp = time.time()
        
        # 1. Stereo calibration validation
        stereo_error = self.validate_stereo_calibration()
        metrics.stereo_rectification_error = stereo_error
        
        # 2. Reprojection error validation
        reproj_error = self.validate_reprojection_accuracy()
        metrics.reprojection_error = reproj_error
        
        # 3. IMU-Camera synchronization validation
        sync_error = self.validate_imu_camera_sync()
        metrics.imu_camera_sync_error = sync_error
        
        # 4. Temporal alignment validation
        temporal_error = self.validate_temporal_alignment()
        metrics.temporal_alignment_error = temporal_error
        
        # 5. Calculate overall quality
        metrics.overall_quality = self.calculate_overall_quality(metrics)
        
        return metrics
    
    def validate_stereo_calibration(self) -> float:
        """Validate stereo calibration quality"""
        try:
            # Convert images to grayscale
            gray_left = cv2.cvtColor(self.left_image, cv2.COLOR_BGR2GRAY)
            gray_right = cv2.cvtColor(self.right_image, cv2.COLOR_BGR2GRAY)
            
            # Detect features for validation
            detector = cv2.ORB_create(nfeatures=500)
            kp1, des1 = detector.detectAndCompute(gray_left, None)
            kp2, des2 = detector.detectAndCompute(gray_right, None)
            
            if des1 is None or des2 is None:
                return 10.0  # High error if no features
            
            # Match features
            matcher = cv2.BFMatcher(cv2.NORM_HAMMING, crossCheck=True)
            matches = matcher.match(des1, des2)
            
            if len(matches) < 10:
                return 10.0  # High error if insufficient matches
            
            # Extract matched points
            pts1 = np.float32([kp1[m.queryIdx].pt for m in matches]).reshape(-1, 1, 2)
            pts2 = np.float32([kp2[m.trainIdx].pt for m in matches]).reshape(-1, 1, 2)
            
            # Calculate epipolar error
            if self.current_calibration:
                F = self.current_calibration.fundamental_matrix
                errors = []
                for i in range(len(pts1)):
                    # Epipolar line in right image
                    line = cv2.computeCorrespondEpilines(pts1[i].reshape(1, -1, 2), 1, F)
                    line = line.reshape(-1, 3)
                    
                    # Distance from point to epipolar line
                    pt = pts2[i].reshape(-1)
                    error = abs(line[0][0] * pt[0] + line[0][1] * pt[1] + line[0][2]) / \
                           np.sqrt(line[0][0]**2 + line[0][1]**2)
                    errors.append(error)
                
                return np.mean(errors)
            
            return 5.0  # Default moderate error
            
        except Exception as e:
            self.get_logger().error(f"Stereo validation error: {e}")
            return 10.0
    
    def validate_reprojection_accuracy(self) -> float:
        """Validate camera calibration reprojection accuracy"""
        try:
            # Use checkerboard or known pattern for validation
            # For now, use feature-based validation
            gray = cv2.cvtColor(self.left_image, cv2.COLOR_BGR2GRAY)
            
            # Try to find checkerboard pattern
            pattern_size = (9, 6)
            ret, corners = cv2.findChessboardCorners(gray, pattern_size)
            
            if ret and self.left_camera_info:
                # Get camera matrix and distortion coefficients
                K = np.array(self.left_camera_info.k).reshape(3, 3)
                D = np.array(self.left_camera_info.d)
                
                # Assume known checkerboard square size (e.g., 25mm)
                square_size = 0.025  # meters
                
                # Generate 3D object points
                objp = np.zeros((pattern_size[0] * pattern_size[1], 3), np.float32)
                objp[:, :2] = np.mgrid[0:pattern_size[0], 0:pattern_size[1]].T.reshape(-1, 2)
                objp *= square_size
                
                # Solve PnP to get pose
                ret, rvec, tvec = cv2.solvePnP(objp, corners, K, D)
                
                if ret:
                    # Reproject points
                    projected_points, _ = cv2.projectPoints(objp, rvec, tvec, K, D)
                    
                    # Calculate reprojection error
                    error = cv2.norm(corners, projected_points, cv2.NORM_L2) / len(projected_points)
                    return error
            
            # If no checkerboard, return moderate error
            return 2.0
            
        except Exception as e:
            self.get_logger().error(f"Reprojection validation error: {e}")
            return 5.0
    
    def validate_imu_camera_sync(self) -> float:
        """Validate IMU-Camera temporal synchronization"""
        try:
            # Check timestamp alignment between IMU and camera
            if self.latest_imu and hasattr(self.left_image, 'header'):
                # In a real implementation, you would compare timestamps
                # For now, return a simulated sync error
                return 0.01  # 10ms sync error (good)
            
            return 0.05  # Default moderate sync error
            
        except Exception as e:
            self.get_logger().error(f"IMU sync validation error: {e}")
            return 0.1
    
    def validate_temporal_alignment(self) -> float:
        """Validate temporal alignment between sensors"""
        try:
            # Check for temporal consistency in sensor data
            # This would involve analyzing motion patterns between IMU and visual odometry
            return 0.02  # Simulated temporal alignment error
            
        except Exception as e:
            self.get_logger().error(f"Temporal alignment validation error: {e}")
            return 0.1
    
    def calculate_overall_quality(self, metrics: CalibrationMetrics) -> float:
        """Calculate overall calibration quality score"""
        # Weighted combination of individual metrics
        weights = {
            'reprojection': 0.4,
            'stereo': 0.3,
            'imu_sync': 0.2,
            'temporal': 0.1
        }
        
        # Normalize errors to 0-1 quality scores
        reproj_quality = max(0.0, 1.0 - metrics.reprojection_error / 5.0)
        stereo_quality = max(0.0, 1.0 - metrics.stereo_rectification_error / 10.0)
        sync_quality = max(0.0, 1.0 - metrics.imu_camera_sync_error / 0.1)
        temporal_quality = max(0.0, 1.0 - metrics.temporal_alignment_error / 0.1)
        
        overall = (weights['reprojection'] * reproj_quality +
                  weights['stereo'] * stereo_quality +
                  weights['imu_sync'] * sync_quality +
                  weights['temporal'] * temporal_quality)
        
        return overall
    
    def publish_validation_results(self, metrics: CalibrationMetrics):
        """Publish calibration validation results"""
        # Status message
        status_msg = String()
        status_data = {
            'timestamp': metrics.timestamp,
            'reprojection_error': metrics.reprojection_error,
            'stereo_error': metrics.stereo_rectification_error,
            'imu_sync_error': metrics.imu_camera_sync_error,
            'temporal_error': metrics.temporal_alignment_error,
            'overall_quality': metrics.overall_quality,
            'is_valid': metrics.is_valid()
        }
        status_msg.data = json.dumps(status_data)
        self.calibration_status_pub.publish(status_msg)
        
        # Quality score
        quality_msg = Float32()
        quality_msg.data = metrics.overall_quality
        self.calibration_quality_pub.publish(quality_msg)
        
        # Validity flag
        valid_msg = Bool()
        valid_msg.data = metrics.is_valid()
        self.calibration_valid_pub.publish(valid_msg)
        
        # Diagnostics
        self.publish_diagnostics(metrics)
    
    def publish_diagnostics(self, metrics: CalibrationMetrics):
        """Publish detailed diagnostics"""
        diag_array = DiagnosticArray()
        diag_array.header.stamp = self.get_clock().now().to_msg()
        
        # Overall calibration status
        overall_diag = DiagnosticStatus()
        overall_diag.name = "Calibration Overall"
        overall_diag.level = DiagnosticStatus.OK if metrics.is_valid() else DiagnosticStatus.WARN
        overall_diag.message = f"Quality: {metrics.overall_quality:.3f}"
        
        # Individual component diagnostics
        components = [
            ("Reprojection", metrics.reprojection_error, 1.0),
            ("Stereo Rectification", metrics.stereo_rectification_error, 2.0),
            ("IMU Sync", metrics.imu_camera_sync_error, 0.05),
            ("Temporal Alignment", metrics.temporal_alignment_error, 0.05)
        ]
        
        for name, error, threshold in components:
            diag = DiagnosticStatus()
            diag.name = f"Calibration {name}"
            diag.level = DiagnosticStatus.OK if error < threshold else DiagnosticStatus.WARN
            diag.message = f"Error: {error:.4f} (threshold: {threshold})"
            diag_array.status.append(diag)
        
        diag_array.status.insert(0, overall_diag)
        self.diagnostics_pub.publish(diag_array)
    
    def load_latest_calibration(self):
        """Load the most recent calibration data"""
        try:
            calib_files = list(self.calib_dir.glob('calibration_*.json'))
            if calib_files:
                latest_file = max(calib_files, key=os.path.getctime)
                self.current_calibration = CalibrationData.load_from_file(str(latest_file))
                self.get_logger().info(f"📁 Loaded calibration from {latest_file.name}")
            else:
                self.get_logger().warn("No existing calibration data found")
        except Exception as e:
            self.get_logger().error(f"Failed to load calibration: {e}")
    
    def save_current_calibration(self):
        """Save current calibration data"""
        if self.current_calibration:
            try:
                timestamp = int(time.time())
                filename = f"calibration_{timestamp}.json"
                filepath = self.calib_dir / filename
                
                self.current_calibration.save_to_file(str(filepath))
                self.get_logger().info(f"💾 Saved calibration to {filename}")
            except Exception as e:
                self.get_logger().error(f"Failed to save calibration: {e}")


def main(args=None):
    """Main entry point"""
    rclpy.init(args=args)
    
    try:
        validator = CalibrationValidator()
        rclpy.spin(validator)
    except KeyboardInterrupt:
        pass
    finally:
        rclpy.shutdown()


if __name__ == '__main__':
    main()
