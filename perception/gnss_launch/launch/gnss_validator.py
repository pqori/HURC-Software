#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
from sensor_msgs.msg import NavSatFix
from std_msgs.msg import String, Bool, Float32
from std_srvs.srv import Trigger
import time
import math
import numpy as np
from typing import Optional, List
from collections import deque


class GNSSValidator(Node):
    """Validate GNSS signal quality and readiness for fusion"""

    def __init__(self):
        super().__init__('gnss_validator')

        # Parameters
        self.declare_parameter('gnss_topic', '/gps/fix')
        self.declare_parameter('validation_rate', 1.0)
        self.declare_parameter('min_satellites', 4)
        self.declare_parameter('max_hdop', 5.0)
        self.declare_parameter('max_vdop', 10.0)
        self.declare_parameter('position_stability_threshold', 2.0)
        self.declare_parameter('validation_window_size', 10)

        # Get parameters
        self.gnss_topic = self.get_parameter('gnss_topic').value
        validation_rate = self.get_parameter('validation_rate').value
        self.min_satellites = self.get_parameter('min_satellites').value
        self.max_hdop = self.get_parameter('max_hdop').value
        self.max_vdop = self.get_parameter('max_vdop').value
        self.stability_threshold = self.get_parameter(
            'position_stability_threshold').value
        window_size = self.get_parameter('validation_window_size').value

        # Validation state
        self.fix_history = deque(maxlen=window_size)
        self.validation_passed = False
        self.last_validation_time = 0.0

        # Subscribers
        self.create_subscription(
            NavSatFix, self.gnss_topic, self.cb_gnss_fix, 10)

        # Publishers
        self.validation_pub = self.create_publisher(
            Bool, '/gnss/validation_passed', 10)
        self.stability_pub = self.create_publisher(
            Float32, '/gnss/position_stability', 10)
        self.readiness_pub = self.create_publisher(
            String, '/gnss/fusion_readiness', 10)

        # Services
        self.create_service(Trigger, '/gnss/validate_signal',
                            self.validate_signal_service)
        self.create_service(
            Trigger, '/gnss/reset_validation', self.reset_validation)

        # Validation timer
        self.create_timer(1.0 / validation_rate, self.perform_validation)

        self.get_logger().info("🔍 GNSS Signal Validator initialized")
        self.get_logger().info(f"   Topic: {self.gnss_topic}")
        self.get_logger().info(f"   Min satellites: {self.min_satellites}")
        self.get_logger().info(f"   Max HDOP: {self.max_hdop}")

    def cb_gnss_fix(self, msg: NavSatFix):
        """Store GNSS fix for validation"""
        self.fix_history.append({
            'timestamp': time.time(),
            'fix': msg,
            'latitude': msg.latitude,
            'longitude': msg.longitude,
            'altitude': msg.altitude,
            'status': msg.status.status,
            'satellites': getattr(msg.status, 'satellites_used', 0)
        })

    def perform_validation(self):
        """Perform comprehensive GNSS validation"""
        if len(self.fix_history) < 3:
            self.validation_passed = False
            self.publish_validation_results()
            return

        try:
            # Get latest fix
            latest_fix = self.fix_history[-1]

            # Basic fix validation
            basic_valid = self.validate_basic_fix_quality(latest_fix)

            # Position stability validation
            stability_valid, stability_score = self.validate_position_stability()

            # Signal consistency validation
            consistency_valid = self.validate_signal_consistency()

            # Overall validation
            self.validation_passed = basic_valid and stability_valid and consistency_valid

            # Update validation time
            self.last_validation_time = time.time()

            # Publish results
            self.publish_validation_results(stability_score)

        except Exception as e:
            self.get_logger().error(f"GNSS validation failed: {e}")
            self.validation_passed = False
            self.publish_validation_results()

    def validate_basic_fix_quality(self, fix_data: dict) -> bool:
        """Validate basic fix quality parameters"""
        fix_msg = fix_data['fix']

        # Check fix status
        if fix_msg.status.status < 1:  # No fix
            return False

        # Check satellite count
        if fix_data['satellites'] < self.min_satellites:
            return False

        # Check HDOP (from covariance)
        if len(fix_msg.position_covariance) >= 9:
            h_cov = (
                fix_msg.position_covariance[0] + fix_msg.position_covariance[4]) / 2.0
            hdop = max(1.0, h_cov ** 0.5)
            if hdop > self.max_hdop:
                return False

        # Check for reasonable coordinates
        if (abs(fix_data['latitude']) > 90.0 or
            abs(fix_data['longitude']) > 180.0 or
                abs(fix_data['altitude']) > 10000.0):
            return False

        return True

    def validate_position_stability(self) -> tuple[bool, float]:
        """Validate position stability over time"""
        if len(self.fix_history) < 5:
            return False, 0.0

        # Calculate position variance over recent fixes
        recent_fixes = list(self.fix_history)[-5:]

        lats = [fix['latitude'] for fix in recent_fixes]
        lons = [fix['longitude'] for fix in recent_fixes]
        alts = [fix['altitude'] for fix in recent_fixes]

        # Convert to meters for stability calculation
        lat_std = np.std(lats) * 111000.0  # Degrees to meters
        lon_std = np.std(lons) * 111000.0 * \
            math.cos(math.radians(np.mean(lats)))
        alt_std = np.std(alts)

        # Overall position stability (RMS)
        position_std = math.sqrt(lat_std**2 + lon_std**2 + alt_std**2)
        stability_score = max(0.0, 1.0 - position_std /
                              self.stability_threshold)

        is_stable = position_std < self.stability_threshold

        return is_stable, stability_score

    def validate_signal_consistency(self) -> bool:
        """Validate signal consistency over time"""
        if len(self.fix_history) < 3:
            return False

        # Check for consistent fix status
        recent_statuses = [fix['status']
                           for fix in list(self.fix_history)[-3:]]

        # All recent fixes should have valid status
        return all(status >= 1 for status in recent_statuses)

    def publish_validation_results(self, stability_score: float = 0.0):
        """Publish validation results"""
        # Validation passed
        validation_msg = Bool()
        validation_msg.data = self.validation_passed
        self.validation_pub.publish(validation_msg)

        # Position stability
        stability_msg = Float32()
        stability_msg.data = stability_score
        self.stability_pub.publish(stability_msg)

        # Fusion readiness assessment
        readiness_msg = String()
        if self.validation_passed:
            if stability_score > 0.8:
                readiness_msg.data = "excellent"
            elif stability_score > 0.6:
                readiness_msg.data = "good"
            else:
                readiness_msg.data = "fair"
        else:
            readiness_msg.data = "not_ready"

        self.readiness_pub.publish(readiness_msg)

    def validate_signal_service(self, request, response):
        """Service to trigger immediate validation"""
        self.perform_validation()

        response.success = self.validation_passed
        if self.validation_passed and len(self.fix_history) > 0:
            latest = self.fix_history[-1]
            response.message = f"GNSS validation passed - ready for fusion (satellites: {latest['satellites']})"
        elif self.validation_passed:
            response.message = "GNSS validation passed - ready for fusion"
        else:
            response.message = "GNSS validation failed - not ready for fusion"

        return response

    def reset_validation(self, request, response):
        """Service to reset validation history"""
        self.fix_history.clear()
        self.validation_passed = False

        response.success = True
        response.message = "GNSS validation history reset"
        self.get_logger().info("🔄 GNSS validation reset")

        return response


def main(args=None):
    """Main entry point"""
    rclpy.init(args=args)

    try:
        validator = GNSSValidator()
        rclpy.spin(validator)
    except KeyboardInterrupt:
        pass
    finally:
        rclpy.shutdown()


if __name__ == '__main__':
    main()
