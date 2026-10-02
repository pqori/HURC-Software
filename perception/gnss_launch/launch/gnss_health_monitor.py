#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
from sensor_msgs.msg import NavSatFix
from diagnostic_msgs.msg import DiagnosticArray, DiagnosticStatus
from std_msgs.msg import String, Float32, Bool, Int32
import time
from typing import Optional
from enum import Enum


class GNSSHealth(Enum):
    """GNSS health states"""
    EXCELLENT = "excellent"    # RTK fix, high accuracy
    GOOD = "good"             # 3D fix, good accuracy
    FAIR = "fair"             # 3D fix, moderate accuracy
    POOR = "poor"             # 2D fix or high uncertainty
    NO_FIX = "no_fix"         # No valid position fix
    NO_SIGNAL = "no_signal"   # No GNSS data received


class GNSSHealthMonitor(Node):
    """Monitor GNSS receiver health and signal quality"""

    def __init__(self):
        super().__init__('gnss_health_monitor')

        # Parameters
        self.declare_parameter('gnss_topic', '/gps/fix')
        self.declare_parameter('health_check_rate', 1.0)
        self.declare_parameter('signal_timeout', 5.0)
        self.declare_parameter('min_satellites', 4)
        self.declare_parameter('hdop_excellent', 1.0)
        self.declare_parameter('hdop_good', 2.0)
        self.declare_parameter('hdop_fair', 5.0)

        # Get parameters
        self.gnss_topic = self.get_parameter('gnss_topic').value
        check_rate = self.get_parameter('health_check_rate').value
        self.signal_timeout = self.get_parameter('signal_timeout').value
        self.min_satellites = self.get_parameter('min_satellites').value
        self.hdop_excellent = self.get_parameter('hdop_excellent').value
        self.hdop_good = self.get_parameter('hdop_good').value
        self.hdop_fair = self.get_parameter('hdop_fair').value

        # State tracking
        self.current_health = GNSSHealth.NO_SIGNAL
        self.last_fix_time = 0.0
        self.current_fix: Optional[NavSatFix] = None
        self.satellite_count = 0
        self.hdop_value = 99.0
        self.signal_strength = -999.0

        # Subscribers
        self.create_subscription(
            NavSatFix, self.gnss_topic, self.cb_gnss_fix, 10)

        # Publishers
        self.health_pub = self.create_publisher(
            String, '/gnss/health_status', 10)
        self.quality_pub = self.create_publisher(
            Float32, '/gnss/signal_quality', 10)
        self.satellite_pub = self.create_publisher(
            Int32, '/gnss/satellite_count', 10)
        self.ready_pub = self.create_publisher(
            Bool, '/gnss/ready_for_fusion', 10)
        self.diagnostics_pub = self.create_publisher(
            DiagnosticArray, '/gnss/diagnostics', 10)

        # Health monitoring timer
        self.create_timer(1.0 / check_rate, self.monitor_gnss_health)

        self.get_logger().info("📡 GNSS Health Monitor initialized")
        self.get_logger().info(f"   Monitoring: {self.gnss_topic}")
        self.get_logger().info(f"   Min satellites: {self.min_satellites}")

    def cb_gnss_fix(self, msg: NavSatFix):
        """Process GNSS fix message"""
        self.current_fix = msg
        self.last_fix_time = time.time()

        # Extract quality metrics
        self.satellite_count = getattr(msg.status, 'satellites_used', 0)

        # Get HDOP from covariance (simplified)
        if len(msg.position_covariance) >= 9:
            # Approximate HDOP from position covariance
            h_cov = (msg.position_covariance[0] +
                     msg.position_covariance[4]) / 2.0
            self.hdop_value = max(1.0, h_cov ** 0.5)

        # Assess health based on fix data
        self.assess_gnss_health()

    def assess_gnss_health(self):
        """Assess current GNSS health status"""
        if not self.current_fix:
            self.current_health = GNSSHealth.NO_SIGNAL
            return

        fix_status = self.current_fix.status.status

        # Check fix type and quality
        if fix_status < 0:  # No fix
            self.current_health = GNSSHealth.NO_FIX
        elif fix_status == 0:  # Invalid fix
            self.current_health = GNSSHealth.NO_FIX
        elif fix_status == 1:  # GPS fix (autonomous)
            if self.hdop_value <= self.hdop_excellent and self.satellite_count >= 8:
                self.current_health = GNSSHealth.EXCELLENT
            elif self.hdop_value <= self.hdop_good and self.satellite_count >= 6:
                self.current_health = GNSSHealth.GOOD
            elif self.hdop_value <= self.hdop_fair and self.satellite_count >= self.min_satellites:
                self.current_health = GNSSHealth.FAIR
            else:
                self.current_health = GNSSHealth.POOR
        elif fix_status == 2:  # DGPS/RTK fix
            self.current_health = GNSSHealth.EXCELLENT
        else:
            self.current_health = GNSSHealth.FAIR

    def monitor_gnss_health(self):
        """Monitor GNSS health and publish status"""
        current_time = time.time()

        # Check for signal timeout
        if current_time - self.last_fix_time > self.signal_timeout:
            self.current_health = GNSSHealth.NO_SIGNAL

        # Publish health status
        self.publish_health_status()

        # Publish diagnostics
        self.publish_diagnostics()

    def publish_health_status(self):
        """Publish GNSS health and quality metrics"""
        # Health status
        health_msg = String()
        health_msg.data = self.current_health.value
        self.health_pub.publish(health_msg)

        # Signal quality (0-1 scale)
        quality_score = self.calculate_quality_score()
        quality_msg = Float32()
        quality_msg.data = quality_score
        self.quality_pub.publish(quality_msg)

        # Satellite count
        sat_msg = Int32()
        sat_msg.data = self.satellite_count
        self.satellite_pub.publish(sat_msg)

        # Ready for fusion
        ready_msg = Bool()
        ready_msg.data = self.is_ready_for_fusion()
        self.ready_pub.publish(ready_msg)

    def calculate_quality_score(self) -> float:
        """Calculate overall GNSS quality score (0-1)"""
        if self.current_health == GNSSHealth.NO_SIGNAL:
            return 0.0
        elif self.current_health == GNSSHealth.NO_FIX:
            return 0.1
        elif self.current_health == GNSSHealth.POOR:
            return 0.3
        elif self.current_health == GNSSHealth.FAIR:
            return 0.6
        elif self.current_health == GNSSHealth.GOOD:
            return 0.8
        elif self.current_health == GNSSHealth.EXCELLENT:
            return 1.0
        else:
            return 0.0

    def is_ready_for_fusion(self) -> bool:
        """Check if GNSS is ready for camera fusion"""
        return (self.current_health in [GNSSHealth.FAIR, GNSSHealth.GOOD, GNSSHealth.EXCELLENT] and
                self.satellite_count >= self.min_satellites and
                self.hdop_value <= self.hdop_fair)

    def publish_diagnostics(self):
        """Publish detailed diagnostics"""
        diag_array = DiagnosticArray()
        diag_array.header.stamp = self.get_clock().now().to_msg()

        # Overall GNSS status
        gnss_diag = DiagnosticStatus()
        gnss_diag.name = "GNSS Receiver"

        if self.current_health in [GNSSHealth.EXCELLENT, GNSSHealth.GOOD]:
            gnss_diag.level = DiagnosticStatus.OK
        elif self.current_health in [GNSSHealth.FAIR, GNSSHealth.POOR]:
            gnss_diag.level = DiagnosticStatus.WARN
        else:
            gnss_diag.level = DiagnosticStatus.ERROR

        gnss_diag.message = f"Health: {self.current_health.value}"

        # Add key-value diagnostics
        gnss_diag.values.append(self.make_kv(
            "Health Status", self.current_health.value))
        gnss_diag.values.append(self.make_kv(
            "Satellites", str(self.satellite_count)))
        gnss_diag.values.append(self.make_kv("HDOP", f"{self.hdop_value:.2f}"))
        gnss_diag.values.append(self.make_kv(
            "Quality Score", f"{self.calculate_quality_score():.2f}"))
        gnss_diag.values.append(self.make_kv(
            "Ready for Fusion", str(self.is_ready_for_fusion())))

        if self.current_fix:
            gnss_diag.values.append(self.make_kv(
                "Fix Type", str(self.current_fix.status.status)))
            gnss_diag.values.append(self.make_kv(
                "Latitude", f"{self.current_fix.latitude:.6f}"))
            gnss_diag.values.append(self.make_kv(
                "Longitude", f"{self.current_fix.longitude:.6f}"))
            gnss_diag.values.append(self.make_kv(
                "Altitude", f"{self.current_fix.altitude:.2f}m"))

        diag_array.status.append(gnss_diag)
        self.diagnostics_pub.publish(diag_array)

    def make_kv(self, key: str, value: str):
        """Create KeyValue diagnostic pair"""
        from diagnostic_msgs.msg import KeyValue
        kv = KeyValue()
        kv.key = key
        kv.value = value
        return kv


def main(args=None):
    """Main entry point"""
    rclpy.init(args=args)

    try:
        monitor = GNSSHealthMonitor()
        rclpy.spin(monitor)
    except KeyboardInterrupt:
        pass
    finally:
        rclpy.shutdown()


if __name__ == '__main__':
    main()
