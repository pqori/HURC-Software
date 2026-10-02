#!/usr/bin/env python3
"""
ZED GNSS Fusion Node

Fuses ZED camera VIO with external GNSS data using ZED SDK Fusion module.
Publishes fused poses in both local and geographic coordinates.
"""

import rclpy
from rclpy.node import Node
from sensor_msgs.msg import NavSatFix
from geometry_msgs.msg import PoseStamped, TransformStamped
from geographic_msgs.msg import GeoPoseStamped, GeoPoint
from nav_msgs.msg import Odometry, Path
from std_msgs.msg import String
from diagnostic_msgs.msg import DiagnosticArray, DiagnosticStatus, KeyValue
import pyzed.sl as sl
from tf2_ros import TransformBroadcaster
import time


class ZedGnssFusion(Node):
    """Main fusion node integrating ZED camera with GNSS"""

    def __init__(self):
        super().__init__('zed_gnss_fusion')

        # Parameters
        self.declare_parameter('camera_sn', 0)
        self.declare_parameter('camera_resolution', 'HD1080')
        self.declare_parameter('gnss_topic', '/gps/fix')
        self.declare_parameter('target_yaw_uncertainty', 0.1)
        self.declare_parameter('target_translation_uncertainty', 0.15)
        self.declare_parameter('enable_reinitialization', True)
        self.declare_parameter('gnss_vio_reinit_threshold', 5.0)
        self.declare_parameter('publish_tf', True)
        self.declare_parameter('publish_path', True)
        self.declare_parameter('path_max_length', 1000)

        # Get parameters
        self.camera_sn = self.get_parameter('camera_sn').value
        self.gnss_topic = self.get_parameter('gnss_topic').value
        self.publish_tf_param = self.get_parameter('publish_tf').value
        self.publish_path_param = self.get_parameter('publish_path').value
        self.path_max_length = self.get_parameter('path_max_length').value

        # State
        self.camera = None
        self.fusion = None
        self.calibration_complete = False
        self.last_gnss_time = 0.0
        self.frame_count = 0

        # Paths for visualization
        self.fused_path = Path()
        self.fused_path.header.frame_id = 'map'

        # TF broadcaster
        if self.publish_tf_param:
            self.tf_broadcaster = TransformBroadcaster(self)

        # Publishers
        self.fused_odom_pub = self.create_publisher(
            Odometry, '~/fused_odom', 10)
        self.geo_pose_pub = self.create_publisher(
            GeoPoseStamped, '~/geo_pose', 10)
        self.fused_path_pub = self.create_publisher(Path, '~/fused_path', 10)
        self.calibration_status_pub = self.create_publisher(
            String, '~/calibration_status', 10)
        self.diagnostics_pub = self.create_publisher(
            DiagnosticArray, '~/diagnostics', 10)

        # Subscribers
        self.gnss_sub = self.create_subscription(
            NavSatFix, self.gnss_topic, self.gnss_callback, 10)

        # Initialize ZED and Fusion
        if not self.initialize_zed():
            self.get_logger().error('Failed to initialize ZED camera')
            raise RuntimeError('ZED initialization failed')

        if not self.initialize_fusion():
            self.get_logger().error('Failed to initialize Fusion module')
            raise RuntimeError('Fusion initialization failed')

        # Processing timer
        self.create_timer(0.02, self.process_loop)  # 50 Hz

        # Diagnostics timer
        self.create_timer(1.0, self.publish_diagnostics)

        self.get_logger().info('🌍 ZED GNSS Fusion Node initialized')
        self.get_logger().info(f'   Subscribing to: {self.gnss_topic}')
        self.get_logger().info('   Move camera to calibrate VIO/GNSS alignment')

    def initialize_zed(self):
        """Initialize ZED camera with positional tracking"""
        try:
            self.camera = sl.Camera()

            # Camera initialization parameters
            init_params = sl.InitParameters()
            init_params.camera_resolution = getattr(sl.RESOLUTION,
                                                    self.get_parameter('camera_resolution').value)
            init_params.coordinate_units = sl.UNIT.METER
            init_params.coordinate_system = sl.COORDINATE_SYSTEM.RIGHT_HANDED_Y_UP

            if self.camera_sn > 0:
                init_params.set_from_serial_number(self.camera_sn)

            # Open camera
            status = self.camera.open(init_params)
            if status != sl.ERROR_CODE.SUCCESS:
                self.get_logger().error(f'Camera open failed: {status}')
                return False

            # Enable positional tracking
            tracking_params = sl.PositionalTrackingParameters()
            tracking_params.mode = sl.POSITIONAL_TRACKING_MODE.GEN_2
            tracking_params.enable_imu_fusion = True  # Critical for GNSS fusion
            tracking_params.set_gravity_as_origin = True
            tracking_params.enable_area_memory = False

            status = self.camera.enable_positional_tracking(tracking_params)
            if status != sl.ERROR_CODE.SUCCESS:
                self.get_logger().error(f'Tracking enable failed: {status}')
                return False

            # Start publishing for fusion
            comm_params = sl.CommunicationParameters()
            self.camera.start_publishing(comm_params)

            # Warm up camera
            if self.camera.grab() != sl.ERROR_CODE.SUCCESS:
                self.get_logger().error('Camera grab failed during warmup')
                return False

            camera_info = self.camera.get_camera_information()
            self.get_logger().info(
                f'✅ ZED Camera opened: SN {camera_info.serial_number}')

            return True

        except Exception as e:
            self.get_logger().error(f'ZED initialization exception: {e}')
            return False

    def initialize_fusion(self):
        """Initialize Fusion module with GNSS"""
        try:
            self.fusion = sl.Fusion()

            # Fusion initialization
            fusion_init_params = sl.InitFusionParameters()
            fusion_init_params.coordinate_system = sl.COORDINATE_SYSTEM.RIGHT_HANDED_Y_UP
            fusion_init_params.coordinate_units = sl.UNIT.METER
            fusion_init_params.verbose = False

            status = self.fusion.init(fusion_init_params)
            if status != sl.FUSION_ERROR_CODE.SUCCESS:
                self.get_logger().error(f'Fusion init failed: {status}')
                return False

            # Subscribe camera to fusion
            camera_info = self.camera.get_camera_information()
            uuid = sl.CameraIdentifier(camera_info.serial_number)
            comm_params = sl.CommunicationParameters()

            status = self.fusion.subscribe(
                uuid, comm_params, sl.Transform(0, 0, 0))
            if status != sl.FUSION_ERROR_CODE.SUCCESS:
                self.get_logger().error(f'Fusion subscribe failed: {status}')
                return False

            # Configure GNSS fusion parameters
            pt_fusion_params = sl.PositionalTrackingFusionParameters()
            pt_fusion_params.enable_GNSS_fusion = True

            gnss_calib = sl.GNSSCalibrationParameters()
            gnss_calib.target_yaw_uncertainty = self.get_parameter(
                'target_yaw_uncertainty').value
            gnss_calib.enable_translation_uncertainty_target = True
            gnss_calib.target_translation_uncertainty = self.get_parameter(
                'target_translation_uncertainty').value
            gnss_calib.enable_reinitialization = self.get_parameter(
                'enable_reinitialization').value
            gnss_calib.gnss_vio_reinit_threshold = self.get_parameter(
                'gnss_vio_reinit_threshold').value

            pt_fusion_params.gnss_calibration_parameters = gnss_calib

            status = self.fusion.enable_positionnal_tracking(pt_fusion_params)
            if status != sl.FUSION_ERROR_CODE.SUCCESS:
                self.get_logger().error(
                    f'Fusion tracking enable failed: {status}')
                return False

            self.get_logger().info('✅ Fusion module initialized with GNSS')

            return True

        except Exception as e:
            self.get_logger().error(f'Fusion initialization exception: {e}')
            return False

    def gnss_callback(self, msg: NavSatFix):
        """Receive GNSS data and ingest into fusion"""
        try:
            # Check for valid fix
            if msg.status.status < 0:
                return

            # Create GNSSData
            gnss_data = sl.GNSSData()
            gnss_data.set_coordinates(
                msg.latitude, msg.longitude, msg.altitude, False)

            # Set timestamp
            gnss_data.ts = sl.Timestamp()
            gnss_data.ts.set_milliseconds(int(time.time() * 1000))

            # Set covariance
            if len(msg.position_covariance) == 9:
                gnss_data.position_covariances = list(msg.position_covariance)
            else:
                # Default covariance if not provided
                default_cov = 2.0  # 2 meter standard deviation
                gnss_data.position_covariances = [
                    default_cov**2, 0, 0,
                    0, default_cov**2, 0,
                    0, 0, (default_cov*2)**2
                ]

            # Set GNSS status
            status_map = {
                -1: sl.GNSS_STATUS.UNKNOWN,
                0: sl.GNSS_STATUS.SINGLE,
                1: sl.GNSS_STATUS.DGNSS,
                2: sl.GNSS_STATUS.RTK_FIX
            }
            gnss_data.gnss_status = status_map.get(
                msg.status.status, sl.GNSS_STATUS.SINGLE)

            # Ingest into fusion
            ingest_status = self.fusion.ingest_gnss_data(gnss_data)
            if ingest_status != sl.FUSION_ERROR_CODE.SUCCESS:
                if ingest_status != sl.FUSION_ERROR_CODE.NO_NEW_DATA_AVAILABLE:
                    self.get_logger().warning(
                        f'GNSS ingest error: {ingest_status}')

            self.last_gnss_time = time.time()

        except Exception as e:
            self.get_logger().error(f'GNSS callback exception: {e}')

    def process_loop(self):
        """Main processing loop"""
        try:
            # Grab camera frame
            if self.camera.grab() != sl.ERROR_CODE.SUCCESS:
                return

            self.frame_count += 1

            # Process fusion
            if self.fusion.process() != sl.FUSION_ERROR_CODE.SUCCESS:
                return

            # Get fused position in camera frame
            fused_pose = sl.Pose()
            tracking_state = self.fusion.get_position(
                fused_pose, sl.REFERENCE_FRAME.WORLD)

            if tracking_state != sl.POSITIONAL_TRACKING_STATE.OK:
                return

            # Publish fused odometry
            self.publish_fused_odometry(fused_pose)

            # Check calibration status
            calib_std = self.fusion.get_current_gnss_calibration_std()

            if not self.calibration_complete:
                if calib_std.translation < 0.2 and calib_std.yaw < 0.15:
                    self.calibration_complete = True
                    self.get_logger().info('✅ GNSS/VIO Calibration complete!')
                    self.get_logger().info(
                        f'   Translation uncertainty: {calib_std.translation:.3f} m')
                    self.get_logger().info(
                        f'   Yaw uncertainty: {calib_std.yaw:.3f} rad')

                    # Publish status
                    status_msg = String()
                    status_msg.data = 'converged'
                    self.calibration_status_pub.publish(status_msg)

            # Get geographic pose (only available after calibration)
            geo_pose = sl.GeoPose()
            geo_status = self.fusion.get_geo_pose(geo_pose)

            if geo_status == sl.GNSS_FUSION_STATUS.OK:
                self.publish_geo_pose(geo_pose)

                # Add to path
                if self.publish_path_param and self.frame_count % 5 == 0:
                    self.add_to_path(geo_pose)

        except Exception as e:
            self.get_logger().error(f'Process loop exception: {e}')

    def publish_fused_odometry(self, pose: sl.Pose):
        """Publish fused odometry"""
        odom_msg = Odometry()
        odom_msg.header.stamp = self.get_clock().now().to_msg()
        odom_msg.header.frame_id = 'map'
        odom_msg.child_frame_id = 'base_link'

        # Position
        translation = pose.get_translation()
        odom_msg.pose.pose.position.x = translation.get()[0]
        odom_msg.pose.pose.position.y = translation.get()[1]
        odom_msg.pose.pose.position.z = translation.get()[2]

        # Orientation
        orientation = pose.get_orientation()
        odom_msg.pose.pose.orientation.x = orientation.get()[0]
        odom_msg.pose.pose.orientation.y = orientation.get()[1]
        odom_msg.pose.pose.orientation.z = orientation.get()[2]
        odom_msg.pose.pose.orientation.w = orientation.get()[3]

        # Covariance (simplified)
        odom_msg.pose.covariance[0] = 0.01  # x
        odom_msg.pose.covariance[7] = 0.01  # y
        odom_msg.pose.covariance[14] = 0.01  # z
        odom_msg.pose.covariance[21] = 0.01  # roll
        odom_msg.pose.covariance[28] = 0.01  # pitch
        odom_msg.pose.covariance[35] = 0.01  # yaw

        self.fused_odom_pub.publish(odom_msg)

        # Publish TF if enabled
        if self.publish_tf_param:
            self.publish_transform(odom_msg)

    def publish_geo_pose(self, geo_pose: sl.GeoPose):
        """Publish geographic pose"""
        geo_msg = GeoPoseStamped()
        geo_msg.header.stamp = self.get_clock().now().to_msg()
        geo_msg.header.frame_id = 'map'

        # Position
        geo_msg.pose.position.latitude = geo_pose.latlng_coordinates.get_latitude(
            False)
        geo_msg.pose.position.longitude = geo_pose.latlng_coordinates.get_longitude(
            False)
        geo_msg.pose.position.altitude = geo_pose.latlng_coordinates.get_altitude()

        # Orientation (identity for now)
        geo_msg.pose.orientation.w = 1.0

        self.geo_pose_pub.publish(geo_msg)

    def publish_transform(self, odom_msg: Odometry):
        """Publish TF transform"""
        t = TransformStamped()
        t.header = odom_msg.header
        t.child_frame_id = odom_msg.child_frame_id

        t.transform.translation.x = odom_msg.pose.pose.position.x
        t.transform.translation.y = odom_msg.pose.pose.position.y
        t.transform.translation.z = odom_msg.pose.pose.position.z
        t.transform.rotation = odom_msg.pose.pose.orientation

        self.tf_broadcaster.sendTransform(t)

    def add_to_path(self, geo_pose: sl.GeoPose):
        """Add current pose to path for visualization"""
        pose_stamped = PoseStamped()
        pose_stamped.header.stamp = self.get_clock().now().to_msg()
        pose_stamped.header.frame_id = 'map'

        # Store lat/lon in position for visualization
        pose_stamped.pose.position.x = geo_pose.latlng_coordinates.get_longitude(
            False)
        pose_stamped.pose.position.y = geo_pose.latlng_coordinates.get_latitude(
            False)
        pose_stamped.pose.position.z = geo_pose.latlng_coordinates.get_altitude()
        pose_stamped.pose.orientation.w = 1.0

        self.fused_path.poses.append(pose_stamped)

        # Limit path length
        if len(self.fused_path.poses) > self.path_max_length:
            self.fused_path.poses.pop(0)

        self.fused_path.header.stamp = self.get_clock().now().to_msg()
        self.fused_path_pub.publish(self.fused_path)

    def publish_diagnostics(self):
        """Publish diagnostic information"""
        diag_array = DiagnosticArray()
        diag_array.header.stamp = self.get_clock().now().to_msg()

        status = DiagnosticStatus()
        status.name = 'ZED GNSS Fusion'
        status.hardware_id = 'zed_camera'

        # Overall status
        gnss_age = time.time() - self.last_gnss_time
        if self.calibration_complete and gnss_age < 5.0:
            status.level = DiagnosticStatus.OK
            status.message = 'Fusion operational'
        elif gnss_age < 5.0:
            status.level = DiagnosticStatus.WARN
            status.message = 'Calibrating...'
        else:
            status.level = DiagnosticStatus.ERROR
            status.message = 'No GNSS data'

        # Add diagnostic values
        status.values.append(
            KeyValue(key='Calibrated', value=str(self.calibration_complete)))
        status.values.append(
            KeyValue(key='Frame Count', value=str(self.frame_count)))
        status.values.append(
            KeyValue(key='GNSS Age', value=f'{gnss_age:.1f}s'))

        if self.fusion:
            calib_std = self.fusion.get_current_gnss_calibration_std()
            status.values.append(KeyValue(key='Translation Uncertainty',
                                          value=f'{calib_std.translation:.3f}m'))
            status.values.append(KeyValue(key='Yaw Uncertainty',
                                          value=f'{calib_std.yaw:.3f}rad'))

        diag_array.status.append(status)
        self.diagnostics_pub.publish(diag_array)

    def destroy_node(self):
        """Cleanup on shutdown"""
        self.get_logger().info('Shutting down ZED GNSS Fusion...')

        if self.fusion:
            self.fusion.close()

        if self.camera:
            self.camera.close()

        super().destroy_node()


def main(args=None):
    rclpy.init(args=args)

    try:
        node = ZedGnssFusion()
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    except Exception as e:
        print(f'Fatal error: {e}')
    finally:
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == '__main__':
    main()
