from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import PathJoinSubstitution

from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare

def generate_launch_description():
    gnss_driver = Node(
        package='ublox_gps',
        executable='ublox_gps_node',
        name='gnss_driver',
        output='screen',
        parameters=[{
            'device': '/dev/gps',
            'frame_id': 'base_link',
            'baudrate': 38400,
            'rate': 1.0,
            'nav_rate': 1,
            'enable_ppp': False,
            'tmode3': 0,
            'debug': 1,
            'dgnss_mode': 3,  # RTK Fixed mode
            'configure_gps': False,  # Don't reconfigure GPS
        }],
        remappings=[
            ('fix', '/gps/fix'),
            ('fix_velocity', '/gps/velocity')
        ]
    )

    ekf_local_node = Node(
        package='robot_localization',
        executable='ekf_node',
        name='ekf_local_node',
        output='screen',
        parameters=[
            PathJoinSubstitution([
                FindPackageShare("rover_navigation"),
                "config",
                "localization.yaml",
            ]), {'use_sim_time': False}]
    )

    navsat_transform_node = Node(
        package='robot_localization',
        executable='navsat_transform_node',
        name='navsat_transform_node',
        output='screen',
        parameters=[
            PathJoinSubstitution([
                FindPackageShare("rover_navigation"),
                "config",
                "localization.yaml",
            ]), {'use_sim_time': False}],
        remappings=[
            ('imu/data', '/Front_Zed/zed_node/imu/data'),
            ('gps/fix', '/gps/fix')
        ]
    )

    ekf_global_node = Node(
        package='robot_localization',
        executable='ekf_node',
        name='ekf_global_node',
        output='screen',
        parameters=[
            PathJoinSubstitution([
                FindPackageShare("rover_navigation"),
                "config",
                "localization.yaml",
            ]), {'use_sim_time': False}]
    )

    return LaunchDescription([
        gnss_driver,
        ekf_local_node,
        navsat_transform_node,
        ekf_global_node,
    ])