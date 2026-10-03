"""Simulation navigation stack: ekf_local + static map->odom + Nav2.

    ros2 launch rover_navigation sim_nav.launch.py
    ros2 launch rover_navigation sim_nav.launch.py params_file:=/path/nav.yaml

Expects a running sim (rover_description gazebo.launch.py or rover_sim
sim.launch.py) that publishes /clock, /rover_drive_controller/odom and the
robot TF, and takes TwistStamped on /rover_drive_controller/cmd_vel.
"""

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch_ros.actions import Node
from launch_ros.descriptions import ParameterValue
from launch_ros.substitutions import FindPackageShare

LIFECYCLE_NODES = [
    "controller_server",
    "planner_server",
    "behavior_server",
    "velocity_smoother",
    "bt_navigator",
]


def generate_launch_description():
    pkg_share = FindPackageShare("rover_navigation")
    use_sim_time = LaunchConfiguration("use_sim_time")
    params_file = LaunchConfiguration("params_file")
    localization_params_file = LaunchConfiguration("localization_params_file")
    autostart = LaunchConfiguration("autostart")
    log_level = LaunchConfiguration("log_level")

    sim_time = {"use_sim_time": ParameterValue(use_sim_time, value_type=bool)}
    ros_args = ["--ros-args", "--log-level", log_level]

    args = [
        DeclareLaunchArgument(
            "use_sim_time", default_value="true",
            description="Use /clock from the simulator"),
        DeclareLaunchArgument(
            "params_file",
            default_value=PathJoinSubstitution([pkg_share, "config", "sim_navigation.yaml"]),
            description="Nav2 parameters file"),
        DeclareLaunchArgument(
            "localization_params_file",
            default_value=PathJoinSubstitution([pkg_share, "config", "sim_localization.yaml"]),
            description="robot_localization parameters for ekf_local"),
        DeclareLaunchArgument(
            "autostart", default_value="true",
            description="Configure and activate the Nav2 lifecycle nodes automatically"),
        DeclareLaunchArgument(
            "log_level", default_value="info", description="ROS log level"),
    ]

    ekf_local = Node(
        package="robot_localization",
        executable="ekf_node",
        name="ekf_local",
        output="screen",
        parameters=[localization_params_file, sim_time],
        remappings=[("odometry/filtered", "/odometry/filtered")],
        arguments=ros_args,
    )

    # No GPS or map server in sim: map and odom coincide.
    map_to_odom = Node(
        package="tf2_ros",
        executable="static_transform_publisher",
        name="map_to_odom_static",
        output="log",
        arguments=["--frame-id", "map", "--child-frame-id", "odom"],
        parameters=[sim_time],
    )

    def nav2_node(package, executable, remappings=()):
        return Node(
            package=package,
            executable=executable,
            name=executable,
            output="screen",
            parameters=[params_file, sim_time],
            remappings=list(remappings),
            arguments=ros_args,
        )

    controller_server = nav2_node(
        "nav2_controller", "controller_server")
    planner_server = nav2_node("nav2_planner", "planner_server")
    behavior_server = nav2_node("nav2_behaviors", "behavior_server")
    bt_navigator = nav2_node("nav2_bt_navigator", "bt_navigator")
    # controller_server and behavior_server publish TwistStamped on cmd_vel;
    # the smoother forwards it to the diff drive controller.
    velocity_smoother = nav2_node(
        "nav2_velocity_smoother", "velocity_smoother",
        remappings=[("cmd_vel_smoothed", "/rover_drive_controller/cmd_vel")])

    lifecycle_manager = Node(
        package="nav2_lifecycle_manager",
        executable="lifecycle_manager",
        name="lifecycle_manager_navigation",
        output="screen",
        parameters=[sim_time, {
            "autostart": ParameterValue(autostart, value_type=bool),
            "node_names": LIFECYCLE_NODES,
            "bond_timeout": 10.0,
        }],
        arguments=ros_args,
    )

    return LaunchDescription(args + [
        ekf_local,
        map_to_odom,
        controller_server,
        planner_server,
        behavior_server,
        velocity_smoother,
        bt_navigator,
        lifecycle_manager,
    ])
