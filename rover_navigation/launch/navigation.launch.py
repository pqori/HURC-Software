from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import PathJoinSubstitution

from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare

def generate_launch_description():
    nav2_params = PathJoinSubstitution([
        FindPackageShare("rover_navigation"),
        "config",
        "navigation.yaml",
    ])

    controller_server = Node(
        package="nav2_controller",
        executable="controller_server",
        name="controller_server",
        output="screen",
        parameters=[nav2_params],
        remappings=[('cmd_vel', '/cmd_vel')]
    )

    planner_server = Node(
        package="nav2_planner",
        executable="planner_server",
        name="planner_server",
        output="screen",
        parameters=[nav2_params],
    )

    bt_navigator = Node(
        package="nav2_bt_navigator",
        executable="bt_navigator",
        name="bt_navigator",
        output="screen",
        parameters=[nav2_params],
    )

    behavior_server = Node(
        package="nav2_behaviors",
        executable="behavior_server",
        name="behavior_server",
        output="screen",
        parameters=[nav2_params]
    )

    velocity_smoother = Node(
        package="nav2_velocity_smoother",
        executable="velocity_smoother",
        name="velocity_smoother",
        output="screen",
        parameters=[nav2_params],
        remappings=[('cmd_vel_smoothed', '/rover_drive_controller/cmd_vel_unstamped')]
    )

    lifecycle_manager = Node(
        package="nav2_lifecycle_manager",
        executable="lifecycle_manager",
        name="lifecycle_manager_navigation",
        output="screen",
        parameters=[{
            "use_sim_time": False,
            "autostart": True,
            "node_names": [
                "planner_server",
                "controller_server",
                "bt_navigator",
                "behavior_server",
                "velocity_smoother"
            ]
        }],
    )

    return LaunchDescription([
        controller_server,
        planner_server,
        bt_navigator,
        behavior_server,
        velocity_smoother,
        lifecycle_manager
    ])