"""Rover simulation for the Nav2 obstacle-avoidance demo (Gazebo Harmonic).

Everything rover_description/launch/gazebo.launch.py does (macOS server-only
Gazebo, gz_ros2_control plugin path workaround, one controller spawner), plus:
  - world:=<name or path>, default obstacles.sdf; a bare file name is looked up
    in rover_sim/worlds first, otherwise handed to Gazebo as is (empty.sdf).
  - the rover spawns at z = 0.6 m (spawn_z:=...).
  - the URDF is expanded with controllers_file:=rover_sim/config/
    rover_controllers_sim.yaml (wheel geometry that matches the model).
  - the simulated ZED 2i topics and /clock are bridged with
    rover_sim/config/bridge.yaml.

  ros2 launch rover_sim sim.launch.py headless:=true rviz:=false
"""

import os
import platform

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import (
    DeclareLaunchArgument,
    IncludeLaunchDescription,
    LogInfo,
    OpaqueFunction,
    RegisterEventHandler,
    SetEnvironmentVariable,
)
from launch.conditions import IfCondition, UnlessCondition
from launch.event_handlers import OnProcessExit
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import (
    Command,
    EnvironmentVariable,
    FindExecutable,
    LaunchConfiguration,
    PathJoinSubstitution,
)
from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterValue
from launch_ros.substitutions import FindPackagePrefix, FindPackageShare


def _resolve_world(context):
    """Return the world to pass to gz sim (absolute path if it is ours)."""
    world = LaunchConfiguration("world").perform(context)
    if os.path.isabs(world) or os.path.sep in world:
        return world
    candidate = os.path.join(
        get_package_share_directory("rover_sim"), "worlds", world)
    return candidate if os.path.isfile(candidate) else world


def _gazebo(context):
    is_macos = platform.system() == "Darwin"
    headless = LaunchConfiguration("headless").perform(context) == "true"
    server_only = "-s " if (headless or is_macos) else ""
    gz_args = f"{server_only}-r -v 3 {_resolve_world(context)}"
    return [
        LogInfo(msg=f"rover_sim: gz sim {gz_args}"),
        IncludeLaunchDescription(
            PythonLaunchDescriptionSource(
                [FindPackageShare("ros_gz_sim"), "/launch/gz_sim.launch.py"]),
            launch_arguments=[("gz_args", gz_args)],
        ),
    ]


def generate_launch_description():
    is_macos = platform.system() == "Darwin"

    args = [
        DeclareLaunchArgument("rviz", default_value="true",
                              description="Start rviz2"),
        DeclareLaunchArgument("headless", default_value="false",
                              description="Gazebo server only (no Gazebo GUI)"),
        DeclareLaunchArgument("world", default_value="obstacles.sdf",
                              description="World file: a name in rover_sim/worlds, "
                                          "a path, or a Gazebo world name"),
        DeclareLaunchArgument("spawn_x", default_value="0.0"),
        DeclareLaunchArgument("spawn_y", default_value="0.0"),
        DeclareLaunchArgument("spawn_z", default_value="0.6"),
        DeclareLaunchArgument("spawn_yaw", default_value="0.0"),
        DeclareLaunchArgument(
            "controllers_file",
            default_value=PathJoinSubstitution([
                FindPackageShare("rover_sim"), "config",
                "rover_controllers_sim.yaml"]),
            description="ros2_control parameters given to gz_ros2_control"),
    ]

    robot_description = ParameterValue(Command([
        PathJoinSubstitution([FindExecutable(name="xacro")]), " ",
        PathJoinSubstitution([
            FindPackageShare("rover_description"), "urdf", "rover.urdf.xacro"]),
        " use_gazebo:=true",
        " controllers_file:=", LaunchConfiguration("controllers_file"),
    ]), value_type=str)

    # package://rover_description/... meshes and our worlds.
    share_dir = PathJoinSubstitution([FindPackageShare("rover_description"), ".."])
    worlds_dir = PathJoinSubstitution([FindPackageShare("rover_sim"), "worlds"])
    gz_resource_path = SetEnvironmentVariable(
        name="GZ_SIM_RESOURCE_PATH",
        value=[share_dir, ":", worlds_dir, ":",
               EnvironmentVariable("GZ_SIM_RESOURCE_PATH", default_value="")],
    )

    # macOS strips DYLD_*, so Gazebo cannot find gz_ros2_control-system through
    # the library path; point the system plugin path at it (see gazebo.launch.py).
    gz_plugin_dir = PathJoinSubstitution([FindPackagePrefix("gz_ros2_control"), "lib"])
    gz_plugin_path = SetEnvironmentVariable(
        name="GZ_SIM_SYSTEM_PLUGIN_PATH",
        value=[gz_plugin_dir, ":",
               EnvironmentVariable("GZ_SIM_SYSTEM_PLUGIN_PATH", default_value="")],
    )

    macos_gui_note = LogInfo(
        msg="macOS: Gazebo runs server-only; watch the rover in rviz2, or run "
            "`pixi run gz-gui` in a second terminal for the Gazebo GUI.",
        condition=UnlessCondition(LaunchConfiguration("headless")),
    )

    # /clock + simulated ZED 2i topics (names/frames match the real zed_wrapper).
    bridge = Node(
        package="ros_gz_bridge",
        executable="parameter_bridge",
        name="rover_sim_bridge",
        output="screen",
        parameters=[{
            "config_file": PathJoinSubstitution([
                FindPackageShare("rover_sim"), "config", "bridge.yaml"]),
            "use_sim_time": True,
        }],
    )

    spawn_rover = Node(
        package="ros_gz_sim",
        executable="create",
        name="spawn_rover",
        output="screen",
        arguments=[
            "-topic", "robot_description",
            "-name", "rover",
            "-allow_renaming", "true",
            "-x", LaunchConfiguration("spawn_x"),
            "-y", LaunchConfiguration("spawn_y"),
            "-z", LaunchConfiguration("spawn_z"),
            "-Y", LaunchConfiguration("spawn_yaw"),
        ],
    )

    robot_state_publisher = Node(
        package="robot_state_publisher",
        executable="robot_state_publisher",
        output="both",
        parameters=[{"robot_description": robot_description,
                     "use_sim_time": True}],
    )

    # One spawner for all controllers (separate spawners race on
    # switch_controller); generous timeouts because the sim may run below
    # real time and switching happens in sim time.
    spawn_all_controllers = Node(
        package="controller_manager",
        executable="spawner",
        arguments=[
            "joint_state_broadcaster",
            "rover_drive_controller",
            "rover_arm_controller",
            "--switch-timeout", "60",
            "--service-call-timeout", "90",
        ],
        output="screen",
    )
    spawn_controllers = RegisterEventHandler(
        OnProcessExit(target_action=spawn_rover, on_exit=[spawn_all_controllers]))

    rviz = Node(
        package="rviz2",
        executable="rviz2",
        name="rviz2",
        output="log",
        arguments=[
            "--display-config",
            PathJoinSubstitution([FindPackageShare("rover_description"),
                                  "config", "rover.rviz"]),
            "--fixed-frame", "odom",
        ],
        parameters=[{"use_sim_time": True}],
        condition=IfCondition(LaunchConfiguration("rviz")),
    )

    return LaunchDescription([
        *args,
        gz_resource_path,
        gz_plugin_path,
        OpaqueFunction(function=_gazebo),
        *([macos_gui_note] if is_macos else []),
        bridge,
        spawn_rover,
        robot_state_publisher,
        spawn_controllers,
        rviz,
    ])
