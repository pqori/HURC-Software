import platform

from launch import LaunchDescription
from launch.actions import (
    DeclareLaunchArgument,
    IncludeLaunchDescription,
    LogInfo,
    RegisterEventHandler,
    SetEnvironmentVariable,
)
from launch.conditions import IfCondition, UnlessCondition
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import (
    Command,
    EnvironmentVariable,
    FindExecutable,
    LaunchConfiguration,
    PathJoinSubstitution,
    PythonExpression,
)
from launch.event_handlers import OnProcessExit

from launch_ros.actions import Node
from launch_ros.substitutions import FindPackagePrefix, FindPackageShare

def generate_launch_description():
    rviz_arg = DeclareLaunchArgument(
        "rviz", default_value="true", description="Start rviz2"
    )
    headless_arg = DeclareLaunchArgument(
        "headless",
        default_value="false",
        description="Run the Gazebo server only (no Gazebo GUI)",
    )
    world_arg = DeclareLaunchArgument(
        "world", default_value="empty.sdf", description="Gazebo world to load"
    )

    rviz_config_file = PathJoinSubstitution(
        [FindPackageShare("rover_description"), "config", "rover.rviz"]
    )

    robot_description = Command([
        PathJoinSubstitution([FindExecutable(name="xacro")]),
        " ",
        PathJoinSubstitution([
            FindPackageShare("rover_description"), "urdf", "rover.urdf.xacro"
        ]),
        " ",
        "use_gazebo:=true",
    ])

    # Let Gazebo resolve package://rover_description/... mesh URIs.
    # Fortress (ign-gazebo6) reads IGN_GAZEBO_RESOURCE_PATH; newer Gazebo
    # reads GZ_SIM_RESOURCE_PATH. Set both, keeping any existing entries.
    share_dir = PathJoinSubstitution([
        FindPackageShare("rover_description"),
        "..",  # go from share/rover_description -> share
    ])
    gz_resource_path = SetEnvironmentVariable(
        name="GZ_SIM_RESOURCE_PATH",
        value=[share_dir, ":", EnvironmentVariable("GZ_SIM_RESOURCE_PATH", default_value="")],
    )
    ign_resource_path = SetEnvironmentVariable(
        name="IGN_GAZEBO_RESOURCE_PATH",
        value=[share_dir, ":", EnvironmentVariable("IGN_GAZEBO_RESOURCE_PATH", default_value="")],
    )

    # Let Gazebo find the gz_ros2_control system plugin. On Linux it is found
    # through LD_LIBRARY_PATH; macOS strips DYLD_* variables, so point
    # Gazebo's plugin search path at the package's lib directory explicitly.
    gz_plugin_dir = PathJoinSubstitution([FindPackagePrefix("gz_ros2_control"), "lib"])
    gz_plugin_path = SetEnvironmentVariable(
        name="GZ_SIM_SYSTEM_PLUGIN_PATH",
        value=[gz_plugin_dir, ":", EnvironmentVariable("GZ_SIM_SYSTEM_PLUGIN_PATH", default_value="")],
    )
    ign_plugin_path = SetEnvironmentVariable(
        name="IGN_GAZEBO_SYSTEM_PLUGIN_PATH",
        value=[gz_plugin_dir, ":", EnvironmentVariable("IGN_GAZEBO_SYSTEM_PLUGIN_PATH", default_value="")],
    )

    # headless:=true adds -s (server only). On macOS the Gazebo Fortress GUI
    # cannot run at all (the `ign gazebo` front end refuses anything but -s,
    # and the GUI's Ogre2 scene needs the Cocoa main thread; see
    # https://github.com/gazebosim/gz-sim/issues/44), so there the server
    # always runs with -s and rviz2 is the visualizer.
    is_macos = platform.system() == "Darwin"
    gz_args = [
        PythonExpression([
            "'-s ' if ('", LaunchConfiguration("headless"), "' == 'true' or ",
            str(is_macos), ") else ''",
        ]),
        "-r -v 4 ",
        LaunchConfiguration("world"),
    ]

    macos_gui_note = LogInfo(
        msg="macOS: Gazebo runs server-only (no Gazebo GUI on macOS); use rviz2 to watch the rover.",
        condition=UnlessCondition(LaunchConfiguration("headless")),
    )

    gazebo = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            [FindPackageShare("ros_gz_sim"), "/launch/gz_sim.launch.py"]
        ),
        launch_arguments=[("gz_args", gz_args)],
    )

    gazebo_bridge = Node(
        package="ros_gz_bridge",
        executable="parameter_bridge",
        arguments=["/clock@rosgraph_msgs/msg/Clock[gz.msgs.Clock"],
        output="screen",
        parameters=[{"use_sim_time": True}],
    )

    gazebo_spawn_rover = Node(
        package="ros_gz_sim",
        executable="create",
        name='spawn_rover',
        output="screen",
        arguments=[
            "-topic", "robot_description",
            "-name", "rover",
            "-allow_renaming", "true",
            "-z", "2.0"
        ],
    )

    robot_state_publisher = Node(
        package="robot_state_publisher",
        executable="robot_state_publisher",
        output="both",
        parameters=[
            {"robot_description": robot_description},
            {"use_sim_time": True},
        ],
    )

    # One spawner for all controllers: separate spawners started at the same
    # time race on /controller_manager/switch_controller, and under load one
    # of them (often joint_state_broadcaster) can time out and stay inactive.
    # Generous timeouts because the sim can run well below real time on a
    # busy laptop, and controller switching happens in sim time.
    spawn_all_controllers = Node(
        package='controller_manager',
        executable='spawner',
        arguments=[
            'joint_state_broadcaster',
            'rover_drive_controller',
            'rover_arm_controller',
            '--switch-timeout', '60',
            '--service-call-timeout', '90',
        ],
        output='screen',
    )

    spawn_controllers = RegisterEventHandler(
        OnProcessExit(
            target_action=gazebo_spawn_rover,
            on_exit=[spawn_all_controllers],
        )
    )

    rviz = Node(
        package="rviz2",
        executable="rviz2",
        name="rviz2",
        output="log",
        arguments=[
            "--display-config", rviz_config_file,
            "--fixed-frame", "odom"
        ],
        parameters=[{"use_sim_time": True}],
        condition=IfCondition(LaunchConfiguration("rviz")),
    )

    return LaunchDescription([
        rviz_arg,
        headless_arg,
        world_arg,
        gz_resource_path,
        ign_resource_path,
        gz_plugin_path,
        ign_plugin_path,
        gazebo,
        *([macos_gui_note] if is_macos else []),
        gazebo_bridge,
        gazebo_spawn_rover,
        robot_state_publisher,
        spawn_controllers,
        rviz,
    ])