from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription, SetEnvironmentVariable, RegisterEventHandler
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import Command, PathJoinSubstitution, FindExecutable
from launch.event_handlers import OnProcessExit

from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare

def generate_launch_description():
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

    gz_resource_path = SetEnvironmentVariable(
        name="GZ_SIM_RESOURCE_PATH",
        value=PathJoinSubstitution([
            FindPackageShare("rover_description"),
            "..",  # go from share/rover_description -> share
        ]),
    )

    gazebo = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            [FindPackageShare("ros_gz_sim"), "/launch/gz_sim.launch.py"]
        ),
        launch_arguments=[("ign_args", " -r -v 4 empty.sdf")],
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

    joint_state_broadcaster = Node(
        package='controller_manager',
        executable='spawner',
        arguments=['joint_state_broadcaster'],
        output='screen',
    )

    drive_controller = Node(
        package='controller_manager',
        executable='spawner',
        arguments=['rover_drive_controller'],
        output='screen',
    )

    arm_controller = Node(
        package='controller_manager',
        executable='spawner',
        arguments=['rover_arm_controller'],
        output='screen',
    )

    spawn_controllers = RegisterEventHandler(
        OnProcessExit(
            target_action=gazebo_spawn_rover,
            on_exit=[
                joint_state_broadcaster,
                drive_controller,
                arm_controller,
            ],
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
    )

    return LaunchDescription([
        gz_resource_path,
        gazebo,
        gazebo_bridge,
        gazebo_spawn_rover,
        robot_state_publisher,
        spawn_controllers,
        rviz,
    ])