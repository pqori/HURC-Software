from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription, SetEnvironmentVariable, RegisterEventHandler
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import Command, PathJoinSubstitution, FindExecutable
from launch.event_handlers import OnProcessStart

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
        "use_gazebo:=false",
    ])

    robot_state_publisher = Node(
        package="robot_state_publisher",
        executable="robot_state_publisher",
        output="both",
        parameters=[
            {"robot_description": robot_description},
        ],
    )

    drive_pdb_uros_agent = Node(
        package="micro_ros_agent",
        executable="micro_ros_agent",
        output="both",
        arguments=[
            'serial',
            '--dev', '/dev/teensy_drive'
        ]
    )

    arm_uros_agent = Node(
        package="micro_ros_agent",
        executable="micro_ros_agent",
        output="both",
        arguments=[
            'serial',
            '--dev', '/dev/teensy_arm'
        ]
    )

    controller_manager = Node(
        package="controller_manager",
        executable="ros2_control_node",
        parameters=[
            {"robot_description": robot_description},
            PathJoinSubstitution([
                FindPackageShare("rover_description"),
                "config",
                "rover_controllers.yaml",
            ]),
        ],
        output="screen",
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

    gripper_controller = Node(
        package='controller_manager',
        executable='spawner',
        arguments=['rover_gripper_controller'],
        output='screen'
    )

    spawn_controllers = RegisterEventHandler(
        OnProcessStart(
            target_action=controller_manager,
            on_start=[
                joint_state_broadcaster,
                drive_controller,
                arm_controller,
                gripper_controller
            ],
        )
    )

    return LaunchDescription([
        robot_state_publisher,
        drive_pdb_uros_agent,
        arm_uros_agent,
        controller_manager,
        spawn_controllers
    ])