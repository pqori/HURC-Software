"""View the I2RT YAM arm in RViz with joint sliders.

    ros2 launch yam_description view_yam.launch.py
    ros2 launch yam_description view_yam.launch.py gripper:=crank_4310
    ros2 launch yam_description view_yam.launch.py camera:=c270 tool:=stylus
    ros2 launch yam_description view_yam.launch.py gui:=false   # headless

With gui:=false neither RViz nor the slider GUI is started; a plain
joint_state_publisher publishes zeros so TF is still complete.
"""

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.conditions import IfCondition, UnlessCondition
from launch.substitutions import (
    Command,
    FindExecutable,
    LaunchConfiguration,
    PathJoinSubstitution,
)
from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterValue
from launch_ros.substitutions import FindPackageShare


def generate_launch_description():
    gripper = LaunchConfiguration("gripper")
    prefix = LaunchConfiguration("prefix")
    gui = LaunchConfiguration("gui")
    camera = LaunchConfiguration("camera")
    tool = LaunchConfiguration("tool")

    rviz_config_file = PathJoinSubstitution(
        [FindPackageShare("yam_description"), "rviz", "yam.rviz"]
    )

    robot_description = ParameterValue(
        Command([
            PathJoinSubstitution([FindExecutable(name="xacro")]),
            " ",
            PathJoinSubstitution([
                FindPackageShare("yam_description"), "urdf", "yam.urdf.xacro"
            ]),
            " gripper:=", gripper,
            " prefix:=", prefix,
            " camera:=", camera,
            " tool:=", tool,
        ]),
        value_type=str,
    )

    robot_state_publisher_node = Node(
        package="robot_state_publisher",
        executable="robot_state_publisher",
        output="both",
        parameters=[{"robot_description": robot_description}],
    )
    joint_state_publisher_gui_node = Node(
        package="joint_state_publisher_gui",
        executable="joint_state_publisher_gui",
        condition=IfCondition(gui),
    )
    joint_state_publisher_node = Node(
        package="joint_state_publisher",
        executable="joint_state_publisher",
        condition=UnlessCondition(gui),
    )
    rviz_node = Node(
        package="rviz2",
        executable="rviz2",
        name="rviz2",
        output="log",
        arguments=["-d", rviz_config_file],
        condition=IfCondition(gui),
    )

    return LaunchDescription([
        DeclareLaunchArgument(
            "gripper", default_value="linear_4310",
            description="Gripper: linear_4310 | crank_4310 | none"),
        DeclareLaunchArgument(
            "prefix", default_value="yam_",
            description="Prefix for all link and joint names "
                        "(rviz/yam.rviz assumes yam_)"),
        DeclareLaunchArgument(
            "camera", default_value="c920",
            description="Wrist webcam: c920 | c270 | none"),
        DeclareLaunchArgument(
            "tool", default_value="none",
            description="Tool held by the gripper: none | stylus"),
        DeclareLaunchArgument(
            "gui", default_value="true",
            description="Start rviz2 and joint_state_publisher_gui"),
        robot_state_publisher_node,
        joint_state_publisher_gui_node,
        joint_state_publisher_node,
        rviz_node,
    ])
