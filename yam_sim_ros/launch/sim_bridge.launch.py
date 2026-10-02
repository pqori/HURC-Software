"""YAM arm bridge (MuJoCo sim or real arm) + robot_state_publisher (+ optional RViz).

    ros2 launch yam_sim_ros sim_bridge.launch.py                       # sim, headless
    ros2 launch yam_sim_ros sim_bridge.launch.py rviz:=true
    ros2 launch yam_sim_ros sim_bridge.launch.py gripper:=crank_4310
    ros2 launch yam_sim_ros sim_bridge.launch.py backend:=real channel:=can0   # Linux host with i2rt

There is no MuJoCo viewer option. yam_sim's viewer (yam_sim.scripts.run_sim) builds its own
robot in its own process and cannot attach to the robot this bridge owns. Use RViz to watch the
arm, or run the viewer on its own instead of this launch file.
"""

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.conditions import IfCondition
from launch.substitutions import Command, FindExecutable, LaunchConfiguration, PathJoinSubstitution
from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterValue
from launch_ros.substitutions import FindPackageShare


def generate_launch_description():
    gripper = LaunchConfiguration("gripper")
    prefix = LaunchConfiguration("prefix")

    robot_description = ParameterValue(
        Command([
            PathJoinSubstitution([FindExecutable(name="xacro")]), " ",
            PathJoinSubstitution([FindPackageShare("yam_description"), "urdf", "yam.urdf.xacro"]),
            " gripper:=", gripper, " prefix:=", prefix,
        ]),
        value_type=str,
    )

    bridge = Node(
        package="yam_sim_ros",
        executable="bridge_node",
        name="yam_sim_bridge",
        output="screen",
        parameters=[{
            "prefix": prefix,
            "gripper": gripper,
            "backend": LaunchConfiguration("backend"),
            "channel": LaunchConfiguration("channel"),
            "rate": ParameterValue(LaunchConfiguration("rate"), value_type=float),
        }],
    )
    rsp = Node(
        package="robot_state_publisher",
        executable="robot_state_publisher",
        output="both",
        parameters=[{"robot_description": robot_description}],
    )
    rviz = Node(
        package="rviz2",
        executable="rviz2",
        name="rviz2",
        output="log",
        arguments=["-d", PathJoinSubstitution([FindPackageShare("yam_description"), "rviz", "yam.rviz"])],
        condition=IfCondition(LaunchConfiguration("rviz")),
    )

    return LaunchDescription([
        DeclareLaunchArgument("gripper", default_value="linear_4310",
                              description="linear_4310 | crank_4310 | none (must match the URDF)"),
        DeclareLaunchArgument("prefix", default_value="yam_",
                              description="joint/link name prefix (yam_description configs assume yam_)"),
        DeclareLaunchArgument("backend", default_value="sim", description="sim | real"),
        DeclareLaunchArgument("channel", default_value="can0", description="CAN interface for backend:=real"),
        DeclareLaunchArgument("rate", default_value="50.0", description="/joint_states rate (Hz)"),
        DeclareLaunchArgument("rviz", default_value="false", description="start rviz2"),
        bridge,
        rsp,
        rviz,
    ])
