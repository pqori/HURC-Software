"""YAM arm bridge (MuJoCo sim or real arm) + robot_state_publisher (+ optional RViz).

    ros2 launch yam_sim_ros sim_bridge.launch.py                       # sim, headless
    ros2 launch yam_sim_ros sim_bridge.launch.py rviz:=true
    ros2 launch yam_sim_ros sim_bridge.launch.py gripper:=crank_4310
    ros2 launch yam_sim_ros sim_bridge.launch.py objects:=keyboard tool:=stylus rviz:=true
    ros2 launch yam_sim_ros sim_bridge.launch.py camera:=c270 camera_rate:=30 camera_resolution:=640x360
    ros2 launch yam_sim_ros sim_bridge.launch.py camera:=none          # no wrist webcam
    ros2 launch yam_sim_ros sim_bridge.launch.py backend:=real channel:=can0   # Linux host with i2rt

`camera` and `tool` go to both the bridge (MuJoCo scene) and xacro (URDF), so the TF tree and
the sim always agree on where the webcam and the stylus tip are.

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
    camera = LaunchConfiguration("camera")
    tool = LaunchConfiguration("tool")

    robot_description = ParameterValue(
        Command([
            PathJoinSubstitution([FindExecutable(name="xacro")]), " ",
            PathJoinSubstitution([FindPackageShare("yam_description"), "urdf", "yam.urdf.xacro"]),
            " gripper:=", gripper, " prefix:=", prefix, " camera:=", camera, " tool:=", tool,
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
            "camera": camera,
            "camera_rate": ParameterValue(LaunchConfiguration("camera_rate"), value_type=float),
            "camera_resolution": ParameterValue(LaunchConfiguration("camera_resolution"), value_type=str),
            "camera_device": ParameterValue(LaunchConfiguration("camera_device"), value_type=str),
            "objects": ParameterValue(LaunchConfiguration("objects"), value_type=str),
            "tool": tool,
            "publish_key_markers": ParameterValue(LaunchConfiguration("publish_key_markers"), value_type=bool),
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
        DeclareLaunchArgument("camera", default_value="c920",
                              description="wrist webcam: c920 | c270 | none (bridge + URDF)"),
        DeclareLaunchArgument("camera_rate", default_value="15.0", description="wrist camera rate (Hz)"),
        DeclareLaunchArgument("camera_resolution", default_value="",
                              description="e.g. 1280x720 or 640x360; empty = the camera's default (1280x720)"),
        DeclareLaunchArgument("camera_device", default_value="0",
                              description="backend:=real: OpenCV webcam index or device path"),
        DeclareLaunchArgument("objects", default_value="",
                              description="sim scene objects: '' | keyboard | cube | cube,keyboard"),
        DeclareLaunchArgument("tool", default_value="none",
                              description="tool in the gripper: none | stylus (bridge + URDF)"),
        DeclareLaunchArgument("publish_key_markers", default_value="true",
                              description="publish /yam_keyboard/markers (objects:=keyboard)"),
        bridge,
        rsp,
        rviz,
    ])
