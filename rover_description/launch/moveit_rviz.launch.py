import os
from launch import LaunchDescription
from launch_ros.actions import Node
from ament_index_python.packages import get_package_share_directory
from moveit_configs_utils import MoveItConfigsBuilder

def generate_launch_description():
    # 1. Resolve the path to your REAL robot description (Must exist on the Mac!)
    rover_desc_path = get_package_share_directory('rover_description')
    real_urdf_path = os.path.join(rover_desc_path, 'urdf', 'rover.urdf.xacro')

    # 2. Build the config dictionary for RViz
    # This ensures RViz gets the kinematics.yaml it was crying about earlier
    moveit_config = (
        MoveItConfigsBuilder("HURCRover", package_name="rover_moveit_config")
        .robot_description(file_path=real_urdf_path)
        .robot_description_semantic(file_path="config/HURCRover.srdf")
        .robot_description_kinematics(file_path="config/kinematics.yaml")
        .to_moveit_configs()
    )

    # 3. Define the RViz Node
    rviz_node = Node(
        package="rviz2",
        executable="rviz2",
        name="rviz2",
        output="log",
        # Automatically load the MoveIt RViz layout
        arguments=["-d", os.path.join(get_package_share_directory("hurcrover_moveit_config"), "config", "moveit.rviz")],
        parameters=[
            moveit_config.robot_description,
            moveit_config.robot_description_semantic,
            moveit_config.robot_description_kinematics,
            moveit_config.joint_limits,
            # {"use_sim_time": True} # <-- Uncomment if your Jetson is running Gazebo instead of real hardware
        ],
    )

    return LaunchDescription([
        rviz_node
    ])