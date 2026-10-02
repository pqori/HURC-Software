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

    servo_node = Node(
        package='moveit_servo',
        executable='servo_node_main',
        name='servo_node',
        parameters=[
            os.path.join(get_package_share_directory("rover_moveit_config"), 'config', 'servo_parameters.yaml'),
            moveit_config.robot_description,
            moveit_config.robot_description_semantic,
            moveit_config.robot_description_kinematics,
        ],
        output='screen',
    )

    return LaunchDescription([
        servo_node
    ])