import os
from launch import LaunchDescription
from launch_ros.actions import Node
from ament_index_python.packages import get_package_share_directory
from moveit_configs_utils import MoveItConfigsBuilder
from moveit_configs_utils.launches import generate_move_group_launch

def generate_launch_description():
    rover_desc_path = get_package_share_directory('rover_description')
    real_urdf_path = os.path.join(rover_desc_path, 'urdf', 'rover.urdf.xacro')

    moveit_config = (
        MoveItConfigsBuilder("HURCRover", package_name="rover_moveit_config")
        .robot_description(file_path=real_urdf_path)
        .to_moveit_configs()
    )

    return generate_move_group_launch(moveit_config)