#!/usr/bin/env python3
import os
from launch import LaunchDescription
from launch_ros.actions import Node
from ament_index_python.packages import get_package_share_directory

def generate_launch_description():
    pkg = 'object_detection'
    pkg_share = get_package_share_directory(pkg)
    
    # Resolve model paths
    model_path = os.path.join(pkg_share, 'model', 'model.onnx')
    # Use COCO classes for pre-trained model, switch to classes.txt for custom URC model
    classes_file = os.path.join(pkg_share, 'model', 'classes_coco.txt')
    
    return LaunchDescription([
        Node(
            package=pkg,
            executable='object_detection',
            name='urc_object_detection',
            output='screen',
            parameters=[{
                'model_path': model_path,
                'classes_file': classes_file,
                'confidence_threshold': 0.5,
                'iou_threshold': 0.45,
                'input_size': 640,
                'image_topic': '/zed2i/left/image_rect_color',
                'output_topic': '/detected_objects',
                'max_detections': 100
            }]),
    ])
