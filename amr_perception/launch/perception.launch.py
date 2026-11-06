"""Start the object detector (and optionally the YOLO node)."""

import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.conditions import IfCondition
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description():
    config = os.path.join(get_package_share_directory('amr_perception'),
                          'config', 'detector.yaml')
    target_frame = LaunchConfiguration('target_frame')

    detector = Node(
        package='amr_perception',
        executable='object_detector',
        output='screen',
        parameters=[config, {'use_sim_time': True, 'target_frame': target_frame}],
    )

    yolo = Node(
        package='amr_perception',
        executable='yolo_detector',
        output='screen',
        parameters=[{'use_sim_time': True}],
        condition=IfCondition(LaunchConfiguration('yolo')),
    )

    return LaunchDescription([
        DeclareLaunchArgument('target_frame', default_value='map',
                              description='Frame the detected objects are expressed in'),
        DeclareLaunchArgument('yolo', default_value='false',
                              description='Also run the optional YOLO node'),
        detector,
        yolo,
    ])
