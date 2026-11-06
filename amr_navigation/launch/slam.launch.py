"""Online SLAM with SLAM Toolbox: publishes /map and the map -> odom transform."""

import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource


def generate_launch_description():
    slam_dir = get_package_share_directory('slam_toolbox')
    params = os.path.join(get_package_share_directory('amr_navigation'),
                          'config', 'slam_params.yaml')

    slam = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(slam_dir, 'launch', 'online_async_launch.py')),
        launch_arguments={'slam_params_file': params, 'use_sim_time': 'true'}.items(),
    )

    return LaunchDescription([slam])
