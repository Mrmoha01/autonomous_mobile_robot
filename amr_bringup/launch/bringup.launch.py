"""Full system: Gazebo + robot + SLAM + Nav2 + object detection + RViz2 (+ optional patrol).

    ros2 launch amr_bringup bringup.launch.py
    ros2 launch amr_bringup bringup.launch.py patrol:=true
    ros2 launch amr_bringup bringup.launch.py gui:=false     # lighter on a VM
"""

import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription, TimerAction
from launch.conditions import IfCondition
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description():
    def share(pkg, *parts):
        return os.path.join(get_package_share_directory(pkg), *parts)

    gui = LaunchConfiguration('gui')
    rviz = LaunchConfiguration('rviz')
    patrol = LaunchConfiguration('patrol')
    yolo = LaunchConfiguration('yolo')

    sim = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(share('amr_gazebo', 'launch', 'sim.launch.py')),
        launch_arguments={'gui': gui}.items(),
    )

    slam = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(share('amr_navigation', 'launch', 'slam.launch.py')))

    navigation = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(share('amr_navigation', 'launch', 'navigation.launch.py')))

    perception = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(share('amr_perception', 'launch', 'perception.launch.py')),
        launch_arguments={'yolo': yolo}.items(),
    )

    rviz_node = Node(
        package='rviz2',
        executable='rviz2',
        arguments=['-d', share('amr_bringup', 'rviz', 'amr.rviz')],
        parameters=[{'use_sim_time': True}],
        output='screen',
        condition=IfCondition(rviz),
    )

    patrol_node = Node(
        package='amr_navigation',
        executable='patrol',
        output='screen',
        ros_arguments=['-p', 'use_sim_time:=true'],
        condition=IfCondition(patrol),
    )

    return LaunchDescription([
        DeclareLaunchArgument('gui', default_value='true',
                              description='Show the Gazebo window'),
        DeclareLaunchArgument('rviz', default_value='true', description='Start RViz2'),
        DeclareLaunchArgument('patrol', default_value='false',
                              description='Start the autonomous waypoint patrol'),
        DeclareLaunchArgument('yolo', default_value='false',
                              description='Also run the optional YOLO node'),
        sim,
        # Give Gazebo a head start, then bring up the rest.
        TimerAction(period=5.0, actions=[slam, perception, rviz_node]),
        TimerAction(period=10.0, actions=[navigation]),
        TimerAction(period=12.0, actions=[patrol_node]),
    ])
