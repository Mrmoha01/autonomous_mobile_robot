"""Start the Nav2 servers (planner, controller, behaviours, BT navigator).

Only the servers that are stable across Jazzy releases are started, so the launch does not
depend on the exact nav2_bringup version installed. cmd_vel goes straight to the robot.
"""

import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node, SetParameter

LIFECYCLE_NODES = [
    'controller_server',
    'smoother_server',
    'planner_server',
    'behavior_server',
    'bt_navigator',
    'waypoint_follower',
]


def generate_launch_description():
    default_params = os.path.join(get_package_share_directory('amr_navigation'),
                                  'config', 'nav2_params.yaml')
    params_file = LaunchConfiguration('params_file')

    def server(package, executable, name):
        return Node(
            package=package,
            executable=executable,
            name=name,
            output='screen',
            parameters=[params_file],
        )

    return LaunchDescription([
        DeclareLaunchArgument('params_file', default_value=default_params,
                              description='Nav2 parameters file'),
        SetParameter('use_sim_time', True),
        server('nav2_controller', 'controller_server', 'controller_server'),
        server('nav2_smoother', 'smoother_server', 'smoother_server'),
        server('nav2_planner', 'planner_server', 'planner_server'),
        server('nav2_behaviors', 'behavior_server', 'behavior_server'),
        server('nav2_bt_navigator', 'bt_navigator', 'bt_navigator'),
        server('nav2_waypoint_follower', 'waypoint_follower', 'waypoint_follower'),
        Node(
            package='nav2_lifecycle_manager',
            executable='lifecycle_manager',
            name='lifecycle_manager_navigation',
            output='screen',
            parameters=[{'autostart': True, 'node_names': LIFECYCLE_NODES}],
        ),
    ])
