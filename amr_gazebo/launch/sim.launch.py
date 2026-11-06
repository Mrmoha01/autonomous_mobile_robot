"""Start Gazebo with the room world, spawn the robot and bridge its topics to ROS 2."""

import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import Command, LaunchConfiguration, PythonExpression
from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterValue


def generate_launch_description():
    gazebo_dir = get_package_share_directory('amr_gazebo')
    description_dir = get_package_share_directory('amr_description')
    ros_gz_sim_dir = get_package_share_directory('ros_gz_sim')

    world = os.path.join(gazebo_dir, 'worlds', 'room.sdf')
    bridge_config = os.path.join(gazebo_dir, 'config', 'bridge.yaml')
    urdf_xacro = os.path.join(description_dir, 'urdf', 'amr.urdf.xacro')

    gui = LaunchConfiguration('gui')
    spawn_x = LaunchConfiguration('x')
    spawn_y = LaunchConfiguration('y')
    spawn_yaw = LaunchConfiguration('yaw')

    # "-r" starts the simulation running, "-s" runs the server only (no Gazebo window).
    gz_flags = PythonExpression(["'-r -s' if '", gui, "' == 'false' else '-r'"])

    gazebo = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(ros_gz_sim_dir, 'launch', 'gz_sim.launch.py')),
        launch_arguments={'gz_args': [gz_flags, ' ', world]}.items(),
    )

    robot_description = ParameterValue(Command(['xacro ', urdf_xacro]), value_type=str)

    robot_state_publisher = Node(
        package='robot_state_publisher',
        executable='robot_state_publisher',
        output='screen',
        parameters=[{'robot_description': robot_description, 'use_sim_time': True}],
    )

    spawn_robot = Node(
        package='ros_gz_sim',
        executable='create',
        output='screen',
        arguments=['-name', 'amr', '-topic', 'robot_description',
                   '-x', spawn_x, '-y', spawn_y, '-z', '0.1', '-Y', spawn_yaw],
    )

    bridge = Node(
        package='ros_gz_bridge',
        executable='parameter_bridge',
        output='screen',
        parameters=[{'config_file': bridge_config}],
    )

    return LaunchDescription([
        DeclareLaunchArgument('gui', default_value='true',
                              description='Show the Gazebo window (false = server only)'),
        DeclareLaunchArgument('x', default_value='0.0', description='Spawn x (world frame)'),
        DeclareLaunchArgument('y', default_value='0.0', description='Spawn y (world frame)'),
        DeclareLaunchArgument('yaw', default_value='0.0', description='Spawn yaw [rad]'),
        gazebo,
        robot_state_publisher,
        spawn_robot,
        bridge,
    ])
