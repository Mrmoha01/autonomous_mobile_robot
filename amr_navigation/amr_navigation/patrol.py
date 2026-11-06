"""Autonomous patrol: drive through a fixed list of waypoints in the room using Nav2.

Usage::

    ros2 run amr_navigation patrol --ros-args -p use_sim_time:=true
    ros2 run amr_navigation patrol --loops 3 --wait 2.0 --ros-args -p use_sim_time:=true

Waypoints are in the ``map`` frame. SLAM starts at the robot's spawn pose, which is the
world origin, so these are also world coordinates of the room (x, y in [-1, 5]).
"""

import argparse
import math
import sys
import time

from geometry_msgs.msg import PoseStamped
from nav2_simple_commander.robot_navigator import BasicNavigator, TaskResult
import rclpy
from rclpy.utilities import remove_ros_args

# (x, y, yaw[rad]) in the map frame
WAYPOINTS = [
    (1.5, 0.5, 0.0),
    (4.3, 0.7, math.pi / 2),
    (2.6, 4.2, math.pi),
    (0.5, 2.2, -math.pi / 2),
]


def make_pose(x, y, yaw):
    pose = PoseStamped()
    pose.header.frame_id = 'map'   # stamp left at 0 -> "use the latest transform"
    pose.pose.position.x = x
    pose.pose.position.y = y
    pose.pose.orientation.z = math.sin(yaw / 2.0)
    pose.pose.orientation.w = math.cos(yaw / 2.0)
    return pose


def main(argv=None):
    parser = argparse.ArgumentParser(description='Patrol waypoints with Nav2')
    parser.add_argument('--loops', type=int, default=0,
                        help='number of laps (0 = repeat forever)')
    parser.add_argument('--wait', type=float, default=1.0,
                        help='seconds to pause at each waypoint')
    args = parser.parse_args(remove_ros_args(sys.argv)[1:] if argv is None else argv)

    rclpy.init()
    navigator = BasicNavigator()
    # SLAM Toolbox is our localiser (there is no AMCL), so wait for it instead.
    navigator.waitUntilNav2Active(localizer='slam_toolbox')
    navigator.get_logger().info('Nav2 is active - starting patrol')

    lap = 0
    try:
        while rclpy.ok() and (args.loops == 0 or lap < args.loops):
            lap += 1
            for i, (x, y, yaw) in enumerate(WAYPOINTS, start=1):
                navigator.get_logger().info(
                    f'Lap {lap}: going to waypoint {i}/{len(WAYPOINTS)} ({x:.1f}, {y:.1f})')
                navigator.goToPose(make_pose(x, y, yaw))
                while not navigator.isTaskComplete():
                    time.sleep(0.2)

                result = navigator.getResult()
                if result == TaskResult.SUCCEEDED:
                    navigator.get_logger().info(f'Reached waypoint {i}')
                    time.sleep(args.wait)
                elif result == TaskResult.CANCELED:
                    navigator.get_logger().info('Patrol cancelled')
                    return
                else:
                    navigator.get_logger().warn(f'Waypoint {i} failed - skipping it')
    except KeyboardInterrupt:
        navigator.cancelTask()
    finally:
        navigator.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == '__main__':
    main()
