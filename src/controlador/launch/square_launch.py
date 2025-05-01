import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch_ros.actions import Node

def generate_launch_description():
    pkg_share = get_package_share_directory('controlador')
    param_file = os.path.join(pkg_share, 'config', 'waypoints.yaml')

    return LaunchDescription([
        Node(
            package='odometry_pkg',
            executable='dead_reckoning',
            name='dead_reckoning',
            output='screen',
        ),
        Node(
            package='controlador',
            executable='square_pid_control',
            name='square_pose_control',
            output='screen',
            parameters=[param_file],
        ),
    ])