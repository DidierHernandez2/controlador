import launch
from launch_ros.actions import Node

def generate_launch_description():
    return launch.LaunchDescription([

        # 1) Nodo de odometría (dead_reckoning) del paquete odometry_pkg
        Node(
            package='odometry_pkg',
            executable='dead_reckoning',
            name='dead_reckoning',
            output='screen',
        ),

        # 2) Nodo controlador de trayectoria (square_pid_control) del paquete controlador
        Node(
            package='controlador',
            executable='square_pid_control',
            name='square_pid_control',
            output='screen',
        ),
    ])
