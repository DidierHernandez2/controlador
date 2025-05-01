import rclpy
from rclpy.node import Node

from geometry_msgs.msg import Twist
from nav_msgs.msg import Odometry
from rclpy.qos import qos_profile_sensor_data

import numpy as np

def normalize_angle(angle):
    return np.arctan2(np.sin(angle), np.cos(angle))

class SquarePoseController(Node):
    def __init__(self):
        super().__init__('square_pose_control')

        # Umbrales
        self.declare_parameter('dist_tol', 0.01)
        self.declare_parameter('angle_tol', 0.05)
        self.dist_tol  = self.get_parameter('dist_tol').value
        self.angle_tol = self.get_parameter('angle_tol').value

        # Waypoints desde parámetro (YAML)
        default_wps = [[2.0, 0.0],
                       [2.0, 2.0],
                       [0.0, 2.0],
                       [0.0, 0.0]]
        self.declare_parameter('waypoints', default_wps)
        raw_wps = self.get_parameter('waypoints').value
        self.waypoints = [(float(x), float(y)) for x,y in raw_wps]

        self.wp_idx = 0

        # Pose actual
        self.x = self.y = self.th = None

        # Suscripción a /odom
        self.create_subscription(
            Odometry, '/odom', self.odom_cb, qos_profile_sensor_data)

        # Publicador de comandos
        self.cmd_pub = self.create_publisher(
            Twist, '/cmd_vel', qos_profile_sensor_data)

        # Timer a 20 Hz
        self.create_timer(0.05, self.control_loop)

        self.get_logger().info(
            f'SquarePoseController iniciado: dist_tol={self.dist_tol}, '
            f'angle_tol={self.angle_tol}, ω=0.1, v=0.2, waypoints={self.waypoints}'
        )

    def odom_cb(self, msg: Odometry):
        # Extrae pose
        self.x = msg.pose.pose.position.x
        self.y = msg.pose.pose.position.y
        q = msg.pose.pose.orientation
        siny = 2.0*(q.w*q.z + q.x*q.y)
        cosy = 1.0 - 2.0*(q.y*q.y + q.z*q.z)
        self.th = np.arctan2(siny, cosy)

    def control_loop(self):
        # Espera primera odometría
        if self.x is None:
            return

        # Objetivo actual
        gx, gy = self.waypoints[self.wp_idx]
        dx, dy = gx - self.x, gy - self.y
        rho   = np.hypot(dx, dy)
        alpha = normalize_angle(np.arctan2(dy, dx) - self.th)

        # Si estamos cerca del waypoint, cambiamos al siguiente
        if rho < self.dist_tol:
            self.get_logger().info(
                f'WP {self.wp_idx} alcanzado (x={self.x:.2f}, y={self.y:.2f})'
            )
            self.wp_idx = (self.wp_idx + 1) % len(self.waypoints)
            # Detener el robot momentáneamente
            self.cmd_pub.publish(Twist())
            return

        cmd = Twist()
        # 1) Si no estamos alineados, giramos a ω=±0.1 rad/s
        if abs(alpha) > self.angle_tol:
            cmd.angular.z = 0.1 * np.sign(alpha)
            cmd.linear.x  = 0.0
        else:
            # 2) Ya alineados, avanzamos a v=0.2 m/s
            cmd.linear.x  = 0.2
            cmd.angular.z = 0.0

        # Publica el comando
        self.cmd_pub.publish(cmd)

def main(args=None):
    rclpy.init(args=args)
    node = SquarePoseController()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()

if __name__=='__main__':
    main()
