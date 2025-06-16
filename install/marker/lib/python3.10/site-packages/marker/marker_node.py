#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
import numpy as np
from sensor_msgs.msg import JointState
from visualization_msgs.msg import Marker

class PelvisMarker(Node):
    def __init__(self):
        super().__init__('pelvis_marker_publisher')

        # Tabla DH
        self.dh_params = [
            {"a": 0.0466, "d": 0.363, "alpha": -np.pi/2, "theta_offset": 0.0},
            {"a": 0.0027, "d": -0.076, "alpha": np.pi/2, "theta_offset": -np.pi/2},
            {"a": 0.0696, "d": -0.000785, "alpha": -np.pi/2, "theta_offset": 0.0},
            {"a": 0.0793, "d": -0.0777, "alpha": 0.0, "theta_offset": 0.0},
            {"a": 0.0152, "d": -0.13, "alpha": 0.0, "theta_offset": 0.0}
        ]

        self.subscription = self.create_subscription(
            JointState,
            '/joint_states',
            self.joint_callback,
            10
        )

        self.marker_pub = self.create_publisher(Marker, '/pelvis_marker', 10)

        self.marker = Marker()
        self.marker.header.frame_id = "base_link"
        self.marker.type = Marker.SPHERE
        self.marker.action = Marker.ADD
        self.marker.scale.x = 0.05
        self.marker.scale.y = 0.05
        self.marker.scale.z = 0.05
        self.marker.color.a = 1.0
        self.marker.color.r = 0.0
        self.marker.color.g = 1.0
        self.marker.color.b = 0.0
        self.marker.pose.orientation.w = 1.0

    def dh_matrix(self, a, d, alpha, theta):
        """Matriz de transformación DH usando solo NumPy."""
        ca, sa = np.cos(alpha), np.sin(alpha)
        ct, st = np.cos(theta), np.sin(theta)
        return np.array([
            [ct, -st * ca,  st * sa, a * ct],
            [st,  ct * ca, -ct * sa, a * st],
            [0,       sa,       ca,     d],
            [0,        0,        0,     1]
        ])

    def joint_callback(self, msg):
        try:
            if len(msg.position) < 3:
                self.get_logger().warn("No hay suficientes ángulos en /joint_states")
                return

            T = np.identity(4)
            for i in range(5):
                params = self.dh_params[i]
                theta = msg.position[i] + params["theta_offset"]
                T_i = self.dh_matrix(params["a"], params["d"], params["alpha"], theta)
                T = T @ T_i  # Multiplicación de matrices

            pelvis_pos = T[:3, 3]

            self.marker.header.stamp = self.get_clock().now().to_msg()
            self.marker.pose.position.x = pelvis_pos[0]
            self.marker.pose.position.y = pelvis_pos[1]
            self.marker.pose.position.z = pelvis_pos[2]
            self.marker_pub.publish(self.marker)

            self.get_logger().info(f"Pelvis en: x={pelvis_pos[0]:.3f}, y={pelvis_pos[1]:.3f}, z={pelvis_pos[2]:.3f}")

        except Exception as e:
            self.get_logger().error(f"Error en cinemática: {e}")

def main(args=None):
    rclpy.init(args=args)
    node = PelvisMarker()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
