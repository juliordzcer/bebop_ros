
#!/usr/bin/env python3


# Copyright 2025 Julio César Rodríguez
# Licensed under the Apache License, Version 2.0
# https://www.apache.org/licenses/LICENSE-2.0


import numpy as np
import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Pose
from std_msgs.msg import Int32
from rclpy.qos import QoSProfile
from tf_transformations import quaternion_from_euler, euler_from_quaternion


class TrajectoryCircle(Node):
    def __init__(self):
        super().__init__('trajectory_circle')

        # Inicialización de variables
        self.r = 0.0
        self.h = 0.0
        self.t = 0.0
        self.rt = 100.0  # Frecuencia de publicación

        self.button_pressed = False
        self.start_time = self.get_clock().now()

        # Bandera para almacenar valores iniciales solo una vez
        self.initial_values_stored = False

        self.state_msg = -1  # Valor inicial por defecto

        self.declare_parameter('lider_name', 'bebop2')
        self.lider_name = self.get_parameter('lider_name').value.strip()

        qos = QoSProfile(depth=10)

        self.pub = self.create_publisher(Pose, '/goal', qos)

        self.ini_pos_sub = self.create_subscription(Pose, f"/{self.lider_name}/set_pose", self.pos_changed, qos)
        self.state_sub = self.create_subscription(Int32, "/state", self.state, qos)

        self.declare_parameter('xi', 0.0)
        self.declare_parameter('yi', 0.0)
        self.declare_parameter('zi', 0.0)
        self.declare_parameter('h', 0.0)
        self.declare_parameter('r', 0.0)
        self.declare_parameter('yawi', 0.0)
        self.declare_parameter('angular_frequency', np.pi / 12)
        self.declare_parameter('smoothing_parameter', 15.0)

        self.xi = self.get_parameter('xi').value
        self.yi = self.get_parameter('yi').value
        self.zi = self.get_parameter('zi').value
        self.altura = self.get_parameter('h').value
        self.radio = self.get_parameter('r').value
        self.yawi = self.get_parameter('yawi').value
        self.w = self.get_parameter('angular_frequency').value
        self.p = self.get_parameter('smoothing_parameter').value

        if self.radio < 0 or self.altura < 0:
            self.get_logger().error("El radio y la altura deben ser valores positivos.")
            raise ValueError("Parámetros inválidos.")

        self.xii = 0.0
        self.yii = 0.0
        self.zii = 0.0
        self.yawii = 0.0

        self.trajectory_timer = self.create_timer(1.0 / self.rt, self.trajectory_circle)

    def state(self, msg):
        if msg.data == 1 and not self.button_pressed:
            self.get_logger().info('Iniciando trayectoria')
            self.button_pressed = True
            self.r = self.radio
            self.h = self.altura
            self.t = 0.0
            self.start_time = self.get_clock().now()

            if not self.initial_values_stored:
                try:
                    self.xii = self.initial_pose.position.x
                    self.yii = self.initial_pose.position.y
                    self.zii = self.initial_pose.position.z

                    euler_angles = euler_from_quaternion([
                        self.initial_pose.orientation.x,
                        self.initial_pose.orientation.y,
                        self.initial_pose.orientation.z,
                        self.initial_pose.orientation.w
                    ])

                    self.yawii = euler_angles[2]
                    self.initial_values_stored = True

                    self.get_logger().info(f'Valores iniciales almacenados: x={self.xii}, y={self.yii}, z={self.zii}, yaw={self.yawii}')
                except AttributeError:
                    self.get_logger().warn('Pose inicial no disponible aún.')

        elif msg.data == 6:
            self.get_logger().info('Reiniciando trayectoria')
            self.r = 0.0
            self.h = 0.0
            self.t = 0.0
            self.button_pressed = False
            self.start_time = self.get_clock().now()

    def pos_changed(self, msg):
        self.initial_pose = msg

    def trajectory_circle(self):
        if not self.button_pressed:
            return

        current_time = self.get_clock().now()
        elapsed_time = (current_time - self.start_time).nanoseconds / 1e9

        if elapsed_time >= 60:
            self.get_logger().info('Trayectoria Terminada')
            self.button_pressed = False
            return

        pose_msg = Pose()

        self.t = elapsed_time
        scale = self.r * (np.arctan(self.p) + np.arctan(self.t - self.p))
        x = scale * np.cos(self.w * self.t)
        y = scale * np.sin(self.w * self.t)
        z = 1.0 #(self.h / 2) * (1 + np.tanh(self.t - 2.5))
        yaw = 0

        # q = quaternion_from_euler(0, 0, yaw + self.yawii)
        q = quaternion_from_euler(0, 0, yaw)

        pose_msg.position.x = x + self.xii
        pose_msg.position.y = y + self.yii
        pose_msg.position.z = z + self.zii
        pose_msg.orientation.x = q[0]
        pose_msg.orientation.y = q[1]
        pose_msg.orientation.z = q[2]
        pose_msg.orientation.w = q[3]

        self.pub.publish(pose_msg)


def main(args=None):
    rclpy.init(args=args)
    traj_circle = TrajectoryCircle()
    rclpy.spin(traj_circle)
    traj_circle.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()