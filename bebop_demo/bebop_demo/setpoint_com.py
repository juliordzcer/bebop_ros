#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile
from geometry_msgs.msg import Pose
from sensor_msgs.msg import Joy
from tf_transformations import quaternion_from_euler, euler_from_quaternion, quaternion_multiply
import numpy as np
import json

class JoystickButtons:
    """Enumeración para los botones del joystick."""
    BUTTON_2 = 3  # Botón que iniciará la trayectoria
    BUTTON_10 = 10  # Botón que reiniciará la trayectoria

class CombinedSetpoint(Node):
    def __init__(self):
        super().__init__('combined_setpoint')

        # Inicialización de variables
        self.r = 0.0  # Radio de la trayectoria circular
        self.h = 0.0  # Altura de la trayectoria
        self.t = 0.0  # Tiempo de la trayectoria
        self.rt = 100.0  # Frecuencia de publicación
        self.button_pressed = False  # Estado del botón
        self.start_time = self.get_clock().now()  # Tiempo de inicio
        self.initial_values_stored = False  # Bandera para almacenar valores iniciales

        # Declaración de parámetros
        self.declare_parameter('lider_name', 'bebop3')
        self.declare_parameter('robot_names', '["bebop1", "bebop2", "bebop3"]')
        self.declare_parameter('formation', '[[1.0, 0.0, 0.0, 0.0], [-1.0, 0.0, 0.0, 0.0]]')
        self.declare_parameter('xi', 0.0)
        self.declare_parameter('yi', 0.0)
        self.declare_parameter('zi', 0.0)
        self.declare_parameter('h', 1.0)
        self.declare_parameter('r', 1.0)
        self.declare_parameter('yawi', 0.0)
        self.declare_parameter('angular_frequency', np.pi / 12)
        self.declare_parameter('smoothing_parameter', 15.0)

        # Obtener parámetros
        self.lider_name = self.get_parameter('lider_name').value.strip()
        self.robot_names = json.loads(self.get_parameter('robot_names').value)
        self.formation = json.loads(self.get_parameter('formation').value)
        self.xi = self.get_parameter('xi').value
        self.yi = self.get_parameter('yi').value
        self.zi = self.get_parameter('zi').value
        self.altura = self.get_parameter('h').value
        self.radio = self.get_parameter('r').value
        self.yawi = self.get_parameter('yawi').value
        self.w = self.get_parameter('angular_frequency').value
        self.p = self.get_parameter('smoothing_parameter').value

        # Validación de parámetros
        if self.radio < 0 or self.altura < 0:
            self.get_logger().error("El radio y la altura deben ser valores positivos.")
            raise ValueError("Parámetros inválidos.")

        # Variables para almacenar los valores iniciales
        self.xii = 0.0
        self.yii = 0.0
        self.zii = 0.0
        self.yawii = 0.0

        # Configuración de QoS
        qos = QoSProfile(depth=10)

        # Suscripción al joystick
        self.joy_sub = self.create_subscription(Joy, 'joy', self.joy_callback, qos)

        # Publicador para el líder
        self.pub_leader = self.create_publisher(Pose, '/goal', qos)

        # Publicadores para los seguidores
        self.pub_followers = [
            self.create_publisher(Pose, f'/{name}/setpoint', qos)
            for name in self.robot_names if name != self.lider_name
        ]

        # Temporizador para la trayectoria
        self.timer = self.create_timer(1.0 / self.rt, self.trajectory_circle)

    def joy_callback(self, joy_msg):
        """Callback para el joystick."""
        if joy_msg.buttons[JoystickButtons.BUTTON_2] == 1 and not self.button_pressed:
            self.get_logger().info('Botón 2 presionado: Iniciando trayectoria')
            self.button_pressed = True
            self.r = self.radio
            self.h = self.altura
            self.t = 0.0
            self.start_time = self.get_clock().now()

            # Almacenar valores iniciales solo la primera vez
            if not self.initial_values_stored:
                self.xii = self.xi
                self.yii = self.yi
                self.zii = self.zi
                self.yawii = self.yawi
                self.initial_values_stored = True
                self.get_logger().info(f'Valores iniciales almacenados: x={self.xii}, y={self.yii}, z={self.zii}, yaw={self.yawii}')

        elif joy_msg.buttons[JoystickButtons.BUTTON_10] == 1 and self.button_pressed:
            self.get_logger().info('Botón 10 presionado: Reiniciando trayectoria')
            self.button_pressed = False
            self.r = 0.0
            self.h = 0.0
            self.t = 0.0
            self.start_time = self.get_clock().now()

    def trajectory_circle(self):
        """Publica la trayectoria circular para el líder y los setpoints para los seguidores."""
        if not self.button_pressed:
            return

        current_time = self.get_clock().now()
        elapsed_time = (current_time - self.start_time).nanoseconds / 1e9

        if elapsed_time >= 60:
            self.get_logger().info('Trayectoria Terminada')
            self.button_pressed = False
            return

        # Calcular la trayectoria del líder
        self.t = elapsed_time
        x = self.r * (np.arctan(self.p) + np.arctan(self.t - self.p)) * np.cos(self.w * self.t)
        y = self.r * (np.arctan(self.p) + np.arctan(self.t - self.p)) * np.sin(self.w * self.t)
        z = (self.h / 2) * (1 + np.tanh(self.t - 2.5))
        yaw = 0  # yaw fijo para simplificar

        # Crear mensaje de pose para el líder
        pose_leader = Pose()
        pose_leader.position.x = x + self.xii
        pose_leader.position.y = y + self.yii
        pose_leader.position.z = z + self.zii
        pose_leader.orientation = self.quaternion_from_yaw(yaw + self.yawii)

        # Publicar la pose del líder
        self.pub_leader.publish(pose_leader)

        # Calcular y publicar las poses de los seguidores
        for i, (name, conditions) in enumerate(zip(self.robot_names, self.formation)):
            if name == self.lider_name:
                continue  # Saltar al líder

            pose_follower = self.calculate_follower_pose(pose_leader, conditions)
            self.pub_followers[i].publish(pose_follower)

    def calculate_follower_pose(self, leader_pose, conditions):
        """Calcula la pose del seguidor basada en la pose del líder y las condiciones de formación."""
        pose = Pose()
        pose.position.x = leader_pose.position.x + conditions[0]
        pose.position.y = leader_pose.position.y + conditions[1]
        pose.position.z = leader_pose.position.z + conditions[2]

        # Calcular la orientación del seguidor
        q_offset = quaternion_from_euler(0, 0, conditions[3])
        q_leader = [
            leader_pose.orientation.x,
            leader_pose.orientation.y,
            leader_pose.orientation.z,
            leader_pose.orientation.w,
        ]
        q_result = quaternion_multiply(q_leader, q_offset)

        pose.orientation.x = q_result[0]
        pose.orientation.y = q_result[1]
        pose.orientation.z = q_result[2]
        pose.orientation.w = q_result[3]

        return pose

    def quaternion_from_yaw(self, yaw):
        """Convierte un ángulo de yaw a un cuaternión."""
        q = quaternion_from_euler(0, 0, yaw)
        return Pose.Orientation(x=q[0], y=q[1], z=q[2], w=q[3])

def main(args=None):
    rclpy.init(args=args)
    node = CombinedSetpoint()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()