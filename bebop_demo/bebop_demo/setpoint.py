#!/usr/bin/env python3

import numpy as np
import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Pose
from sensor_msgs.msg import Joy
from rclpy.qos import QoSProfile
from tf_transformations import quaternion_from_euler

class JoystickButtons:
    """Enumeración para los botones del joystick."""
    BUTTON_2 = 2
    BUTTON_10 = 10

class TrajectoryCircle(Node):
    def __init__(self):
        super().__init__('trajectory_circle')

        # Inicialización de variables
        self.r = 0.0
        self.h = 0.0
        self.t = 0.0
        self.rt = 50.0  # Frecuencia de publicación
        
        self.button_pressed = False
        self.start_time = self.get_clock().now()

        # Bandera para almacenar valores iniciales solo una vez
        self.initial_values_stored = False

        # Configuración de QoS
        qos = QoSProfile(depth=10)

        # Publicador para el tópico /goal
        self.pub = self.create_publisher(Pose, '/goal', qos)

        # Declaración de parámetros
        self.declare_parameter('xi', 0.0)
        self.declare_parameter('yi', 0.0)
        self.declare_parameter('zi', 0.0)
        self.declare_parameter('h', 0.0)
        self.declare_parameter('r', 0.0)
        self.declare_parameter('yawi', 0.0)  # Ángulo de yaw inicial
        self.declare_parameter('angular_frequency', np.pi / 6)
        self.declare_parameter('smoothing_parameter', 15.0)

        # Obtención de parámetros
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

        # Suscripción al tópico del joystick
        self.joy_sub = self.create_subscription(Joy, 'joy', self.joy_callback, qos)
        self.get_logger().info('Nodo inicializado y suscrito al tópico /joy')

        # Iniciar el bucle de la trayectoria
        self.trajectory_timer = self.create_timer(1.0 / self.rt, self.trajectory_circle)

    def joy_callback(self, joy_msg):
        """Callback para el tópico /joy. Inicia o reinicia la trayectoria según los botones presionados."""
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

        # Reiniciar trayectoria si se presiona el botón 10
        elif joy_msg.buttons[JoystickButtons.BUTTON_10] == 1 and self.button_pressed:
            self.get_logger().info('Botón 10 presionado: Reiniciando trayectoria')
            self.r = 0.0
            self.h = 0.0
            self.t = 0.0
            self.button_pressed = False
            self.start_time = self.get_clock().now()

    def trajectory_circle(self):
        """Publica la trayectoria circular en el tópico /goal."""
        if not self.button_pressed:
            return

        current_time = self.get_clock().now()
        elapsed_time = (current_time - self.start_time).nanoseconds / 1e9

        if elapsed_time >= 60:
            self.get_logger().info('Trayectoria Terminada')
            self.button_pressed = False
            return

        # Generación del mensaje de posición
        pose_msg = Pose()

        # Cálculo de la trayectoria circular
        self.t = elapsed_time
        x = self.r * (np.arctan(self.p) + np.arctan(self.t - self.p)) * np.cos(self.w * self.t)
        y = self.r * (np.arctan(self.p) + np.arctan(self.t - self.p)) * np.sin(self.w * self.t)
        z = (self.h / 2) * (1 + np.tanh(self.t - 2.5))
        yaw = 0.0
        roll = 0.0
        pitch = 0.0

        # Conversión de ángulos de Euler a cuaternión
        q = quaternion_from_euler(roll, pitch, yaw + self.yawii)

        # Asignación de valores al mensaje
        pose_msg.position.x = x + self.xii
        pose_msg.position.y = y + self.yii
        pose_msg.position.z = z + self.zii
        pose_msg.orientation.x = q[0]
        pose_msg.orientation.y = q[1]
        pose_msg.orientation.z = q[2]
        pose_msg.orientation.w = q[3]

        # Publicación del mensaje
        self.pub.publish(pose_msg)

def main(args=None):
    rclpy.init(args=args)
    traj_circle = TrajectoryCircle()
    rclpy.spin(traj_circle)
    traj_circle.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()