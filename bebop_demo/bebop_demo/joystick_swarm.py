#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist
from sensor_msgs.msg import Joy

class JoystickNode(Node):
    def __init__(self):
        super().__init__('joystick_node')

        # Declarar parámetros
        self.declare_parameter('robot_name', 'bebop2')  
        self.declare_parameter('scale', 0.6)  # Factor de escala configurable

        # Obtener parámetros
        robot_name = self.get_parameter('robot_name').get_parameter_value().string_value
        self.scale = self.get_parameter('scale').get_parameter_value().double_value

        # Validar que el nombre del robot no esté vacío
        if not robot_name.strip():
            self.get_logger().error('El parámetro "robot_name" está vacío. Se usará "bebop2" por defecto.')
            robot_name = 'bebop2'

        # Tópico dinámico
        cmd_vel_topic = f"/{robot_name}/cmd_vel"

        # Publisher y Subscriber
        self.publisher_ = self.create_publisher(Twist, cmd_vel_topic, 10)
        self.subscription = self.create_subscription(Joy, '/joy', self.joy_callback, 10)

        # Inicialización del mensaje Twist
        self.cmd = Twist()

        self.get_logger().info(f"Joystick Node iniciado. Publicando en: {cmd_vel_topic}")
        self.get_logger().info(f"Factor de escala configurado en: {self.scale}")

    def joy_callback(self, msg):
        """
        Callback que se ejecuta cuando llegan los datos del joystick.
        """
        try:
            self.cmd.linear.x = float(msg.axes[1]) * self.scale
            self.cmd.linear.y = float(msg.axes[0]) * self.scale
            self.cmd.linear.z = float(msg.axes[4]) * self.scale
            self.cmd.angular.z = float(msg.axes[3]) * self.scale

            # Publicar el comando de movimiento
            self.publisher_.publish(self.cmd)

        except (IndexError, TypeError) as e:
            self.get_logger().error(f"Error en la lectura del joystick: {e}")

def main(args=None):
    rclpy.init(args=args)
    node = JoystickNode()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
