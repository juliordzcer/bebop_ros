#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist
from sensor_msgs.msg import Joy

class JoystickNode(Node):
    def __init__(self):
        super().__init__('joystick_node')

        # Publisher para enviar comandos de movimiento
        self.publisher_ = self.create_publisher(Twist, '/parrot_bebop_2/cmd_vel', 10)

        # Subscriber para leer los datos del joystick
        self.subscription = self.create_subscription(
            Joy,
            '/joy',
            self.joy_callback,
            10
        )

        # Configuración inicial del comando de movimiento
        self.cmd = Twist()

        # Factor de escala para reducir la velocidad
        self.scale = 0.6  # Ajusta este valor para controlar la velocidad (0.1 = lento, 1.0 = rápido)

    def joy_callback(self, msg):
        """
        Callback que se ejecuta cuando llegan los datos del joystick.
        """
        # Asignación de ejes y botones (ajusta según tu control)
        self.cmd.linear.x = float(msg.axes[1]) * self.scale  # Eje izquierdo (arriba/abajo)
        self.cmd.linear.y = float(msg.axes[0]) * self.scale  # Eje izquierdo (izquierda/derecha)
        self.cmd.linear.z = float(msg.buttons[4] - msg.buttons[5]) * self.scale  # Botones LB (4) y RB (5)
        self.cmd.angular.z = float(msg.axes[3]) * self.scale  # Eje derecho (izquierda/derecha)

        # Publicar el comando de movimiento
        self.publisher_.publish(self.cmd)

def main(args=None):
    rclpy.init(args=args)
    node = JoystickNode()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()