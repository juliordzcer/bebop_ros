

#!/usr/bin/env python3

import numpy as np
import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Pose
from sensor_msgs.msg import Joy
from rclpy.qos import QoSProfile
from tf_transformations import quaternion_from_euler

class TrajectoryCircle(Node):
    def __init__(self):
        super().__init__('trajectory_circle')

        # Inicialización de variables
        self.r = 0.0
        self.h = 0.0
        self.t = 0.0
        self.rt = 50.0  # Frecuencia de publicación
        self.hd = 0.05075
        self.button_pressed = False
        self.start_time = self.get_clock().now()

        # Configuración de QoS
        qos = QoSProfile(depth=10)

        # Publicador para el tópico /goal
        self.pub = self.create_publisher(Pose, '/goal', qos)

        self.declare_parameter('robot_name', 'bebop2')
        robot_name = self.get_parameter('robot_name').get_parameter_value().string_value
  
        # Validar que el nombre del robot no esté vacío
        if not robot_name.strip():
            self.get_logger().error('El parámetro "robot_name" está vacío. Se usará "bebop2" por defecto.')
            robot_name = 'bebop2'

        # Tópico dinámico
        cmd_set_pose_topic = f"/{robot_name}/set_pose"



        # Publicador para el tópico /set_pose
        self.pubinipos = self.create_publisher(Pose, cmd_set_pose_topic, qos)

        # Declaración de parámetros
        self.declare_parameter('xi', 0.0)
        self.declare_parameter('yi', 0.0)
        self.declare_parameter('zi', 0.0)
        self.declare_parameter('h', 0.0)
        self.declare_parameter('r', 0.0)
        self.declare_parameter('yawi', 0.0)  # Ángulo de yaw inicial

        # Obtención de parámetros
        self.xi = self.get_parameter('xi').get_parameter_value().double_value
        self.yi = self.get_parameter('yi').get_parameter_value().double_value
        self.zi = self.get_parameter('zi').get_parameter_value().double_value 
        self.altura = self.get_parameter('h').get_parameter_value().double_value
        self.radio = self.get_parameter('r').get_parameter_value().double_value
        self.yawi = self.get_parameter('yawi').get_parameter_value().double_value

        # Publicar condiciones iniciales después de un retraso de 2 segundos
        self.timer = self.create_timer(2.0, self.publish_initial_pose)

        # Suscripción al tópico del joystick
        self.joy_sub = self.create_subscription(Joy, 'joy', self.joy_callback, qos)
        self.get_logger().info('Nodo inicializado y suscrito al tópico /joy')

        # Iniciar el bucle de la trayectoria
        self.trajectory_timer = self.create_timer(1.0 / self.rt, self.trajectory_circle)

    def publish_initial_pose(self):
        # Condiciones iniciales
        posei_msg = Pose()
        # Conversión de ángulos de Euler a cuaternión
        q_i = quaternion_from_euler(0, 0, self.yawi)

        # Asignación de valores al mensaje
        posei_msg.position.x = self.xi
        posei_msg.position.y = self.yi
        posei_msg.position.z = self.zi + self.hd
        posei_msg.orientation.x = q_i[0]
        posei_msg.orientation.y = q_i[1]
        posei_msg.orientation.z = q_i[2]
        posei_msg.orientation.w = q_i[3]

        # Publicación del mensaje
        self.pubinipos.publish(posei_msg)
        self.get_logger().info(f'Pose inicial publicada: {posei_msg}')

        self.xii = self.xi
        self.yii = self.yi
        self.zii = self.zi
        self.yawii = self.yawi

        # Cancelar el temporizador después de la primera ejecución
        self.timer.cancel()

    def joy_callback(self, joy_msg):
        # Iniciar trayectoria si se presiona el botón 2
        if joy_msg.buttons[2] == 1 and not self.button_pressed:
            self.get_logger().info('Botón 2 presionado: Iniciando trayectoria')
            self.button_pressed = True
            self.r = self.radio
            self.h = self.altura
            self.t = 0.0
            self.start_time = self.get_clock().now()

        # Reiniciar trayectoria si se presiona el botón 10
        elif joy_msg.buttons[10] == 1 and self.button_pressed:
            self.get_logger().info('Botón 10 presionado: Reiniciando trayectoria')
            self.r = 0.0
            self.h = 0.0
            self.t = 0.0
            self.button_pressed = False
            self.start_time = self.get_clock().now()

    def trajectory_circle(self):
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
        w = np.pi / 6  # Frecuencia angular
        p = 15  # Parámetro de suavizado

        x = self.r * (np.arctan(p) + np.arctan(self.t - p)) * np.cos(w * self.t)
        y = self.r * (np.arctan(p) + np.arctan(self.t - p)) * np.sin(w * self.t)
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