#!/usr/bin/env python3
import os
import numpy as np
import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Pose
from nav_msgs.msg import Odometry
from scipy import io
from tf_transformations import euler_from_quaternion
from threading import Timer

class DataRecorder(Node):
    def __init__(self):
        super().__init__('data_recorder')

        # Obtener el parámetro desde ROS
        self.declare_parameter('recording_time', 60)  # Valor por defecto de 60 segundos
        self.recording_time = self.get_parameter('recording_time').get_parameter_value().integer_value

        # Variables para guardar las posiciones, errores y tiempo
        self.time = []
        self.x_actual = []
        self.y_actual = []
        self.z_actual = []
        self.yaw_actual = []
        self.error_x = []
        self.error_y = []
        self.error_z = []
        self.error_yaw = []
        self.is_saving_data = False
        
        # Inicializar las posiciones deseadas
        self.x_deseada_val = 0.0
        self.y_deseada_val = 0.0
        self.z_deseada_val = 0.0
        self.yaw_deseada_val = 0.0

        # Suscripción a la odometría y la meta (goal)
        self.create_subscription(Odometry, '/model/parrot_bebop_2/odometry', self.odometry_callback, 10)
        self.create_subscription(Pose, '/goal', self.goal_callback, 10)

        # Iniciar la grabación de datos
        self.start_recording()

    def start_recording(self):
        self.is_saving_data = True
        self.start_time = self.get_clock().now().seconds_nanoseconds()[0]
        self.get_logger().info("Comenzando a guardar los datos...")
        # Iniciar el temporizador para guardar datos cada 0.001 segundos
        self.timer = self.create_timer(0.001, self.record_data)
        # Detener la grabación después de `recording_time` segundos
        Timer(self.recording_time, self.stop_recording).start()

    def stop_recording(self):
        self.is_saving_data = False
        self.timer.cancel()
        self.get_logger().info("Tiempo de grabación completado. Dejando de guardar los datos...")
        self.save_data()

    def record_data(self):
        if not self.is_saving_data:
            return

        # Extraemos el tiempo desde que comenzó la grabación
        current_time = self.get_clock().now().seconds_nanoseconds()[0] - self.start_time
        self.time.append(current_time)

    def odometry_callback(self, msg):
        if not self.is_saving_data:
            return

        # Extraemos las posiciones actuales
        self.x_actual.append(msg.pose.pose.position.x)
        self.y_actual.append(msg.pose.pose.position.y)
        self.z_actual.append(msg.pose.pose.position.z)

        # Calcular el error en las posiciones
        self.error_x.append((self.x_deseada_val - msg.pose.pose.position.x))
        self.error_y.append((self.y_deseada_val - msg.pose.pose.position.y))
        self.error_z.append((self.z_deseada_val - msg.pose.pose.position.z))

        # Extraemos el yaw de la orientación
        q = msg.pose.pose.orientation
        rpy = euler_from_quaternion([q.x, q.y, q.z, q.w])
        self.yaw_actual.append(rpy[2])

        # Calcular el error de yaw
        self.error_yaw.append((self.yaw_deseada_val - rpy[2]))

    def goal_callback(self, msg):
        # Guardamos las posiciones deseadas del goal
        self.x_deseada_val = msg.position.x
        self.y_deseada_val = msg.position.y
        self.z_deseada_val = msg.position.z
        # Si lo deseas, también puedes obtener el yaw deseado de la orientación del goal
        q = msg.orientation
        rpy = euler_from_quaternion([q.x, q.y, q.z, q.w])
        self.yaw_deseada_val = rpy[2]

    def save_data(self):
        # Verificar que se tengan datos para guardar
        if not self.time:
            self.get_logger().warn("No se han recopilado datos para guardar.")
            return
        
        # Asegurarse de que todas las listas tengan la misma longitud
        min_length = min(len(self.time), len(self.x_actual), len(self.y_actual), len(self.z_actual), len(self.yaw_actual), len(self.error_x), len(self.error_y), len(self.error_z), len(self.error_yaw))
        self.time = self.time[:min_length]
        self.x_actual = self.x_actual[:min_length]
        self.y_actual = self.y_actual[:min_length]
        self.z_actual = self.z_actual[:min_length]
        self.yaw_actual = self.yaw_actual[:min_length]
        self.error_x = self.error_x[:min_length]
        self.error_y = self.error_y[:min_length]
        self.error_z = self.error_z[:min_length]
        self.error_yaw = self.error_yaw[:min_length]

        # Guardar los datos en archivos
        home_dir = os.path.expanduser("~/Experimentos")
        if not os.path.exists(home_dir):
            os.makedirs(home_dir)
        
        data_dict = {
            'time': self.time,
            'x_actual': self.x_actual,
            'y_actual': self.y_actual,
            'z_actual': self.z_actual,
            'yaw_actual': self.yaw_actual,
            'x_deseada': [self.x_deseada_val] * len(self.time),
            'y_deseada': [self.y_deseada_val] * len(self.time),
            'z_deseada': [self.z_deseada_val] * len(self.time),
            'yaw_deseada': [self.yaw_deseada_val] * len(self.time),
            'error_x': self.error_x,
            'error_y': self.error_y,
            'error_z': self.error_z,
            'error_yaw': self.error_yaw
        }

        mat_path = os.path.join(home_dir, "dron_data.mat")
        io.savemat(mat_path, data_dict)
        self.get_logger().info(f"Datos guardados en: {mat_path}")

def main(args=None):
    rclpy.init(args=args)
    
    recorder = DataRecorder()
    
    try:
        rclpy.spin(recorder)
    except KeyboardInterrupt:
        pass
    finally:
        # Es posible que ya se haya guardado antes, pero lo dejamos aquí por si acaso
        recorder.save_data()
        rclpy.shutdown()

if __name__ == '__main__':
    main()