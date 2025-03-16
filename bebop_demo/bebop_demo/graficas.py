#!/usr/bin/env python3
import os
import numpy as np
import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Pose
from nav_msgs.msg import Odometry
from sensor_msgs.msg import Joy
from scipy import io
from tf_transformations import euler_from_quaternion
from threading import Timer
from rclpy.qos import QoSProfile

class DataRecorder(Node):
    def __init__(self):
        super().__init__('data_recorder')

        # Variables para almacenar datos
        self.time = []
        self.pose_1 = {'x': [], 'y': [], 'z': [], 'yaw': []}
        self.pose_2 = {'x': [], 'y': [], 'z': [], 'yaw': []}
        self.setpoint_1 = {'x': [], 'y': [], 'z': [], 'yaw': []}
        self.setpoint_2 = {'x': [], 'y': [], 'z': [], 'yaw': []}

        self.is_saving_data = False
        self.recording_started = False  

        # Inicializar posiciones y velocidades deseadas
        self.x_d1, self.y_d1, self.z_d1, self.yaw_d1 = 0.0, 0.0, 0.0, 0.0
        self.x_d2, self.y_d2, self.z_d2, self.yaw_d2 = 0.0, 0.0, 0.0, 0.0

        qos_profile = QoSProfile(depth=10)
        # Suscripciones
        self.create_subscription(Pose, '/bebop1/pose', self.pose1_callback, qos_profile)
        self.create_subscription(Pose, '/bebop2/pose', self.pose2_callback, qos_profile)
        self.create_subscription(Pose, '/bebop1/setpointG', self.goal1_callback, qos_profile)
        self.create_subscription(Pose, '/bebop2/setpointG', self.goal2_callback, qos_profile)
        self.create_subscription(Joy, '/joy', self.joy_callback, qos_profile)

        self.get_logger().info("Esperando botón X del control de Xbox para iniciar...")

    def joy_callback(self, msg):
        if len(msg.buttons) > 0 and msg.buttons[2] == 1:  # Botón X presionado
            if not self.recording_started:
                self.start_recording()

    def start_recording(self):
        self.recording_started = True
        self.is_saving_data = True
        self.start_time = self.get_clock().now().seconds_nanoseconds()[0]
        self.get_logger().info("Grabación iniciada...")

        self.timer = self.create_timer(0.001, self.record_data)  
        self.stop_timer = Timer(60.0, self.stop_recording)
        self.stop_timer.start()

    def stop_recording(self):
        if self.is_saving_data:
            self.is_saving_data = False
            self.recording_started = False
            if hasattr(self, 'timer') and self.timer:
                self.timer.cancel()
            if hasattr(self, 'stop_timer') and self.stop_timer:
                self.stop_timer.cancel()
            self.get_logger().info("Grabación finalizada. Guardando datos...")
            self.save_data()

    def record_data(self):
        if not self.is_saving_data:
            return

        current_time = self.get_clock().now().seconds_nanoseconds()[0] - self.start_time
        self.time.append(current_time)

        self.setpoint_1['x'].append(self.x_d1)
        self.setpoint_1['y'].append(self.y_d1)
        self.setpoint_1['z'].append(self.z_d1)
        self.setpoint_1['yaw'].append(self.yaw_d1)

        self.setpoint_2['x'].append(self.x_d2)
        self.setpoint_2['y'].append(self.y_d2)
        self.setpoint_2['z'].append(self.z_d2)
        self.setpoint_2['yaw'].append(self.yaw_d2)

    def pose1_callback(self, msg):
        if not self.is_saving_data:
            return

        self.pose_1['x'].append(msg.position.x)
        self.pose_1['y'].append(msg.position.y)
        self.pose_1['z'].append(msg.position.z)

        q = msg.orientation
        _, _, yaw = euler_from_quaternion([q.x, q.y, q.z, q.w])
        self.pose_1['yaw'].append(yaw)

    def pose2_callback(self, msg):
        if not self.is_saving_data:
            return

        self.pose_2['x'].append(msg.position.x)
        self.pose_2['y'].append(msg.position.y)
        self.pose_2['z'].append(msg.position.z)

        q = msg.orientation
        _, _, yaw = euler_from_quaternion([q.x, q.y, q.z, q.w])
        self.pose_2['yaw'].append(yaw)

    def goal1_callback(self, msg):
        self.x_d1 = msg.position.x
        self.y_d1 = msg.position.y
        self.z_d1 = msg.position.z

        q = msg.orientation
        _, _, self.yaw_d1 = euler_from_quaternion([q.x, q.y, q.z, q.w])

    def goal2_callback(self, msg):
        self.x_d2 = msg.position.x
        self.y_d2 = msg.position.y
        self.z_d2 = msg.position.z

        q = msg.orientation
        _, _, self.yaw_d2 = euler_from_quaternion([q.x, q.y, q.z, q.w])

    def save_data(self):
        if not self.time:
            self.get_logger().warn("No hay datos para guardar.")
            return

        min_length = min(len(self.time), len(self.pose_1['x']))
        self.time = self.time[:min_length]

        home_dir = os.path.expanduser("~/Experimentos")
        if not os.path.exists(home_dir):
            os.makedirs(home_dir)

        data_dict = {
            'time': self.time,
            'pose1': self.pose_1,
            'pose2': self.pose_2,
            'setpoint1': self.setpoint_1,
            'setpoint2': self.setpoint_2,
        }

        mat_path = os.path.join(home_dir, "drones_data.mat")
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
        recorder.save_data()
        rclpy.shutdown()

if __name__ == '__main__':
    main()