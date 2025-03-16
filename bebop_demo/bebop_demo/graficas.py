import os
import json
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

class MultiRobotDataRecorder(Node):
    def __init__(self):
        super().__init__('multi_robot_data_recorder')
        
        # Leer nombres de robots desde parámetro
        self.declare_parameter('robot_names', '[]')  # JSON con nombres de los robots
        raw_robot_names = self.get_parameter('robot_names').value
        
        try:
            self.robot_names = json.loads(raw_robot_names)
        except json.JSONDecodeError as e:
            self.get_logger().error(f"Error en JSON de robot_names: {str(e)}")
            return
        
        if not self.robot_names:
            self.get_logger().error("No se han definido robots para grabar.")
            return
        
        # Inicializar almacenamiento de datos
        self.time = []
        self.poses = {name: {'x': [], 'y': [], 'z': [], 'yaw': []} for name in self.robot_names}
        self.setpoints = {name: {'x': [], 'y': [], 'z': [], 'yaw': []} for name in self.robot_names}
        
        self.is_saving_data = False
        self.recording_started = False  
        
        qos_profile = QoSProfile(depth=10)
        
        # Suscripciones dinámicas
        for name in self.robot_names:
            self.create_subscription(Pose, f'/{name}/pose', lambda msg, n=name: self.pose_callback(msg, n), qos_profile)
            self.create_subscription(Pose, f'/{name}/setpointG', lambda msg, n=name: self.goal_callback(msg, n), qos_profile)
        
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
        
        self.timer = self.create_timer(0.01, self.record_data)  # Cada 10 ms
        self.stop_timer = Timer(60.0, self.stop_recording)
        self.stop_timer.start()
    
    def stop_recording(self):
        if self.is_saving_data:
            self.is_saving_data = False
            self.recording_started = False
            if hasattr(self, 'timer') and self.timer:
                self.destroy_timer(self.timer)
            if hasattr(self, 'stop_timer') and self.stop_timer:
                self.stop_timer.cancel()
            self.get_logger().info("Grabación finalizada. Guardando datos...")
            self.save_data()
    
    def record_data(self):
        if not self.is_saving_data:
            return
        
        current_time = self.get_clock().now().seconds_nanoseconds()[0] - self.start_time
        self.time.append(current_time)
    
    def pose_callback(self, msg, name):
        if not self.is_saving_data:
            return
        
        self.poses[name]['x'].append(msg.position.x)
        self.poses[name]['y'].append(msg.position.y)
        self.poses[name]['z'].append(msg.position.z)
        
        q = msg.orientation
        _, _, yaw = euler_from_quaternion([q.x, q.y, q.z, q.w])
        self.poses[name]['yaw'].append(yaw)
    
    def goal_callback(self, msg, name):
        self.setpoints[name]['x'] = msg.position.x
        self.setpoints[name]['y'] = msg.position.y
        self.setpoints[name]['z'] = msg.position.z
        
        q = msg.orientation
        _, _, self.setpoints[name]['yaw'] = euler_from_quaternion([q.x, q.y, q.z, q.w])
    
    def save_data(self):
        if not self.time:
            self.get_logger().warn("No hay datos para guardar.")
            return
        
        home_dir = os.path.expanduser("~/Experimentos")
        if not os.path.exists(home_dir):
            os.makedirs(home_dir)
        
        data_dict = {'time': self.time, 'poses': self.poses, 'setpoints': self.setpoints}
        mat_path = os.path.join(home_dir, "drones_data.mat")
        
        try:
            io.savemat(mat_path, data_dict)
            self.get_logger().info(f"Datos guardados en: {mat_path}")
        except Exception as e:
            self.get_logger().error(f"Error al guardar datos: {e}")
    

def main(args=None):
    rclpy.init(args=args)
    recorder = MultiRobotDataRecorder()
    
    try:
        rclpy.spin(recorder)
    except KeyboardInterrupt:
        recorder.save_data()
    finally:
        recorder.destroy_node()
        rclpy.shutdown()
    

if __name__ == '__main__':
    main()