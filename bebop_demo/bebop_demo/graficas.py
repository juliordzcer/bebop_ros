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
from rclpy.qos import QoSProfile
import atexit

class MultiRobotDataRecorder(Node):
    def __init__(self):
        super().__init__('multi_robot_data_recorder')
        
        # Declarar y obtener el parámetro 'n' (número de robots)
        self.declare_parameter('n', 1)  
        self.n = self.get_parameter('n').value
        
        # Validar que el número de robots sea válido
        if self.n < 1:
            self.get_logger().error("El número de robots debe ser al menos 1.")
            raise ValueError("El número de robots debe ser al menos 1.")
        
        # Inicializar variables
        self.robot_names = [f'bebop{i+1}' for i in range(self.n)]
        self.time = []
        self.poses = {name: {'x': [], 'y': [], 'z': [], 'yaw': []} for name in self.robot_names}
        self.setpoints = {name: {'x': [], 'y': [], 'z': [], 'yaw': []} for name in self.robot_names}
        
        self.is_saving_data = False
        self.recording_started = False  
        
        # Configurar QoS
        qos_profile = QoSProfile(depth=10)
        
        # Crear suscripciones para cada robot
        for name in self.robot_names:
            self.create_subscription(Pose, f'/{name}/pose', lambda msg, n=name: self.pose_callback(msg, n), qos_profile)
            self.create_subscription(Pose, f'/{name}/setpointG', lambda msg, n=name: self.setpoint_callback(msg, n), qos_profile)
        
        # Suscribirse al topic /joy para iniciar la grabación
        self.create_subscription(Joy, '/joy', self.joy_callback, qos_profile)
        self.get_logger().info(f"Esperando botón X del control de Xbox para iniciar... Número de robots: {self.n}")
    
    def joy_callback(self, msg):
        """Callback para el joystick. Inicia la grabación si se presiona el botón X."""
        if len(msg.buttons) > 0 and msg.buttons[3] == 1:  # Botón X del control de Xbox
            if not self.recording_started:
                self.start_recording()
    
    def start_recording(self):
        """Inicia la grabación de datos."""
        self.recording_started = True
        self.is_saving_data = True
        self.start_time = self.get_clock().now().seconds_nanoseconds()[0]
        self.get_logger().info("Grabación iniciada...")
        
        # Crear un temporizador para guardar datos periódicamente
        self.timer = self.create_timer(0.001, self.record_data)
        # Crear un temporizador para detener la grabación después de 60 segundos
        self.stop_timer_ros = self.create_timer(60.0, self.stop_recording)
    
    def stop_recording(self):
        """Detiene la grabación y guarda los datos."""
        if self.is_saving_data:
            self.get_logger().info("Deteniendo la grabación...")
            self.is_saving_data = False
            self.recording_started = False
            if hasattr(self, 'timer') and self.timer:
                self.destroy_timer(self.timer)
            self.get_logger().info("Grabación finalizada. Guardando datos...")
            self.save_data()
    
    def record_data(self):
        """Guarda el tiempo actual en la lista de tiempos."""
        if not self.is_saving_data:
            return
        
        current_time = self.get_clock().now().seconds_nanoseconds()[0] - self.start_time
        self.time.append(current_time)
    
    def pose_callback(self, msg, name):
        """Callback para la pose de los robots."""
        if not self.is_saving_data:
            return
        
        self.poses[name]['x'].append(msg.position.x)
        self.poses[name]['y'].append(msg.position.y)
        self.poses[name]['z'].append(msg.position.z)
        
        # Convertir la orientación (quaternion) a ángulo de guiñada (yaw)
        q = msg.orientation
        _, _, yaw = euler_from_quaternion([q.x, q.y, q.z, q.w])
        self.poses[name]['yaw'].append(yaw)
    
    def setpoint_callback(self, msg, name):
        """Callback para los setpoints de los robots."""
        if not self.is_saving_data:
            return
        
        self.setpoints[name]['x'].append(msg.position.x)
        self.setpoints[name]['y'].append(msg.position.y)
        self.setpoints[name]['z'].append(msg.position.z)
        
        # Convertir la orientación (quaternion) a ángulo de guiñada (yaw)
        q = msg.orientation
        _, _, yaw = euler_from_quaternion([q.x, q.y, q.z, q.w])
        self.setpoints[name]['yaw'].append(yaw)
    
    def save_data(self):
        """Guarda los datos en un archivo .mat."""
        if not self.time:
            self.get_logger().warn("No hay datos para guardar.")
            return
        
        # Crear el directorio si no existe
        home_dir = os.path.expanduser("~/Experimentos")
        if not os.path.exists(home_dir):
            try:
                os.makedirs(home_dir)
            except Exception as e:
                self.get_logger().error(f"Error al crear directorio: {e}")
                return
        
        # Preparar los datos para guardar
        data_dict = {'time': self.time, 'poses': self.poses, 'setpoints': self.setpoints}
        mat_path = os.path.join(home_dir, "drones_data.mat")
        
        # Verificar la cantidad de datos
        self.get_logger().info(f"Número de registros en time: {len(self.time)}")
        self.get_logger().info(f"Número de registros en poses: {sum(len(v['x']) for v in self.poses.values())}")
        
        # Guardar los datos en un archivo .mat
        try:
            io.savemat(mat_path, data_dict)
            self.get_logger().info(f"Datos guardados en: {mat_path}")
        except Exception as e:
            self.get_logger().error(f"Error al guardar datos: {e}")
    

def main(args=None):
    rclpy.init(args=args)
    recorder = MultiRobotDataRecorder()
    
    # Registrar la función save_data para que se llame al salir
    atexit.register(recorder.save_data)

    try:
        rclpy.spin(recorder)
    except KeyboardInterrupt:
        recorder.save_data()
    finally:
        recorder.destroy_node()
        rclpy.shutdown()
    

if __name__ == '__main__':
    main()