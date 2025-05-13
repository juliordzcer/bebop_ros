#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, QoSReliabilityPolicy, QoSHistoryPolicy, DurabilityPolicy
from geometry_msgs.msg import Twist, Pose
from std_msgs.msg import Bool
from std_srvs.srv import Empty
import numpy as np
import json
from collections import deque
from rclpy.time import Time
import os
import scipy.special

class PID:
    def __init__(self, node, kp, kd, ki, min_output, max_output, integrator_min, integrator_max, dt, axis):
        self.node = node
        self.kp = kp
        self.kd = kd
        self.ki = ki
        self.min_output = min_output
        self.max_output = max_output
        self.integrator_min = integrator_min
        self.integrator_max = integrator_max
        self.dt = dt
        self.axis = axis
        
        self.prev_error = 0.0
        self.integral = 0.0
        
    def update(self, current_value, target_value):
        error = target_value - current_value
        
        p_term = self.kp * error
        derivative = (error - self.prev_error) / self.dt
        d_term = self.kd * derivative
        
        self.integral += error * self.dt
        self.integral = np.clip(self.integral, self.integrator_min, self.integrator_max)
        i_term = self.ki * self.integral
        
        output = p_term + i_term + d_term
        output = np.clip(output, self.min_output, self.max_output)
        
        self.prev_error = error
        return output

class BearingBasedController(Node):
    
    class State:
        IDLE = 0
        AUTOMATIC = 1
        TAKING_OFF = 2
        LANDING = 3
        EMERGENCY_STOP = 4

    def __init__(self):
        super().__init__('bearing_based_controller')

        # Configuración QoS
        self.qos_profile = QoSProfile(
            reliability=QoSReliabilityPolicy.RELIABLE,
            history=QoSHistoryPolicy.KEEP_LAST,
            depth=10,
            durability=DurabilityPolicy.VOLATILE
        )

        # Declaración de parámetros
        self.declare_parameters(namespace='',
            parameters=[
                ('robot_names', '["bebop1", "bebop2", "bebop3", "bebop4"]'),
                ('kp', 3.0), ('kv', 3.0), ('frequency', 100.0),
                ('takeoff_height', 1.0),  # Altura de 1 metro
                ('landing_threshold', 0.1),
                ('velocity_window_size', 5), ('max_linear_velocity', 1.0),
                ('formation_scale', 1.0), ('leader_speed', 0.5),
                ('PIDs.Z.kp', 1.2), ('PIDs.Z.kd', 0.04), ('PIDs.Z.ki', 0.12),
                ('PIDs.Z.minOutput', -1.5), ('PIDs.Z.maxOutput', 1.5),
                ('PIDs.Z.integratorMin', -0.5), ('PIDs.Z.integratorMax', 0.5),
                ('trajectory_file1', 'lider1_ace.dat'),
                ('trajectory_file2', 'lider2_ace.dat')
            ])

        # Cargar trayectorias
        self.leader1_acc = self.load_trajectory(self.get_parameter('trajectory_file1').value)
        self.leader2_acc = self.load_trajectory(self.get_parameter('trajectory_file2').value)
        self.trajectory_index = 0
        self.trajectory_time = 10.0
        self.dt_trajectory = 0.01

        # Inicialización de variables
        self.robot_names = json.loads(self.get_parameter('robot_names').value)
        self.initialize_structures()
        self.setup_bearings()
        self.initialize_pids()
        self.setup_communication()
        self.initialize_state()

        self.get_logger().info("Controlador inicializado correctamente")

    def load_trajectory(self, filename):
        """Carga los datos de aceleración desde archivo"""
        if not os.path.exists(filename):
            self.get_logger().error(f"Archivo de trayectoria no encontrado: {filename}")
            return np.zeros((1, 3))
        
        try:
            data = np.loadtxt(filename, skiprows=1)
            return data[:, 1:]  # Saltar la columna de tiempo
        except Exception as e:
            self.get_logger().error(f"Error cargando trayectoria: {str(e)}")
            return np.zeros((1, 3))

    def initialize_structures(self):
        """Inicializa estructuras de datos"""
        self.robot_poses = {name: None for name in self.robot_names}
        self.robot_velocities = {name: np.zeros(3) for name in self.robot_names}
        self.pose_history = {name: deque(maxlen=self.get_parameter('velocity_window_size').value) 
                           for name in self.robot_names}
        self.last_pose_time = {name: None for name in self.robot_names}

    def setup_bearings(self):
        """Configura la formación deseada"""
        scale = self.get_parameter('formation_scale').value
        self.desired_positions = np.array([
            [1.0, 1.0, 0.0], [-1.0, 1.0, 0.0],
            [1.0, -1.0, 0.0], [-1.0, -1.0, 0.0]
        ]) * scale
        
        n = len(self.robot_names)
        self.gijA = np.zeros((n, n, 3))
        for i in range(n):
            for j in range(n):
                if i != j:
                    diff = self.desired_positions[j] - self.desired_positions[i]
                    norm = np.linalg.norm(diff)
                    self.gijA[i, j, :] = diff / norm if norm > 0 else np.zeros(3)

    def initialize_pids(self):
        """Inicializa los controladores PID"""
        self.pid_z = {
            name: PID(self,
                   self.get_parameter('PIDs.Z.kp').value,
                   self.get_parameter('PIDs.Z.kd').value,
                   self.get_parameter('PIDs.Z.ki').value,
                   self.get_parameter('PIDs.Z.minOutput').value,
                   self.get_parameter('PIDs.Z.maxOutput').value,
                   self.get_parameter('PIDs.Z.integratorMin').value,
                   self.get_parameter('PIDs.Z.integratorMax').value,
                   1.0 / self.get_parameter('frequency').value,
                   'z')
            for name in self.robot_names
        }

    def setup_communication(self):
        """Configura publicadores, suscriptores y servicios"""
        self.cmd_pubs = {
            name: self.create_publisher(Twist, f'/{name}_cmd', self.qos_profile)
            for name in self.robot_names
        }

        self.pose_subs = [
            self.create_subscription(
                Pose, f'/{name}/pose',
                lambda msg, robot_name=name: self.pose_callback(msg, robot_name),
                self.qos_profile
            ) for name in self.robot_names
        ]

        self.takeoff_srv = self.create_service(Empty, 'takeoff', self.takeoff_callback)
        self.land_srv = self.create_service(Empty, 'land', self.land_callback)
        self.emergency_stop_srv = self.create_service(Empty, 'emergency_stop', self.emergency_stop_callback)

    def initialize_state(self):
        """Inicializa el estado del controlador"""
        self.state = self.State.IDLE
        self.enable = False
        self.takeoff_complete = False
        self.emergency_stop = False
        self.control_timer = self.create_timer(1.0 / self.get_parameter('frequency').value, self.control_loop)
        self.trajectory_timer = self.create_timer(self.dt_trajectory, self.update_trajectory)

    def update_trajectory(self):
        """Actualiza el índice de la trayectoria para los líderes"""
        if self.state == self.State.AUTOMATIC and self.enable:
            self.trajectory_time += self.dt_trajectory
            self.trajectory_index = int(self.trajectory_time / self.dt_trajectory)
            
            if self.trajectory_index >= len(self.leader1_acc):
                self.trajectory_index = 0
                self.trajectory_time = 0.0

    def takeoff_callback(self, request, response):
        """Maneja la solicitud de despegue"""
        if self.state != self.State.TAKING_OFF and not self.emergency_stop:
            self.get_logger().info("Iniciando secuencia de despegue")
            self.state = self.State.TAKING_OFF
            self.takeoff_complete = False
            self.enable = True
        return response

    def land_callback(self, request, response):
        """Maneja la solicitud de aterrizaje"""
        if self.state != self.State.LANDING and not self.emergency_stop:
            self.get_logger().info("Iniciando secuencia de aterrizaje")
            self.state = self.State.LANDING
            self.enable = True
        return response

    def emergency_stop_callback(self, request, response):
        """Maneja la solicitud de parada de emergencia"""
        self.get_logger().warn("¡PARADA DE EMERGENCIA ACTIVADA!")
        self.state = self.State.EMERGENCY_STOP
        self.emergency_stop = True
        self.enable = False
        self.stop_all_robots()
        return response

    def pose_callback(self, msg, robot_name):
        """Actualiza pose y estima velocidad"""
        current_time = self.get_clock().now()
        new_pose = np.array([msg.position.x, msg.position.y, msg.position.z])
        
        if self.last_pose_time[robot_name] is not None:
            dt = (current_time - self.last_pose_time[robot_name]).nanoseconds * 1e-9
            if dt > 1e-6 and self.robot_poses[robot_name] is not None:
                velocity = (new_pose - self.robot_poses[robot_name]) / dt
                self.pose_history[robot_name].append(velocity)
                self.robot_velocities[robot_name] = np.mean(self.pose_history[robot_name], axis=0) 
        
        self.robot_poses[robot_name] = new_pose
        self.last_pose_time[robot_name] = current_time

    def control_loop(self):
        """Bucle principal de control"""
        if self.emergency_stop:
            self.stop_all_robots()
            return
            
        if self.state == self.State.TAKING_OFF:
            self.handle_takeoff()
        elif self.state == self.State.LANDING:
            self.handle_landing()
        elif self.state == self.State.AUTOMATIC and self.enable:
            self.bearing_control()
        else:
            self.stop_all_robots()

    def handle_takeoff(self):
        """Controla la secuencia de despegue"""
        all_at_height = True
        takeoff_height = self.get_parameter('takeoff_height').value
        
        for name in self.robot_names:
            if self.robot_poses[name] is None:
                all_at_height = False
                continue
                
            current_z = self.robot_poses[name][2]
            cmd = Twist()
            
            if not self.takeoff_complete:
                if current_z < takeoff_height - 0.05:
                    all_at_height = False
                    cmd.linear.z = float(self.pid_z[name].update(current_z, takeoff_height))
                else:
                    cmd.linear.z = 0.0
                
                self.cmd_pubs[name].publish(cmd)
            
        if all_at_height and not self.takeoff_complete:
            self.get_logger().info("Altura de despegue alcanzada")
            self.takeoff_complete = True
            self.state = self.State.AUTOMATIC

    def handle_landing(self):
        """Controla la secuencia de aterrizaje"""
        all_landed = True
        landing_threshold = self.get_parameter('landing_threshold').value
        
        for name in self.robot_names:
            if self.robot_poses[name] is None:
                all_landed = False
                continue
                
            current_z = self.robot_poses[name][2]
            cmd = Twist()
            
            if current_z > landing_threshold:
                all_landed = False
                cmd.linear.z = float(self.pid_z[name].update(current_z, 0.0))
            else:
                cmd.linear.z = 0.0
            
            self.cmd_pubs[name].publish(cmd)
        
        if all_landed:
            self.get_logger().info("Aterrizaje completado")
            self.state = self.State.IDLE
            self.enable = False

    def bearing_control(self):
        """Implementa el control de formación con trayectorias de Bézier"""
        try:
            if None in self.robot_poses.values():
                self.get_logger().warn("Faltan datos de pose", throttle_duration_sec=1.0)
                return
            
            kp = self.get_parameter('kp').value
            kv = self.get_parameter('kv').value
            max_vel = self.get_parameter('max_linear_velocity').value
            takeoff_height = self.get_parameter('takeoff_height').value
            
            positions = np.array([self.robot_poses[name] for name in self.robot_names])
            velocities = np.array([self.robot_velocities[name] for name in self.robot_names])
            
            n = len(self.robot_names)
            commands = np.zeros((n, 3))
            
            for i, name in enumerate(self.robot_names):
                if name == 'bebop1' and self.trajectory_index < len(self.leader1_acc):
                    commands[i] = self.leader1_acc[self.trajectory_index] * 0.1
                elif name == 'bebop2' and self.trajectory_index < len(self.leader2_acc):
                    commands[i] = self.leader2_acc[self.trajectory_index] * 0.1
                elif i > 1:
                    sum_P = np.zeros((3, 3))
                    sum_control = np.zeros(3)
                    
                    for j in range(n):
                        if i != j and np.any(self.gijA[i, j]):
                            gij = self.gijA[i, j]
                            Pgij = np.eye(3) - np.outer(gij, gij) / np.dot(gij, gij)
                            sum_P += Pgij
                            
                            pos_diff = positions[i] - positions[j]
                            vel_diff = velocities[i] - velocities[j]
                            control_term = kp * pos_diff + kv * vel_diff
                            
                            sum_control += np.linalg.pinv(sum_P) @ Pgij @ control_term
                    
                    commands[i] = -sum_control
            
            # Publicar comandos
            for i, name in enumerate(self.robot_names):
                cmd = Twist()
                cmd.linear.x = float(np.clip(commands[i, 0], -max_vel, max_vel))
                cmd.linear.y = float(np.clip(commands[i, 1], -max_vel, max_vel))
                cmd.linear.z = float(self.pid_z[name].update(self.robot_poses[name][2], takeoff_height))
                self.cmd_pubs[name].publish(cmd)
                
        except Exception as e:
            self.get_logger().error(f"Error en control: {str(e)}", throttle_duration_sec=1.0)
            self.enable = False
            self.stop_all_robots()

    def stop_all_robots(self):
        """Detiene todos los robots"""
        for name in self.robot_names:
            self.cmd_pubs[name].publish(Twist())

def main(args=None):
    rclpy.init(args=args)
    controller = BearingBasedController()
    
    try:
        rclpy.spin(controller)
    except KeyboardInterrupt:
        controller.get_logger().info("Apagando controlador...")
    except Exception as e:
        controller.get_logger().error(f"Error fatal: {str(e)}")
    finally:
        controller.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()