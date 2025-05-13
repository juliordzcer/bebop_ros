#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile
from geometry_msgs.msg import Twist, Pose
from std_msgs.msg import Bool, Int32, Float32MultiArray
from std_srvs.srv import Empty
from tf_transformations import euler_from_quaternion
import numpy as np
import json
from collections import deque
from enum import IntEnum
from .pid import PID

class FormationController(Node):
    class State(IntEnum):
        IDLE = 0
        AUTOMATIC = 1
        TAKING_OFF = 2
        LANDING = 3
        EMERGENCY_STOP = 4

    def __init__(self):
        super().__init__('bearing_based_controller')

        # Configuration parameters
        self.declare_parameters(namespace='',
            parameters=[
                ('robot_names', '["bebop1", "bebop2", "bebop3", "bebop4"]'),
                ('frequency', 100.0),  # Reducido de 500Hz a 100Hz
                ('takeoff_height', 1.0),
                ('takeoff_threshold', 0.04),
                ('landing_threshold', 0.08),
                ('formation_scale', 1.0),
                ('kp', 1.5),  # Aumentado de 1.5
                ('kv', 0.07),  # Aumentado de 0.5
                ('max_linear_velocity', 2.0),  # Reducido de 2.0
                ('max_angular_velocity', 5.0),  # Reducido de 5.5
                ('trajectory_speed', 0.5)  # Nuevo parámetro
            ])

        # PID parameters for X axis
        self.declare_parameters(namespace='PIDs.X',
            parameters=[
                ('kp', 1.5),
                ('kd', 0.05),
                ('ki', 0.2),
                ('minOutput', -1.0),  # Reducido de -10.0
                ('maxOutput', 1.0),   # Reducido de 10.0
                ('integratorMin', -0.5),  # Reducido de -1.5
                ('integratorMax', 0.5)   # Reducido de 1.5
            ])
        
        # PID parameters for Y axis
        self.declare_parameters(namespace='PIDs.Y',
            parameters=[
                ('kp', 1.2),
                ('kd', 0.04),
                ('ki', 0.12),
                ('minOutput', -1.0),  # Reducido
                ('maxOutput', 1.0),   # Reducido
                ('integratorMin', -0.5),  # Reducido
                ('integratorMax', 0.5)   # Reducido
            ])
        
        # PID parameters for Z axis
        self.declare_parameters(namespace='PIDs.Z',
            parameters=[
                ('kp', 1.2),
                ('kd', 0.04),
                ('ki', 0.12),
                ('minOutput', -1.0),  # Reducido
                ('maxOutput', 1.0),   # Reducido
                ('integratorMin', -0.2),  # Reducido
                ('integratorMax', 0.2)   # Reducido
            ])
        
        # PID parameters for Yaw axis
        self.declare_parameters(namespace='PIDs.Yaw',
            parameters=[
                ('kp', 0.8),
                ('kd', 0.3),
                ('ki', 0.05),
                ('minOutput', -1.0),  # Reducido
                ('maxOutput', 1.0),   # Reducido
                ('integratorMin', -0.5),  # Reducido
                ('integratorMax', 0.5)   # Reducido
            ])

        # Get parameters
        self.robot_names = json.loads(self.get_parameter('robot_names').value)
        self.frequency = self.get_parameter('frequency').value
        self.takeoff_height = self.get_parameter('takeoff_height').value
        self.takeoff_threshold = self.get_parameter('takeoff_threshold').value
        self.landing_threshold = self.get_parameter('landing_threshold').value
        self.formation_scale = self.get_parameter('formation_scale').value
        self.max_linear_velocity = self.get_parameter('max_linear_velocity').value
        self.max_angular_velocity = self.get_parameter('max_angular_velocity').value
        self.trajectory_speed = self.get_parameter('trajectory_speed').value

        """Initialize data structures"""
        self.robot_poses = {name: None for name in self.robot_names}
        self.robot_orientations = {name: None for name in self.robot_names}
        self.robot_velocities = {name: np.zeros(3) for name in self.robot_names}
        self.pose_history = {name: deque(maxlen=5) for name in self.robot_names}
        self.last_pose_time = {name: None for name in self.robot_names}
        self.automatic_start_time = None

        """Setup ROS communication"""
        qos = QoSProfile(depth=10)
        
        # Publishers
        self.cmd_pubs = {
            name: self.create_publisher(Twist, f'/{name}/cmd_vel', qos)
            for name in self.robot_names
        }
        self.enable_pubs = {
            name: self.create_publisher(Bool, f'/{name}/enable', qos)
            for name in self.robot_names
        }

        # Publishers para errores
        self.bearing_error_pub = self.create_publisher(Float32MultiArray, '/bearing_errors', qos)
        self.formation_error_pub = self.create_publisher(Float32MultiArray, '/formation_errors', qos)

        # Subscribers
        self.pose_subs = [
            self.create_subscription(
                Pose, f'/{name}/pose',
                lambda msg, name=name: self.pose_callback(msg, name),
                qos
            ) for name in self.robot_names
        ]
        self.state_sub = self.create_subscription(
            Int32, '/state', self.state_changed, qos
        )

        # Services
        self.takeoff_srv = self.create_service(Empty, 'takeoff', self.takeoff_callback)
        self.land_srv = self.create_service(Empty, 'land', self.land_callback)

        # Initialize PID controllers
        dt = 1.0 / self.frequency
        self.pids = {
            name: {
                'x': self.create_pid('X', dt),
                'y': self.create_pid('Y', dt),
                'z': self.create_pid('Z', dt),
                'yaw': self.create_pid('Yaw', dt)
            }
            for name in self.robot_names
        }

        # Initialize formation
        self.setup_bearings()

        # State
        self.state = self.State.IDLE
        self.enable = False
        self.takeoff_complete = False

        self.control_timer = self.create_timer(dt, self.control_loop)

    def create_pid(self, axis, dt):
        """Create a PID controller for specific axis"""
        params = self.get_parameters([f'PIDs.{axis}.{param}' 
                                    for param in ['kp', 'kd', 'ki', 'minOutput', 'maxOutput', 'integratorMin', 'integratorMax']])
        return PID(
            self,
            params[0].value,  # kp
            params[1].value,  # kd
            params[2].value,  # ki
            params[3].value,  # minOutput
            params[4].value,  # maxOutput
            params[5].value,  # integratorMin
            params[6].value,  # integratorMax
            dt,
            axis.lower()
        )

    def setup_bearings(self):
        """Configura la formación deseada"""
        base_positions = np.array([
            [ 1.0,  1.0, 0.0],   # bebop1 (lider derecho)
            [ 1.0, -1.0, 0.0],  # bebop2 (seguidor derecho)
            [-1.0, -1.0, 0.0], # bebop3 (seguidor izquierdo)
            [-1.0,  1.0, 0.0]   # bebop4 (lider izquierdo)
        ])
        
        # Aplicar escalado
        self.desired_positions = base_positions * self.formation_scale
        
        n = len(self.robot_names)
        self.gijA = np.zeros((n, n, 3))
        for i in range(n):
            for j in range(n):
                if i != j:
                    diff = self.desired_positions[j] - self.desired_positions[i]
                    norm = np.linalg.norm(diff)
                    self.gijA[i, j, :] = diff / norm if norm > 0 else np.zeros(3)
        
        # self.get_logger().info(f"Formación deseada:\n{self.desired_positions}")
        self.get_logger().info(f"Bearings deseados:\n{self.gijA}")

    def calculate_bearing_errors(self, positions):
        """Calcula los errores de bearings entre todos los robots"""
        n = len(self.robot_names)
        bearing_errors = np.zeros((n, n))
        
        for i in range(n):
            for j in range(n):
                if i != j:
                    # Vector relativo actual
                    actual_diff = positions[j] - positions[i]
                    actual_norm = np.linalg.norm(actual_diff)
                    
                    if actual_norm > 0:
                        actual_gij = actual_diff / actual_norm
                    else:
                        actual_gij = np.zeros(3)
                    
                    # Bearing deseado
                    desired_gij = self.gijA[i, j]
                    
                    # Error de bearing (diferencia angular)
                    error = np.arccos(np.clip(np.dot(actual_gij, desired_gij), -1.0, 1.0))
                    bearing_errors[i, j] = error
                    
        return bearing_errors

    def calculate_formation_errors(self, positions):
        """Calcula los errores de posición relativa respecto a la formación deseada"""
        n = len(self.robot_names)
        formation_errors = np.zeros(n)
        
        for i in range(n):
            error_sum = 0.0
            count = 0
            
            for j in range(n):
                if i != j:
                    # Error de posición relativa
                    desired_rel_pos = self.desired_positions[j] - self.desired_positions[i]
                    actual_rel_pos = positions[j] - positions[i]
                    
                    error_sum += np.linalg.norm(actual_rel_pos - desired_rel_pos)
                    count += 1
                    
            if count > 0:
                formation_errors[i] = error_sum / count
                
        return formation_errors

    def ortProj(self, x: np.array):
        """
        Calcula la matriz de proyección ortogonal de un vector
        """
        m = len(x)
        x = x.reshape(m, 1)
        return np.eye(m) - np.dot(x, x.T) / np.linalg.norm(x)**2 if np.linalg.norm(x) != 0 else np.zeros((m, m))

    def state_changed(self, msg):
        """Handle state changes"""
        new_state = msg.data
        
        if new_state == self.State.IDLE and self.state != self.State.IDLE:
            self.get_logger().info("Changing to IDLE state")
            self.state = self.State.IDLE
            self.enable = False
            self.automatic_start_time = None
                
        elif new_state == self.State.AUTOMATIC:
            if self.state == self.State.TAKING_OFF and self.takeoff_complete:
                self.get_logger().info("Changing to AUTOMATIC state")
                self.state = self.State.AUTOMATIC
                self.enable = True
                self.automatic_start_time = self.get_clock().now()
            elif self.state != self.State.TAKING_OFF:
                self.get_logger().warn("Cannot switch to AUTOMATIC without completing takeoff first")
                
        elif new_state == self.State.TAKING_OFF and self.state != self.State.TAKING_OFF:
            self.get_logger().info("Changing to TAKING_OFF state")
            self.state = self.State.TAKING_OFF
            self.takeoff_complete = False
            self.enable = True
                
        elif new_state == self.State.LANDING and self.state != self.State.LANDING:
            self.get_logger().info("Changing to LANDING state")
            self.state = self.State.LANDING
            self.enable = True
                
        elif new_state == self.State.EMERGENCY_STOP and self.state != self.State.EMERGENCY_STOP:
            self.get_logger().warn("EMERGENCY STOP activated!")
            self.state = self.State.EMERGENCY_STOP
            self.enable = False
        
        # Publish enable state to all robots
        self.publish_enable_state()

    def publish_enable_state(self):
        """Publish current enable state to all robots"""
        enable_msg = Bool(data=self.enable)
        for pub in self.enable_pubs.values():
            pub.publish(enable_msg)

    def pose_callback(self, msg, robot_name):
        """Update pose and estimate velocity"""
        current_time = self.get_clock().now()
        new_pose = np.array([msg.position.x, msg.position.y, msg.position.z])
        
        # Update orientation
        orientation_q = msg.orientation
        self.robot_orientations[robot_name] = euler_from_quaternion([
            orientation_q.x, orientation_q.y, orientation_q.z, orientation_q.w
        ])
        
        # Estimate velocity
        if self.last_pose_time[robot_name] is not None:
            dt = (current_time - self.last_pose_time[robot_name]).nanoseconds * 1e-9
            if dt > 0 and self.robot_poses[robot_name] is not None:
                velocity = (new_pose - self.robot_poses[robot_name]) / dt
                self.pose_history[robot_name].append(velocity)
                self.robot_velocities[robot_name] = np.mean(self.pose_history[robot_name], axis=0)
        
        self.robot_poses[robot_name] = new_pose
        self.last_pose_time[robot_name] = current_time

    def takeoff_callback(self, request, response):
        self.get_logger().info('Takeoff requested!')
        if self.state != self.State.TAKING_OFF:
            self.state = self.State.TAKING_OFF
            self.takeoff_complete = False
            self.enable = True
            self.publish_enable_state()
        return response

    def land_callback(self, request, response):
        self.get_logger().info('Landing requested!')
        if self.state != self.State.LANDING:
            self.state = self.State.LANDING
            self.enable = True
            self.publish_enable_state()
        return response

    def control_loop(self):
        if self.state == self.State.TAKING_OFF:
            self.handle_takeoff()
        elif self.state == self.State.LANDING:
            self.handle_landing()
        elif self.state in [self.State.IDLE, self.State.EMERGENCY_STOP]:
            self.handle_stop_state()
        elif self.state == self.State.AUTOMATIC:
            self.automatic_state()

    def handle_takeoff(self):
        """Handle takeoff control logic"""
        for name in self.robot_names:
            if self.robot_poses[name] is None:
                continue
                
            current_z = self.robot_poses[name][2]
            
            if not self.takeoff_complete:
                if current_z < self.takeoff_height - 0.05:  
                    msg = Twist()
                    msg.linear.z = float(self.pids[name]['z'].update(current_z, self.takeoff_height))
                    self.cmd_pubs[name].publish(msg)
                else:
                    self.get_logger().info(f"Takeoff height reached: {current_z:.2f}m")
                    self.takeoff_complete = True
            else:
                msg = Twist()
                msg.linear.z = float(self.pids[name]['z'].update(current_z, self.takeoff_height))
                self.cmd_pubs[name].publish(msg)

    def handle_landing(self):
        """Handle landing control logic"""
        for name in self.robot_names:
            if self.robot_poses[name] is None:
                continue
                
            current_z = self.robot_poses[name][2]
            msg = Twist()
            
            if current_z > self.landing_threshold:
                msg.linear.z = float(self.pids[name]['z'].update(current_z, 0.05)) * 0.5
                self.cmd_pubs[name].publish(msg)
            else:
                self.get_logger().info(f"Landing completed for {name}!")
                self.cmd_pubs[name].publish(Twist())
        
        # Verificar si todos han aterrizado
        if all(self.robot_poses[name][2] <= self.landing_threshold for name in self.robot_names if self.robot_poses[name] is not None):
            self.state = self.State.IDLE
            self.enable = False
            self.publish_enable_state()

    def handle_stop_state(self):
        """Handle IDLE and EMERGENCY states"""
        self.enable = False
        self.publish_enable_state()
        for pub in self.cmd_pubs.values():
            pub.publish(Twist())

    def automatic_state(self):
        """Implementa el control de formación con trayectorias lineales"""
        try:
            # Verificar datos faltantes
            if any(pos is None for pos in self.robot_poses.values()):
                self.get_logger().warn("Faltan datos de pose", throttle_duration_sec=1.0)
                return

            # Obtener el tiempo transcurrido
            current_time = self.get_clock().now()
            if self.automatic_start_time is None:
                self.automatic_start_time = current_time
            elapsed_time = (current_time - self.automatic_start_time).nanoseconds * 1e-9

            # Parámetros de control
            kp = self.get_parameter('kp').value
            kv = self.get_parameter('kv').value
            max_vel = self.get_parameter('max_linear_velocity').value
            trajectory_speed = self.get_parameter('trajectory_speed').value

            # Obtener posiciones y velocidades actuales
            positions = np.array([self.robot_poses[name] for name in self.robot_names])
            velocities = np.array([self.robot_velocities[name] for name in self.robot_names])
            
            # Calcular errores
            bearing_errors = self.calculate_bearing_errors(positions)
            formation_errors = self.calculate_formation_errors(positions)
            
            # Publicar errores de bearings
            bearing_msg = Float32MultiArray()
            bearing_msg.data = bearing_errors.flatten().tolist()
            self.bearing_error_pub.publish(bearing_msg)
            
            # Publicar errores de formación
            formation_msg = Float32MultiArray()
            formation_msg.data = formation_errors.tolist()
            self.formation_error_pub.publish(formation_msg)



            n = len(self.robot_names)
            commands = np.zeros((n, 3))

            for i, name in enumerate(self.robot_names):
                current_pos = self.robot_poses[name]
                if current_pos is None:
                    continue
                
                if name == 'bebop1':  # Líder 1
                    # Posición deseada con offset de formación
                    x_d = min(elapsed_time * trajectory_speed, 10.0) + self.desired_positions[i][0]
                    y_d = self.desired_positions[i][1]
                    z_d = self.takeoff_height + self.desired_positions[i][2]

                    commands[i, 0] = float(np.clip(
                        self.pids[name]['x'].update(current_pos[0], x_d),
                        -max_vel, max_vel
                    ))
                    commands[i, 1] = float(np.clip(
                        self.pids[name]['y'].update(current_pos[1], y_d),
                        -max_vel, max_vel
                    ))
                    commands[i, 2] = float(np.clip(
                        self.pids[name]['z'].update(current_pos[2], z_d),
                        -max_vel/2, max_vel/2
                    ))

                elif name == 'bebop2':  # Líder 2
                    # Trayectoria lineal para bebop2: se mueve en línea recta en X con offset en Y
                    current_pos = self.robot_poses[name]
                    if current_pos is None:
                        continue
                    
                    # Punto deseado se mueve linealmente con el tiempo
                    x_d = min(elapsed_time * trajectory_speed, 10.0) + self.desired_positions[i][0]  # Máximo 10 metros
                    y_d = self.desired_positions[i][1]  # Offset fijo en Y
                    z_d = 1.0 + self.desired_positions[i][2]  # Altura fija
                    commands[i, 0] = float(np.clip(
                        self.pids[name]['x'].update(current_pos[0], x_d),
                        -max_vel, max_vel
                    ))
                    commands[i, 1] = float(np.clip(
                        self.pids[name]['y'].update(current_pos[1], y_d),
                        -max_vel, max_vel
                    ))
                    commands[i, 2] = float(np.clip(
                        self.pids[name]['z'].update(current_pos[2], z_d),
                        -max_vel, max_vel
                    ))

                else:  # Seguidores
                    sum_P = np.zeros((3, 3))
                    sum_control = np.zeros(3)
                    valid_bearings = 0
                    dt = 1.0 / self.frequency

                    # Inicializar i_e si no existe (probablemente como atributo de clase)
                    if not hasattr(self, 'i_e'):
                        self.i_e = np.zeros(3)  # Para cada agente, o usa un diccionario/array
                    
                    for j in range(n):
                        if i != j and np.any(self.gijA[i, j]):
                            gij = self.gijA[i, j]
                            Pgij = self.ortProj(gij)
                            
                            # Calcular error relativo
                            desired_relative_pos = self.desired_positions[j] - self.desired_positions[i]
                            actual_relative_pos = positions[j] - positions[i]
                            e = actual_relative_pos - desired_relative_pos
                            
                            self.i_e = self.i_e + e * dt
                            
                            control = (kp * e + kv * self.i_e)  
                            
                            sum_P += Pgij
                            sum_control += Pgij @ control

                            valid_bearings += 1
                    
                    if valid_bearings > 0:
                        commands[i] = np.linalg.pinv(sum_P) @ sum_control
                        # commands[i] =  sum_control
                    else:
                        commands[i] = np.zeros(3)

                # else:  # Seguidores
                #     sum_P = np.zeros((3, 3))
                #     sum_control = np.zeros(3)
                #     valid_bearings = 0
                    
                #     for j in range(n):
                #         if i != j and np.any(self.gijA[i, j]):
                #             gij = self.gijA[i, j]
                #             Pgij = self.ortProj(gij)
                            
                #             # Calcular error relativo a la formación deseada
                #             desired_relative_pos = self.desired_positions[j] - self.desired_positions[i]
                #             actual_relative_pos = positions[j] - positions[i]
                            
                #             pos_error = actual_relative_pos - desired_relative_pos
                #             vel_error = velocities[i] - velocities[j]

                #             control_term = kp * pos_error + kv * vel_error

                #             bearing_control = np.dot(Pgij, control_term)
                            
                #             sum_P += Pgij
                #             sum_control += bearing_control
                #             valid_bearings += 1
                    
                #     # if valid_bearings > 0 and np.linalg.matrix_rank(sum_P) == 3:
                #     #     commands[i] = -np.linalg.pinv(sum_P) @ sum_control / valid_bearings
                #     if valid_bearings > 0 and np.linalg.matrix_rank(sum_P) == 3:
                #         commands[i] = (np.linalg.pinv(sum_P) @ sum_control) 
                #     else:
                #         commands[i] = np.zeros(3)

            # Publicar comandos con suavizado
            for i, name in enumerate(self.robot_names):
                if self.robot_poses[name] is None:
                    continue
                    
                cmd = Twist()
                cmd.linear.x = float(np.clip(commands[i, 0], -max_vel, max_vel))
                cmd.linear.y = float(np.clip(commands[i, 1], -max_vel, max_vel))
                cmd.linear.z = float(np.clip(commands[i, 2], -max_vel/2, max_vel/2))
                cmd.angular.z = 0.0
                self.cmd_pubs[name].publish(cmd)


            # Dentro de automatic_state, después de calcular los errores:
            if int(elapsed_time * 10) % 10 == 0:  # Cada 2 segundos
                self.get_logger().info(f"Errores de bearings promedio: {np.rad2deg(np.mean(bearing_errors)):.4f} deg")
                self.get_logger().info(f"Errores de formación promedio: {np.mean(formation_errors)*100.0:.4f} cm")
            # # Log de diagnóstico cada 2 segundos
            # if int(elapsed_time * 10) % 20 == 0:
            #     self.get_logger().info(f"\nTiempo: {elapsed_time:.1f}s")
            #     self.get_logger().info("Posiciones actuales:\n" + 
            #         "\n".join(f"{name}: {self.robot_poses[name]}" for name in self.robot_names))
            #     self.get_logger().info("Comandos calculados:\n" + 
            #         "\n".join(f"{name}: {commands[i]}" for i, name in enumerate(self.robot_names)))

            # # Dentro de automatic_state, después de calcular los errores:
            # if int(elapsed_time * 10) % 20 == 0:  # Cada 2 segundos
            #     self.get_logger().info(f"Errores de bearings promedio: {np.mean(bearing_errors):.4f} rad")
            #     self.get_logger().info(f"Errores de formación promedio: {np.mean(formation_errors):.4f} m")

        except Exception as e:
            self.get_logger().error(f"Error en control: {str(e)}", throttle_duration_sec=1.0)
            self.enable = False
            self.publish_enable_state()

def main(args=None):
    rclpy.init(args=args)
    controller = FormationController()
    
    try:
        rclpy.spin(controller)
    except KeyboardInterrupt:
        controller.get_logger().info("Shutting down controller...")
    finally:
        controller.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()
