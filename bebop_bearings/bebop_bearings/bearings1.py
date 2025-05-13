#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile
from geometry_msgs.msg import Twist, Pose
from std_msgs.msg import Bool, Int32
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
                ('frequency', 500.0),
                ('takeoff_height', 1.0),
                ('landing_threshold', 0.01),  # 5 cm
            ])

        # PID parameters for Z axis only
        self.declare_parameter('PIDs.Z.kp', 1.2)
        self.declare_parameter('PIDs.Z.kd', 0.04)
        self.declare_parameter('PIDs.Z.ki', 0.12)
        self.declare_parameter('PIDs.Z.minOutput', -1.5)
        self.declare_parameter('PIDs.Z.maxOutput', 1.5)
        self.declare_parameter('PIDs.Z.integratorMin', -0.5)
        self.declare_parameter('PIDs.Z.integratorMax', 0.5)



        self.declare_parameter('takeoff_threshold', 0.04)
        self.declare_parameter('landing_threshold', 0.08)
        self.declare_parameter('takeoff_height', 1.0) 

        self.takeoff_threshold = self.get_parameter('takeoff_threshold').value
        self.landing_threshold = self.get_parameter('landing_threshold').value
        self.takeoff_height = self.get_parameter('takeoff_height').value



        # Initialization
        self.robot_names = json.loads(self.get_parameter('robot_names').value)

        """Initialize data structures"""
        self.robot_poses = {name: None for name in self.robot_names}
        self.robot_orientations = {name: None for name in self.robot_names}
        self.robot_velocities = {name: np.zeros(3) for name in self.robot_names}
        self.pose_history = {name: deque(maxlen=5) for name in self.robot_names}
        self.last_pose_time = {name: None for name in self.robot_names}

        """Setup ROS communication"""
        qos = QoSProfile(depth=10)
        
        # Publishers
        self.cmd_pubs = {
            name: self.create_publisher(Twist, f'/{name}/cmd_vel', qos)
            for name in self.robot_names
        }
        
        # Nuevo: Publisher individual para enable de cada dron
        self.enable_pubs = {
            name: self.create_publisher(Bool, f'/{name}/enable', qos)
            for name in self.robot_names
        }

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
        # self.emergency_srv = self.create_service(Empty, 'emergency_stop', self.emergency_callback)


        # Inicialize PID
        """Initialize PID controllers"""
        dt = 1.0 / self.get_parameter('frequency').value
        self.pids = {}
        
        for name in self.robot_names:
            self.pids[name] = {
                'z': self.create_pid('Z', dt)
            }

        # State
        self.state = self.State.IDLE
        self.enable = False
        self.takeoff_complete = False

        # Asegurar que los motores comiencen APAGADOS
        # self.publish_enable_to_all(self.enable)
        # self.stop_all_robots()
        
        self.control_timer = self.create_timer(
            1.0/self.get_parameter('frequency').value, 
            self.control_loop
        )

    def create_pid(self, axis, dt):
        """Create a PID controller for specific axis"""
        prefix = f'PIDs.{axis}.'
        return PID(
            self,
            self.get_parameter(prefix + 'kp').value,
            self.get_parameter(prefix + 'kd').value,
            self.get_parameter(prefix + 'ki').value,
            self.get_parameter(prefix + 'minOutput').value,
            self.get_parameter(prefix + 'maxOutput').value,
            self.get_parameter(prefix + 'integratorMin').value,
            self.get_parameter(prefix + 'integratorMax').value,
            dt,
            axis.lower()
        )











    def state_changed(self, msg):
        """Maneja cambios en el estado desde el tópico /state"""
        new_state = msg.data
        
        if new_state == self.State.IDLE:
            if self.state != self.State.IDLE:
                self.get_logger().info("Cambiando a estado IDLE")
                self.state = self.State.IDLE
                self.enable = False
                
        elif new_state == self.State.AUTOMATIC:
            if self.state == self.State.TAKING_OFF and self.takeoff_complete:
                self.get_logger().info("Cambiando a estado AUTOMATIC")
                self.state = self.State.AUTOMATIC
                self.enable = True
            elif self.state != self.State.TAKING_OFF:
                self.get_logger().warn("No se puede cambiar a AUTOMATIC sin completar el despegue primero")
                
        elif new_state == self.State.TAKING_OFF:
            if self.state != self.State.TAKING_OFF:
                self.get_logger().info("Cambiando a estado TAKING_OFF")
                self.state = self.State.TAKING_OFF
                self.takeoff_complete = False
                self.enable = True
                
        elif new_state == self.State.LANDING:
            if self.state != self.State.LANDING:
                self.get_logger().info("Cambiando a estado LANDING")
                self.state = self.State.LANDING
                self.enable = True
                
        elif new_state == self.State.EMERGENCY_STOP:
            if self.state != self.State.EMERGENCY_STOP:
                self.get_logger().warn("¡EMERGENCY STOP activado!")
                self.state = self.State.EMERGENCY_STOP
                self.enable = False
        
        # Publicar estado de enable
        for name in self.robot_names:
            self.enable_pubs[name].publish(Bool(data=self.enable))



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
            for name in self.robot_names:
                self.enable_pubs[name].publish(Bool(data=self.enable))
        return response

    def land_callback(self, request, response):
        self.get_logger().info('Landing requested!')
        if self.state != self.State.LANDING:
            self.state = self.State.LANDING
            self.enable = True
            for name in self.robot_names:
                self.enable_pubs[name].publish(Bool(data=self.enable))
        return response


    def control_loop(self):
        if self.state == self.State.TAKING_OFF:
            all_at_heitght = True
            takeoff_height = self.get_parameter('takeoff_height').value

            for name in self.robot_names:
                current_z = self.robot_poses[name][2]
                
                if not self.takeoff_complete:
                    if current_z < self.takeoff_height - 0.05:  
                        msg = Twist()
                        msg.linear.z = float(self.pids[name]['z'].update(current_z, takeoff_height))
                        self.cmd_pubs[name].publish(msg)
                    else:
                        # Altura alcanzada
                        self.get_logger().info(f"Altura de despegue alcanzada: {current_z:.2f}m")
                        self.takeoff_complete = True
                else:
                    # Mantener la altura con PID
                    msg = Twist()
                    msg.linear.z = float(self.pids[name]['z'].update(current_z, takeoff_height))
                    self.cmd_pubs[name].publish(msg)

        elif self.state == self.State.LANDING:
            for name in self.robot_names:
                current_z = self.robot_poses[name][2]
                msg = Twist()
                
                if current_z > self.landing_threshold:
                    # Descender controladamente
                    msg.linear.z = float(self.pids[name]['z'].update(current_z, 0.5)) * 0.5  # Reducir velocidad
                    self.cmd_pubs[name].publish(msg)

                else:
                    # Aterrizaje completado
                    self.get_logger().info("¡Aterrizaje completado!")
                    self.state = self.State.IDLE
                    self.enable = False
                    self.enable_pubs[name].publish(Bool(data=self.enable))
                    self.cmd_pubs[name].publish(Twist())
          
        elif self.state in [self.State.IDLE, self.State.EMERGENCY_STOP]:
            
            self.enable = False
            for name in self.robot_names:
                self.cmd_pubs[name].publish(Twist())
                self.enable_pubs[name].publish(Bool(data=self.enable))

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




