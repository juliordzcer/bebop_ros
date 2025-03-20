#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
from rclpy.parameter import Parameter
from geometry_msgs.msg import Twist, Pose
from std_srvs.srv import Empty
from tf2_ros import Buffer, TransformListener
from tf_transformations import euler_from_quaternion, quaternion_from_euler
from rclpy.qos import QoSProfile
from nav_msgs.msg import Odometry
from std_msgs.msg import Bool
from sensor_msgs.msg import Joy
from .sta import STA

class JoystickButtons:
    A = 0
    B = 1
    Y = 3

class Controller(Node):
    class State:
        IDLE = 0
        AUTOMATIC = 1
        TAKING_OFF = 2
        LANDING = 3
        EMERGENCY_STOP = 4

    def __init__(self):
        super().__init__('controller')
        
        # Declaración de parámetros
        self.declare_parameter('frequency',50.0)
        self.declare_parameter('robot_name', 'bebop3')
        self.declare_parameter('goal_name', 'goal')
        self.declare_parameter('takeoff_threshold', 0.04)
        self.declare_parameter('landing_threshold', 0.08)
        
        # Obtención de parámetros
        self.frequency = self.get_parameter('frequency').value
        self.robot_name = self.get_parameter('robot_name').value.strip()
        self.goal_name = self.get_parameter("goal_name").value.strip()
        self.takeoff_threshold = self.get_parameter('takeoff_threshold').value
        self.landing_threshold = self.get_parameter('landing_threshold').value

        if not self.robot_name:
            self.get_logger().warn('El parámetro "robot_name" está vacío. Se usará "bebop3" por defecto.')
            self.robot_name = 'bebop3'

        self.get_logger().info(f"Robot Name: {self.robot_name}")

        # Definición de tópicos dinámicos
        qos_profile = QoSProfile(depth=10)
        self.cmd_pub = self.create_publisher(Twist, f"/{self.robot_name}/cmd_vel", qos_profile)
        self.cmd_des = self.create_publisher(Pose, f"/{self.robot_name}/setpointG", qos_profile)
        self.cmd_enable = self.create_publisher(Bool, f"/{self.robot_name}/enable", qos_profile)
        self.goal_sub = self.create_subscription(Pose, f"/{self.goal_name}", self.goal_changed, qos_profile)
        self.pos_sub = self.create_subscription(Pose, f"/{self.robot_name}/pose", self.pos_changed, qos_profile)
        self.joy_sub = self.create_subscription(Joy, 'joy', self.joy_callback, qos_profile)
        
        # Servicios
        self.takeoff_srv = self.create_service(Empty, 'takeoff', self.takeoff_callback)
        self.land_srv = self.create_service(Empty, 'land', self.land_callback)
        
        # TF Listener
        self.tf_buffer = Buffer()
        self.tf_listener = TransformListener(self.tf_buffer, self)
        
        # stas
        self.sta_x = self.create_sta('X')
        self.sta_y = self.create_sta('Y')
        self.sta_z = self.create_sta('Z')
        self.sta_yaw = self.create_sta('Yaw')
        
        self.state = self.State.IDLE
        self.goal = Pose()
        self.current_pose = Pose()
        self.enable = False
        
        timer_period = 1.0 / self.frequency
        self.timer = self.create_timer(timer_period, self.iteration)

    def joy_callback(self, msg):
        if msg.buttons[JoystickButtons.A] == 1:
            self.get_logger().info("Landing initiated")
            self.state = self.State.LANDING
        elif msg.buttons[JoystickButtons.B] == 1:
            self.get_logger().info("Emergency stop activated!")
            self.state = self.State.EMERGENCY_STOP
        elif msg.buttons[JoystickButtons.Y] == 1:
            self.get_logger().info("Takeoff initiated")
            self.state = self.State.TAKING_OFF

    def create_sta(self, axis):
        prefix = f'STAs.{axis}.'
        k0 = self.get_param_or(prefix + 'k0', 0.0)
        k1 = self.get_param_or(prefix + 'k1', 0.0)
        k2 = self.get_param_or(prefix + 'k2', 0.0)
        min_output = self.get_param_or(prefix + 'minOutput', -1.0)
        max_output = self.get_param_or(prefix + 'maxOutput', 1.0)
        integrator_min = self.get_param_or(prefix + 'integratorMin', -0.5)
        integrator_max = self.get_param_or(prefix + 'integratorMax', 0.5)
        dt = 1.0 / self.frequency
        
        return STA(self, k0, k1, k2, min_output, max_output, integrator_min, integrator_max, dt, axis.lower())

    def get_param_or(self, name, default):
        self.declare_parameter(name, default)
        return self.get_parameter(name).get_parameter_value().double_value

    def goal_changed(self, msg):
        self.goal = msg

    def pos_changed(self, msg):
        self.current_pose = msg

    def takeoff_callback(self, request, response):
        self.get_logger().info('Takeoff requested!')
        self.state = self.State.TAKING_OFF
        return response

    def land_callback(self, request, response):
        self.get_logger().info('Landing requested!')
        self.state = self.State.LANDING
        return response

    def iteration(self):
        if self.state == self.State.TAKING_OFF:
            if self.current_pose.position.z > self.takeoff_threshold:
                self.state = self.State.AUTOMATIC
                self.enable = True
            else:
                self.enable = False
            self.cmd_enable.publish(Bool(data=self.enable))

        elif self.state == self.State.LANDING:
            msg = Twist()
            msg.linear.z = float(self.sta_z.update(self.current_pose.position.z, 0.0)) * 0.1
            self.cmd_pub.publish(msg)
            if self.current_pose.position.z <= self.landing_threshold:
                self.state = self.State.IDLE
                self.enable = False
                self.cmd_enable.publish(Bool(data=self.enable))
                self.cmd_pub.publish(Twist())

        elif self.state == self.State.AUTOMATIC:

            euler_angles_c = euler_from_quaternion([
                self.current_pose.orientation.x,
                self.current_pose.orientation.y,
                self.current_pose.orientation.z,
                self.current_pose.orientation.w
            ])

            euler_angles_d = euler_from_quaternion([
                self.goal.orientation.x,
                self.goal.orientation.y,
                self.goal.orientation.z,
                self.goal.orientation.w
            ])

            x_d, y_d, z_d, yaw_d = self.goal.position.x, self.goal.position.y, self.goal.position.z, euler_angles_d[2]


            msg = Twist()
            msg.linear.x = float(self.sta_x.update(self.current_pose.position.x, x_d))*3.0
            msg.linear.y = float(self.sta_y.update(self.current_pose.position.y, y_d))*5.0
            msg.linear.z = float(self.sta_z.update(self.current_pose.position.z, z_d))*4.0
            msg.angular.z = 0.0#float(self.sta_z.update(euler_angles_c[2], yaw_d))
            self.cmd_pub.publish(msg)


            msg_goal = Pose()
            msg_goal.position.x = x_d
            msg_goal.position.y = y_d
            msg_goal.position.z = z_d

            # Convertir ángulos de Euler a cuaternión
            quat = quaternion_from_euler(0, 0, yaw_d)

            # Asignar el cuaternión al mensaje
            msg_goal.orientation.x = quat[0]
            msg_goal.orientation.y = quat[1]
            msg_goal.orientation.z = quat[2]
            msg_goal.orientation.w = quat[3]

            # Publicar el mensaje
            self.cmd_des.publish(msg_goal)

        elif self.state in [self.State.IDLE, self.State.EMERGENCY_STOP]:
            self.cmd_pub.publish(Twist())
            self.enable = False
            self.cmd_enable.publish(Bool(data=self.enable))


def main(args=None):
    rclpy.init(args=args)
    controller = Controller()
    rclpy.spin(controller)
    controller.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()