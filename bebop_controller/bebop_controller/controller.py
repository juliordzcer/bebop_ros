#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
from rclpy.parameter import Parameter
from geometry_msgs.msg import Twist, Pose
from std_srvs.srv import Empty
from tf2_ros import Buffer, TransformListener
from tf_transformations import euler_from_quaternion
from rclpy.qos import QoSProfile
from nav_msgs.msg import Odometry 
from std_msgs.msg import Bool
from sensor_msgs.msg import Joy

# Suponiendo que tienes una clase PID similar en Python
from .pid import PID

class Controller(Node):
    class State:
        IDLE = 0
        AUTOMATIC = 1
        TAKING_OFF = 2
        LANDING = 3
        EMERGENCY_STOP = 4  # Añadir estado de parada de emergencia

    def __init__(self):
        super().__init__('controller')
        
        # Parámetros
        self.declare_parameter('frequency', 50.0)
        
        self.frequency = self.get_parameter('frequency').get_parameter_value().double_value

        qos_profile = QoSProfile(depth=10)

        # Publisher de comando de velocidad y activacion de drone
        self.cmd_pub = self.create_publisher(Twist, '/parrot_bebop_2/cmd_vel', qos_profile)
        self.cmd_enable = self.create_publisher(Bool, '/parrot_bebop_2/enable', qos_profile)

        # Suscriptor de setpoint y odometria
        self.goal_sub = self.create_subscription(Pose, 'goal', self.goal_changed, qos_profile)
        self.pos_sub  = self.create_subscription(Odometry, '/model/parrot_bebop_2/odometry', self.pos_changed, qos_profile)
        self.joy_sub = self.create_subscription(Joy, 'joy', self.joy_callback, 10)

        # Servicios de despegue y aterrizaje
        self.takeoff_srv = self.create_service(Empty, 'takeoff', self.takeoff_callback)
        self.land_srv = self.create_service(Empty, 'land', self.land_callback)

        # TF Listener
        self.tf_buffer = Buffer()
        self.tf_listener = TransformListener(self.tf_buffer, self)

        # PIDs
        self.pid_x = self.create_pid('X')
        self.pid_y = self.create_pid('Y')
        self.pid_z = self.create_pid('Z')
        self.pid_yaw = self.create_pid('Yaw')

        self.state = self.State.IDLE
        self.goal = Pose()
        self.current_pose = Pose()  # Inicializar current_pose
        self.enable = False

        timer_period = 1.0 / self.frequency
        self.timer = self.create_timer(timer_period, self.iteration)

    def joy_callback(self, msg):        
        if msg.buttons[0] == 1:  # A button
            self.get_logger().info("Landing initiated")
            self.state = self.State.LANDING
        elif msg.buttons[1] == 1:  # B button
            self.get_logger().info("Emergency stop activated!")
            self.state = self.State.EMERGENCY_STOP
        elif msg.buttons[3] == 1:  # Y button
            self.get_logger().info("Takeoff initiated")
            self.state = self.State.TAKING_OFF

    def create_pid(self, axis):
        prefix = f'PIDs/{axis}/'
        kp = self.get_param_or(prefix + 'kp', 0.0)
        kd = self.get_param_or(prefix + 'kd', 0.0)
        ki = self.get_param_or(prefix + 'ki', 0.0)
        min_output = self.get_param_or(prefix + 'minOutput', -1.0)
        max_output = self.get_param_or(prefix + 'maxOutput', 1.0)
        integrator_min = self.get_param_or(prefix + 'integratorMin', -0.5)
        integrator_max = self.get_param_or(prefix + 'integratorMax', 0.5)

        # Añadir logs para verificar los parámetros
        self.get_logger().info(f'[{axis}] kp: {kp}, kd: {kd}, ki: {ki}, min_output: {min_output}, max_output: {max_output}, integrator_min: {integrator_min}, integrator_max: {integrator_max}')

        return PID(
            self,
            kp,
            kd,
            ki,
            min_output,
            max_output,
            integrator_min,
            integrator_max,
            axis.lower()
        )

    def get_param_or(self, name, default):
        self.declare_parameter(name, default)
        param = self.get_parameter(name)
        self.get_logger().info(f'Parameter {name}: {param.get_parameter_value().double_value}')
        return param.get_parameter_value().double_value

    def goal_changed(self, msg):
        self.goal = msg

    def pos_changed(self, msg):
        self.current_pose.position = msg.pose.pose.position
        self.current_pose.orientation = msg.pose.pose.orientation

    def takeoff_callback(self, request, response):
        self.get_logger().info('Takeoff requested!')
        self.state = self.State.TAKING_OFF
        return response

    def land_callback(self, request, response):
        self.get_logger().info('Landing requested!')
        self.state = self.State.LANDING
        return response

    def pid_reset(self):
        self.pid_x.reset()
        self.pid_y.reset()
        self.pid_z.reset()
        self.pid_yaw.reset()

    def iteration(self):
        if self.state == self.State.TAKING_OFF:
            if self.current_pose.position.z > 0.05:
                self.pid_reset()
                self.state = self.State.AUTOMATIC
                self.enable = True
            else:
                self.enable = False
            self.cmd_enable.publish(Bool(data=self.enable))

        elif self.state == self.State.LANDING:
            self.goal.position.z = 0.05 
            if self.current_pose.position.z <= 0.05:
                self.state = self.State.IDLE
                self.enable = False
                self.cmd_enable.publish(Bool(data=self.enable))
                self.cmd_pub.publish(Twist())

        elif self.state == self.State.AUTOMATIC:
            euler_angles_a = euler_from_quaternion([
                self.current_pose.orientation.x,
                self.current_pose.orientation.y,
                self.current_pose.orientation.z,
                self.current_pose.orientation.w
            ])

            x_a = self.current_pose.position.x
            y_a = self.current_pose.position.y
            z_a = self.current_pose.position.z
            yaw_a = euler_angles_a[2]

            euler_angles_d = euler_from_quaternion([
                self.goal.orientation.x,
                self.goal.orientation.y,
                self.goal.orientation.z,
                self.goal.orientation.w
            ])

            x_d = self.goal.position.x  
            y_d = self.goal.position.y  
            z_d = self.goal.position.z  
            yaw_d = euler_angles_d[2]

            msg = Twist()
            msg.linear.x = float(self.pid_x.update(x_a, x_d))
            msg.linear.y = float(self.pid_y.update(y_a, y_d))
            msg.linear.z = float(self.pid_z.update(z_a, z_d))
            msg.angular.z = float(self.pid_yaw.update(yaw_a, yaw_d))
            self.cmd_pub.publish(msg)

        elif self.state == self.State.IDLE:
            self.cmd_pub.publish(Twist())
            self.enable = False
            self.cmd_enable.publish(Bool(data=self.enable))

        elif self.state == self.State.EMERGENCY_STOP:
            self.cmd_pub.publish(Twist())
            self.enable = False
            self.cmd_enable.publish(Bool(data=self.enable))
            # self.get_logger().info("Emergency stop activated!")

def main(args=None):
    rclpy.init(args=args)
    controller = Controller()
    rclpy.spin(controller)
    controller.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
