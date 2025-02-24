#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
import math
from geometry_msgs.msg import Twist, Pose
from nav_msgs.msg import Odometry
from sensor_msgs.msg import Joy
from std_srvs.srv import Empty as EmptySrv
from .pid import PID

class Controller(Node):
    Idle = 0
    Automatic = 1
    TakingOff = 2
    Landing = 3
    EmergencyStop = 4

    def __init__(self):
        super().__init__('controller')

        # Publisher for drone commands
        self.pub_nav = self.create_publisher(Twist, '/parrot_bebop_2/gazebo/command/twist', 10)

        # Subscribers
        self.create_subscription(Odometry, '/model/parrot_bebop_2/odometry', self.odometry_callback, 10)
        self.create_subscription(Pose, 'goal', self._pose_changed, 10)
        self.joy_sub = self.create_subscription(Joy, 'joy', self.joy_callback, 10)

        # Services
        self.create_service(EmptySrv, 'takeoff', self._takeoff)
        self.create_service(EmptySrv, 'land', self._land)

        # Timer for control loop
        self.timer = self.create_timer(0.01, self.run)

        # Current and goal poses
        self.current_pose = Pose()
        self.goal = Pose()

        # PID controllers
        self.pid_x = PID(self, 3.5, 1.0, 0.0, -16.0, 16.0, "x")  
        self.pid_y = PID(self, 3.5, 1.0, 0.0, -16.0, 16.0, "y")  
        self.pid_z = PID(self, 4.0, 3.00, 2.00,-6.0, 6.0, "z")  
        self.pid_yaw = PID(self, 5.00, 1.0, 0.20, -200.0, 200.0, "yaw")  

        # State of the controller
        self.state = Controller.Idle

        # Scale factor for joystick input
        self.scale = 0.6

    def odometry_callback(self, msg):
        self.current_pose.position = msg.pose.pose.position
        self.current_pose.orientation = msg.pose.pose.orientation

    def joy_callback(self, msg):
        # self.get_logger().info(f"Joy message received: {msg}")
        # self.get_logger().info(f"Button states: {msg.buttons}")
        
        if msg.buttons[0] == 1:  # A button
            self.get_logger().info("Landing initiated via Xbox controller.")
            self.state = Controller.Landing
        elif msg.buttons[1] == 1:  # B button
            self.get_logger().info("Emergency stop triggered via Xbox controller.")
            self.state = Controller.EmergencyStop
        elif msg.buttons[3] == 1:  # Y button
            self.get_logger().info("Takeoff initiated via Xbox controller.")
            self.state = Controller.TakingOff

    def _pose_changed(self, msg):
        self.goal = msg

    def _takeoff(self, request, response):
        self.get_logger().info("Takeoff requested!")
        self.state = Controller.TakingOff
        return response

    def _land(self, request, response):
        self.get_logger().info("Landing requested!")
        self.state = Controller.Landing
        return response

    def run(self):
        msg = Twist()

        if self.state == Controller.EmergencyStop:
            self.pub_nav.publish(msg)
            return

        if self.state == Controller.TakingOff:
            msg.linear.z = 0.0  # Adjust this value as needed
            self.pub_nav.publish(msg)
            self.state = Controller.Automatic  # Transition to automatic control after takeoff

        if self.state == Controller.Landing:
            msg.linear.z = 0.0  # Adjust this value as needed
            self.pub_nav.publish(msg)
            self.state = Controller.Idle  # Transition to idle after landing



        if self.state == Controller.Automatic:
            # PID control logic
            ex = self.goal.position.x - self.current_pose.position.x
            ey = self.goal.position.y - self.current_pose.position.y
            ez = self.goal.position.z - self.current_pose.position.z
            eyaw = self.goal.orientation.z - self.current_pose.orientation.z

            msg.linear.x = float(self.pid_x.update(0.0, ex))*0.01
            msg.linear.y = float(self.pid_y.update(0.0, ey))*0.01
            msg.linear.z = float(self.pid_z.update(0.0, ez))*0.01
            msg.angular.z = 0.0 #float(self.pid_z.update(0.0, eyaw))*0.01

            self.pub_nav.publish(msg)

        if self.state == Controller.Idle:
            self.pub_nav.publish(msg)

def main(args=None):
    rclpy.init(args=args)
    controller = Controller()
    rclpy.spin(controller)
    controller.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()