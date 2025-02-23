#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
import math
import numpy as np
from tf2_ros import TransformException, Buffer, TransformListener
from std_msgs.msg import Empty, Float32
from geometry_msgs.msg import Twist, PoseStamped, Pose
from std_srvs.srv import Empty as EmptySrv
from rclpy.qos import QoSProfile, ReliabilityPolicy
from tf2_geometry_msgs import do_transform_pose
from pid import PID

class Controller(Node):
    Idle = 0
    Automatic = 1
    TakingOff = 2
    Landing = 3

    def __init__(self, frame):
        super().__init__('controller')
        self.frame = frame
        qos = QoSProfile(depth=10, reliability=ReliabilityPolicy.BEST_EFFORT)
        self.pub_nav = self.create_publisher(Twist, 'cmd_vel', qos)
        self.tf_buffer = Buffer()
        self.tf_listener = TransformListener(self.tf_buffer, self)
        self.pid_x = PID(35, 10, 0.0, -20, 20, "x")
        self.pid_y = PID(-35, -10, -0.0, -20, 20, "y")
        self.pid_z = PID(4000, 3000.0, 2000.0, 10000, 60000, "z")
        self.pid_yaw = PID(-50.0, 0.0, 0.0, -200.0, 200.0, "yaw")
        self.state = Controller.Idle
        self.goal = Pose()
        self.create_subscription(Pose, 'goal', self._pose_changed, qos)
        self.create_service(EmptySrv, 'takeoff', self._takeoff)
        self.create_service(EmptySrv, 'land', self._land)
        self.timer = self.create_timer(0.01, self.run)

    def get_transform(self, source_frame, target_frame):
        try:
            now = self.get_clock().now()
            transform = self.tf_buffer.lookup_transform(source_frame, target_frame, now)
            position = transform.transform.translation
            quaternion = transform.transform.rotation
            return position, quaternion, now
        except TransformException as ex:
            self.get_logger().error(f'Could not transform {source_frame} to {target_frame}: {ex}')
            return None

    def pid_reset(self):
        self.pid_x.reset()
        self.pid_y.reset()
        self.pid_z.reset()
        self.pid_yaw.reset()

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
        thrust = 0
        if self.state == Controller.TakingOff:
            transform = self.get_transform("world", self.frame)
            if transform:
                position, quaternion, t = transform
                if position.z > 0.05 or thrust > 50000:
                    self.pid_reset()
                    self.pid_z.integral = thrust / self.pid_z.ki
                    self.target_z = 0.5
                    self.state = Controller.Automatic
                    thrust = 0
                else:
                    thrust += 100
                    msg = Twist()
                    msg.linear.z = float(thrust)
                    self.pub_nav.publish(msg)
            else:
                self.get_logger().error(f"Could not transform from world to {self.frame}.")

        if self.state == Controller.Landing:
            self.goal.position.z = 0.05
            transform = self.get_transform("world", self.frame)
            if transform:
                position, quaternion, t = transform
                if position.z <= 0.1:
                    self.state = Controller.Idle
                    msg = Twist()
                    self.pub_nav.publish(msg)
            else:
                self.get_logger().error(f"Could not transform from world to {self.frame}.")

        if self.state == Controller.Automatic or self.state == Controller.Landing:
            transform = self.get_transform("world", self.frame)
            if transform:
                position, quaternion, t = transform
                target_world = PoseStamped()
                target_world.header.stamp = t.to_msg()
                target_world.header.frame_id = "world"
                target_world.pose = self.goal

                target_drone = do_transform_pose(target_world, transform)

                quaternion = (
                    target_drone.pose.orientation.x,
                    target_drone.pose.orientation.y,
                    target_drone.pose.orientation.z,
                    target_drone.pose.orientation.w)
                euler = self.euler_from_quaternion(quaternion)

                msg = Twist()
                msg.linear.x = float(self.pid_x.update(0.0, target_drone.pose.position.x))
                msg.linear.y = float(self.pid_y.update(0.0, target_drone.pose.position.y))
                msg.linear.z = float(self.pid_z.update(0.0, target_drone.pose.position.z))
                msg.angular.z = float(self.pid_yaw.update(0.0, euler[2]))
                self.pub_nav.publish(msg)
            else:
                self.get_logger().error(f"Could not transform from world to {self.frame}.")

        if self.state == Controller.Idle:
            msg = Twist()
            self.pub_nav.publish(msg)

    def euler_from_quaternion(self, quaternion):
        x = quaternion[0]
        y = quaternion[1]
        z = quaternion[2]
        w = quaternion[3]

        sinr_cosp = 2 * (w * x + y * z)
        cosr_cosp = 1 - 2 * (x * x + y * y)
        roll = math.atan2(sinr_cosp, cosr_cosp)

        sinp = 2 * (w * y - z * x)
        pitch = math.asin(sinp)

        siny_cosp = 2 * (w * z + x * y)
        cosy_cosp = 1 - 2 * (y * y + z * z)
        yaw = math.atan2(siny_cosp, cosy_cosp)

        return roll, pitch, yaw

def main(args=None):
    rclpy.init(args=args)
    frame = "bebop"  # You can get this from a parameter if needed
    controller = Controller(frame)
    rclpy.spin(controller)
    controller.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()