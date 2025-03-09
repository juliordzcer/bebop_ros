import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist

class CrazyflieController(Node):
    def __init__(self):
        super().__init__('crazyflie_controller')
        self.publisher = self.create_publisher(Twist, '/crazyflie/cmd_vel', 10)
        self.timer = self.create_timer(0.1, self.timer_callback)

    def timer_callback(self):
        msg = Twist()
        msg.linear.x = 0.0  # Velocidad lineal en X
        msg.angular.z = 0.5  # Velocidad angular en Z
        self.publisher.publish(msg)
        self.get_logger().info('Publicando comando de control')

def main(args=None):
    rclpy.init(args=args)
    controller = CrazyflieController()
    rclpy.spin(controller)
    controller.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()