import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Point

class PositionControlNode(Node):
    def __init__(self):
        super().__init__('position_control_node')
        self.publisher_ = self.create_publisher(Point, '/parrot_bebop_2/position_command', 10)
        self.timer = self.create_timer(1.0, self.timer_callback)
        self.target_position = Point()
        self.target_position.x = 0.0
        self.target_position.y = 0.0
        self.target_position.z = 1.0  # Desired altitude

    def timer_callback(self):
        self.publisher_.publish(self.target_position)
        self.get_logger().info(f'Publishing target position: {self.target_position.x}, {self.target_position.y}, {self.target_position.z}')

def main(args=None):
    rclpy.init(args=args)
    node = PositionControlNode()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
