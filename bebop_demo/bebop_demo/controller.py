import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Point
from sensor_msgs import Joy
from std_srvs import Empty


class Controller(Node):
    def __init__(self, use_controller, joy_topic):
        super().__init__('bebop_demo_controller')

        self._emergency = self.create_client(Empty, 'emergency')
        while not self._emergency.wait_for_service(timeout_sec=1.0):
            self.get_logger().info('emergency service not available, waiting again...')
        self.get_logger().info("found emergency service")

        if use_controller:
            self._land = self.create_client(Empty, 'land')
            while not self._land.wait_for_service(timeout_sec=1.0):
                self.get_logger().info('land service not available, waiting again...')
            self.get_logger().info("found land service")

            self._takeoff = self.create_client(Empty, 'takeoff')
            while not self._takeoff.wait_for_service(timeout_sec=1.0):
                self.get_logger().info('takeoff service not available, waiting again...')
            self.get_logger().info("found takeoff service")
        else:
            self._land = None
            self._takeoff = None

        self._buttons = None
        self._joy_sub = self.create_subscription(Joy, joy_topic, self._joyChanged, 10)


    def _joyChanged(self, data):
        for i in range(0, len(data.buttons)):
            if self._buttons is None or data.buttons[i] != self._buttons[i]:
                if i == 0 and data.buttons[i] == 1 and self._land is not None:
                    self._land.call_async(Empty.Request())
                if i == 1 and data.buttons[i] == 1:
                    self._emergency.call_async(Empty.Request())
                if i == 2 and data.buttons[i] == 1 and self._takeoff is not None:
                    self._takeoff.call_async(Empty.Request())

        self._buttons = data.buttons

def main(args=None):
    rclpy.init(args=args)
    use_controller = rclpy.parameter.Parameter('use_bebop_controller', rclpy.Parameter.Type.BOOL, False)
    joy_topic = rclpy.parameter.Parameter('joy_topic', rclpy.Parameter.Type.STRING, 'joy')
    controller = Controller(use_controller.value, joy_topic.value)
    rclpy.spin(controller)
    controller.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()