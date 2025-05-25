import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Image
from cv_bridge import CvBridge, CvBridgeError
import cv2
from rclpy.qos import QoSProfile, QoSReliabilityPolicy, QoSHistoryPolicy

class ImageSubscriber(Node):

    def __init__(self):
        super().__init__('image_subscriber')

        self.declare_parameter('robot_names', ['bebop1', 'bebop2', 'bebop3'])
        self.declare_parameter('world_name', 'bebop')

        robot_names_param = self.get_parameter('robot_names').get_parameter_value().string_array_value
        self.robot_names = robot_names_param
        world_name = self.get_parameter('world_name').get_parameter_value().string_value

        self.topic_names = [
            f'/world/{world_name}/model/{robot_name}/link/body/sensor/rgb_camera_sensor/image'
            for robot_name in self.robot_names
        ]

        self._subscriptions = []
        self.cv_images = {robot_name: None for robot_name in self.robot_names}
        self.bridge = CvBridge()
        self.frame_counter = 0  # Contador de fotogramas
        self.display_rate = 5  # Mostrar cada 5 fotogramas

        qos_profile = QoSProfile(
            reliability=QoSReliabilityPolicy.BEST_EFFORT,
            history=QoSHistoryPolicy.KEEP_LAST,
            depth=10
        )

        for topic in self.topic_names:
            subscription = self.create_subscription(
                Image,
                topic,
                lambda msg, topic=topic: self.image_callback(msg, topic),
                qos_profile=qos_profile
            )
            self._subscriptions.append(subscription)
            self.get_logger().info(f"Subscribed to {topic}")

    def image_callback(self, msg, topic_name):
        try:
            robot_name = next(r for r in self.robot_names if f'/{r}/' in topic_name)
            cv_image = self.bridge.imgmsg_to_cv2(msg, desired_encoding='bgr8')
            self.cv_images[robot_name] = cv_image
            self.frame_counter += 1

            if self.frame_counter % self.display_rate == 0:
                for robot_name, cv_image in self.cv_images.items():
                    if cv_image is not None:
                        cv2.imshow(f"Camera Image {robot_name}", cv_image)
                cv2.waitKey(1)

        except CvBridgeError as e:
            self.get_logger().error(f"Error converting image: {e}")
        except KeyError as e:
            self.get_logger().error(f"Robot name not found in topic: {e}")
        except Exception as e:
            self.get_logger().error(f"Unexpected error: {e}")

    def __del__(self):
        cv2.destroyAllWindows()

def main(args=None):
    rclpy.init(args=args)
    node = ImageSubscriber()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()