import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Image
from cv_bridge import CvBridge
import cv2

class ImageSubscriber(Node):

    def __init__(self):
        super().__init__('image_subscriber')

        # Declarar los parámetros
        self.declare_parameter('robot_names', ['bebop1', 'bebop2'])  # Lista de cadenas
        self.declare_parameter('world_name', 'bebop')  # Nombre del mundo

        # Obtener el parámetro de nombres de robots
        robot_names_param = self.get_parameter('robot_names').get_parameter_value().string_array_value
        self.robot_names = robot_names_param  # Directamente como una lista
        world_name = self.get_parameter('world_name').get_parameter_value().string_value

        # Crear una lista de tópicos basada en los nombres de los robots
        self.topic_names = [
            f'/world/{world_name}/model/{robot_name}/link/rgb_camera_link/sensor/rgb_camera_sensor/image'
            for robot_name in self.robot_names
        ]

        # Inicializar la lista de suscripciones
        self._subscriptions = []  # Usamos un nombre diferente para evitar conflictos

        # Crear suscripciones dinámicas a los tópicos de los robots
        for topic in self.topic_names:
            subscription = self.create_subscription(
                Image,
                topic,
                lambda msg, topic=topic: self.image_callback(msg, topic),  # Pasar el nombre del tópico al callback
                10)
            self._subscriptions.append(subscription)
            self.get_logger().info(f"Subscribed to {topic}")  # Log para verificar la suscripción

        # Diccionario para almacenar las imágenes de cada robot
        self.cv_images = {robot_name: None for robot_name in self.robot_names}

        self.bridge = CvBridge()

    def image_callback(self, msg, topic_name):
        try:
            # Obtener el robot correspondiente
            robot_name = next(r for r in self.robot_names if f'/{r}/' in topic_name)

            # Convertir la imagen
            cv_image = self.bridge.imgmsg_to_cv2(msg, desired_encoding='bgr8')

            # Almacenar la imagen
            self.cv_images[robot_name] = cv_image

            # Mostrar las imágenes de todos los robots
            for robot_name, cv_image in self.cv_images.items():
                if cv_image is not None:
                    cv2.imshow(f"Camera Image {robot_name}", cv_image)
                    # self.get_logger().info(f"Showing image for {robot_name}")  # Log para verificar la visualización
            cv2.waitKey(1)

        except Exception as e:
            self.get_logger().error(f"Error converting image: {e}")


def main(args=None):
    rclpy.init(args=args)

    # Crear un nodo
    node = ImageSubscriber()

    # Iniciar el nodo
    rclpy.spin(node)

    # Limpiar
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()