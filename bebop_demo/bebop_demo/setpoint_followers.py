import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile
from geometry_msgs.msg import Pose
from tf_transformations import quaternion_from_euler, quaternion_multiply
from sensor_msgs.msg import Joy  # Mensaje del joystick
import json

class JoystickButtons:
    """Enumeración para los botones del joystick."""
    BUTTON_2 = 3  # Botón que iniciará la grabación


class SetpointFollowers(Node):
    def __init__(self):
        super().__init__('multi_robot_pose_publisher')

        # Estado para controlar cuándo iniciar las gráficas
        self.recording_enabled = False  

        # Declarar parámetros
        self.declare_parameter('robot_names', '["bebop1", "bebop2", "bebop3"]')
        self.declare_parameter('formation', '[[1.0, 2.0, 0.0, 0.0], [3.0, 4.0, 0.0, 1.57]], [3.0, 4.0, 0.0, 1.57]]')
        self.declare_parameter('lider_name', 'bebop3')

        # Obtener y validar parámetros
        self.load_parameters()

        # Configuración de QoS
        qos = QoSProfile(depth=10)

        # Suscribirse al joystick
        self.joy_sub = self.create_subscription(Joy, '/joy', self.joy_callback, qos)

        # Suscribirse a la pose del líder
        self.pose_leader = None  
        self.pos_leader_sub = self.create_subscription(Pose, f"/{self.lider_name}/pose", self.pos_changed, qos)

        # Crear publicadores para cada robot seguidor
        self._publishers = [
            self.create_publisher(Pose, f'/{name}/setpoint', qos)
            for name in self.robot_names
        ]

        # Crear un temporizador para la publicación de setpoints (10 Hz)
        self.create_timer(0.001, self.publish_setpoint)  

    def load_parameters(self):
        """ Carga y valida los parámetros desde ROS2 """
        self.lider_name = self.get_parameter('lider_name').value.strip()
        raw_robot_names = self.get_parameter('robot_names').value
        raw_formation = self.get_parameter('formation').value

        try:
            self.robot_names = json.loads(raw_robot_names)
            self.formation = json.loads(raw_formation)
        except json.JSONDecodeError as e:
            self.get_logger().error(f"ERROR de JSON: {str(e)}")
            raise

        if len(self.robot_names) != len(self.formation):
            raise ValueError("Error: La cantidad de robots y formaciones no coincide.")

        for condition in self.formation:
            if not isinstance(condition, list) or len(condition) != 4:
                raise ValueError("Error: Los datos de formación deben ser listas de 4 números.")

        # Excluir al líder de la lista de seguidores
        if self.lider_name in self.robot_names:
            self.robot_names.remove(self.lider_name)

    def joy_callback(self, msg):
        """ Callback para recibir los datos del joystick """
        if len(msg.buttons) > JoystickButtons.BUTTON_2 and msg.buttons[JoystickButtons.BUTTON_2] == 1:
            if not self.recording_enabled:
                self.recording_enabled = True  # Habilita la publicación
                self.get_logger().info("¡Botón 2 presionado! Iniciando gráficas...")

    def pos_changed(self, msg):
        """ Callback que actualiza la pose del líder """
        self.pose_leader = msg

    def calculate_follower_pose(self, leader_pose, formation_conditions):
        """ Calcula la pose del seguidor basada en la pose del líder y las condiciones de formación """
        pose = Pose()
        pose.position.x = leader_pose.position.x + formation_conditions[0]
        pose.position.y = leader_pose.position.y + formation_conditions[1]
        pose.position.z = leader_pose.position.z + formation_conditions[2]

        # Convertir ángulo de yaw a cuaternión
        q_offset = quaternion_from_euler(0, 0, formation_conditions[3])
        q_leader = [
            leader_pose.orientation.x,
            leader_pose.orientation.y,
            leader_pose.orientation.z,
            leader_pose.orientation.w,
        ]
        q_result = quaternion_multiply(q_leader, q_offset)

        # Asignar la orientación resultante
        pose.orientation.x = q_result[0]
        pose.orientation.y = q_result[1]
        pose.orientation.z = q_result[2]
        pose.orientation.w = q_result[3]

        return pose

    def publish_setpoint(self):
        """ Publica las posiciones de los seguidores basadas en el líder """
        if not self.recording_enabled:
            return  # No publica hasta que el botón 2 sea presionado

        if self.pose_leader is None:
            return  # Aún no se ha recibido la pose del líder

        for i, (name, conditions) in enumerate(zip(self.robot_names, self.formation)):
            try:
                pose = self.calculate_follower_pose(self.pose_leader, conditions)
                self._publishers[i].publish(pose)
            except Exception as e:
                self.get_logger().error(f"Error al calcular o publicar la pose para {name}: {str(e)}")

def main(args=None):
    rclpy.init(args=args)
    try:
        node = SetpointFollowers()
        rclpy.spin(node)
    except Exception as e:
        if 'node' in locals() and node is not None:
            node.get_logger().error(f"ERROR FATAL: {str(e)}")
        else:
            print(f"ERROR FATAL: {str(e)}")
    finally:
        if 'node' in locals():
            node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()
