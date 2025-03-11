import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Pose
from tf_transformations import quaternion_from_euler
import json

class MultiRobotPosePublisher(Node):
    def __init__(self):
        super().__init__('multi_robot_pose_publisher')
        
        # Declarar parámetros
        self.declare_parameter('robot_names', '["bebop1", "bebop2"]')  # Cadena JSON
        self.declare_parameter('initial_conditions', '[[1.0, 2.0, 0.0, 0.0], [3.0, 4.0, 0.0, 1.57]]')  # Cadena JSON
        
        # Obtener parámetros
        raw_robot_names = self.get_parameter('robot_names').value
        raw_initial_conditions = self.get_parameter('initial_conditions').value
        
        # Parsear JSON
        try:
            self.robot_names = json.loads(raw_robot_names)
            self.initial_conditions = json.loads(raw_initial_conditions)
        except json.JSONDecodeError as e:
            self.get_logger().error(f"ERROR de JSON: {str(e)}")
            raise
        
        # Crear publicadores
        self._publishers = [
            self.create_publisher(Pose, f'{name}/set_pose', 10)
            for name in self.robot_names
        ]
        
        # Temporizador de un solo disparo con un retraso de 2 segundos
        self.timer = self.create_timer(2.0, self.publish_initial_poses)

    def publish_initial_poses(self):
        # Publicar las poses una sola vez
        for i, (name, conditions) in enumerate(zip(self.robot_names, self.initial_conditions)):
            self.hd = 0.05075
            pose = Pose()
            pose.position.x = conditions[0]
            pose.position.y = conditions[1]
            pose.position.z = conditions[2] + self.hd
            
            # Convertir ángulo de yaw a cuaternión
            q = quaternion_from_euler(0, 0, conditions[3])
            pose.orientation.x = q[0]
            pose.orientation.y = q[1]
            pose.orientation.z = q[2]
            pose.orientation.w = q[3]
            
            # Publicar la pose
            self._publishers[i].publish(pose)
            self.get_logger().info(f"Publicada pose para {name}: {conditions}")
        
        # Detener el temporizador después de la primera publicación
        self.timer.cancel()

def main(args=None):
    rclpy.init(args=args)
    try:
        node = MultiRobotPosePublisher()
        rclpy.spin(node)
    except Exception as e:
        node.get_logger().error(f"ERROR FATAL: {str(e)}")
    finally:
        if 'node' in locals():  # Para evitar el error cuando `node` no se crea
            node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()