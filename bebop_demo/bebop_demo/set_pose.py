import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Pose
from tf_transformations import quaternion_from_euler
import json

class MultiRobotPosePublisher(Node):
    def __init__(self):
        super().__init__('multi_robot_pose_publisher')
        
        # Declarar parámetros con valores por defecto genéricos
        self.declare_parameter('robot_names', '[]')  # Lista vacía por defecto
        self.declare_parameter('initial_conditions', '[]')  # Lista vacía por defecto
        
        # Obtener y parsear parámetros
        try:
            self.robot_names = json.loads(self.get_parameter('robot_names').value)
            self.initial_conditions = json.loads(self.get_parameter('initial_conditions').value)
        except json.JSONDecodeError as e:
            self.get_logger().error(f"ERROR de JSON: {str(e)}")
            raise
        
        # Validar que coincidan las longitudes
        if len(self.robot_names) != len(self.initial_conditions):
            self.get_logger().error(
                f"Número de robots ({len(self.robot_names)}) "
                f"no coincide con condiciones iniciales ({len(self.initial_conditions)})"
            )
            raise ValueError("Longitudes de parámetros no coinciden")
        
        # Crear publicadores dinámicamente
        self._publishers = [
            self.create_publisher(Pose, f'{name}/set_pose', 10)
            for name in self.robot_names
        ]
        
        # Temporizador de un solo disparo
        self.timer = self.create_timer(2.0, self.publish_initial_poses)

    def publish_initial_poses(self):
        """Publica las poses iniciales para todos los drones."""
        self.hd = 0.05075  # Altura de despegue (ajustable)
        
        for i, (name, conditions) in enumerate(zip(self.robot_names, self.initial_conditions)):
            if len(conditions) < 4:
                self.get_logger().error(f"Condiciones iniciales inválidas para {name}: {conditions}")
                continue
            
            pose = Pose()
            pose.position.x = conditions[0]
            pose.position.y = conditions[1]
            pose.position.z = conditions[2] + self.hd  # Añadir altura de despegue
            
            # Convertir yaw (en radianes) a cuaternión
            q = quaternion_from_euler(0, 0, conditions[3])
            pose.orientation.x = q[0]
            pose.orientation.y = q[1]
            pose.orientation.z = q[2]
            pose.orientation.w = q[3]
            
            self._publishers[i].publish(pose)
            self.get_logger().info(f"Publicada pose inicial para {name}: {conditions}")
        
        self.timer.cancel()  # Detener después de publicar

def main(args=None):
    rclpy.init(args=args)
    try:
        node = MultiRobotPosePublisher()
        rclpy.spin(node)
    except Exception as e:
        node.get_logger().error(f"ERROR: {str(e)}")
    finally:
        if 'node' in locals():
            node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()