import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Pose
import matplotlib.pyplot as plt
from mpl_toolkits.mplot3d import Axes3D
import numpy as np
import time
from tf_transformations import euler_from_quaternion
from collections import deque

class RealTimePlotter(Node):
    def __init__(self):
        super().__init__('real_time_plotter')

        # Suscriptores a las poses y setpoints de los robots
        self.sub_pose1 = self.create_subscription(Pose, '/bebop1/pose', lambda msg: self.pose_callback(msg, 'bebop1'), 10)
        self.sub_setpoint1 = self.create_subscription(Pose, '/bebop1/setpoint', lambda msg: self.setpoint_callback(msg, 'bebop1'), 10)
        self.sub_pose2 = self.create_subscription(Pose, '/bebop2/pose', lambda msg: self.pose_callback(msg, 'bebop2'), 10)
        self.sub_setpoint2 = self.create_subscription(Pose, '/goal', lambda msg: self.setpoint_callback(msg, 'bebop2'), 10)

        # Variables para almacenar poses y setpoints
        self.poses = {'bebop1': None, 'bebop2': None}
        self.setpoints = {'bebop1': None, 'bebop2': None}

        # Almacenar trayectorias (usando deque para limitar el número de puntos)
        self.max_history_length = 1000
        self.pose_history = {'bebop1': deque(maxlen=self.max_history_length), 'bebop2': deque(maxlen=self.max_history_length)}
        self.setpoint_history = {'bebop1': deque(maxlen=self.max_history_length), 'bebop2': deque(maxlen=self.max_history_length)}

        # Almacenar errores
        self.error_history = {
            'bebop1': {'x': deque(maxlen=self.max_history_length), 'y': deque(maxlen=self.max_history_length), 
                       'z': deque(maxlen=self.max_history_length), 'yaw': deque(maxlen=self.max_history_length)},
            'bebop2': {'x': deque(maxlen=self.max_history_length), 'y': deque(maxlen=self.max_history_length), 
                       'z': deque(maxlen=self.max_history_length), 'yaw': deque(maxlen=self.max_history_length)}
        }

        # Inicialización de la figura de trayectoria 3D
        plt.ion()
        self.fig_traj = plt.figure()
        self.ax_traj = self.fig_traj.add_subplot(111, projection='3d')
        
        # Inicialización de la figura de error con 4 subgráficos
        self.fig_error, self.axs_error = plt.subplots(2, 2, figsize=(12, 8))
        self.fig_error.suptitle('Errores de seguimiento')

        self.start_time = time.time()

    def pose_callback(self, msg, robot_name):
        """Callback para actualizar la pose de un robot."""
        pose = np.array([msg.position.x, msg.position.y, msg.position.z])
        quat = np.array([msg.orientation.x, msg.orientation.y, msg.orientation.z, msg.orientation.w])
        self.poses[robot_name] = (pose, quat)
        self.pose_history[robot_name].append(pose)

    def setpoint_callback(self, msg, robot_name):
        """Callback para actualizar el setpoint de un robot."""
        setpoint = np.array([msg.position.x, msg.position.y, msg.position.z])
        quat = np.array([msg.orientation.x, msg.orientation.y, msg.orientation.z, msg.orientation.w])
        self.setpoints[robot_name] = (setpoint, quat)
        self.setpoint_history[robot_name].append(setpoint)

    def calculate_yaw_error(self, pose_quat, setpoint_quat):
        """Calcula el error de yaw entre la pose y el setpoint."""
        pose_euler = euler_from_quaternion(pose_quat)
        setpoint_euler = euler_from_quaternion(setpoint_quat)
        return setpoint_euler[2] - pose_euler[2]

    def plot_data(self):
        """Grafica las trayectorias y errores en tiempo real."""
        try:
            self.ax_traj.clear()
            for robot_name in ['bebop1', 'bebop2']:
                if self.poses[robot_name] is not None and self.setpoints[robot_name] is not None:
                    pose, pose_quat = self.poses[robot_name]
                    setpoint, setpoint_quat = self.setpoints[robot_name]

                    # Calcular errores
                    error_x = setpoint[0] - pose[0]
                    error_y = setpoint[1] - pose[1]
                    error_z = setpoint[2] - pose[2]
                    error_yaw = self.calculate_yaw_error(pose_quat, setpoint_quat)

                    # Almacenar errores
                    self.error_history[robot_name]['x'].append(error_x)
                    self.error_history[robot_name]['y'].append(error_y)
                    self.error_history[robot_name]['z'].append(error_z)
                    self.error_history[robot_name]['yaw'].append(error_yaw)

                    # Graficar trayectorias
                    if self.pose_history[robot_name]:
                        pose_array = np.array(self.pose_history[robot_name])
                        self.ax_traj.plot(pose_array[:, 0], pose_array[:, 1], pose_array[:, 2], label=f'Trayectoria {robot_name}')
                    if self.setpoint_history[robot_name]:
                        setpoint_array = np.array(self.setpoint_history[robot_name])
                        self.ax_traj.plot(setpoint_array[:, 0], setpoint_array[:, 1], setpoint_array[:, 2], '--', label=f'Setpoint {robot_name}')

            # Configurar gráfico de trayectorias
            self.ax_traj.set_xlabel('X')
            self.ax_traj.set_ylabel('Y')
            self.ax_traj.set_zlabel('Z')
            self.ax_traj.legend()

            # Graficar errores
            for i, (error_type, ax) in enumerate(zip(['x', 'y', 'z', 'yaw'], self.axs_error.flat)):
                ax.clear()
                for robot_name in ['bebop1', 'bebop2']:
                    if self.error_history[robot_name][error_type]:
                        ax.plot(self.error_history[robot_name][error_type], label=f'Error {error_type} {robot_name}')
                ax.set_title(f'Error {error_type.upper()}')
                ax.legend()
                ax.grid(True)

            plt.draw()
            plt.pause(0.0001)  # Reducir la frecuencia de actualización

        except Exception as e:
            self.get_logger().error(f"Error al graficar: {str(e)}")

    def run(self):
        """Bucle principal para actualizar los gráficos."""
        while rclpy.ok():
            rclpy.spin_once(self, timeout_sec=0.1)
            self.plot_data()

def main():
    rclpy.init()
    plotter = RealTimePlotter()
    plotter.run()
    plotter.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()