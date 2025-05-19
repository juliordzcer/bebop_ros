from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription, ExecuteProcess, TimerAction
from launch.launch_description_sources import PythonLaunchDescriptionSource
from ros_gz_bridge.actions import RosGzBridge
from launch_ros.actions import Node
from ament_index_python.packages import get_package_share_directory
import os
import json
import numpy as np
from math import sqrt

def is_valid_position(new_pos, existing_positions, min_distance):
    """Verifica si una nueva posición está suficientemente lejos de las existentes."""
    for pos in existing_positions:
        distance = sqrt((new_pos[0] - pos[0])**2 + (new_pos[1] - pos[1])**2)
        if distance < min_distance:
            return False
    return True

def generate_launch_description():
    # =====================================================
    # Configuración de parámetros
    # =====================================================
    num_drones = 5
    min_distance = 0.8  # Separación mínima de 50 cm entre drones
    max_attempts = 100  # Intentos máximos para colocar cada dron
    robot_names = [f"bebop{i+1}" for i in range(num_drones)]
    leaders = ["bebop1"]  # Definimos los líderes

    # =====================================================
    # Generación de condiciones iniciales con separación
    # =====================================================
    np.random.seed(42)  # Semilla para reproducibilidad
    x_range = (-num_drones/5, num_drones/5)
    y_range = (-num_drones/5, num_drones/5)
    z = 0.0

    initial_conditions = []
    for _ in range(num_drones):
        attempts = 0
        while attempts < max_attempts:
            x = np.random.uniform(*x_range)
            y = np.random.uniform(*y_range)
            yaw = 0 #np.random.uniform(-np.pi, np.pi)
            
            if is_valid_position([x, y], initial_conditions, min_distance):
                initial_conditions.append([x, y, z, yaw])
                break
            attempts += 1
        
        if attempts == max_attempts:
            raise RuntimeError(f"No se pudo encontrar posición válida para el dron {len(initial_conditions) + 1}")

    pkg_ros_gz_sim = get_package_share_directory('ros_gz_sim')
    pkg_bebop_demo = get_package_share_directory('bebop_demo')
    pkg_bebop_bearings = get_package_share_directory('bebop_bearings')

    # 1. World Generator (sin delay)
    world_generator = ExecuteProcess(
        cmd=[
            'python3',
            os.path.expanduser('~/ws_bebop/src/bebop_ros/bebop_gz/world_generator.py'),
            f'num_drones={num_drones}'
        ],
        output='screen'
    )

    # 2. Nodos con delay de 2 segundos
    delayed_nodes = [
        IncludeLaunchDescription(
            PythonLaunchDescriptionSource(
                os.path.join(pkg_ros_gz_sim, 'launch', 'gz_sim.launch.py')
            ),
            launch_arguments={
                'gz_args': '-r -z 1000000 bebop_multi.world',
            }.items(),
        ),
        RosGzBridge(
            bridge_name='ros_gz_bridge',
            config_file=os.path.join(pkg_bebop_demo, 'config', 'bebop_bridge.yaml'),
        ),
        Node(
            package='bebop_demo',
            executable='set_pose',
            name='set_pose',
            output='screen',
            parameters=[
                {'robot_names': json.dumps(robot_names)},
                {'initial_conditions': json.dumps(initial_conditions)}
            ]
        ),
        Node(
            package='bebop_controller',
            executable='formation_controller',
            name='formation_controller',
            output='screen',
            parameters=[
                {'robot_names': json.dumps(robot_names)},
                {'leaders': json.dumps(leaders)},
                {'formation_spacing': 1.0},  # 1 metro de separación
                {'trajectory_speed': 0.5},   # 0.5 m/s de velocidad
                {'takeoff_height': 1.0},     # 1 metro de altura
                
                # Parámetros PID (ajustar según necesidad)
                {'PIDs.X.kp': 1.5}, {'PIDs.X.kd': 0.05}, {'PIDs.X.ki': 0.2},
                {'PIDs.Y.kp': 1.5}, {'PIDs.Y.kd': 0.05}, {'PIDs.Y.ki': 0.2},
                {'PIDs.Z.kp': 1.2}, {'PIDs.Z.kd': 0.04}, {'PIDs.Z.ki': 0.12},
                {'PIDs.Yaw.kp': 0.8}, {'PIDs.Yaw.kd': 0.3}, {'PIDs.Yaw.ki': 0.05},
                
                {'max_linear_velocity': 2.0},
                {'max_angular_velocity': 5.5}
            ]
        )
    ]

    # 3. GUI con delay adicional
    state_gui = TimerAction(
        period=2.0,
        actions=[
            Node(
                package='bebop_gui',
                executable='bebop_gui_swarm',
                name='bebop_gui',
                output='screen',
                parameters=[{'num_drones': num_drones}] 
            )
        ]
    )

    return LaunchDescription([
        world_generator,
        TimerAction(
            period=4.0,
            actions=delayed_nodes
        ),
        state_gui
    ])