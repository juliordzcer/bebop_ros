from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription, ExecuteProcess
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration, TextSubstitution
from ros_gz_bridge.actions import RosGzBridge
from launch_ros.actions import Node
import os
from ament_index_python.packages import get_package_share_directory

def generate_launch_description():
    # Obtener rutas de los paquetes
    pkg_ros_gz_sim = get_package_share_directory('ros_gz_sim')
    pkg_ros_gz_sim_demos = get_package_share_directory('bebop_demo')

    # Definir nombres de robots y condiciones iniciales como cadenas JSON
    robot_names = '["bebop1", "bebop2", "bebop3", "bebop4"]'  # Cadena JSON
    initial_conditions = '[[-1.0, -1.0, 0.0, 0.0], [-1.0, 1.0, 0.0, 0.0], [-2.0, 0.0, 0.0, 0.0], [0.0, 0.0, 0.0, 0.0]]'  # Cadena JSON
    formation = '[[-1.0, -1.0, 0.0, 0.0], [-1.0, 1.0, 0.0, 0.0], [-2.0, 0.0, 0.0, 0.0], [0.0, 0.0, 0.0, 0.0]]'  # Cadena JSON
    lider = 'bebop4'
    world_name = 'bebop'

    # Lanzar Gazebo
    gz_sim = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(pkg_ros_gz_sim, 'launch', 'gz_sim.launch.py')
        ),
        launch_arguments={
            'gz_args': '-r -z  2000000 bebop4.sdf'
        }.items(),
    )
    # Lanzar el puente ROS-Gazebo
    ros_gz_bridge = RosGzBridge(
        bridge_name='ros_gz_bridge',
        config_file=os.path.join(pkg_ros_gz_sim_demos, 'config', 'bebop_swarm4.yaml'),
    )

    # Lanzar el nodo de setpoint
    setpoint = ExecuteProcess(
        cmd=[
            'ros2', 'run', 'bebop_demo', 'setpoint',
            '--ros-args',
            '-p', 'h:=2.0',
            '-p', 'r:=0.5',
            '-p', f'lider_name:={lider}',
        ],
        output='screen'
    )

    setpoint_followers = ExecuteProcess(
        cmd=[
            'ros2', 'run', 'bebop_demo', 'setpoint_followers',
            '--ros-args',
            "-p", f"robot_names:='{robot_names}'",  # Corrección: Comillas simples externas, dobles internas
            "-p", f"formation:='{formation}'", # Corrección: Comillas simples externas, dobles internas
            "-p", f"lider_name:='{lider}'",
        ],
        output='screen'
    )

    # Lanzar el nodo de joystick
    joystick = ExecuteProcess(
        cmd=['ros2', 'run', 'joy', 'joy_node'],
        output='screen'
    )

    # Lanzar el nodo MultiRobotPosePublisher
    set_pose = Node(
        package='bebop_demo',
        executable='set_pose',  # Nombre del ejecutable
        name='set_pose',
        output='screen',
        parameters=[
            {'robot_names': robot_names},
            {'initial_conditions': initial_conditions}
        ]
    )

    # Lanzar el controlador PID
    controller_leader = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(get_package_share_directory('bebop_controller'), 'launch', 'pid.launch.py')
        ),
        launch_arguments={
            'robot_name': f'{lider}',
            'goal_name': 'goal'
        }.items()
    )

    # Lanzar el controlador PID
    controller_follower_1 = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(get_package_share_directory('bebop_controller'), 'launch', 'pid.launch.py')
        ),
        launch_arguments={
            'robot_name': 'bebop1',
            'goal_name': 'bebop1/setpoint'
        }.items()
    )

    # Lanzar el controlador PID
    controller_follower_2 = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(get_package_share_directory('bebop_controller'), 'launch', 'pid.launch.py')
        ),
        launch_arguments={
            'robot_name': 'bebop2',
            'goal_name': 'bebop2/setpoint'
        }.items()
    )

    # Lanzar el controlador PID
    controller_follower_3 = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(get_package_share_directory('bebop_controller'), 'launch', 'pid.launch.py')
        ),
        launch_arguments={
            'robot_name': 'bebop3',
            'goal_name': 'bebop3/setpoint'
        }.items()
    )


    # Lanzar el nodo de joystick
    DATA = ExecuteProcess(
        cmd=['ros2', 'run', 'bebop_demo', 'graficas'],
        output='screen'
    )


    # Visor de imagenes
    imagenes = ExecuteProcess(
        cmd=[
            'ros2', 'run', 'bebop_demo', 'imagenes',
            '--ros-args',
            '-p', f'robot_name:={robot_names}',
            '-p', f'world_name:={world_name}',           
            ],
        output='screen'
    )

    return LaunchDescription([
        gz_sim,
        ros_gz_bridge,
        setpoint,
        joystick,
        set_pose,
        controller_leader,
        controller_follower_1,
        controller_follower_2,
        setpoint_followers,
        controller_follower_3,
    ])