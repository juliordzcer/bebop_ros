from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription, ExecuteProcess
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration
from ros_gz_bridge.actions import RosGzBridge
from launch_ros.actions import Node
import os
from ament_index_python.packages import get_package_share_directory

def generate_launch_description():
    pkg_ros_gz_sim = get_package_share_directory('ros_gz_sim')
    pkg_ros_gz_sim_demos = get_package_share_directory('bebop_demo')

    # Declarar el parámetro 'robot_names' como una lista de cadenas
    declare_robot_names = DeclareLaunchArgument(
        'robot_names',
        default_value='["bebop1", "bebop2"]',  # Pasa los nombres como una lista de cadenas
        description='List of robots to show images from'
    )
    
    # Declarar el parámetro 'world_name'
    declare_world_name = DeclareLaunchArgument(
        'world_name',
        default_value='bebop',  # Nombre del mundo
        description='Name of the world to load'
    )

    # Gazebo
    gz_sim = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(pkg_ros_gz_sim, 'launch', 'gz_sim.launch.py')),
        launch_arguments={
            'gz_args': '-r -z 1000000 bebop.sdf'
        }.items(),
    )

    # Bridge
    ros_gz_bridge = RosGzBridge(
        bridge_name='ros_gz_bridge',
        config_file=os.path.join(pkg_ros_gz_sim_demos, 'config', 'bebop_swarm.yaml'),
    )

    # Position Control Node (Usando el primer robot de la lista)
    position_control = Node(
        package='bebop_demo',
        executable='joystick_swarm',
        name='joystick_swarm',
        parameters=[{'robot_name': 'bebop2'}],  # Seleccionar el robot aquí
        output='screen'
    )

    # Joy Node
    joystick = ExecuteProcess(
        cmd=['ros2', 'run', 'joy', 'joy_node'],
        output='screen'
    )

    # Visor de imágenes para múltiples robots
    imagenes = Node(
        package='bebop_demo',
        executable='imagenes',
        name='imagenes',
        parameters=[{'robot_names': LaunchConfiguration('robot_names')}],  # Usar LaunchConfiguration para obtener el valor
        output='screen'
    )

    return LaunchDescription([
        declare_robot_names,  # Declarar el parámetro para los robots
        declare_world_name,
        gz_sim,
        ros_gz_bridge,
        position_control,
        joystick,
        imagenes
    ])