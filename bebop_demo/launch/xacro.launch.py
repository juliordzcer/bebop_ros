import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription, ExecuteProcess
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch_ros.actions import Node

def generate_launch_description():
    # Ruta al archivo Xacro
    pkg_share = get_package_share_directory('bebop_gz')
    xacro_file = os.path.join(pkg_share, 'urdf', 'bebop2.xacro')

    # Convertir Xacro a URDF
    urdf_file = os.path.join(pkg_share, 'urdf', 'bebop2.urdf')
    if not os.path.exists(urdf_file):
        os.system(f'xacro {xacro_file} -o {urdf_file}')

    # Iniciar Gazebo Ignition
    gazebo_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(get_package_share_directory('ros_gz_sim'), 'launch', 'gz_sim.launch.py')
        ),
        launch_arguments={'gz_args': '-r empty.sdf'}.items()
    )

    # Publicar el modelo del robot en Gazebo
    spawn_entity = Node(
        package='ros_gz_sim',
        executable='create',
        arguments=['-file', urdf_file, '-name', 'bebop2'],
        output='screen'
    )

    return LaunchDescription([
        gazebo_launch,
        spawn_entity
    ])