from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription, ExecuteProcess
from launch.launch_description_sources import PythonLaunchDescriptionSource
from ros_gz_bridge.actions import RosGzBridge
from launch_ros.actions import Node
import os
from ament_index_python.packages import get_package_share_directory

def generate_launch_description():
    # Obtener rutas de los paquetes
    pkg_ros_gz_sim = get_package_share_directory('ros_gz_sim')
    pkg_ros_gz_sim_demos = get_package_share_directory('bebop_demo')

    # Definir nombres de robots y condiciones iniciales como cadenas JSON
    robot_names = '["bebop1"]'  # Cadena JSON
    initial_conditions = '[[2.5, -0.5, 0.0, 1.0]]'  # Cadena JSON
    lider = 'bebop1'

    State = ExecuteProcess(
        cmd=['ros2', 'run', 'bebop_gui', 'bebop_gui_one',],
        output='screen'
    )

    # Lanzar Gazebo
    gz_sim = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(pkg_ros_gz_sim, 'launch', 'gz_sim.launch.py')
        ),
        launch_arguments={
            'gz_args': '-r -z  1000000 bebop1.sdf'
        }.items(),
    )
    # Lanzar el puente ROS-Gazebo
    ros_gz_bridge = RosGzBridge(
        bridge_name='ros_gz_bridge',
        config_file=os.path.join(pkg_ros_gz_sim_demos, 'config', 'bebop1.yaml'),
    )

    # Lanzar el nodo de setpoint
    setpoint = ExecuteProcess(
        cmd=[
            'ros2', 'run', 'bebop_demo', 'setpoint',
            '--ros-args',
            '-p', 'h:=0.3',
            '-p', 'r:=0.2',
            '-p', f'lider_name:={lider}',
        ],
        output='screen'
    )


    # Lanzar el nodo MultiRobotPosePublisher
    set_pose = Node(
        package='bebop_demo',
        executable='set_pose', 
        name='set_pose',
        output='screen',
        parameters=[
            {'robot_names': robot_names},
            {'initial_conditions': initial_conditions}
        ]
    )

    # Lanzar el controlador PID
    controller = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(get_package_share_directory('bebop_controller'), 'launch', 'pid.launch.py')
        ),
        launch_arguments={
            'robot_name': f'{lider}',
            'goal_name': 'goal'
        }.items()
    )

    return LaunchDescription([
        State,
        gz_sim,
        ros_gz_bridge,
        setpoint,
        set_pose,
        controller,
    ])