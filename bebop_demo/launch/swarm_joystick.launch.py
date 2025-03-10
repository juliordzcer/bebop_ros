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

    lider_1 = "bebop2"
    world_name = "bebop"
    robot_names = '["bebop1", "bebop2"]'

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

    # joystick
    joystick_controller = ExecuteProcess(
        cmd=[
            'ros2', 'run', 'bebop_demo', 'joystick',
            '--ros-args',
            '-p', f'robot_name:={lider_1}',
            ],
        output='screen'
    )

    setpoint = ExecuteProcess(
        cmd=[
            'ros2', 'run', 'bebop_demo', 'setpoint',
            '--ros-args', 
            '-p', 'xi:=1.0',
            '-p', 'yi:=1.0',
            '-p', 'zi:=0.0',
            '-p', 'h:=0.5',
            '-p', 'r:=1.0',
            '-p', 'yawi:=0.0',
            '-p', f'robot_name:={lider_1}',
        ],
        output='screen'
    )


    # Joy Node
    joystick = ExecuteProcess(
        cmd=['ros2', 'run', 'joy', 'joy_node'],
        output='screen'
    )

    # # Visor de imagenes
    # imagenes = ExecuteProcess(
    #     cmd=[
    #         'ros2', 'run', 'bebop_demo', 'imagenes',
    #         '--ros-args',
    #         '-p', f'robot_name:={robot_names}',
    #         '-p', f'world_name:={world_name}',           
    #         ],
    #     output='screen'
    # )

    # Position Control Node
    controller_leader = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(get_package_share_directory('bebop_controller'), 'launch', 'pid.launch.py')
        ),
        launch_arguments={
            'robot_name': f'{lider_1}'
        }.items()
    )

    return LaunchDescription([
        gz_sim,
        ros_gz_bridge,
        joystick_controller,
        joystick,
        # imagenes,
        setpoint,
        controller_leader
    ])
