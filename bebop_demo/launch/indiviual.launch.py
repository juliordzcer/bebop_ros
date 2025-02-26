import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription, ExecuteProcess
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from ros_gz_bridge.actions import RosGzBridge
from launch_ros.actions import Node

def generate_launch_description():

    # Gazebo
    gz_sim = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(get_package_share_directory('ros_gz_sim'), 'launch', 'gz_sim.launch.py')),
        launch_arguments={
            'gz_args': '-r -z 1000000 bebop.sdf'
        }.items(),
    )

    # Bridge
    ros_gz_bridge = RosGzBridge(
        bridge_name='ros_gz_bridge',
        config_file=os.path.join(get_package_share_directory('bebop_demo'), 'config', 'bebop.yaml'),
    )

    # Setpoint Node
    setpoint = ExecuteProcess(
        cmd=[
            'ros2', 'run', 'bebop_demo', 'setpoint',
            '--ros-args', 
            '-p', 'xi:=1.0',
            '-p', 'yi:=1.0',
            '-p', 'zi:=0.0',
            '-p', 'h:=0.5',
            '-p', 'r:=1.0',
            '-p', 'yawi:=0.0'
        ],
        output='screen'
    )

    # Position Control Node
    controller = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(get_package_share_directory('bebop_controller'), 'launch', 'pid.launch.py')
        )
    )
     
    # Joy
    Joystick = ExecuteProcess(
        cmd=['ros2', 'run', 'joy', 'joy_node'],
        output='screen'
    )

    # Graficas
    Graficas = ExecuteProcess(
        cmd=['ros2', 'run', 'bebop_demo', 'graficas'],
        output='screen'
    )

    return LaunchDescription([
        gz_sim,
        ros_gz_bridge,
        setpoint,
        Joystick,
        controller,
        Graficas
    ])
