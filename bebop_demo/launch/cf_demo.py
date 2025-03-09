import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription, ExecuteProcess
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from ros_gz_bridge.actions import RosGzBridge
from launch_ros.actions import Node



def generate_launch_description():

    pkg_ros_gz_sim = get_package_share_directory('ros_gz_sim')
    pkg_ros_gz_sim_demos = get_package_share_directory('bebop_demo')

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
        config_file=os.path.join(pkg_ros_gz_sim_demos, 'config', 'bebop.yaml'),
    )

    # Position Control Node
    position_control = ExecuteProcess(
        cmd=['ros2', 'run', 'bebop_demo', 'joystick'],
        output='screen'
    )
     
    # Joy
    Joystick = ExecuteProcess(
        cmd=['ros2', 'run', 'joy', 'joy_node'],
        output='screen'
    )

    return LaunchDescription([
        gz_sim,
        ros_gz_bridge,
        position_control,
        Joystick
    ])
