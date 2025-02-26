from launch import LaunchDescription
from launch_ros.actions import Node
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch_ros.substitutions import FindPackageShare

def generate_launch_description():
    config_file = PathJoinSubstitution([
        FindPackageShare('bebop_controller'),
        'config',
        'pid.yaml'
    ])

    return LaunchDescription([
        DeclareLaunchArgument(
            'config_file',
            default_value=config_file,
            description='Path to the configuration file for the PID controller'
        ),
        Node(
            package='bebop_controller',
            executable='controller_node',
            name='controller',
            output='screen',
            parameters=[LaunchConfiguration('config_file')]
        )
    ])
