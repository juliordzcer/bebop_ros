from launch import LaunchDescription
from launch_ros.actions import Node
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration

def generate_launch_description():
    # Declarar argumentos
    frequency = DeclareLaunchArgument('frequency', default_value='100.0', description='Frequency of control loop')
    robot_name = DeclareLaunchArgument('robot_name', default_value='bebop1', description='Name of the robot')
    goal_name = DeclareLaunchArgument('goal_name', default_value='goal', description='Name of the robot')

    # Parámetros PID
    pid_params = {
        'PIDs.X.kp': 1.9,
        'PIDs.X.kd': 0.055,
        'PIDs.X.ki': 0.22,
        'PIDs.X.minOutput': -10.0,
        'PIDs.X.maxOutput': 10.0,
        'PIDs.X.integratorMin': -10.0,
        'PIDs.X.integratorMax': 10.0,
        'PIDs.Y.kp': 1.9,
        'PIDs.Y.kd': 0.055,
        'PIDs.Y.ki': 0.22,
        'PIDs.Y.minOutput': -10.0,
        'PIDs.Y.maxOutput': 10.0,
        'PIDs.Y.integratorMin': -10.0,
        'PIDs.Y.integratorMax': 10.0,
        'PIDs.Z.kp': 1.2,
        'PIDs.Z.kd': 0.04,
        'PIDs.Z.ki': 0.12,
        'PIDs.Z.minOutput': -6.0,
        'PIDs.Z.maxOutput': 6.0,
        'PIDs.Z.integratorMin': -10.0,
        'PIDs.Z.integratorMax': 10.0,
        'PIDs.Yaw.kp': 1.5,
        'PIDs.Yaw.kd': 0.80,
        'PIDs.Yaw.ki': 0.1,
        'PIDs.Yaw.minOutput': -200.0,
        'PIDs.Yaw.maxOutput': 200.0,
        'PIDs.Yaw.integratorMin': 0.0,
        'PIDs.Yaw.integratorMax': 0.0,
    }

    return LaunchDescription([
        frequency,
        robot_name,
        goal_name,
        Node(
            package='bebop_controller',
            executable='controller_pid',
            name='controller',
            output='screen',
            parameters=[pid_params, {
                'robot_name': LaunchConfiguration('robot_name'),
                'goal_name': LaunchConfiguration('goal_name')
            }],
            remappings=[]
        ),
    ])
