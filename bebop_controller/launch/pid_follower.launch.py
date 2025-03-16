from launch import LaunchDescription
from launch_ros.actions import Node
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration

def generate_launch_description():
    # Declarar argumentos
    frequency = DeclareLaunchArgument('frequency', default_value='50.0', description='Frequency of control loop')
    robot_name = DeclareLaunchArgument('robot_name', default_value='bebop1', description='Name of the robot')
    lider_name = DeclareLaunchArgument('lider_name', default_value='bebop2', description='Name of the leader')

    # Parámetros PID
    pid_params = {
        'PIDs.X.kp': 1.2,
        'PIDs.X.kd': 0.4,
        'PIDs.X.ki': 0.1,
        'PIDs.X.minOutput': -10.0,
        'PIDs.X.maxOutput': 10.0,
        'PIDs.X.integratorMin': -0.1,
        'PIDs.X.integratorMax': 0.1,
        'PIDs.Y.kp': 1.2,
        'PIDs.Y.kd': 0.4,
        'PIDs.Y.ki': 0.1,
        'PIDs.Y.minOutput': -10.0,
        'PIDs.Y.maxOutput': 10.0,
        'PIDs.Y.integratorMin': -0.1,
        'PIDs.Y.integratorMax': 0.1,
        'PIDs.Z.kp': 2.0,
        'PIDs.Z.kd': 0.5,
        'PIDs.Z.ki': 0.2,
        'PIDs.Z.minOutput': -6.0,
        'PIDs.Z.maxOutput': 6.0,
        'PIDs.Z.integratorMin': -10.0,
        'PIDs.Z.integratorMax': 10.0,
        'PIDs.Yaw.kp': 2.500,
        'PIDs.Yaw.kd': 3.00,
        'PIDs.Yaw.ki': 0.2,
        'PIDs.Yaw.minOutput': -200.0,
        'PIDs.Yaw.maxOutput': 200.0,
        'PIDs.Yaw.integratorMin': 0.0,
        'PIDs.Yaw.integratorMax': 0.0,
    }

    return LaunchDescription([
        frequency,
        robot_name,
        lider_name,
        Node(
            package='bebop_controller',
            executable='controller_follower',
            name='controller',
            output='screen',
            parameters=[pid_params, {
                'robot_name': LaunchConfiguration('robot_name'),
                'lider_name': LaunchConfiguration('lider_name')
            }],
            remappings=[]
        ),
    ])
