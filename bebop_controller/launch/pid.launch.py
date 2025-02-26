from launch import LaunchDescription
from launch_ros.actions import Node

def generate_launch_description():
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
        'PIDs.Z.kp': 6.0,
        'PIDs.Z.kd': 1.0,
        'PIDs.Z.ki': 0.2,
        'PIDs.Z.minOutput': -6.0,
        'PIDs.Z.maxOutput': 6.0,
        'PIDs.Z.integratorMin': -10.0,
        'PIDs.Z.integratorMax': 10.0,
        'PIDs.Yaw.kp': 2.500,
        'PIDs.Yaw.kd': 3.00,
        'PIDs.Yaw.ki': 0.0,
        'PIDs.Yaw.minOutput': -200.0,
        'PIDs.Yaw.maxOutput': 200.0,
        'PIDs.Yaw.integratorMin': 0.0,
        'PIDs.Yaw.integratorMax': 0.0,
    }

    return LaunchDescription([
        Node(
            package='bebop_controller',
            executable='controller_node',
            name='controller',
            output='screen',
            parameters=[pid_params]
        )
    ])
