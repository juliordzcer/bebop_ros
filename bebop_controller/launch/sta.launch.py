from launch import LaunchDescription
from launch_ros.actions import Node
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
import math


def generate_launch_description():
    # Declarar argumentos
    frequency = DeclareLaunchArgument('frequency', default_value='50.0', description='Frequency of control loop')
    robot_name = DeclareLaunchArgument('robot_name', default_value='bebop1', description='Name of the robot')
    goal_name = DeclareLaunchArgument('goal_name', default_value='goal', description='Name of the goal')

    k0_x = 9.5/5.0
    k1_x = 0.16/5.0
    k2_x = 0.02/5.0

    k0_y = 9.5/5.0
    k1_y = 0.16/5.0
    k2_y = 0.02/5.0 

    k0_z = 9.5/5.0
    k1_z = 0.16/5.0
    k2_z = 0.02/5.0 

    

    k0_yaw = 0.0
    k1_yaw, k2_yaw = 0.0, 0.0 

    # Parámetros STA
    sta_params = {
        'STAs.X.k0': k0_x,
        'STAs.X.k1': k1_x,
        'STAs.X.k2': k2_x,
        'STAs.X.minOutput': -10.0,
        'STAs.X.maxOutput': 10.0,
        'STAs.X.integratorMin': -10.0,
        'STAs.X.integratorMax': 10.0,
        'STAs.Y.k0': k0_y,
        'STAs.Y.k1': k1_y,
        'STAs.Y.k2': k2_y,
        'STAs.Y.minOutput': -10.0,
        'STAs.Y.maxOutput': 10.0,
        'STAs.Y.integratorMin': -10.0,
        'STAs.Y.integratorMax': 10.0,
        'STAs.Z.k0': k0_z,
        'STAs.Z.k1': k1_z,
        'STAs.Z.k2': k2_z,
        'STAs.Z.minOutput': -6.0,
        'STAs.Z.maxOutput': 6.0,
        'STAs.Z.integratorMin': -10.0,
        'STAs.Z.integratorMax': 10.0,
        'STAs.Yaw.k0': k0_yaw,
        'STAs.Yaw.k1': k1_yaw,
        'STAs.Yaw.k2': k2_yaw,
        'STAs.Yaw.minOutput': -200.0,
        'STAs.Yaw.maxOutput': 200.0,
        'STAs.Yaw.integratorMin': 0.0,
        'STAs.Yaw.integratorMax': 0.0,
    }

    return LaunchDescription([
        frequency,
        robot_name,
        goal_name,
        Node(
            package='bebop_controller',
            executable='controller_sta',
            name='controller',
            output='screen',
            parameters=[sta_params, {
                'robot_name': LaunchConfiguration('robot_name'),
                'goal_name': LaunchConfiguration('goal_name')
            }],
            remappings=[]
        ),
    ])
