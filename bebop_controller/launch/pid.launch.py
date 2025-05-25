# Copyright 2025 Julio César Rodríguez
# Licensed under the Apache License, Version 2.0
# https://www.apache.org/licenses/LICENSE-2.0


from launch import LaunchDescription
from launch_ros.actions import Node
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration

def generate_launch_description():
    # Declarar argumentos
    frequency_arg = DeclareLaunchArgument(
        'frequency',
        default_value='100.0',
        description='Frequency (Hz) of control loop'
    )
    
    robot_name_arg = DeclareLaunchArgument(
        'robot_name', 
        default_value='bebop1',
        description='Namespace for the robot'
    )
    
    goal_name_arg = DeclareLaunchArgument(
        'goal_name',
        default_value='goal',
        description='Topic name for goal position'
    )

    # Parámetros base
    base_params = {
        'frequency': LaunchConfiguration('frequency'),
        'robot_name': LaunchConfiguration('robot_name'),
        'goal_name': LaunchConfiguration('goal_name'),
        'takeoff_threshold': 0.05,  # [m]
        'landing_threshold': 0.03,   # [m]
        'hover_height': 1.5          # [m]
    }

    # Parámetros PID (valores más conservadores)
    pid_params = {
        # PID para eje X
        'PIDs.X.kp': 1.5,
        'PIDs.X.kd': 0.05,
        'PIDs.X.ki': 0.2,
        'PIDs.X.minOutput': -2.0,
        'PIDs.X.maxOutput': 2.0,
        'PIDs.X.integratorMin': -1.0,
        'PIDs.X.integratorMax': 1.0,
        
        # PID para eje Y
        'PIDs.Y.kp': 1.5,
        'PIDs.Y.kd': 0.05,
        'PIDs.Y.ki': 0.2,
        'PIDs.Y.minOutput': -2.0,
        'PIDs.Y.maxOutput': 2.0,
        'PIDs.Y.integratorMin': -1.0,
        'PIDs.Y.integratorMax': 1.0,
        
        # PID para eje Z
        'PIDs.Z.kp': 1.2,
        'PIDs.Z.kd': 0.04,
        'PIDs.Z.ki': 0.12,
        'PIDs.Z.minOutput': -1.5,
        'PIDs.Z.maxOutput': 1.5,
        'PIDs.Z.integratorMin': -0.5,
        'PIDs.Z.integratorMax': 0.5,
        
        # PID para Yaw
        'PIDs.Yaw.kp': 0.8,       # Valor reducido
        'PIDs.Yaw.kd': 0.3,       # Valor reducido
        'PIDs.Yaw.ki': 0.05,      # Valor reducido
        'PIDs.Yaw.minOutput': -45.0,  # Grados/seg
        'PIDs.Yaw.maxOutput': 45.0,   # Grados/seg
        'PIDs.Yaw.integratorMin': -5.0,
        'PIDs.Yaw.integratorMax': 5.0
    }

    # Combinar todos los parámetros
    node_params = [base_params, pid_params]

    return LaunchDescription([
        frequency_arg,
        robot_name_arg,
        goal_name_arg,
        
        Node(
            package='bebop_controller',
            executable='controller_pid',
            name='controller',
            output='screen',
            parameters=node_params,
            remappings=[
                # Ejemplo de remapeo si fuera necesario:
                # ('/cmd_vel', '/bebop1/cmd_vel')
            ]
        )
    ])