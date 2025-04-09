from launch import LaunchDescription
from launch.actions import  ExecuteProcess


def generate_launch_description():
    # Lanzar el nodo de joystick
    State = ExecuteProcess(
        cmd=['ros2', 'run', 'bebop_gui', 'bebop_gui'],
        output='screen'
    )

    return LaunchDescription([
        State
    ])