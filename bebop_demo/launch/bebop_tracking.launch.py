from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription, ExecuteProcess
from launch.launch_description_sources import PythonLaunchDescriptionSource
from ros_gz_bridge.actions import RosGzBridge
from launch_ros.actions import Node
from ament_index_python.packages import get_package_share_directory
import os

def generate_launch_description():
    # =====================================================
    # Simulation Parameters
    # =====================================================
    num_drones = 1
    robot_names = '["bebop1"]'
    initial_conditions = '[[2.5, -0.5, 0.0, 1.0]]'
    lider = 'bebop1'

    # =====================================================
    # Package Paths
    # =====================================================
    pkg_ros_gz_sim = get_package_share_directory('ros_gz_sim')
    pkg_bebop_demo = get_package_share_directory('bebop_demo')
    pkg_bebop_controller = get_package_share_directory('bebop_controller')

    # =====================================================
    # World Generator Script
    # =====================================================
    world_generator = ExecuteProcess(
        cmd=[
            'python3',
            os.path.expanduser('~/ws_bebop/src/bebop_ros/bebop_gz/world_generator.py'),
            f'num_drones={num_drones}'
        ],
        output='screen'
    )

    # =====================================================
    # Custom GUI Node
    # =====================================================
    state_gui = ExecuteProcess(
        cmd=['ros2', 'run', 'bebop_gui', 'bebop_gui'],
        output='screen'
    )

    # =====================================================
    # Launch Gazebo Simulator
    # =====================================================
    gz_sim = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(pkg_ros_gz_sim, 'launch', 'gz_sim.launch.py')
        ),
        launch_arguments={
            'gz_args': '-r -z  1000000 bebop_multi.world',
        }.items(),
    )

    # =====================================================
    # ROS-Gazebo Bridge
    # =====================================================
    ros_gz_bridge = RosGzBridge(
        bridge_name='ros_gz_bridge',
        config_file=os.path.join(pkg_bebop_demo, 'config', 'bebop_bridge.yaml'),
    )

    # =====================================================
    # Trajectory Generator Node (Setpoint)
    # =====================================================
    setpoint = ExecuteProcess(
        cmd=[
            'ros2', 'run', 'bebop_demo', 'setpoint',
            '--ros-args',
            '-p', 'h:=0.3',
            '-p', 'r:=0.2',
            '-p', f'lider_name:={lider}',
        ],
        output='screen'
    )

    # =====================================================
    # Initial Pose Publisher Node
    # =====================================================
    set_pose = Node(
        package='bebop_demo',
        executable='set_pose',
        name='set_pose',
        output='screen',
        parameters=[
            {'robot_names': robot_names},
            {'initial_conditions': initial_conditions}
        ]
    )

    # =====================================================
    # PID Controller Node
    # =====================================================
    controller = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(pkg_bebop_controller, 'launch', 'pid.launch.py')
        ),
        launch_arguments={
            'robot_name': lider,
            'goal_name': 'goal'
        }.items()
    )

    # =====================================================
    # Launch Description
    # =====================================================
    return LaunchDescription([
        world_generator,
        state_gui,
        gz_sim,
        ros_gz_bridge,
        setpoint,
        set_pose,
        controller,
    ])