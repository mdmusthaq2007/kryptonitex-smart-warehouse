import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription, TimerAction
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import Command
from launch_ros.actions import Node


def generate_launch_description():
    pkg = get_package_share_directory('forklift_robot')
    world = os.path.join(pkg, 'worlds', 'final_smart_warehouse_48x48x3.sdf')
    xacro_file = os.path.join(pkg, 'urdf', 'forklift.urdf.xacro')
    robot_description = Command(['xacro ', xacro_file])

    gz = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(get_package_share_directory('ros_gz_sim'), 'launch', 'gz_sim.launch.py')
        ),
        launch_arguments={'gz_args': ['-r ', world]}.items(),
    )
    rsp = Node(
        package='robot_state_publisher', executable='robot_state_publisher', output='screen',
        parameters=[{'robot_description': robot_description, 'use_sim_time': True}],
    )
    spawn = Node(
        package='ros_gz_sim', executable='create', output='screen',
        arguments=['-topic', 'robot_description', '-name', 'kryptonitex_forklift', '-x', '0', '-y', '0', '-z', '0.1'],
    )
    jsb = Node(package='controller_manager', executable='spawner', arguments=['joint_state_broadcaster'], output='screen')
    diff = Node(package='controller_manager', executable='spawner', arguments=['diff_drive_controller'], output='screen')
    fork = Node(package='controller_manager', executable='spawner', arguments=['fork_position_controller'], output='screen')
    return LaunchDescription([
        gz, rsp,
        TimerAction(period=3.0, actions=[spawn]),
        TimerAction(period=5.0, actions=[jsb, diff, fork]),
    ])
