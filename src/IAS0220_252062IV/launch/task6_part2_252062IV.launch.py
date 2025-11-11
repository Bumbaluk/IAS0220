import os
from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from ament_index_python.packages import get_package_share_directory
from launch_ros.actions import Node


def generate_launch_description():

    mvt_launch_path = os.path.join(
        get_package_share_directory('machine_vision_part2'),
        'launch',
        'mvt_main.launch.py'
    )

    # Include mvt_main.launch.py
    mvt_main = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(mvt_launch_path)
    )

    # Spawn robot in Gazebo
    spawn_robot_gazebo = Node(
        package='gazebo_ros',
        executable='spawn_entity.py',
        name='spawn_ddrobot',
        namespace='ddrobot',
        output='screen',
        arguments=[
            '-topic', '/ddrobot/robot_description',
            '-entity', 'ddrobot',
            '-x', '0.6',
            '-y', '-7.5',
            '-z', '0.0',
            '-Y', '1.6'
        ]
    )

    return LaunchDescription([
        mvt_main,
        spawn_robot_gazebo
    ])
