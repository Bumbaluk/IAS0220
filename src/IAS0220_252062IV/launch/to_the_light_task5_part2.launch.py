import os
from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription
from launch.actions import ExecuteProcess
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import PathJoinSubstitution
from ament_index_python.packages import get_package_share_directory
from launch_ros.actions import Node


def generate_launch_description():
    # Declare Arguments

    # Package paths
    pkg_own = get_package_share_directory('IAS0220_252062IV')
    pkg_gazebo = get_package_share_directory('setup_gazebo_ias0220')

    # URDF and Bag paths
    xacro_file = os.path.join(pkg_own, 'urdf',
                              'differential_robot_simu_task4_part2.urdf')
    bag_path = PathJoinSubstitution([pkg_own, 'bags',
                                     'rosbag_to_light_task5_part2'])

    # Launch Gazebo simulation
    gazebo_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(pkg_gazebo, 'launch', 'gazebo.launch.py')
        ),
        launch_arguments={'xacro_file': xacro_file}.items()
    )

    # Launch rosbag play
    bag_replay = ExecuteProcess(
        cmd=['ros2', 'bag', 'play', '-r', '1.0', '-l', bag_path],
        output='screen'
    )

    # Launch steering node
    steering_node = Node(
        package='IAS0220_252062IV',
        executable='steering_node',
        name='steering_node',
        output='screen'
    )

    return LaunchDescription([
        gazebo_launch,
        bag_replay,
        steering_node
    ])
