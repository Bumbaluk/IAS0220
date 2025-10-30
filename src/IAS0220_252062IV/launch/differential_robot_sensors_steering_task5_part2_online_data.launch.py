import os
from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch_ros.actions import Node
from ament_index_python.packages import get_package_share_directory


def generate_launch_description():

    # Package paths
    pkg_own = get_package_share_directory('IAS0220_252062IV')
    pkg_gazebo = get_package_share_directory('setup_gazebo_ias0220')

    # URDF and Bag paths
    xacro_file = os.path.join(pkg_own, 'urdf',
                              'differential_robot_simu_task4_part2.urdf')

    # Launch Gazebo simulation
    gazebo_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(pkg_gazebo, 'launch', 'gazebo.launch.py')
        ),
        launch_arguments={'xacro_file': xacro_file}.items()
    )
    # serial interface
    serial_interface_node = Node(
        package='ias0220_sensors',
        executable='serial_interface',
        name='serial_interface',
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
        serial_interface_node,
        steering_node
    ])
