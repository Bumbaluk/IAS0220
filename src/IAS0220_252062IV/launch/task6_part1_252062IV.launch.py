from launch import LaunchDescription
from launch_ros.actions import Node
from ament_index_python.packages import get_package_share_directory
import os


def generate_launch_description():
    package_name = 'IAS0220_252062IV'
    rviz_config_path = os.path.join(
        get_package_share_directory(package_name),
        'config',
        'task6_part1_252062IV.rviz'
    )

    publisher_node = Node(
        package='IAS0220_252062IV',
        executable='image_publisher',
        name='image_publisher',
        output='screen',
    )

    calibration_node = Node(
        package='IAS0220_252062IV',
        executable='camera_calibration',
        name='camera_calibration',
        output='screen',
    )

    image_proc = Node(
        package='image_proc',
        executable='image_proc',
        name='image_processor',
        remappings=[
                ('image', '/image_raw'),
                ('camera_info', '/camera_info'),
                ('image_processed', '/image_processed'),
            ],
        output='screen',
    )
    rvizz_node = Node(
        package='rviz2',
        executable='rviz2',
        name='rviz2',
        arguments=['-d', rviz_config_path],
        output='screen',
    )

    return LaunchDescription([
        publisher_node,
        calibration_node,
        image_proc,
        rvizz_node
    ])
