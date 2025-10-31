import os
import launch
from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription, ExecuteProcess
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

    # Directory for recorded rosbag
    bags_folder = os.path.expanduser('~/ros2_ws/src/IAS0220_252062IV/bags')
    os.makedirs(bags_folder, exist_ok=True)
    bag_file_path = os.path.join(bags_folder, 'rosbag_to_light')

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

    # Record all ROS topics into rosbag_to_light.bag
    record_bag = ExecuteProcess(
        cmd=['ros2', 'bag', 'record', '-o',
             bag_file_path, '/tf', '/tf_static', '/imu', '/distance'],
        output='screen',
        on_exit=launch.actions.Shutdown()
    )

    return LaunchDescription([
        gazebo_launch,
        # static_tf_pub,
        serial_interface_node,
        steering_node,
        record_bag
    ])


'''
    # Static transform publisher (map -> imu_link)
    static_tf_pub = Node(
        package='tf2_ros',
        executable='static_transform_publisher',
        name='static_tf_pub',
        arguments=['0', '0', '0', '0', '0', '0', 'map', 'imu_link'],
        output='screen'
    )
'''
