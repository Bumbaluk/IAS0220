import launch
from launch_ros.actions import Node
from launch.actions import ExecuteProcess
import os


def generate_launch_description():

    # Get the package directory for your package (where bags will be saved)
    # Create path for bags folder
    bags_folder = os.path.expanduser('~/ros2_ws/src/IAS0220_252062IV/bags')

    if not os.path.exists(bags_folder):
        os.makedirs(bags_folder)

    # Full path for recorded bag file
    bag_file_path = os.path.join(bags_folder, 'recorded')

    return launch.LaunchDescription([

        # Static transform publisher: map -> imu_link
        Node(
            package='tf2_ros',
            executable='static_transform_publisher',
            name='static_tf_pub',
            arguments=['0', '0', '0', '0', '0', '0', 'map', 'imu_link'],
            output='screen'
        ),

        # Serial interface node
        Node(
            package="ias0220_sensors",
            executable="serial_interface",
            name="serial_interface",
            output="screen",
        ),

        # ros2 bag record all topics
        ExecuteProcess(
            cmd=['ros2', 'bag', 'record', '-o', bag_file_path, '-a'],
            output='screen',
            on_exit=launch.actions.Shutdown()
        )
    ])
