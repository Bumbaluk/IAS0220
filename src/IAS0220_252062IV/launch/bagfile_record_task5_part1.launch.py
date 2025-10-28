import launch
from launch_ros.actions import Node
from launch.actions import ExecuteProcess
from ament_index_python.packages import get_package_share_directory
import os


def generate_launch_description():

    # Get the package directory for your package (where bags will be saved)
    pkg_dir = get_package_share_directory('ias0220_sensors')

    # Create path for bags folder
    bags_folder = os.path.join(pkg_dir, 'bags')
    if not os.path.exists(bags_folder):
        os.makedirs(bags_folder)

    # Full path for recorded bag file
    bag_file_path = os.path.join(bags_folder, 'recorded')

    return launch.LaunchDescription([

        # 1️⃣ Static transform publisher: map -> imu_link
        Node(
            package='tf2_ros',
            executable='static_transform_publisher',
            name='static_tf_pub',
            arguments=['0', '0', '0', '0', '0', '0', 'map', 'imu_link'],
            output='screen'
        ),

        # 2️⃣ Serial interface node (reads Arduino sensors)
        Node(
            package='ias0220_sensors',
            executable='serial_interface.py',
            name='serial_interface',
            output='screen'
        ),

        # 3️⃣ ros2 bag record all topics
        ExecuteProcess(
            cmd=['ros2', 'bag', 'record', '-o', bag_file_path, '-a'],
            output='screen'
        )
    ])
