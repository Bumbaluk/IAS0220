import launch
from launch.actions import DeclareLaunchArgument, ExecuteProcess
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch_ros.actions import Node
from ament_index_python.packages import get_package_share_directory
import os


def generate_launch_description():

    # Declare the argument 'which_bag'
    declare_which_bag = DeclareLaunchArgument(
        'which_bag',
        default_value='bag0',
        description='Which bag to use for playback'
    )

    # Get the argument value
    which_bag = LaunchConfiguration('which_bag')

    # Get the package share directory
    pkg_dir = get_package_share_directory('ias0220_sensors')

    # Path to the bags folder
    bag_path = PathJoinSubstitution([pkg_dir, 'bags', which_bag])

    # Path to RViz configuration
    rviz_config_file = os.path.join(pkg_dir, 'rviz', 'task5_part1.rviz')

    # Launch ros2 bag play
    bag_replay = ExecuteProcess(
        cmd=['ros2', 'bag', 'play', '-r', '1', '-l', bag_path],
        output='screen'
    )

    # Launch RViz with your saved configuration
    rviz_node = Node(
        package='rviz2',
        executable='rviz2',
        name='rviz2',
        arguments=['-d', rviz_config_file],
        output='screen'
    )

    return launch.LaunchDescription([
        declare_which_bag,
        bag_replay,
        rviz_node
    ])
