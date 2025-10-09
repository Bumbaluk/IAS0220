import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription
from launch_ros.actions import Node
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration

package_name = "IAS0220_252062IV"


def generate_launch_description():
    # Get paths
    pkg_path = get_package_share_directory(package_name)
    xacro_file = os.path.join(pkg_path,
                              "urdf",
                              "differential_robot_simu_task4_part1.urdf")
    rviz_config_file = os.path.join(pkg_path, "rviz",
                                    "differential_robot_simu_task4_part1.rviz")

    # Declare xacro_file argument to pass to setup_gazebo_ias0220
    declare_xacro_file_arg = DeclareLaunchArgument(
        name='xacro_file',
        default_value=xacro_file,
        description='Path to robot xacro file'
    )

    # Include Gazebo launch from setup_gazebo_ias0220
    gazebo_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(get_package_share_directory('setup_gazebo_ias0220'),
                         'launch',
                         'gazebo.launch.py')
        ),
        launch_arguments={
            'xacro_file': LaunchConfiguration('xacro_file')}.items())

    # RViz
    rviz_node = Node(
        package="rviz2",
        executable="rviz2",
        name="rviz2",
        arguments=["-d", rviz_config_file],
        output="screen"
    )

    # Teleop
    teleop_node = Node(
        package="teleop_twist_keyboard",
        executable="teleop_twist_keyboard",
        name="teleop_keyboard",
        prefix="xterm -e",  # launches in terminal for key input
        output="screen",
        remappings=[('/cmd_vel', '/cmd_vel')]
    )

    # RQT Graph
    rqt_graph_node = Node(
        package='rqt_graph',
        executable='rqt_graph',
        name='rqt_graph',
        output='screen'
    )

    return LaunchDescription([
        declare_xacro_file_arg,
        gazebo_launch,
        rviz_node,
        teleop_node,
        rqt_graph_node
    ])
