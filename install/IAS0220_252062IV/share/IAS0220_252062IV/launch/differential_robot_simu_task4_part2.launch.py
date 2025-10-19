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
                              "differential_robot_simu_task4_part2.urdf")
    rviz_config_file = os.path.join(pkg_path, "config",
                                    "task4_config.rviz")

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

    # Encoders
    encoders_node = Node(
        package="encoders_pkg",
        executable="encoders_node",
        name="encoders_node",
        output="screen",
        parameters=[{'noisy': False}]
    )
    # Odometry
    odometry_node = Node(
        package=package_name,
        executable="odometry",
        name="odometry",
        output="screen"
    )

    # Static transform node
    static_tf_node = Node(
        package='tf2_ros',
        executable='static_transform_publisher',
        name='map_to_odom_broadcaster',
        arguments=['0', '0', '0', '0', '0', '0', 'map', 'odom']
    )

    left_wheel_tf = Node(
        package='tf2_ros',
        executable='static_transform_publisher',
        name='left_wheel_tf',
        arguments=[
            '0', '0.175', '0', '0', '0', '0', 'base_link', 'left_wheel'],
        output='screen'
    )

    right_wheel_tf = Node(
        package='tf2_ros',
        executable='static_transform_publisher',
        name='right_wheel_tf',
        arguments=[
            '0', '-0.175', '0', '0', '0', '0', 'base_link', 'right_wheel'],
        output='screen'
    )

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
        output="screen",
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
        static_tf_node,
        left_wheel_tf,
        right_wheel_tf,
        encoders_node,
        odometry_node,
        rviz_node,
        teleop_node,
        rqt_graph_node,
    ])
