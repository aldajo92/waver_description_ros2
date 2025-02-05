import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch_ros.actions import Node
from launch.substitutions import Command, LaunchConfiguration

def generate_launch_description():
    package_name = 'waver_description'
    urdf_file = 'waver.xacro'
    rviz_config_file = 'config.rviz'

    urdf_file_path = os.path.join(
        get_package_share_directory(package_name),
        'urdf',
        urdf_file
    )

    rviz_config_file_path = os.path.join(
        get_package_share_directory(package_name),
        'rviz',
        rviz_config_file
    )

    robot_state_publisher = Node(
        package='robot_state_publisher',
        executable='robot_state_publisher',
        name='robot_state_publisher',
        output='screen',
        parameters=[{'robot_description': Command(['xacro ', urdf_file_path])}]
    )

    joint_state_publisher_gui = Node(
        package='joint_state_publisher_gui',
        executable='joint_state_publisher_gui',
        output='screen'
    )

    rviz = Node(
        package='rviz2',
        executable='rviz2',
        name='rviz2',
        output='screen',
        arguments=['-d', rviz_config_file_path]
    )

    spawn_entity_node = Node(
        package='gazebo_ros',
        executable='spawn_entity.py',
        arguments=['-entity', 'waver', '-file', urdf_file_path, '-x', '1.15', '-y', '-0.8', '-z', '0.5'],
        output='screen'
    )

    return LaunchDescription([
        DeclareLaunchArgument('use_sim_time', default_value='false', description='Use simulation (Gazebo) clock'),
        robot_state_publisher,
        joint_state_publisher_gui,
        rviz,
        spawn_entity_node
    ])