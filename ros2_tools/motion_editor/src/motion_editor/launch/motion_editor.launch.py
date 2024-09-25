from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
import os
from ament_index_python.packages import get_package_share_directory


def generate_launch_description():
    package_share_directory = get_package_share_directory('motion_editor')
    rviz_config_file = os.path.join(
        package_share_directory, 'rviz', 'default.rviz')

    return LaunchDescription([
        DeclareLaunchArgument(
            'data_path',
            default_value='',  # 设置默认值
        ),

        Node(
            package='motion_editor',
            executable='motion_editor',
            name='motion_editor',
            parameters=[{'data_path': LaunchConfiguration('data_path')}],
        ),
        Node(
            package='rviz2',
            executable='rviz2',
            name='rviz2',
            output='screen',
            arguments=[
                '-d', rviz_config_file] if os.path.exists(rviz_config_file) else []
        )
    ])
