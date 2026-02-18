from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
import os

from ament_index_python.packages import get_package_share_directory


def generate_launch_description():

    nominal_dir_arg = DeclareLaunchArgument(
        'nominal_dir',
        description='Directory containing nominal trajectory CSV files'
    )

    modified_dir_arg = DeclareLaunchArgument(
        'modified_dir',
        description='Directory containing modified trajectory CSV files'
    )

    nominal_dir = LaunchConfiguration('nominal_dir')
    modified_dir = LaunchConfiguration('modified_dir')

    pkg_share = get_package_share_directory('mighty')

    visualizer_node = Node(
        package='mighty',
        executable='plot_comparisons',
        name='plot_comparisons',
        arguments=[nominal_dir, modified_dir],
        output='screen'
    )

    return LaunchDescription([
        nominal_dir_arg,
        modified_dir_arg,
        visualizer_node,
    ])