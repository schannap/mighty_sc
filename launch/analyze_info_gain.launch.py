from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
from launch.conditions import IfCondition

from launch.actions import ExecuteProcess


def generate_launch_description():

    traj_dir_arg = DeclareLaunchArgument(
        'traj_directory',
        description='Absolute path to directory containing trajectory CSV files'
    )

    use_rviz_arg = DeclareLaunchArgument(
        'use_rviz',
        default_value='true',
        description='Launch RViz automatically'
    )

    traj_directory = LaunchConfiguration('traj_directory')
    use_rviz = LaunchConfiguration('use_rviz')

    occlusion_analysis_node = Node(
        package='mighty',
        executable='occlusion_analysis',
        name='benchmark_mighty',
        output='screen',
        parameters=[{
            'traj_directory': traj_directory
        }]
    )

    rviz_node = ExecuteProcess(
        condition=IfCondition(use_rviz),
        cmd=['rviz2'],
        output='screen'
    )

    return LaunchDescription([
        traj_dir_arg,
        occlusion_analysis_node,
    ])
