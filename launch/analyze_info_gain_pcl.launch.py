from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
from launch.conditions import IfCondition
import os
from launch.actions import ExecuteProcess
from launch_ros.parameter_descriptions import ParameterValue
from ament_index_python.packages import get_package_share_directory


def generate_launch_description():
    parameters_path=os.path.join(get_package_share_directory('mighty'), 'config', 'benchmark.yaml')

    traj_path_arg = DeclareLaunchArgument(
        'traj_path',
        description='Absolute path to trajectory CSV files'
    )

    use_rviz_arg = DeclareLaunchArgument(
        'use_rviz',
        default_value='true',
        description='Launch RViz automatically'
    )
    file_identifier_arg = DeclareLaunchArgument(
        'file_identifier',
        default_value='test'
    )   


    traj_path = LaunchConfiguration('traj_path')
    file_identifier = LaunchConfiguration('file_identifier')
    use_rviz = LaunchConfiguration('use_rviz')

    occlusion_analysis_pcl_node = Node(
        package='mighty',
        executable='occlusion_analysis_pcl',
        name='benchmark_mighty',
        output='screen',
        parameters=[parameters_path, {
            'trajectory_csv_path': traj_path,
            'file_identifier': ParameterValue(file_identifier, value_type=str)
        }]
    )

    rviz_node = ExecuteProcess(
        condition=IfCondition(use_rviz),
        cmd=['rviz2'],
        output='screen'
    )

    return LaunchDescription([
        traj_path_arg,
        file_identifier_arg,
        occlusion_analysis_pcl_node,
    ])
