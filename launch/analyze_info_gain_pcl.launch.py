from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
from launch.conditions import IfCondition

from launch.actions import ExecuteProcess
from launch_ros.parameter_descriptions import ParameterValue


def generate_launch_description():

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
        name='occlusion_analysis_pcl_node',
        output='screen',
        parameters=[{
            'trajectory_csv_path': traj_path,
            'initial_wdx':'60.0',
            'initial_wdy':'30.0', 
            'min_wdx':'60.0',
            'min_wdy':'30.0',
            'min_wdz':'3.0',
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
