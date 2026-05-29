from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription, DeclareLaunchArgument, TimerAction, RegisterEventHandler
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch.event_handlers import OnProcessExit
from launch_ros.substitutions import FindPackageShare
from launch_ros.actions import Node
import os
from launch_ros.parameter_descriptions import ParameterValue
from ament_index_python.packages import get_package_share_directory
import yaml
from launch.actions import EmitEvent
from launch.events import Shutdown

def generate_launch_description():
    # The path to the urdf file
    urdf_path=PathJoinSubstitution([FindPackageShare('mighty'), 'urdf', 'quadrotor.urdf.xacro'])
    parameters_path=os.path.join(get_package_share_directory('mighty'), 'config', 'benchmark.yaml')
    weight_jerk_arg = DeclareLaunchArgument('jerk_weight', default_value='0.1')
    weight_occ_arg = DeclareLaunchArgument('occ_weight', default_value='-10.0')
    output_file_id_arg = DeclareLaunchArgument('output_file_id', default_value='mighty')

    weight_jerk = LaunchConfiguration('jerk_weight')
    weight_occ = LaunchConfiguration('occ_weight')
    output_file_id = LaunchConfiguration('output_file_id')

    # Get the dict of parameters from the yaml file
    with open(parameters_path, 'r') as file:
        parameters = yaml.safe_load(file)

    # ========== Launch Arguments ==========

    env_arg = DeclareLaunchArgument(
        'env',
        default_value='hard_forest'
    )

    trajectory_csv_path = LaunchConfiguration('trajectory_csv_path')
    env = LaunchConfiguration('env')

    # ========== Include Base (Gazebo + RViz + Dyn Obstacles) ==========
    base_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            PathJoinSubstitution([
                FindPackageShare('mighty'),
                'launch',
                'base_mighty.launch.py'
            ])
        ),
        launch_arguments={
            'env': env,
            'use_dyn_obs': 'false',
            'use_gazebo_gui': 'false',
            'use_rviz': 'false'
        }.items()
    )

    # ========== Include Mapper ==========
    mapper_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            PathJoinSubstitution([
                FindPackageShare('global_mapper_ros'),
                'launch',
                'global_mapper_node.launch.py'
            ])
        ),
        launch_arguments={
            # 'namespace':'NX01',
            'quad': 'NX01',
            'depth_pointcloud_topic': 'mid360_PointCloud2',
            'pose_topic': 'state',
            'odom_topic': 'odometry/filtered_no',
            'goal_topic': '/move_base_simple/goal',
            'world_dimensions':'[60.0, 30.0, 10.0]'
        }.items()
    )

    # ========== Include Onboard ==========
    onboard_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            PathJoinSubstitution([
                FindPackageShare('mighty'),
                'launch',
                'onboard_mighty.launch.py'
            ])
        ),
        launch_arguments={
            'x': '11.0',
            'y': '0.0',
            'z': '1.0',
            'yaw': '1.57',
            'use_hardware': 'false',
            'initial_wdx': '60.0',
            'initial_wdy': '30.0',
            'initial_wdz': '10.0'
        }.items()
    )

    # ========== Analyzer Node ==========
    analyzer_node = Node(
        package='mighty',
        executable='occlusion_analysis_pcl',
        output='screen',
        parameters=[{
            'trajectory_csv_path': trajectory_csv_path,
            'initial_wdx':'30.0',
            'initial_wdy':'30.0', 
            'min_wdx':'30.0',
            'min_wdy':'30.0',
            'min_wdz':'3.0'
        }]
    )
    # parameters_path=os.path.join(get_package_share_directory('mighty'), 'config', 'benchmark.yaml')


    traj_benchmark_mighty_node = Node(
        package='mighty',
        executable='local_traj_benchmark_mighty',
        name='benchmark_mighty',
        output='screen',
        namespace='NX01',
        # Toggle prefix by setting use_gdb:=true/false
        # prefix=gdb_prefix,
        parameters=[parameters_path, {
            'jerk_weight' : ParameterValue(weight_jerk, value_type=float),
            'occ_weight': ParameterValue(weight_occ, value_type=float),
            'output_file_id': output_file_id
        }],
    )

    # Delay the mapper and onboard mighty
    delayed_mapper = TimerAction(
        period=20.0,
        actions=[mapper_launch]
    )
    delayed_onboard = TimerAction(
        period=20.0,
        actions=[onboard_launch]
    )
    # Delay analyzer start (important)
    delayed_analyzer = TimerAction(
        period=40.0,   # adjust as needed
        actions=[analyzer_node]
    )

    delayed_benchmark = TimerAction(
        period= 40.0,
        actions=[traj_benchmark_mighty_node]
    )

    # ========== Auto Shutdown When Analyzer Exits ==========
    shutdown_on_exit = RegisterEventHandler(
        OnProcessExit(
            target_action=traj_benchmark_mighty_node,
            on_exit=[
                EmitEvent(event=Shutdown(reason='Benchmark finished'))
            ]
        )
    )

    return LaunchDescription([
        env_arg,
        weight_jerk_arg,
        weight_occ_arg,
        output_file_id_arg,
        base_launch,
        delayed_mapper, #mapper_launch,
        delayed_onboard, #onboard_launch,
        delayed_benchmark,
        shutdown_on_exit
    ])
