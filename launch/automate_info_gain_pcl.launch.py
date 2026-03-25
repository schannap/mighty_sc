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

    # Get the dict of parameters from the yaml file
    with open(parameters_path, 'r') as file:
        parameters = yaml.safe_load(file)

    # Extract specific node parameters
    # parameters = parameters['mighty_node']['ros__parameters']
    
    # ========== Launch Arguments ==========
    traj_arg = DeclareLaunchArgument(
        'trajectory_csv_path',
        description='Absolute path to trajectory CSV file'
    )

    env_arg = DeclareLaunchArgument(
        'env',
        default_value='big_obstacle'
    )

    file_identifier_arg = DeclareLaunchArgument(
        'file_identifier',
        default_value='test'
    )   

    trajectory_csv_path = LaunchConfiguration('trajectory_csv_path')
    env = LaunchConfiguration('env')
    file_identifier = LaunchConfiguration('file_identifier')

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

    # mighty_node = Node(
    #             package='mighty',
    #             executable='mighty',
    #             name='mighty_node',
    #             namespace='NX01',
    #             output='screen',
    #             emulate_tty=True,
    #             parameters=[parameters],
    #             remappings=[('lidar_cloud_in', 'mid360_PointCloud2'),
    #                         ('depth_camera_cloud_in', 'd435/depth/color/points')],
    #             # prefix='xterm -e gdb -q -ex run --args', # gdb debugging
    #             # arguments=['--ros-args', '--log-level', 'error']
    # )

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
            'initial_wdx':'60.0',
            'initial_wdy':'30.0',
            'initial_wdz':'10.0'
        }.items()
    )

    # ========== Analyzer Node ==========
    analyzer_node = Node(
        package='mighty',
        executable='occlusion_analysis_pcl',
        output='screen',
        parameters=[{
            'trajectory_csv_path': trajectory_csv_path,
            'initial_wdx':'60.0',
            'initial_wdy':'30.0', 
            'min_wdx':'60.0',
            'min_wdy':'30.0',
            'min_wdz':'3.0',
            'file_identifier': ParameterValue(file_identifier, value_type=str)
        }]
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

    # ========== Auto Shutdown When Analyzer Exits ==========
    shutdown_on_exit = RegisterEventHandler(
        OnProcessExit(
            target_action=analyzer_node,
            on_exit=[
                EmitEvent(event=Shutdown(reason='Analyzer finished'))
            ]
        )
    )

    return LaunchDescription([
        traj_arg,
        env_arg,
        file_identifier_arg,
        base_launch,
        delayed_mapper, #mapper_launch,
        delayed_onboard, #onboard_launch,
        # mighty_node
        delayed_analyzer,
        shutdown_on_exit
    ])
