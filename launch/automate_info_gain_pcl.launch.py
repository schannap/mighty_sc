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
from launch.actions import ExecuteProcess

def generate_launch_description():
    # The path to the urdf file
    urdf_path=PathJoinSubstitution([FindPackageShare('mighty'), 'urdf', 'quadrotor.urdf.xacro'])
    parameters_path=os.path.join(get_package_share_directory('mighty'), 'config', 'benchmark.yaml')

    # Get the dict of parameters from the yaml file
    with open(parameters_path, 'r') as file:
        parameters = yaml.safe_load(file)
    
    # Extract specific node parameters
    parameters = parameters['benchmark_mighty']['ros__parameters']
    
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
    world_dim = f"[{parameters['initial_wdx']}, {parameters['initial_wdy']}, {parameters['initial_wdz']}]"
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
            'world_dimensions':world_dim
        }.items()
    )

    # ========== Include Onboard ==========
    onboard_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            PathJoinSubstitution([
                FindPackageShare('mighty'),
                'launch',
                'onboard_mighty_benchmark.launch.py'
            ])
        ),
        launch_arguments={
            'x': '11.0',
            'y': '0.0',
            'z': '1.0',
            'yaw': '1.57'
        }.items()
    )

    # ========== Analyzer Node ==========
    analyzer_node = Node(
        package='mighty',
        executable='occlusion_analysis_pcl',
        output='screen',
        parameters=[parameters_path, {
            'trajectory_csv_path': trajectory_csv_path,
            'file_identifier': ParameterValue(file_identifier, value_type=str)
        }]
    )

    # publish a goal before analyzer starts to ensure the mapper has a goal to work with and generate the map
    goal_pub = ExecuteProcess(
        cmd=[
            'ros2', 'topic', 'pub', '--once',
            '/NX01/goal',
            'dynus_interfaces/msg/Goal',
            "{header: {frame_id: 'map'}, "
            "p: {x: 2.0, y: 0.0, z: 1.0}, "
            "v: {x: 0.5, y: 0.0, z: 0.0}, "
            "a: {x: 0.0, y: 0.0, z: 0.0}, "
            "j: {x: 0.0, y: 0.0, z: 0.0}, "
            "yaw: 0.0, dyaw: 0.0}"
        ],
        output='screen'
)
    # Delay the mapper and onboard mighty
    delayed_mapper = TimerAction(
        period=10.0,
        actions=[mapper_launch]
    )
    delayed_onboard = TimerAction(
        period=10.0,
        actions=[onboard_launch]
    )

    delayed_goal = TimerAction(
    period=15.0,
    actions=[goal_pub]
    )

    # Delay analyzer start (important)
    delayed_analyzer = TimerAction(
        period=25.0,   # adjust as needed
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
        delayed_goal,
        delayed_analyzer,
        shutdown_on_exit
    ])
