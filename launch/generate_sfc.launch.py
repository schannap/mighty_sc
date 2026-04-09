from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
import os
from ament_index_python.packages import get_package_share_directory
import yaml
from launch_ros.actions import Node

def generate_launch_description():

    # Prefix for gdb debugging
    # -q: quiet
    # -ex run: immediately run after starting gdb
    # --args: pass remaining args as program args
    gdb_prefix = 'xterm -e gdb -q -ex run --args'

    parameters_path=os.path.join(get_package_share_directory('mighty'), 'config', 'benchmark.yaml')
    start_pos_arg = DeclareLaunchArgument('start', default_value="[2.0, 0.0, 1.0]")
    start_pos = LaunchConfiguration('start')
    goal_x_arg = DeclareLaunchArgument('goal_x', default_value='9.0')
    goal_x = LaunchConfiguration('goal_x')
    goal_y_min_arg = DeclareLaunchArgument('goal_y_min', default_value='-2.5')
    goal_y_min = LaunchConfiguration('goal_y_min')
    goal_y_max_arg = DeclareLaunchArgument('goal_y_max', default_value='2.5')
    goal_y_max = LaunchConfiguration('goal_y_max')  

    sfc_node = Node(
        package='mighty',
        executable='corridor_generator_node',
        name='benchmark_mighty',
        output='screen',
        namespace='',
        parameters=[parameters_path, {
            'start': start_pos,
            'goal_x': goal_x,
            'goal_y_min': goal_y_min,
            'goal_y_max': goal_y_max
        }],
    )

    return LaunchDescription([
        start_pos_arg,
        goal_x_arg,
        goal_y_min_arg,
        goal_y_max_arg,
        sfc_node
    ])
