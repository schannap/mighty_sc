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

    parameters_path=os.path.join(get_package_share_directory('mighty'), 'config', 'benchmark_ns.yaml')
    output_file_id_arg = DeclareLaunchArgument('output_file_id', default_value='mighty')
    output_file_id = LaunchConfiguration('output_file_id')

    traj_benchmark_mighty_node = Node(
        package='mighty',
        executable='local_traj_benchmark_mighty',
        name='benchmark_mighty',
        output='screen',
        namespace='NX01',
        # Toggle prefix by setting use_gdb:=true/false
        # prefix=gdb_prefix,
        parameters=[parameters_path, {
            'output_file_id': output_file_id
        }],
    )

    return LaunchDescription([
        output_file_id_arg,
        traj_benchmark_mighty_node
    ])
