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


    traj_benchmark_mighty_node = Node(
        package='mighty',
        executable='local_traj_benchmark_mighty',
        name='local_traj_benchmark_mighty',
        output='screen',
        namespace='NX01',
        # Toggle prefix by setting use_gdb:=true/false
        # prefix=gdb_prefix,
        parameters=[parameters_path],
    )

    return LaunchDescription([
        traj_benchmark_mighty_node,
    ])
