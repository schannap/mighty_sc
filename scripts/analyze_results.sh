#!/bin/bash

# TRAJ_DIR=/home/kkondo/code/mighty_ws/src/mighty/benchmark_data/multi_thread/traj_dump/mighty_N5
TRAJ_DIR=/home/kkondo/code/mighty_ws/src/mighty/benchmark_data/multi_thread/traj_dump/mighty_N5with_occ_

source /home/kkondo/code/mighty_ws/install/setup.bash
source /home/kkondo/code/decomp_ws/install/setup.bash
source /usr/share/gazebo/setup.bash

for traj in ${TRAJ_DIR}/*.csv; do
    echo "Running $traj"
    ros2 launch mighty automate_info_gain_pcl.launch.py trajectory_csv_path:=$traj
    echo "Finished $traj"
    sleep 5
done
