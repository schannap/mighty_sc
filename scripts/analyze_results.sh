#!/bin/bash

# TRAJ_DIR=/home/kkondo/code/mighty_ws/src/mighty/benchmark_data/multi_thread/traj_dump/mighty_N5
TRAJ_DIR=/home/kkondo/code/mighty_ws/src/mighty/benchmark_data/multi_thread/traj_dump/test_endpoint_18_mini

source /home/kkondo/code/mighty_ws/install/setup.bash
source /home/kkondo/code/decomp_ws/install/setup.bash
source /usr/share/gazebo/setup.bash

# for traj in ${TRAJ_DIR}/*.csv; do
#     echo "Running $traj"
#     ros2 launch mighty automate_info_gain_pcl.launch.py trajectory_csv_path:=$traj file_identifier:=nominal_$(basename $traj .csv)
#     echo "Finished $traj"
#     sleep 5
# done

for traj in ${TRAJ_DIR}/*.csv; do
    base=$(basename "$traj")
    echo $traj

    # Extract the 3-digit number before .mysco2.csv
    num=$(echo "$base" | grep -oE '[0-9]{3}\.mysco2\.csv' | grep -oE '[0-9]{3}')

    # Skip if extraction failed
    if [ -z "$num" ]; then
        echo "Skipping (no number found): $base"
        continue
    fi

    # Convert to integer (handles leading zeros correctly)
    num=$((10#$num))

    # Skip if > 50
    if [ "$num" -gt 50 ]; then
        echo "Skipping $base (index $num > 50)"
        continue
    fi

    echo "Running $traj"

    ros2 launch mighty automate_info_gain_pcl.launch.py \
        trajectory_csv_path:="$traj" \
        file_identifier:=18_filtered_vis/$(basename "$traj" .csv)

    echo "Finished $traj"
    sleep 5
done