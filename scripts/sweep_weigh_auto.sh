#!/bin/bash

source /home/kkondo/code/mighty_ws/install/setup.bash
source /home/kkondo/code/decomp_ws/install/setup.bash
source /usr/share/gazebo/setup.bash

# Log-scale exponents
# JERK_EXP_MIN=-3
# JERK_EXP_MAX=-1
# OCC_EXP_MIN=-2
# OCC_EXP_MAX=2
JERK_EXP_MIN=-1
JERK_EXP_MAX=-1
OCC_EXP_MIN=2
OCC_EXP_MAX=2

# Generate jerk weights (positive log scale)
JERK_WEIGHTS=()
for exp in $(seq $JERK_EXP_MIN 1 $JERK_EXP_MAX); do
  val=$(awk "BEGIN {print 10^$exp}")
  JERK_WEIGHTS+=($val)
done

# Generate occ weights (negative log scale)
OCC_WEIGHTS=()
for exp in $(seq $OCC_EXP_MIN 1 $OCC_EXP_MAX); do
  mag=$(awk "BEGIN {print 10^$exp}")
  OCC_WEIGHTS+=($(awk "BEGIN {print -$mag}"))
done

OCC_WEIGHTS=(0)

# Print a list of all the jerk weights
# Print a list of all the occlusion weights

# Print jerk weights
echo "Generated jerk weights:"
for w in "${JERK_WEIGHTS[@]}"; do
  echo "$w"
done

# Print occlusion weights
echo "Generated occlusion weights:"
for w in "${OCC_WEIGHTS[@]}"; do
  echo "$w"
done


counter=0
# Sweep
for jerk in "${JERK_WEIGHTS[@]}"; do
  for occ in "${OCC_WEIGHTS[@]}"; do

    echo "==============================================="
    echo "Running jerk_weight=$jerk occ_weight=$occ"
    echo "==============================================="


    ros2 launch mighty sweep_weights.launch.py \
        jerk_weight:=$jerk \
        occ_weight:=$occ \
        output_file_id:=sweep_test_nominal_$counter \
        env:=big_obstacle

    if [ $? -ne 0 ]; then
        echo "Launch failed. Exiting."
        exit 1
    fi

    ((counter++))

    sleep 5
    

  done
done