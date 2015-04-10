#!/usr/bin/env bash

rosrun pcl_ros pointcloud_to_pcd /input:=$2 &
PCD_PID=$!

N_PCDS=0

while [ $N_PCDS -lt $1 ]; do
  N_PCDS="$(ls -l | wc -l)"
  echo -e "\033[3F$N_PCDS / $1 .pcd files."
  sleep .05
done

kill $PCD_PID

wait $PCD_PID
