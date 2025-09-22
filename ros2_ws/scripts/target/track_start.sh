#!/bin/bash

DIR="/home/sunrise/ros2_px4/ros2_ws/logs"

if [ ! -d "${DIR}" ]; then
    echo "目录 ${DIR} 不存在，正在创建..."
    mkdir -p "${DIR}"
fi

COUNT=$(find "${DIR}" -maxdepth 1 -type f | wc -l)
echo "目录 $DIR 的文件数（不含子目录）: ${COUNT}"

ros2 launch mission_planner visual_track.launch.py 2>&1 | tee ${DIR}/track_${COUNT}.log
