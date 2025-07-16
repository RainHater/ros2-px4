#!/bin/bash

TOP_DIR=$(git rev-parse --show-toplevel)
# echo "顶层路径: $TOP_DIR"
source /opt/ros/humble/setup.bash
source ${TOP_DIR}/ros2_ws/install/setup.sh
