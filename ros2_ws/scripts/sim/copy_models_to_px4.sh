#!/usr/bin/env bash
set -e

TOP_DIR=$(git rev-parse --show-toplevel)

# 源模型目录
SRC_DIR=${TOP_DIR}/ros2_ws/src/simulation

# 目标 PX4 模型目录
DST_DIR=${TOP_DIR}/px4/PX4-Autopilot/Tools/simulation/gazebo-classic/sitl_gazebo-classic

cp -r ${SRC_DIR}/worlds/*.world ${DST_DIR}/worlds
cp -r ${SRC_DIR}/models/* ${DST_DIR}/models

echo "✨ 所有模型已移动完成!"
