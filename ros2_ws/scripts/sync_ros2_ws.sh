#!/bin/bash

TOP_DIR=$(git rev-parse --show-toplevel)

# 检查是否提供了 IP 参数
if [ -z "$1" ]; then
    echo "❌ 用法: $0 <目标IP地址>"
    exit 1
fi

TARGET_IP="$1"
TARGET_USER="sunrise"
TARGET_PATH="/home/sunrise/Desktop/ros2_px4"
SOURCE_PATH="${TOP_DIR}/ros2_ws/"

echo "开始同步到 $TARGET_USER@$TARGET_IP:$TARGET_PATH"

sudo rsync -aAXv \
    --exclude="/.cache/" \
    --exclude="/.venv/" \
    --exclude="/build/" \
    --exclude="/install/" \
    --exclude="/log/" \
    --exclude="/tmp/" \
    --exclude="/scripts/" \
    --exclude="/src/simulation" \
    "$SOURCE_PATH" \
    "$TARGET_USER@$TARGET_IP:$TARGET_PATH"

echo "✅ 同步完成"
