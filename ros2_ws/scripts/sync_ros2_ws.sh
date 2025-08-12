#!/bin/bash
# 密码sunrise

TOP_DIR=$(pwd)

if [ -z "$1" ]; then
    echo "❌ 用法: $0 <目标IP地址>"
    exit 1
fi

TARGET_IP="$1"
TARGET_USER="sunrise"
TARGET_PATH="/home/${TARGET_USER}/ros2_px4"
SOURCE_PATH="${TOP_DIR}/install/"

echo "开始同步到 $TARGET_USER@$TARGET_IP:$TARGET_PATH"

# 本地路径检测
if [ ! -d "$SOURCE_PATH" ]; then
    echo "❌ 本地目录不存在: $SOURCE_PATH"
    exit 1
fi

# 远端目录检测（通过 ssh 远程执行 mkdir -p，保证目录存在）
ssh $TARGET_USER@$TARGET_IP "mkdir -p $TARGET_PATH/install"

# 同步
rsync -aAXv \
    "$SOURCE_PATH" \
    "$TARGET_USER@$TARGET_IP:$TARGET_PATH/install"

echo "✅ 同步完成"
