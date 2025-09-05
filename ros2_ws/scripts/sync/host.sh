#!/bin/bash
#密码sunrise
#请安装 sudo apt install sshpass

TOP_DIR=$(git rev-parse --show-toplevel)

# 检查是否提供了 IP 参数
if [ -z "$1" ]; then
    echo "❌ 用法: $0 <目标IP地址>"
    exit 1
fi

TARGET_IP="$1"
TARGET_USER="sunrise"
TARGET_PATH="/home/${TARGET_USER}/ros2_px4/ros2_ws"
SOURCE_PATH="${TOP_DIR}/ros2_ws/"

sshpass -p "sunrise" ssh ${TARGET_USER}@${TARGET_IP} "mkdir -p ${TARGET_PATH}"

echo "开始同步到 $TARGET_USER@$TARGET_IP:$TARGET_PATH"

sshpass -p "sunrise" rsync -aAXv \
    --exclude="/.cache/" \
    --exclude="/.venv/" \
    --exclude="/build/" \
    --exclude="/install/" \
    --exclude="/log/" \
    --exclude="/scripts/" \
    --exclude="/tmp/" \
    --exclude="/arm/" \
    --exclude="/tools/" \
    --exclude="/src/simulation" \
    "$SOURCE_PATH" \
    "scripts/sync/ros2_ws.sh" \
    "scripts/target/yaml_update.sh" \
    "$TARGET_USER@$TARGET_IP:$TARGET_PATH"

echo "✅ 同步完成"