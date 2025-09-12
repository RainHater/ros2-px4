#!/bin/bash
# 密码sunrise
#请安装 sudo apt install sshpass

TOP_DIR=$(pwd)

echo "当前目录: ${TOP_DIR}"

TARGET_IP="100.76.77.51"
# TARGET_IP="172.20.10.3"
TARGET_USER="sunrise"
TARGET_PASSWORD="sunrise"
TARGET_PATH="/home/${TARGET_USER}/ros2_px4/ros2_ws"
SOURCE_PATH="${TOP_DIR}/arm/install/"

echo "源文件目录: ${SOURCE_PATH}"
echo "开始同步到 ${TARGET_USER}@${TARGET_IP}:${TARGET_PATH}"

# 本地路径检测
if [ ! -d "${SOURCE_PATH}" ]; then
    echo "❌ 本地目录不存在: ${SOURCE_PATH}"
    exit 1
fi

# 远端目录检测（通过 ssh 远程执行 mkdir -p，保证目录存在）
sshpass -p "${TARGET_PASSWORD}" ssh ${TARGET_USER}@${TARGET_IP} "mkdir -p ${TARGET_PATH}/install"

# 同步
sshpass -p "${TARGET_PASSWORD}" rsync -aAXv \
    "${SOURCE_PATH}" \
    "${TARGET_USER}@${TARGET_IP}:${TARGET_PATH}/install"

sshpass -p "${TARGET_PASSWORD}" rsync -aAXv \
    "${SOURCE_PATH}/utilities/share/utilities/config/app.yaml" \
    "${SOURCE_PATH}/mipi_detect/share/mipi_detect/config/m_params.yaml" \
    "${SOURCE_PATH}/mipi_detect/share/mipi_detect/config/params.yaml" \
    "${SOURCE_PATH}/mipi_detect/share/mipi_detect/config/config.json" \
    "${TOP_DIR}/scripts/target/yaml_update.sh" \
    "${TARGET_USER}@${TARGET_IP}:${TARGET_PATH}"

echo "✅ 同步完成"
