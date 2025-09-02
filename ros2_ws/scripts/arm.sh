#!/bin/bash

# 获取当前用户
CURRENT_USER=$USER
CPU_CORES=$(nproc)

echo "当前用户: ${CURRENT_USER}, CPU 核心数量: ${CPU_CORES}"

BUILD_DIR="./arm"

export CMAKE_BUILD_PARALLEL_LEVEL=${CPU_CORES}
colcon \
    --log-base ${BUILD_DIR}/log build \
    --build-base ${BUILD_DIR}/build \
    --install-base ${BUILD_DIR}/install \
    --parallel-workers ${CPU_CORES}

echo "✅构建完成"
