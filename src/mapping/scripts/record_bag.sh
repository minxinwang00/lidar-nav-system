#!/bin/bash

# 1. 统一存放的目录
BAG_BASE_DIR="$HOME/my_bags" 
mkdir -p "$BAG_BASE_DIR"

# 2. 生成当前时间戳
TIMESTAMP=$(date +"%Y%m%d_%H%M%S")
BAG_NAME="bag_${TIMESTAMP}"

echo "======================================"
echo "🎥 开始录制数据..."
echo "保存路径: ${BAG_BASE_DIR}/${BAG_NAME}"
echo "（建图结束后，请按 Ctrl+C 停止录制）"
echo "======================================"

# 3. 录制所有必要话题（使用 \ 换行）
ros2 bag record -o "${BAG_BASE_DIR}/${BAG_NAME}" \
    /cloud_registered \
    /livox/lidar \
    /livox/imu \
    /tf \
    /tf_static

# 4. 结束提示
echo "======================================"
echo "✅ 录制已停止！"
echo "数据已保存在: ${BAG_BASE_DIR}/${BAG_NAME}"
echo "接下来你可以运行 build_map.sh 生成地图了。"
echo "======================================"