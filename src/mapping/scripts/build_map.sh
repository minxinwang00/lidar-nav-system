#!/bin/bash

# 确保遇到错误直接退出
set -e

echo "======================================"
echo "🚀 开始自动构建 2D 栅格地图..."
echo "======================================"

# 获取当前脚本所在的绝对目录
SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" &>/dev/null && pwd)"

# 步骤 1：从最新的 bag 中提取 3D 点云
echo "[1/2] 正在提取 3D 点云..."
python3 "$SCRIPT_DIR/scripts/extract_last_pcd.py"

# 步骤 2：将 3D 点云降维并转换为 2D 栅格地图
echo "[2/2] 正在转换为 2D 栅格地图..."
python3 "$SCRIPT_DIR/scripts/pcd_to_grid.py"

# 步骤 3：自动备份生成的地图，防止覆盖
echo "正在备份本次生成的地图..."
TIMESTAMP=$(date +"%Y%m%d_%H%M%S")
MAP_BACKUP_DIR="$HOME/my_maps"
mkdir -p "$MAP_BACKUP_DIR"

# 推导 ros2_ws 的路径 (nav_task_repo 的上一级)
PKG_DIR="$(dirname "$SCRIPT_DIR")"
WS_DIR="$(dirname "$PKG_DIR")"

# 复制并重命名
cp "$WS_DIR/my_map.pgm" "$MAP_BACKUP_DIR/map_${TIMESTAMP}.pgm"
cp "$WS_DIR/my_map.yaml" "$MAP_BACKUP_DIR/map_${TIMESTAMP}.yaml"

echo "======================================"
echo "🎉 地图构建完成！"
echo "当前地图: $WS_DIR/my_map.pgm"
echo "备份位置: $MAP_BACKUP_DIR/map_${TIMESTAMP}.pgm"
echo "======================================"