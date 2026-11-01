#!/bin/bash

# 确保遇到错误直接退出
set -e

# 默认参数
MODE="bag"        # 默认模式：bag
INPUT_PCD=""      # 仅在 pcd 模式下有效

# 解析命令行参数
while [[ "$#" -gt 0 ]]; do
    case $1 in
        -m|--mode) MODE="$2"; shift ;;       # -m bag 或 -m pcd
        -f|--file) INPUT_PCD="$2"; shift ;;  # -f /path/to/xxx.pcd
        *) echo "未知参数: $1"; echo "用法: bash make_map.sh -m [bag|pcd] [-f 指定pcd路径]"; exit 1 ;;
    esac
    shift
done

# 动态获取路径
SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" &>/dev/null && pwd)"
PKG_DIR="$(dirname "$SCRIPT_DIR")"
WS_DIR="$(dirname "$PKG_DIR")"
FINAL_PCD="$WS_DIR/final_map.pcd"

echo "======================================"
echo "🚀 开始构建 2D 栅格地图，当前模式: $MODE"
echo "======================================"

if [ "$MODE" == "bag" ]; then
    echo "[模式 A] 正在从最新的 Rosbag 中提取点云..."
    python3 "$SCRIPT_DIR/scripts/extract_last_pcd.py"

elif [ "$MODE" == "pcd" ]; then
    echo "[模式 B] 正在准备 FAST-LIO 生成的点云..."
    
    # 如果没指定文件，默认去 ros2_ws 根目录下找 test.pcd (前提是你改了 yaml)
    if [ -z "$INPUT_PCD" ]; then
        DEFAULT_FASTLIO_PCD="$WS_DIR/test.pcd"
        if [ -f "$DEFAULT_FASTLIO_PCD" ]; then
            INPUT_PCD="$DEFAULT_FASTLIO_PCD"
        else
            echo "❌ 错误: 找不到默认的 $DEFAULT_FASTLIO_PCD"
            echo "请通过 -f 参数指定点云路径，例如: bash make_map.sh -m pcd -f /home/mxw/xxx.pcd"
            exit 1
        fi
    fi
    
    # 检查文件是否存在并复制
    if [ -f "$INPUT_PCD" ]; then
        echo "使用点云: $INPUT_PCD"
        cp "$INPUT_PCD" "$FINAL_PCD"
    else
        echo "❌ 错误: 找不到指定的点云文件 $INPUT_PCD"
        exit 1
    fi

else
    echo "❌ 错误: 未知模式 $MODE。请使用 -m bag 或 -m pcd"
    exit 1
fi

# ---------------------------------------------------------
# 公共步骤：不管是哪种模式，最终都有了 final_map.pcd，直接转栅格
# ---------------------------------------------------------
echo "[最终步骤] 正在转换为 2D 栅格地图..."
python3 "$SCRIPT_DIR/scripts/pcd_to_grid.py"

# ---------------------------------------------------------
# 地图备份防覆盖
# ---------------------------------------------------------
echo "正在备份本次生成的地图..."
TIMESTAMP=$(date +"%Y%m%d_%H%M%S")
MAP_BACKUP_DIR="$HOME/my_maps"
mkdir -p "$MAP_BACKUP_DIR"

cp "$WS_DIR/my_map.pgm" "$MAP_BACKUP_DIR/map_${TIMESTAMP}.pgm"
cp "$WS_DIR/my_map.yaml" "$MAP_BACKUP_DIR/map_${TIMESTAMP}.yaml"
cp "$FINAL_PCD" "$MAP_BACKUP_DIR/cloud_${TIMESTAMP}.pcd"

echo "======================================"
echo "🎉 地图构建完成！"
echo "当前地图: $WS_DIR/my_map.pgm"
echo "备份位置: $MAP_BACKUP_DIR (时间戳: $TIMESTAMP)"
echo "======================================"