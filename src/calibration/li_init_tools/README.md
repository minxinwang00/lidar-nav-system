# LI-Init 外参自动标定集成

LI-Init（仓库名 `LiDAR_IMU_Init`）用于估计：

- LiDAR → IMU 旋转 `R`
- LiDAR → IMU 平移 `T`
- LiDAR 与 IMU 时间偏移 `dt`

本包把标定流程接入现有导航系统：

```text
calibrate.launch.py
├── livox 驱动
├── li_init_manager
│   ├── ~/record_bag     录制 /livox/lidar + /livox/imu
│   ├── ~/stop_recording 结束并保存 bag
│   └── ~/apply_result   解析 LI-Init 结果 → 生成 FAST-LIO 配置
└── 可选底盘/RViz

LI-Init（外部运行）
      ↓ 结果文件
mid360_calibrated.yaml
      ↓
mapping.launch.py lio_config:=.../mid360_calibrated.yaml
navigation.launch.py lio_config:=.../mid360_calibrated.yaml
```

## 1. 安装 LI-Init

原版 `hku-mars/LiDAR_IMU_Init` 主要是 **ROS1 Melodic/Noetic** 工程；本工作空间是 ROS2 Humble，因此有两种方式：

### 方式 A：ROS2 分支/移植版

如果使用的分支包含 `ament_cmake`/`ament_python`：

```bash
bash src/calibration/li_init_tools/scripts/install_li_init.sh
```

脚本会 clone 到：

```text
src/calibration/third_party/LiDAR_IMU_Init
```

并尝试 `colcon build`。

### 方式 B：原版 ROS1 + 独立环境/容器

原版在 ROS1 中运行。推荐：

1. 录制 ROS2 bag；
2. 转成 ROS1 bag：

```bash
pip install rosbags
rosbags-convert ~/nav_system_ws/calib_bags/li_init_xxx \
  --dst /tmp/li_init_ros1.bag
```

3. 在 ROS1 Noetic 环境/容器中：

```bash
roscore
rosbag play /tmp/li_init_ros1.bag --clock
roslaunch li_init li_init.launch
```

4. 保存 LI-Init 输出结果。

> 若没有 ROS1 环境，使用 Docker `osrf/ros:noetic-desktop-full` 即可。

## 2. 录制标定数据

```bash
ros2 launch li_init_tools calibrate.launch.py \
  use_chassis:=true \
  bag_duration:=90
```

若底盘由遥控器控制，`use_chassis:=false`。

开始录制：

```bash
ros2 service call /li_init_manager/record_bag std_srvs/srv/Trigger {}
```

录制期间需要充分激励：

- 绕 X/Y/Z 三个轴旋转；
- 前后左右移动；
- 避免长时间静止；
- 建议 60~120 s；
- 确保 `/livox/lidar` 和 `/livox/imu` 持续有数据。

提前结束：

```bash
ros2 service call /li_init_manager/stop_recording std_srvs/srv/Trigger {}
```

bag 位于：

```text
~/nav_system_ws/calib_bags/li_init_YYYYmmdd_HHMMSS
```

查看状态：

```bash
ros2 topic echo /li_init_status
```

## 3. 运行 LI-Init 并保存结果

按 LI-Init 的说明运行，输出的旋转、平移、时间偏移保存为：

```text
~/nav_system_ws/calib_bags/li_init_result.txt
```

支持的文本格式示例：

```text
Rotation LiDAR to IMU:
 0.9999  0.0010 -0.0020
-0.0010  0.9999  0.0005
 0.0020 -0.0005  0.9999
Translation LiDAR to IMU:
 0.012 -0.003 0.045
Time offset: -0.0021
```

也支持 YAML：

```yaml
extrinsic_R: [ ... 9 values ... ]
extrinsic_T: [x, y, z]
time_offset_lidar_to_imu: -0.0021
```

如果 LI-Init 输出格式不同，可直接用命令行覆盖：

```bash
ros2 run li_init_tools apply_li_init \
  --rotation 0.9999 0.001 -0.002 -0.001 0.9999 0.0005 0.002 -0.0005 0.9999 \
  --translation 0.012 -0.003 0.045 \
  --time-offset -0.0021
```

## 4. 生成 FAST-LIO 标定配置

```bash
ros2 service call /li_init_manager/apply_result std_srvs/srv/Trigger {}
```

输出：

```text
src/mapping/FAST_LIO/config/mid360_calibrated.yaml
```

写入内容：

```yaml
common:
  time_offset_lidar_to_imu: <dt>
mapping:
  extrinsic_T: [x, y, z]
  extrinsic_R: [9 values, row-major]
  extrinsic_est_en: false
```

命令行方式：

```bash
ros2 run li_init_tools apply_li_init \
  --result ~/nav_system_ws/calib_bags/li_init_result.txt \
  --input src/mapping/FAST_LIO/config/mid360.yaml \
  --output src/mapping/FAST_LIO/config/mid360_calibrated.yaml
```

脚本会检查旋转矩阵行列式并做 SO(3) 投影。

## 5. 使用标定配置

建图：

```bash
ros2 launch bringup mapping.launch.py \
  lio_config:=$HOME/nav_system_ws/src/mapping/FAST_LIO/config/mid360_calibrated.yaml
```

导航：

```bash
ros2 launch bringup navigation.launch.py \
  map_yaml:=$HOME/nav_system_ws/src/mapping/maps/occupancy_map.yaml \
  lio_config:=$HOME/nav_system_ws/src/mapping/FAST_LIO/config/mid360_calibrated.yaml \
  use_gui:=true use_chassis:=true
```

## 6. 验证标定效果

1. 静止 30 s，观察 `/Odometry` 是否明显漂移；
2. 手持/车载缓慢旋转，观察点云是否“拖影”；
3. 走闭合轨迹，回到起点观察 z 漂移与重影；
4. 对比 `extrinsic_est_en: true` 与标定后的结果；
5. 若 z 方向漂移明显，优先检查平移外参与时间偏移。

## 7. 临时替代方案

没有 LI-Init 结果时，可在 `mid360.yaml` 中打开在线估计：

```yaml
mapping:
  extrinsic_est_en: true
  extrinsic_T: [0.0, 0.0, 0.0]
  extrinsic_R: [1,0,0, 0,1,0, 0,0,1]
```

在线估计只适合短时间建图，长期使用仍建议离线 LI-Init 标定。

## 8. 注意

- 旋转矩阵约定为 **LiDAR → IMU**，与 FAST-LIO 一致。
- 平移单位米。
- 时间偏移单位秒。
- 标定时不要只旋转，必须有平移激励，否则平移外参不可观。
- 标定结果文件路径可通过 `calibrate.launch.py` 的 `result_file` 参数修改。
- 当前环境若无法联网，先运行 `scripts/install_li_init.sh` 的 clone 步骤需在网络可用时执行。
