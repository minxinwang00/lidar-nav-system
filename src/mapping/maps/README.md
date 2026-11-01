# 地图目录

导航层使用的二维占据栅格地图放在这里，格式为 `nav2_map_server` 可读取的
`*.pgm` + `*.yaml`。

两种生成方式：

1. **在线**（推荐）：先运行

   ```bash
   ros2 launch bringup mapping.launch.py
   ```

   建图完成后在 Qt 界面点“结束建图并自动保存”（会生成带时间戳的新地图），
   或只保存当前累积地图：

   ```bash
   ros2 service call /pointcloud_to_grid/save_map std_srvs/srv/Trigger {}
   ```

   控制台也提供“开始建图 / 结束建图并自动保存”按钮。导航启动时如果不指定
   `map_yaml`，会自动选择本目录下修改时间最新的 `*.yaml`。界面的地图下拉框
   同样默认选中最新地图，也可以“导入地图...”后点“加载到导航”。

2. **离线**：用 FAST-LIO 保存的 PCD 转换

   ```bash
   python3 ../scripts/pcd_to_grid.py ../FAST_LIO/PCD/scans.pcd \
       --resolution 0.05 --z-min 0.20 --z-max 2.0 --inflation 0.25
   ```

生成后启动导航：

```bash
ros2 launch bringup navigation.launch.py \
  map_yaml:=$HOME/nav_system_ws/src/mapping/maps/occupancy_map.yaml \
  use_gui:=true
```

> `pcd_to_grid.py` 默认把点云包围盒内的格子视为自由空间（室内全扫描场景）。
> 若有未扫描区域且不希望被当作可通行，加 `--unknown-outside`。
