# 地图说明

本目录保存实机建图导出的二维占据栅格地图副本：

- `occupancy_map.pgm`：栅格图像，0=障碍，254=空闲，205=未知；
- `occupancy_map.yaml`：地图元数据；
- `occupancy_map.png`：README 预览图。

实际导航时 launch 默认读取：

```text
src/mapping/maps/occupancy_map.yaml
```

也可以显式指定：

```bash
ros2 launch bringup navigation.launch.py \
  map_yaml:=$HOME/nav_system_ws/src/mapping/maps/occupancy_map.yaml \
  use_chassis:=true \
  use_gui:=true
```
