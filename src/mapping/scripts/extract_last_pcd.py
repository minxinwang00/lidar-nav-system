import os
import rosbag2_py
from rclpy.serialization import deserialize_message
from sensor_msgs.msg import PointCloud2
import sensor_msgs_py.point_cloud2 as pc2
import open3d as o3d
import numpy as np

bag_path = '/home/mxw/my_pointcloud_bag2'
output_pcd = '/home/mxw/ros2_ws/final_map.pcd'

storage_options = rosbag2_py.StorageOptions(uri=bag_path, storage_id='sqlite3')
converter_options = rosbag2_py.ConverterOptions(
    input_serialization_format='cdr',
    output_serialization_format='cdr'
)
reader = rosbag2_py.SequentialReader()
reader.open(storage_options, converter_options)
reader.set_filter(rosbag2_py.StorageFilter(topics=['/cloud_registered']))

all_points = []
frame_count = 0
while reader.has_next():
    topic, data, timestamp = reader.read_next()
    if topic == '/cloud_registered':
        msg = deserialize_message(data, PointCloud2)
        points = []
        for p in pc2.read_points(msg, field_names=("x", "y", "z"), skip_nans=True):
            points.append([p[0], p[1], p[2]])
        if points:
            all_points.append(np.array(points))
            frame_count += 1

if not all_points:
    print("未找到点云")
    exit(1)

merged = np.vstack(all_points)
print(f"总帧数：{frame_count}，总点数：{len(merged)}")

# 可选：体素降采样，减小文件体积（0.05 米）
pcd = o3d.geometry.PointCloud()
pcd.points = o3d.utility.Vector3dVector(merged)
pcd = pcd.voxel_down_sample(voxel_size=0.05)

o3d.io.write_point_cloud(output_pcd, pcd)
print(f"已保存到 {output_pcd}")
