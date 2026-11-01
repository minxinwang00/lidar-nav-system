import rclpy
from rclpy.node import Node
from sensor_msgs.msg import PointCloud2, PointField
from nav_msgs.msg import OccupancyGrid
from std_msgs.msg import Header
import open3d as o3d
import numpy as np
import cv2
import yaml

class MapPublisher(Node):
    def __init__(self):
        super().__init__('map_publisher')
        self.pcd_pub = self.create_publisher(PointCloud2, '/cloud_registered', 10)
        self.grid_pub = self.create_publisher(OccupancyGrid, '/map', 10)

        # 读取点云并降采样
        pcd = o3d.io.read_point_cloud('/home/mxw/ros2_ws/final_map.pcd')
        print(f"原始点数：{len(pcd.points)}")
        pcd = pcd.voxel_down_sample(voxel_size=0.2)   # 0.2 米降采样
        print(f"降采样后点数：{len(pcd.points)}")
        self.points = np.asarray(pcd.points).astype(np.float32)

        # 读取栅格地图
        with open('/home/mxw/ros2_ws/my_map.yaml', 'r') as f:
            meta = yaml.safe_load(f)
        self.resolution = meta['resolution']
        self.origin = meta['origin']
        self.grid_img = cv2.imread(meta['image'], cv2.IMREAD_GRAYSCALE)
        self.grid_height, self.grid_width = self.grid_img.shape

        # 预先构建 PointCloud2 消息
        header = Header()
        header.frame_id = 'camera_init'
        fields = [
            PointField(name='x', offset=0, datatype=PointField.FLOAT32, count=1),
            PointField(name='y', offset=4, datatype=PointField.FLOAT32, count=1),
            PointField(name='z', offset=8, datatype=PointField.FLOAT32, count=1),
        ]
        self.cloud_msg = PointCloud2()
        self.cloud_msg.header = header
        self.cloud_msg.height = 1
        self.cloud_msg.width = len(self.points)
        self.cloud_msg.fields = fields
        self.cloud_msg.is_bigendian = False
        self.cloud_msg.point_step = 12
        self.cloud_msg.row_step = self.cloud_msg.point_step * self.cloud_msg.width
        self.cloud_msg.is_dense = True
        self.cloud_msg.data = self.points.tobytes()

        # 预先构建 OccupancyGrid 消息
        self.grid_msg = OccupancyGrid()
        self.grid_msg.header = header
        self.grid_msg.info.resolution = self.resolution
        self.grid_msg.info.width = self.grid_width
        self.grid_msg.info.height = self.grid_height
        self.grid_msg.info.origin.position.x = self.origin[0]
        self.grid_msg.info.origin.position.y = self.origin[1]
        self.grid_msg.info.origin.position.z = 0.0
        self.grid_msg.info.origin.orientation.w = 1.0
        data = []
        for row in self.grid_img:
            for val in row:
                data.append(100 if val > 127 else 0)
        self.grid_msg.data = data

        # 定时器，1Hz 发布
        self.timer = self.create_timer(1.0, self.publish_all)

    def publish_all(self):
        now = self.get_clock().now().to_msg()
        self.cloud_msg.header.stamp = now
        self.grid_msg.header.stamp = now
        self.pcd_pub.publish(self.cloud_msg)
        self.grid_pub.publish(self.grid_msg)
        print("Published cloud and map")

def main(args=None):
    rclpy.init(args=args)
    node = MapPublisher()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
