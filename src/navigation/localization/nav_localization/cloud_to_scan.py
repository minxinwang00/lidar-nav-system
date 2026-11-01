#!/usr/bin/env python3
"""把 3D 激光点云转换成 nav2_amcl 所需的 2D 激光扫描（LaserScan）。

订阅 /cloud_registered（世界坐标系），把点变换到 base_link，只保留一个水平
高度带内的点，再把它们压缩成一圈 360 度的 LaserScan。由于 MID-360 本身没有
2D 激光雷达，且本机没有安装 ros-humble-pointcloud-to-laserscan，所以需要这个
桥接节点来充当 AMCL 的激光输入。
"""

import math

import numpy as np

import rclpy
from rclpy.duration import Duration
from rclpy.node import Node
from rclpy.qos import HistoryPolicy, QoSProfile, ReliabilityPolicy
from rclpy.time import Time

import tf2_ros
from sensor_msgs.msg import LaserScan, PointCloud2
from sensor_msgs_py import point_cloud2


def quat_to_matrix(q):
    x, y, z, w = q.x, q.y, q.z, q.w
    return np.array([
        [1 - 2 * (y * y + z * z), 2 * (x * y - z * w), 2 * (x * z + y * w)],
        [2 * (x * y + z * w), 1 - 2 * (x * x + z * z), 2 * (y * z - x * w)],
        [2 * (x * z - y * w), 2 * (y * z + x * w), 1 - 2 * (x * x + y * y)],
    ], dtype=np.float64)


class CloudToScan(Node):
    def __init__(self):
        super().__init__('cloud_to_scan')

        self.declare_parameter('cloud_topic', '/cloud_registered')
        self.declare_parameter('scan_topic', '/scan')
        self.declare_parameter('target_frame', 'base_link')
        self.declare_parameter('z_min', -0.20)
        self.declare_parameter('z_max', 0.80)
        self.declare_parameter('range_min', 0.20)
        self.declare_parameter('range_max', 12.0)
        self.declare_parameter('num_bins', 720)
        self.declare_parameter('max_points', 6000)
        self.declare_parameter('angle_min', -math.pi)
        self.declare_parameter('angle_max', math.pi)

        self.target_frame = str(self.get_parameter('target_frame').value)
        self.z_min = float(self.get_parameter('z_min').value)
        self.z_max = float(self.get_parameter('z_max').value)
        self.range_min = float(self.get_parameter('range_min').value)
        self.range_max = float(self.get_parameter('range_max').value)
        self.num_bins = max(90, int(self.get_parameter('num_bins').value))
        self.max_points = max(100, int(self.get_parameter('max_points').value))
        self.angle_min = float(self.get_parameter('angle_min').value)
        self.angle_max = float(self.get_parameter('angle_max').value)
        self._warned = False

        self.tf_buffer = tf2_ros.Buffer()
        self.tf_listener = tf2_ros.TransformListener(self.tf_buffer, self)

        qos = QoSProfile(depth=5,
                         reliability=ReliabilityPolicy.RELIABLE,
                         history=HistoryPolicy.KEEP_LAST)
        self.create_subscription(
            PointCloud2, str(self.get_parameter('cloud_topic').value),
            self._cloud_cb, qos)
        self.scan_pub = self.create_publisher(
            LaserScan, str(self.get_parameter('scan_topic').value), qos)

        self.get_logger().info(
            'cloud_to_scan: %s -> %s (%d bins, z [%.2f, %.2f])'
            % (str(self.get_parameter('cloud_topic').value),
               str(self.get_parameter('scan_topic').value),
               self.num_bins, self.z_min, self.z_max))

    def _read_xyz(self, msg):
        try:
            pts = point_cloud2.read_points(msg, field_names=('x', 'y', 'z'),
                                           skip_nans=True)
            if isinstance(pts, np.ndarray):
                xyz = (np.stack([pts['x'], pts['y'], pts['z']], axis=1)
                       if pts.dtype.names else pts[:, :3])
            else:
                xyz = np.asarray(list(pts), dtype=np.float64)
        except Exception as exc:
            self.get_logger().warn('read_points failed: %s' % exc)
            return None
        if xyz.size == 0:
            return None
        return np.asarray(xyz, dtype=np.float64).reshape(-1, 3)

    def _to_target(self, xyz, frame_id):
        if not frame_id or frame_id == self.target_frame:
            return xyz
        try:
            tr = self.tf_buffer.lookup_transform(
                self.target_frame, frame_id, Time(), Duration(seconds=0.3))
        except Exception as exc:
            if not self._warned:
                self.get_logger().warn(
                    'No TF %s <- %s (%s); scan not published'
                    % (self.target_frame, frame_id, exc))
                self._warned = True
            return None
        R = quat_to_matrix(tr.transform.rotation)
        t = tr.transform.translation
        return xyz.dot(R.T) + np.array([t.x, t.y, t.z], dtype=np.float64)

    def _cloud_cb(self, msg):
        xyz = self._read_xyz(msg)
        if xyz is None:
            return
        stride = max(1, int(math.ceil(len(xyz) / float(self.max_points))))
        xyz = xyz[::stride]
        xyz = self._to_target(xyz, msg.header.frame_id)
        if xyz is None:
            return

        x, y, z = xyz[:, 0], xyz[:, 1], xyz[:, 2]
        keep = (z > self.z_min) & (z < self.z_max)
        x, y = x[keep], y[keep]
        if x.size == 0:
            return

        r = np.hypot(x, y)
        keep = (r > self.range_min) & (r < self.range_max)
        x, y, r = x[keep], y[keep], r[keep]
        if r.size == 0:
            return

        theta = np.arctan2(y, x)
        span = self.angle_max - self.angle_min
        idx = np.floor((theta - self.angle_min) / span * self.num_bins).astype(np.int32)
        np.clip(idx, 0, self.num_bins - 1, out=idx)

        ranges = np.full(self.num_bins, np.inf, dtype=np.float64)
        np.minimum.at(ranges, idx, r)

        scan = LaserScan()
        scan.header.stamp = msg.header.stamp
        scan.header.frame_id = self.target_frame
        scan.angle_min = self.angle_min
        scan.angle_max = self.angle_max
        scan.angle_increment = span / float(self.num_bins)
        scan.time_increment = 0.0
        scan.scan_time = 0.1
        scan.range_min = self.range_min
        scan.range_max = self.range_max
        scan.ranges = ranges.astype(np.float32).tolist()
        self.scan_pub.publish(scan)


def main(args=None):
    rclpy.init(args=args)
    node = CloudToScan()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
