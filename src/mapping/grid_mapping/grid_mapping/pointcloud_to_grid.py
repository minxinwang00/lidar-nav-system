#!/usr/bin/env python3
"""3D 激光点云 -> 2D 占据栅格地图。

订阅：
  /cloud_registered（sensor_msgs/PointCloud2，世界坐标系，来自 FAST-LIO）
  /Odometry        （nav_msgs/Odometry，同一世界坐标系下的机器人位姿）

发布：
  /local_costmap       （nav_msgs/OccupancyGrid）滑动窗口、已做膨胀，供局部
                       控制器做动态障碍物避让
  /grid_map/global     （nav_msgs/OccupancyGrid，transient local）累积地图

服务：
  ~/save_map（std_srvs/Trigger）写出 <save_dir>/<map_name>.pgm/.yaml，
  供 nav2_map_server 加载用于全局规划。
  ~/start_mapping（std_srvs/Trigger）清空累积地图并开始建图。
  ~/stop_mapping（std_srvs/Trigger）停止累积并立即保存地图。
"""

import math
import os
import time

import numpy as np
from scipy import ndimage

import rclpy
from rclpy.duration import Duration
from rclpy.node import Node
from rclpy.qos import DurabilityPolicy, HistoryPolicy, QoSProfile, ReliabilityPolicy
from rclpy.time import Time

import tf2_ros
from nav_msgs.msg import OccupancyGrid, Odometry
from sensor_msgs.msg import PointCloud2
from sensor_msgs_py import point_cloud2
from std_srvs.srv import Trigger

import cv2


def quat_to_matrix(q):
    """由 geometry_msgs 的 Quaternion 构造 3x3 旋转矩阵。"""
    x, y, z, w = q.x, q.y, q.z, q.w
    return np.array([
        [1 - 2 * (y * y + z * z), 2 * (x * y - z * w), 2 * (x * z + y * w)],
        [2 * (x * y + z * w), 1 - 2 * (x * x + z * z), 2 * (y * z - x * w)],
        [2 * (x * z - y * w), 2 * (y * z + x * w), 1 - 2 * (x * x + y * y)],
    ], dtype=np.float64)


def yaw_from_quat(q):
    return math.atan2(2.0 * (q.w * q.z + q.x * q.y),
                      1.0 - 2.0 * (q.y * q.y + q.z * q.z))


def despeckle_obstacles(grid, min_neighbors=1, min_cluster_cells=3):
    """删除离散的孤立障碍点/过小障碍簇。

    典型噪声：单个点、两三个点组成的小团，彼此之间隔着空白格。
    连续墙线、桌腿等成片/成线的障碍会保留。

    min_neighbors: 8 邻域内至少要有多少个障碍邻居；1 表示只删完全孤立的单格。
    min_cluster_cells: 连通障碍簇的最小格数，小于该值整簇删除。
    """
    occupied = grid >= 50
    if not occupied.any():
        return grid

    kernel = np.ones((3, 3), dtype=np.uint8)
    counts = ndimage.convolve(occupied.astype(np.uint8), kernel,
                              mode='constant', cval=0)
    neighbors = counts - occupied.astype(np.uint8)
    occupied &= neighbors >= int(min_neighbors)

    structure = np.ones((3, 3), dtype=bool)
    labels, count = ndimage.label(occupied, structure=structure)
    if count > 0:
        sizes = np.bincount(labels.ravel(), minlength=count + 1)
        keep = sizes >= int(min_cluster_cells)
        keep[0] = False
    else:
        keep = np.zeros(1, dtype=bool)

    # 清掉原始障碍，再写回保留下来的障碍
    grid[grid >= 50] = 0
    if count > 0 and keep.any():
        grid[keep[labels]] = 100
    return grid


def fill_enclosed_unknown(grid, free_value=0):
    """把被空闲格完全包围的未知连通块填成空地。

    只处理内部未知块：
      - 接触栅格边界的未知区域保留，避免把传感器视野外当成空地；
      - 邻接障碍物的未知区域保留，避免把墙后/柱后的盲区填平；
      - 邻接已知空闲格、且不接触障碍物的封闭未知块，填为 free_value。

    这样稀疏点云之间的“空洞”不会再被规划器当成墙。
    """
    unknown = grid < 0
    if not unknown.any():
        return grid

    structure = np.ones((3, 3), dtype=bool)
    labels, count = ndimage.label(unknown, structure=structure)
    if count == 0:
        return grid

    # 接触边界的未知块不填
    border_labels = np.unique(np.concatenate((
        labels[0, :], labels[-1, :], labels[:, 0], labels[:, -1])))
    border_labels = border_labels[border_labels > 0]

    # 统计每个未知块是否邻接已知格 / 障碍格
    known_dilated = ndimage.binary_dilation(~unknown, structure=structure)
    obstacle_dilated = ndimage.binary_dilation(grid >= 50,
                                               structure=structure)
    has_known = np.bincount(labels[known_dilated],
                            minlength=count + 1) > 0
    has_obstacle = np.bincount(labels[obstacle_dilated],
                               minlength=count + 1) > 0

    fill_labels = has_known & ~has_obstacle
    fill_labels[0] = False
    fill_labels[border_labels] = False
    if fill_labels.any():
        grid[fill_labels[labels]] = free_value
    return grid


class PointCloudToGrid(Node):
    def __init__(self):
        super().__init__('pointcloud_to_grid')

        self.declare_parameter('cloud_topic', '/cloud_registered')
        self.declare_parameter('odom_topic', '/Odometry')
        self.declare_parameter('map_frame', 'map')
        self.declare_parameter('resolution', 0.05)
        self.declare_parameter('local_size', 8.0)
        self.declare_parameter('max_range', 12.0)
        self.declare_parameter('min_range', 0.15)
        self.declare_parameter('z_min', 0.15)
        self.declare_parameter('z_max', 2.0)
        # 局部代价地图可用单独的高度带，避免为了局部避障改坏全局地图
        self.declare_parameter('local_z_min', 0.5)
        self.declare_parameter('local_z_max', 2.0)
        self.declare_parameter('subsample', 2)
        self.declare_parameter('max_points', 4000)
        self.declare_parameter('inflation_radius', 0.35)
        # 是否把障碍物附近的未知格也膨胀成高代价。
        # false：未知格保持 -1，避免传感器没扫到的区域形成大片紫色假障碍。
        self.declare_parameter('inflate_unknown', False)
        # 被空闲格完全包围的未知块填成空地，避免稀疏点云空洞被当成墙
        self.declare_parameter('fill_enclosed_unknown', True)
        # 删除离散孤立障碍点/过小障碍簇
        self.declare_parameter('despeckle_obstacles', True)
        self.declare_parameter('obstacle_min_neighbors', 1)
        self.declare_parameter('obstacle_min_cluster_cells', 3)
        self.declare_parameter('global_resolution', 0.05)
        self.declare_parameter('process_rate', 10.0)
        self.declare_parameter('global_publish_rate', 1.0)
        self.declare_parameter('max_global_cells', 2000000)
        self.declare_parameter('save_dir', '~/nav_system_ws/src/mapping/maps')
        self.declare_parameter('map_name', 'occupancy_map')
        # 累积地图的自动持久化
        self.declare_parameter('auto_save_enable', True)
        self.declare_parameter('auto_save_period', 30.0)
        self.declare_parameter('auto_save_on_shutdown', True)
        self.declare_parameter('auto_save_min_obstacles', 20)
        self.declare_parameter('mapping_active_on_start', True)
        self.declare_parameter('versioned_on_stop', True)

        self.res = float(self.get_parameter('resolution').value)
        self.local_size = float(self.get_parameter('local_size').value)
        self.max_range = float(self.get_parameter('max_range').value)
        self.min_range = float(self.get_parameter('min_range').value)
        self.z_min = float(self.get_parameter('z_min').value)
        self.z_max = float(self.get_parameter('z_max').value)
        self.local_z_min = float(self.get_parameter('local_z_min').value)
        self.local_z_max = float(self.get_parameter('local_z_max').value)
        self.subsample = max(1, int(self.get_parameter('subsample').value))
        self.max_points = max(100, int(self.get_parameter('max_points').value))
        self.inflation_radius = float(self.get_parameter('inflation_radius').value)
        self.inflate_unknown = bool(self.get_parameter('inflate_unknown').value)
        self.fill_enclosed_unknown = bool(
            self.get_parameter('fill_enclosed_unknown').value)
        self.despeckle_obstacles = bool(
            self.get_parameter('despeckle_obstacles').value)
        self.obstacle_min_neighbors = int(
            self.get_parameter('obstacle_min_neighbors').value)
        self.obstacle_min_cluster_cells = int(
            self.get_parameter('obstacle_min_cluster_cells').value)
        self.gres = float(self.get_parameter('global_resolution').value)
        self.map_frame = str(self.get_parameter('map_frame').value)
        self.save_dir = os.path.expanduser(str(self.get_parameter('save_dir').value))
        self.map_name = str(self.get_parameter('map_name').value)
        self.max_cells = int(self.get_parameter('max_global_cells').value)
        self.auto_save_enable = bool(self.get_parameter('auto_save_enable').value)
        self.auto_save_period = float(self.get_parameter('auto_save_period').value)
        self.auto_save_on_shutdown = bool(
            self.get_parameter('auto_save_on_shutdown').value)
        self.auto_save_min_obstacles = int(
            self.get_parameter('auto_save_min_obstacles').value)
        self._mapping_active = bool(
            self.get_parameter('mapping_active_on_start').value)
        self.versioned_on_stop = bool(
            self.get_parameter('versioned_on_stop').value)

        self.tf_buffer = tf2_ros.Buffer()
        self.tf_listener = tf2_ros.TransformListener(self.tf_buffer, self)

        self._cloud = None
        self._cloud_seq = 0
        self._processed_seq = -1
        self._odom = None
        self._warned_tf = False

        self._g_version = 0
        self._last_saved_version = -1
        # 全局累积（map 坐标系、全局分辨率）
        self._g_free = set()
        self._g_occ = set()
        self._g_min = [10 ** 9, 10 ** 9]
        self._g_max = [-10 ** 9, -10 ** 9]
        self._g_dropped = 0

        self._local_grid = None
        self._local_origin = (0.0, 0.0)

        sensor_qos = QoSProfile(depth=5,
                                reliability=ReliabilityPolicy.RELIABLE,
                                history=HistoryPolicy.KEEP_LAST)
        map_qos = QoSProfile(depth=1,
                             reliability=ReliabilityPolicy.RELIABLE,
                             durability=DurabilityPolicy.TRANSIENT_LOCAL,
                             history=HistoryPolicy.KEEP_LAST)

        self.create_subscription(PointCloud2,
                                 str(self.get_parameter('cloud_topic').value),
                                 self._cloud_cb, sensor_qos)
        self.create_subscription(Odometry,
                                 str(self.get_parameter('odom_topic').value),
                                 self._odom_cb, 10)

        self.local_pub = self.create_publisher(OccupancyGrid, '/local_costmap', 5)
        # 调试用：膨胀前的原始占据栅格，便于在 RViz 区分“真实障碍”和“膨胀代价”
        self.local_raw_pub = self.create_publisher(
            OccupancyGrid, '/local_obstacles_raw', 5)
        self.global_pub = self.create_publisher(OccupancyGrid, '/grid_map/global', map_qos)

        self.create_service(Trigger, '~/save_map', self._save_map_cb)
        self.create_service(Trigger, '~/start_mapping',
                            self._start_mapping_cb)
        self.create_service(Trigger, '~/stop_mapping',
                            self._stop_mapping_cb)

        rate = max(1.0, float(self.get_parameter('process_rate').value))
        self.create_timer(1.0 / rate, self._process_timer)
        g_rate = max(0.2, float(self.get_parameter('global_publish_rate').value))
        self.create_timer(1.0 / g_rate, self._publish_global_timer)
        if self.auto_save_enable and self.auto_save_period > 0.0:
            self.create_timer(self.auto_save_period, self._auto_save_timer)
            self.get_logger().info(
                'auto map save every %.1f s -> %s/%s.yaml'
                % (self.auto_save_period, self.save_dir, self.map_name))

        self.get_logger().info(
            'pointcloud_to_grid started: res=%.3f m, local=%.1f m, range=%.1f m'
            % (self.res, self.local_size, self.max_range))

    # ------------------------------------------------------------------ 回调函数
    def _cloud_cb(self, msg):
        self._cloud = msg
        self._cloud_seq += 1

    def _odom_cb(self, msg):
        self._odom = msg

    def _robot_origin(self):
        """返回机器人在 map 坐标系下的位置；取不到时返回 None。"""
        if self._odom is None:
            return None
        p = self._odom.pose.pose.position
        src = self._odom.header.frame_id or self.map_frame
        if src == self.map_frame:
            return np.array([p.x, p.y, p.z], dtype=np.float64)
        try:
            tr = self.tf_buffer.lookup_transform(
                self.map_frame, src, Time(), Duration(seconds=0.2))
        except Exception:
            return None
        R = quat_to_matrix(tr.transform.rotation)
        t = tr.transform.translation
        return R.dot(np.array([p.x, p.y, p.z])) + np.array([t.x, t.y, t.z])

    def _tf_points_to_map(self, xyz, frame_id):
        if not frame_id or frame_id == self.map_frame:
            return xyz
        try:
            tr = self.tf_buffer.lookup_transform(
                self.map_frame, frame_id, Time(), Duration(seconds=0.3))
        except Exception as exc:
            if not self._warned_tf:
                self.get_logger().warn(
                    'No TF %s <- %s (%s); dropping cloud' % (self.map_frame, frame_id, exc))
                self._warned_tf = True
            return None
        R = quat_to_matrix(tr.transform.rotation)
        t = tr.transform.translation
        return xyz.dot(R.T) + np.array([t.x, t.y, t.z], dtype=np.float64)

    def _read_xyz(self, msg):
        try:
            pts = point_cloud2.read_points(msg, field_names=('x', 'y', 'z'),
                                           skip_nans=True)
            if isinstance(pts, np.ndarray):
                if pts.dtype.names:
                    xyz = np.stack([pts['x'], pts['y'], pts['z']], axis=1)
                else:
                    xyz = pts[:, :3]
            else:
                xyz = np.asarray(list(pts), dtype=np.float64)
        except Exception as exc:  # 点云格式异常也绝不能让节点崩溃
            self.get_logger().warn('read_points failed: %s' % exc)
            return None
        if xyz.size == 0:
            return None
        return np.asarray(xyz, dtype=np.float64).reshape(-1, 3)

    # ------------------------------------------------------------------ 数据处理
    def _process_timer(self):
        if self._cloud is None or self._cloud_seq == self._processed_seq:
            return
        origin = self._robot_origin()
        if origin is None:
            return
        self._processed_seq = self._cloud_seq

        xyz = self._read_xyz(self._cloud)
        if xyz is None:
            return
        xyz = self._tf_points_to_map(xyz, self._cloud.header.frame_id)
        if xyz is None:
            return

        # 降采样 + 距离 / 高度滤波（高度为相对传感器的差值）
        stride = max(self.subsample, int(math.ceil(len(xyz) / float(self.max_points))))
        xyz = xyz[::stride]
        ox, oy, oz = origin
        r = np.hypot(xyz[:, 0] - ox, xyz[:, 1] - oy)
        dz = xyz[:, 2] - oz
        base_keep = (r > self.min_range) & (r < self.max_range)
        keep_local = base_keep & (dz > self.local_z_min) & \
            (dz < self.local_z_max)
        keep_global = base_keep & (dz > self.z_min) & (dz < self.z_max)

        pts_local = xyz[keep_local]
        pts_global = xyz[keep_global]
        if pts_local.shape[0] > 0:
            self._build_local(pts_local, ox, oy)
        if pts_global.shape[0] > 0:
            self._update_global(pts_global, ox, oy)

    def _build_local(self, pts, ox, oy):
        res = self.res
        size = max(40, int(round(self.local_size / res)))
        half = size * res / 2.0
        origin_x = ox - half
        origin_y = oy - half

        grid = np.full((size, size), -1, dtype=np.int16)

        # 只保留落在局部窗口内的点
        px, py = pts[:, 0], pts[:, 1]
        inside = ((px > origin_x) & (px < origin_x + size * res) &
                  (py > origin_y) & (py < origin_y + size * res))
        px, py = px[inside], py[inside]

        if px.size > 0:
            # 对自由空间做射线投射（已向量化，每条射线的采样点数量固定）
            k = 32
            t = np.linspace(0.0, 1.0, k, endpoint=False)[None, :]
            rx = ox + (px[:, None] - ox) * t
            ry = oy + (py[:, None] - oy) * t
            ix = np.floor((rx - origin_x) / res).astype(np.int32)
            iy = np.floor((ry - origin_y) / res).astype(np.int32)
            valid = (ix >= 0) & (ix < size) & (iy >= 0) & (iy < size)
            grid[iy[valid], ix[valid]] = 0

            ox_i = np.floor((px - origin_x) / res).astype(np.int32)
            oy_i = np.floor((py - origin_y) / res).astype(np.int32)
            ok = (ox_i >= 0) & (ox_i < size) & (oy_i >= 0) & (oy_i < size)
            grid[oy_i[ok], ox_i[ok]] = 100

        # 先去掉离散孤立障碍，再把被空闲格包围的未知空洞填成空地。
        if self.despeckle_obstacles:
            grid = despeckle_obstacles(
                grid, self.obstacle_min_neighbors,
                self.obstacle_min_cluster_cells)
        if self.fill_enclosed_unknown:
            grid = fill_enclosed_unknown(grid)

        # 膨胀前先发布一份原始栅格：-1=未知，0=空闲，100=障碍
        raw_msg = OccupancyGrid()
        raw_msg.header.stamp = self.get_clock().now().to_msg()
        raw_msg.header.frame_id = self.map_frame
        raw_msg.info.resolution = res
        raw_msg.info.width = size
        raw_msg.info.height = size
        raw_msg.info.origin.position.x = float(origin_x)
        raw_msg.info.origin.position.y = float(origin_y)
        raw_msg.info.origin.orientation.w = 1.0
        raw_msg.data = grid.astype(np.int8).flatten().tolist()
        self.local_raw_pub.publish(raw_msg)

        # 生成膨胀代价 (0..100)，使规划器 / 控制器只需查一次表。
        # 障碍附近的未知格也必须写入代价，否则动态障碍周围会被当成自由空间。
        obs = grid >= 50
        if obs.any() and self.inflation_radius > 0.0:
            dist = ndimage.distance_transform_edt(~obs) * res
            cost = np.clip(100.0 * (1.0 - dist / self.inflation_radius), 0.0, 100.0)
            if self.inflate_unknown:
                near = (~obs) & (dist < self.inflation_radius)
            else:
                near = (grid == 0) & (dist < self.inflation_radius)
            grid[near] = cost[near].astype(np.int16)
            grid[obs] = 100

        self._local_grid = grid.astype(np.int8)
        self._local_origin = (origin_x, origin_y)

        msg = OccupancyGrid()
        msg.header.stamp = self.get_clock().now().to_msg()
        msg.header.frame_id = self.map_frame
        msg.info.resolution = res
        msg.info.width = size
        msg.info.height = size
        msg.info.origin.position.x = float(origin_x)
        msg.info.origin.position.y = float(origin_y)
        msg.info.origin.orientation.w = 1.0
        msg.data = self._local_grid.flatten().tolist()
        self.local_pub.publish(msg)

    def _update_global(self, pts, ox, oy):
        if not self._mapping_active:
            return
        if len(self._g_occ) + len(self._g_free) > self.max_cells:
            self._g_dropped += 1
            return
        g = self.gres
        ix = np.floor(pts[:, 0] / g).astype(np.int32)
        iy = np.floor(pts[:, 1] / g).astype(np.int32)
        occ = set(zip(ix.tolist(), iy.tolist()))

        k = 16
        t = np.linspace(0.0, 1.0, k, endpoint=False)[None, :]
        rx = ox + (pts[:, 0][:, None] - ox) * t
        ry = oy + (pts[:, 1][:, None] - oy) * t
        fx = np.floor(rx / g).astype(np.int32).ravel()
        fy = np.floor(ry / g).astype(np.int32).ravel()
        free = set(zip(fx.tolist(), fy.tolist()))

        self._g_free |= free
        self._g_occ |= occ
        self._g_free -= occ
        self._g_version += 1

        for xs, ys in ((ix, iy), (fx, fy)):
            if xs.size:
                self._g_min[0] = min(self._g_min[0], int(xs.min()))
                self._g_min[1] = min(self._g_min[1], int(ys.min()))
                self._g_max[0] = max(self._g_max[0], int(xs.max()))
                self._g_max[1] = max(self._g_max[1], int(ys.max()))

    def _global_snapshot(self):
        """返回累积地图的 (数组, origin_x, origin_y, 分辨率)。"""
        if not self._g_occ and not self._g_free:
            return None, 0.0, 0.0, self.gres
        margin = int(round(1.0 / self.gres))
        x0 = self._g_min[0] - margin
        y0 = self._g_min[1] - margin
        x1 = self._g_max[0] + margin
        y1 = self._g_max[1] + margin
        w = min(6000, x1 - x0 + 1)
        h = min(6000, y1 - y0 + 1)
        arr = np.full((h, w), -1, dtype=np.int8)

        if self._g_free:
            f = np.array(list(self._g_free), dtype=np.int32)
            fx = f[:, 0] - x0
            fy = f[:, 1] - y0
            ok = (fx >= 0) & (fx < w) & (fy >= 0) & (fy < h)
            arr[fy[ok], fx[ok]] = 0
        if self._g_occ:
            o = np.array(list(self._g_occ), dtype=np.int32)
            ox = o[:, 0] - x0
            oy = o[:, 1] - y0
            ok = (ox >= 0) & (ox < w) & (oy >= 0) & (oy < h)
            arr[oy[ok], ox[ok]] = 100
        if self.despeckle_obstacles:
            arr = despeckle_obstacles(
                arr, self.obstacle_min_neighbors,
                self.obstacle_min_cluster_cells)
        if self.fill_enclosed_unknown:
            arr = fill_enclosed_unknown(arr)
        return arr, x0 * self.gres, y0 * self.gres, self.gres

    def _publish_global_timer(self):
        arr, x0, y0, res = self._global_snapshot()
        if arr is None:
            return
        msg = OccupancyGrid()
        msg.header.stamp = self.get_clock().now().to_msg()
        msg.header.frame_id = self.map_frame
        msg.info.resolution = res
        msg.info.height, msg.info.width = arr.shape
        msg.info.origin.position.x = float(x0)
        msg.info.origin.position.y = float(y0)
        msg.info.origin.orientation.w = 1.0
        msg.data = arr.flatten().tolist()
        self.global_pub.publish(msg)

    # ------------------------------------------------------------------ 保存地图
    def _save_map(self, reason='manual', map_name=None):
        """以原子方式写出 pgm + yaml。返回 (是否成功, 提示信息)。"""
        arr, x0, y0, res = self._global_snapshot()
        if arr is None:
            return False, 'No map data accumulated yet'
        # 允许运行时修改 save_dir，便于测试/切换地图目录
        self.save_dir = os.path.expanduser(
            str(self.get_parameter('save_dir').value))
        name = map_name or self.map_name
        try:
            os.makedirs(self.save_dir, exist_ok=True)
            img = np.full(arr.shape, 205, dtype=np.uint8)   # 未知区域
            img[arr == 0] = 254                             # 空闲区域
            img[arr == 100] = 0                             # 占据区域
            pgm_path = os.path.join(self.save_dir, name + '.pgm')
            yaml_path = os.path.join(self.save_dir, name + '.yaml')
            # 先写临时文件，避免自动保存过程中 map_server 读到写了一半的地图
            pgm_tmp = pgm_path + '.tmp.pgm'
            yaml_tmp = yaml_path + '.tmp.yaml'
            if not cv2.imwrite(pgm_tmp, np.flipud(img)):
                raise RuntimeError('cv2.imwrite failed')
            with open(yaml_tmp, 'w', encoding='utf-8') as f:
                f.write('image: %s\n' % os.path.basename(pgm_path))
                f.write('resolution: %.4f\n' % res)
                f.write('origin: [%.4f, %.4f, 0.0]\n' % (x0, y0))
                f.write('negate: 0\noccupied_thresh: 0.65\nfree_thresh: 0.196\n')
            os.replace(pgm_tmp, pgm_path)
            os.replace(yaml_tmp, yaml_path)
        except Exception as exc:
            return False, 'Save failed (%s): %s' % (reason, exc)
        self._last_saved_version = self._g_version
        return True, 'Saved %s [%s] (%dx%d @ %.3f m)' % (
            yaml_path, reason, arr.shape[1], arr.shape[0], res)

    def _save_map_cb(self, request, response):
        success, message = self._save_map('service')
        response.success = success
        response.message = message
        if success:
            self.get_logger().info(message)
        else:
            self.get_logger().warn(message)
        return response

    def _start_mapping_cb(self, request, response):
        self._g_free.clear()
        self._g_occ.clear()
        self._g_min = [10 ** 9, 10 ** 9]
        self._g_max = [-10 ** 9, -10 ** 9]
        self._g_dropped = 0
        self._g_version = 0
        self._last_saved_version = -1
        self._mapping_active = True
        response.success = True
        response.message = ('Mapping started; global grid cleared, '
                            'auto-save is %s'
                            % ('on' if self.auto_save_enable else 'off'))
        self.get_logger().info(response.message)
        return response

    def _stop_mapping_cb(self, request, response):
        self._mapping_active = False
        if not self._g_occ and not self._g_free:
            response.success = True
            response.message = 'Mapping stopped; no map data to save'
            self.get_logger().warn(response.message)
            return response
        name = None
        if self.versioned_on_stop:
            name = '%s_%s' % (self.map_name,
                              time.strftime('%Y%m%d_%H%M%S'))
        success, message = self._save_map('stop_mapping', map_name=name)
        response.success = success
        response.message = 'Mapping stopped; ' + message
        if success:
            self.get_logger().info(response.message)
        else:
            self.get_logger().warn(response.message)
        return response

    def _auto_save_timer(self):
        if not self._mapping_active:
            return
        if not self.auto_save_enable:
            return
        if self._g_version == self._last_saved_version:
            return
        if len(self._g_occ) < self.auto_save_min_obstacles:
            return
        success, message = self._save_map('auto')
        if success:
            self.get_logger().info(message)
        else:
            self.get_logger().warn(message)

    def destroy_node(self):
        if (self.auto_save_enable and self.auto_save_on_shutdown
                and self._g_version != self._last_saved_version):
            success, message = self._save_map('shutdown')
            if success:
                self.get_logger().info(message)
            else:
                self.get_logger().warn(message)
        super().destroy_node()


def main(args=None):
    rclpy.init(args=args)
    node = PointCloudToGrid()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
