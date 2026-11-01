#!/usr/bin/env python3
"""把 FAST-LIO 的里程计接入决策层所期望的 TF 树。

FAST-LIO 原生发布：
  TF   camera_init -> body        以及 /Odometry（nav_msgs/Odometry）
本节点补上缺失的两级上层坐标变换：
  TF   map  -> odom               （单位变换或偏移，静态）
  TF   odom -> base_link          （由 /Odometry 得到，频率为 tf_rate）
  TF   map  -> camera_init        （单位变换，静态；仅为让 /cloud_registered
                                   能在 map 坐标系下使用）
并把 /Odometry 以 odom / base_link 为坐标系重新发布为 /odom。

如果 base_link 不在 IMU 原点处，请设置 base_to_body_xyz / _yaw，
odom->base_link 变换就会包含这个偏移（由 body 位姿换算到 base 位姿）。
"""

import math

import rclpy
from rclpy.duration import Duration
from rclpy.node import Node
from rclpy.qos import DurabilityPolicy, HistoryPolicy, QoSProfile, ReliabilityPolicy
from rclpy.time import Time

import tf2_ros
from geometry_msgs.msg import PoseStamped, TransformStamped
from nav_msgs.msg import Odometry


def yaw_to_quat(yaw):
    return (0.0, 0.0, math.sin(yaw * 0.5), math.cos(yaw * 0.5))


def normalize_quat(q):
    n = math.sqrt(q.x * q.x + q.y * q.y + q.z * q.z + q.w * q.w)
    if n < 1e-9:
        return (0.0, 0.0, 0.0, 1.0)
    return (q.x / n, q.y / n, q.z / n, q.w / n)


class LioOdomBridge(Node):
    def __init__(self):
        super().__init__('lio_odom_bridge')

        self.declare_parameter('odom_in_topic', '/Odometry')
        self.declare_parameter('odom_out_topic', '/odom')
        self.declare_parameter('map_frame', 'map')
        self.declare_parameter('odom_frame', 'odom')
        self.declare_parameter('base_frame', 'base_link')
        self.declare_parameter('camera_init_frame', 'camera_init')
        self.declare_parameter('publish_camera_init_tf', True)
        self.declare_parameter('camera_init_parent', 'odom')
        # 当 map -> odom 由 nav2_amcl 负责发布时设为 false（它自己会发该 TF）
        self.declare_parameter('publish_map_to_odom', True)
        self.declare_parameter('tf_rate', 50.0)
        # map -> odom 修正量（AMCL 未启用时使用）
        self.declare_parameter('map_to_odom_x', 0.0)
        self.declare_parameter('map_to_odom_y', 0.0)
        self.declare_parameter('map_to_odom_yaw', 0.0)
        # body（IMU）位姿到 base_link 位姿的偏移，在 body 坐标系下表达
        self.declare_parameter('base_to_body_x', 0.0)
        self.declare_parameter('base_to_body_y', 0.0)
        self.declare_parameter('base_to_body_z', 0.0)
        self.declare_parameter('base_to_body_yaw', 0.0)

        self.map_frame = str(self.get_parameter('map_frame').value)
        self.odom_frame = str(self.get_parameter('odom_frame').value)
        self.base_frame = str(self.get_parameter('base_frame').value)
        self.tf_rate = float(self.get_parameter('tf_rate').value)

        self._odom = None

        self.tf_buffer = tf2_ros.Buffer()
        self.tf_listener = tf2_ros.TransformListener(self.tf_buffer, self)
        self.tf_broadcaster = tf2_ros.TransformBroadcaster(self)
        self.static_broadcaster = tf2_ros.StaticTransformBroadcaster(self)
        self._publish_static()

        qos = QoSProfile(depth=20,
                         reliability=ReliabilityPolicy.RELIABLE,
                         history=HistoryPolicy.KEEP_LAST)
        self.create_subscription(Odometry,
                                 str(self.get_parameter('odom_in_topic').value),
                                 self._odom_cb, qos)
        self.odom_pub = self.create_publisher(
            Odometry, str(self.get_parameter('odom_out_topic').value), 20)
        self.pose_pub = self.create_publisher(PoseStamped, '/robot_pose', 10)

        self.create_timer(1.0 / max(1.0, self.tf_rate), self._tf_timer)
        self.create_timer(0.1, self._pose_timer)

    # ------------------------------------------------------------------ 辅助方法
    def _make_static(self, parent, child, x, y, z, yaw):
        t = TransformStamped()
        t.header.stamp = self.get_clock().now().to_msg()
        t.header.frame_id = parent
        t.child_frame_id = child
        t.transform.translation.x = float(x)
        t.transform.translation.y = float(y)
        t.transform.translation.z = float(z)
        qx, qy, qz, qw = yaw_to_quat(yaw)
        t.transform.rotation.x, t.transform.rotation.y = qx, qy
        t.transform.rotation.z, t.transform.rotation.w = qz, qw
        return t

    def _publish_static(self):
        transforms = []
        if bool(self.get_parameter('publish_map_to_odom').value):
            transforms.append(self._make_static(
                self.map_frame, self.odom_frame,
                self.get_parameter('map_to_odom_x').value,
                self.get_parameter('map_to_odom_y').value,
                0.0,
                self.get_parameter('map_to_odom_yaw').value))
        if bool(self.get_parameter('publish_camera_init_tf').value):
            # 把 camera_init 挂在 odom 之下，这样 AMCL 修正后的
            # map->odom 变换也会同时带动 LiDAR 的世界坐标系
            transforms.append(self._make_static(
                str(self.get_parameter('camera_init_parent').value),
                str(self.get_parameter('camera_init_frame').value),
                0.0, 0.0, 0.0, 0.0))
        if transforms:
            self.static_broadcaster.sendTransform(transforms)

    # ------------------------------------------------------------------ 回调函数
    def _odom_cb(self, msg):
        self._odom = msg
        out = Odometry()
        out.header.stamp = msg.header.stamp
        out.header.frame_id = self.odom_frame
        out.child_frame_id = self.base_frame
        out.pose = msg.pose
        out.twist = msg.twist
        self.odom_pub.publish(out)

    def _pose_timer(self):
        """从最终的 TF 树（已经 AMCL 修正）中取出位姿并发布 /robot_pose。"""
        try:
            tr = self.tf_buffer.lookup_transform(
                self.map_frame, self.base_frame, Time(), Duration(seconds=0.05))
        except Exception:
            return
        pose = PoseStamped()
        pose.header.stamp = self.get_clock().now().to_msg()
        pose.header.frame_id = self.map_frame
        pose.pose.position.x = tr.transform.translation.x
        pose.pose.position.y = tr.transform.translation.y
        pose.pose.position.z = tr.transform.translation.z
        pose.pose.orientation = tr.transform.rotation
        self.pose_pub.publish(pose)

    def _tf_timer(self):
        if self._odom is None:
            return
        p = self._odom.pose.pose.position
        q = self._odom.pose.pose.orientation
        qx, qy, qz, qw = normalize_quat(q)

        # 把 base_to_body 偏移（在 body 系下表达）旋转到 odom 系
        bx = float(self.get_parameter('base_to_body_x').value)
        by = float(self.get_parameter('base_to_body_y').value)
        byaw = float(self.get_parameter('base_to_body_yaw').value)
        yaw = math.atan2(2.0 * (qw * qz + qx * qy), 1.0 - 2.0 * (qy * qy + qz * qz))
        cy, sy = math.cos(yaw), math.sin(yaw)
        wx = p.x + cy * bx - sy * by
        wy = p.y + sy * bx + cy * by
        qx2, qy2, qz2, qw2 = yaw_to_quat(yaw + byaw)

        t = TransformStamped()
        t.header.stamp = self._odom.header.stamp
        t.header.frame_id = self.odom_frame
        t.child_frame_id = self.base_frame
        t.transform.translation.x = wx
        t.transform.translation.y = wy
        t.transform.translation.z = p.z + float(
            self.get_parameter('base_to_body_z').value)
        t.transform.rotation.x = qx2
        t.transform.rotation.y = qy2
        t.transform.rotation.z = qz2
        t.transform.rotation.w = qw2
        self.tf_broadcaster.sendTransform(t)


def main(args=None):
    rclpy.init(args=args)
    node = LioOdomBridge()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
