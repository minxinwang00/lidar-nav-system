#!/usr/bin/env python3
"""底盘运动闭环测试工具。

使用 FAST-LIO 的 /Odometry 作为外部运动参考，测试：
  1. 前进是否真的产生位移；
  2. 原地旋转是否真的产生偏航变化。

运行前：
  - 只启动 chassis_bridge（不要启动 navigation.launch.py，否则 bt_main
    会同时发布 /cmd_vel，测试结果无效）；
  - 车轮落地，前方空旷，有人值守，随时可急停。

示例：
  ros2 run chassis chassis_motion_test \
      --ros-args \
      -p forward_speed:=0.1 \
      -p forward_duration:=10.0 \
      -p angular_speed:=0.15 \
      -p angular_duration:=10.0
"""

import math
import time

import rclpy
from rclpy.node import Node

from geometry_msgs.msg import Twist
from nav_msgs.msg import Odometry
from std_msgs.msg import Bool, String


def yaw_from_quat(q):
    return math.atan2(2.0 * (q.w * q.z + q.x * q.y),
                      1.0 - 2.0 * (q.y * q.y + q.z * q.z))


def angle_diff(a, b):
    return math.atan2(math.sin(a - b), math.cos(a - b))


class MotionTest(Node):
    def __init__(self):
        super().__init__('chassis_motion_test')
        self.declare_parameter('forward_speed', 0.1)
        self.declare_parameter('angular_speed', 0.15)
        self.declare_parameter('forward_duration', 10.0)
        self.declare_parameter('angular_duration', 10.0)
        self.declare_parameter('settle_time', 1.0)
        self.declare_parameter('odom_topic', '/Odometry')
        self.declare_parameter('cmd_topic', '/cmd_vel')
        self.declare_parameter('status_topic', '/chassis_status')
        self.declare_parameter('min_forward_delta', 0.05)
        self.declare_parameter('min_yaw_delta_deg', 3.0)

        self.cmd_pub = self.create_publisher(
            Twist, str(self.get_parameter('cmd_topic').value), 10)
        self.estop_pub = self.create_publisher(Bool, '/estop', 10)
        self.nav_cmd_pub = self.create_publisher(String, '/nav_command', 10)

        self._odom = None
        self._status = ''
        self.create_subscription(
            Odometry, str(self.get_parameter('odom_topic').value),
            self._odom_cb, 10)
        self.create_subscription(
            String, str(self.get_parameter('status_topic').value),
            self._status_cb, 10)

    def _odom_cb(self, msg):
        self._odom = msg

    def _status_cb(self, msg):
        self._status = msg.data

    def spin_for(self, seconds):
        deadline = time.monotonic() + seconds
        while time.monotonic() < deadline:
            rclpy.spin_once(self, timeout_sec=0.02)

    def snapshot(self):
        self.spin_for(0.5)
        if self._odom is None:
            return None
        p = self._odom.pose.pose.position
        q = self._odom.pose.pose.orientation
        return (p.x, p.y, yaw_from_quat(q))

    def command(self, vx, wz, duration):
        twist = Twist()
        twist.linear.x = float(vx)
        twist.angular.z = float(wz)
        deadline = time.monotonic() + max(0.1, duration)
        next_pub = time.monotonic()
        while time.monotonic() < deadline:
            now = time.monotonic()
            if now >= next_pub:
                self.cmd_pub.publish(twist)
                next_pub = now + 0.1
            rclpy.spin_once(self, timeout_sec=0.01)
        status_during = self._status
        self.cmd_pub.publish(Twist())
        self.spin_for(float(self.get_parameter('settle_time').value))
        return status_during

    def run(self):
        self.nav_cmd_pub.publish(String(data='cancel'))
        self.estop_pub.publish(Bool(data=False))
        self.spin_for(0.5)

        start = self.snapshot()
        if start is None:
            self.get_logger().error(
                '没有收到 /Odometry，无法判断底盘是否运动。'
                '请先启动 FAST-LIO 或修改 odom_topic。')
            return False
        self.get_logger().info(
            'start x=%.3f y=%.3f yaw=%.1f°'
            % (start[0], start[1], math.degrees(start[2])))
        self.get_logger().info('chassis_status: %s' % self._status)

        fwd = float(self.get_parameter('forward_speed').value)
        fwd_t = float(self.get_parameter('forward_duration').value)
        self.get_logger().info('forward %.2f m/s for %.1fs' % (fwd, fwd_t))
        status_fwd = self.command(fwd, 0.0, fwd_t)
        after_fwd = self.snapshot()
        if after_fwd is None:
            return False
        dxy = math.hypot(after_fwd[0] - start[0],
                         after_fwd[1] - start[1])
        avg_v = dxy / max(1e-3, fwd_t)
        dx = after_fwd[0] - start[0]
        dy = after_fwd[1] - start[1]
        heading_err = None
        if dxy >= 0.05:
            motion_yaw = math.atan2(dy, dx)
            heading_err = math.degrees(angle_diff(motion_yaw, start[2]))
        heading_text = ('%+.1f°' % heading_err
                        if heading_err is not None else 'N/A（位移太小）')
        self.get_logger().info(
            'forward delta_xy=%.3f m avg=%.3f m/s (cmd=%.3f, scale=%.2f) '
            'heading_error=%s status_during=%s'
            % (dxy, avg_v, fwd, (avg_v / abs(fwd)) if abs(fwd) > 1e-3 else 0.0,
               heading_text, status_fwd))
        if heading_err is not None and abs(heading_err) > 15.0:
            self.get_logger().warn(
                '实际运动方向与 base_link 朝向相差 %.1f°：检查 MID-360 安装 yaw '
                '或 lio_odom_bridge 的 base_to_body_yaw 参数。' % heading_err)

        ang = float(self.get_parameter('angular_speed').value)
        ang_t = float(self.get_parameter('angular_duration').value)
        self.get_logger().info('angular %.2f rad/s for %.1fs' % (ang, ang_t))
        status_turn = self.command(0.0, ang, ang_t)
        after_turn = self.snapshot()
        if after_turn is None:
            return False
        dyaw = math.atan2(math.sin(after_turn[2] - after_fwd[2]),
                          math.cos(after_turn[2] - after_fwd[2]))
        dyaw_deg = abs(math.degrees(dyaw))
        avg_w = abs(math.radians(dyaw_deg)) / max(1e-3, ang_t)
        self.get_logger().info(
            'turn delta_yaw=%.2f° avg=%.3f rad/s (cmd=%.3f, scale=%.2f) '
            'status_during=%s'
            % (dyaw_deg, avg_w, abs(ang),
               (avg_w / abs(ang)) if abs(ang) > 1e-3 else 0.0,
               status_turn))

        min_fwd = float(self.get_parameter('min_forward_delta').value)
        min_yaw = float(self.get_parameter('min_yaw_delta_deg').value)
        ok_fwd = dxy >= min_fwd
        ok_turn = dyaw_deg >= min_yaw
        self.get_logger().info(
            'RESULT forward=%s turn=%s'
            % ('PASS' if ok_fwd else 'FAIL',
               'PASS' if ok_turn else 'FAIL'))
        if not ok_fwd:
            self.get_logger().warn(
                '前进没有达到阈值：检查串口/权限/MiniPC 是否执行导航帧')
        if not ok_turn:
            self.get_logger().warn(
                '旋转没有达到阈值：检查底盘类型、w_speed/yaw_speed、'
                '是否支持原地旋转')
        return ok_fwd and ok_turn


def main(args=None):
    rclpy.init(args=args)
    node = MotionTest()
    try:
        ok = node.run()
    except KeyboardInterrupt:
        ok = False
    finally:
        try:
            node.estop_pub.publish(Bool(data=True))
            node.spin_for(0.2)
        except Exception:
            pass
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()
    raise SystemExit(0 if ok else 1)


if __name__ == '__main__':
    main()
