#!/usr/bin/env python3
"""旋转字段自动扫描：找出 MiniPC 到底用哪个字段控制转向。

背景
----
导航帧里有 ``yaw_speed`` 和 ``w_speed`` 两个角速度字段。不同底盘/固件
可能只认其中一个，或者两个都要写。当前的现象是“能前进、不能旋转”，
所以需要把下面几种组合逐个试一遍，并用 FAST-LIO 的 ``/Odometry`` 测量
真实偏航变化：

    w_speed   正/负
    yaw_speed 正/负
    both      正/负

运行前提
--------
1. FAST-LIO 已启动，``/Odometry`` 有数据；
2. ``chassis_bridge`` 已停止，串口没有被占用；
3. ``bt_main`` / 导航没有运行，``/cmd_vel`` 没有其他发布者；
4. 车轮落地，周围空旷，有人值守，随时可以断电/急停。

示例
----
    ros2 run chassis chassis_rotation_sweep --ros-args \
      -p port:=/dev/serial/by-id/usb-STMicroelectronics_STM32_Virtual_ComPort_206532874D31-if00 \
      -p angular_speed:=0.30 \
      -p duration:=5.0

如果怀疑方向反了，默认会同时测试正负两个方向。只想测正向：

    -p signs:=1

测试时间不要低于 4 秒，否则小角速度下偏航变化可能低于噪声。
"""

import csv
import datetime
import math
import os
import sys
import time

import rclpy
from rclpy.node import Node

from nav_msgs.msg import Odometry
from std_msgs.msg import Bool, String

try:
    import serial
except ImportError:
    serial = None

from chassis.chassis_bridge import encode_navigation_frame


def angle_diff(a, b):
    """返回 a-b 归一化到 [-pi, pi]。"""
    return math.atan2(math.sin(a - b), math.cos(a - b))


def yaw_from_quat(q):
    return math.atan2(2.0 * (q.w * q.z + q.x * q.y),
                      1.0 - 2.0 * (q.y * q.y + q.z * q.z))


class RotationSweep(Node):
    def __init__(self):
        super().__init__('chassis_rotation_sweep')

        self.declare_parameter('port', '/dev/ttyACM0')
        self.declare_parameter('baud', 115200)
        self.declare_parameter('angular_speed', 0.30)
        self.declare_parameter('duration', 5.0)
        self.declare_parameter('settle_time', 2.0)
        # 必须接近 chassis_bridge 的 50 Hz；20 Hz 可能被 MiniPC 当成超时
        self.declare_parameter('rate', 50.0)

        self.declare_parameter('fields', 'w_speed,yaw_speed,both')
        # sign_mode: 0=正负都测，1=只测正向，-1=只测负向
        self.declare_parameter('sign_mode', 0)
        # linear_speed > 0 时同时给前进速度，用于测试阿克曼/不能原地转的底盘
        self.declare_parameter('linear_speed', 0.0)
        self.declare_parameter('min_yaw_delta_deg', 10.0)
        self.declare_parameter('force', False)

        self.declare_parameter('frame_sof', 0x55)
        self.declare_parameter('frame_id', 0x04)
        self.declare_parameter('frame_eof', 0xFF)
        self.declare_parameter('byte_order', 'little')
        self.declare_parameter('scan_mode', False)

        self.declare_parameter('odom_topic', '/Odometry')
        self.declare_parameter('cmd_vel_topic', '/cmd_vel')

        self._yaw = None
        self._last_yaw = None
        self._cum_yaw = 0.0
        self._odom_count = 0
        self._max_rate = 0.0

        self.create_subscription(
            Odometry, str(self.get_parameter('odom_topic').value),
            self._odom_cb, 10)
        self.estop_pub = self.create_publisher(Bool, '/estop', 10)
        self.nav_cmd_pub = self.create_publisher(String, '/nav_command', 10)

        self._serial = None

    # ---------------------------------------------------------------- ROS
    def _odom_cb(self, msg):
        yaw = yaw_from_quat(msg.pose.pose.orientation)
        if self._last_yaw is not None:
            self._cum_yaw += angle_diff(yaw, self._last_yaw)
            self._max_rate = max(
                self._max_rate, abs(msg.twist.twist.angular.z))
        self._last_yaw = yaw
        self._yaw = yaw
        self._odom_count += 1

    def spin_for(self, seconds):
        deadline = time.monotonic() + seconds
        while time.monotonic() < deadline:
            rclpy.spin_once(self, timeout_sec=0.02)

    # ---------------------------------------------------------------- 串口
    def open_serial(self):
        if serial is None:
            self.get_logger().error(
                'pyserial 未安装：sudo apt install python3-serial')
            return False
        port = str(self.get_parameter('port').value)
        try:
            self._serial = serial.Serial(
                port, int(self.get_parameter('baud').value), timeout=0)
            self.get_logger().info('串口已打开：%s' % port)
            return True
        except Exception as exc:
            self.get_logger().error(
                '打开串口失败：%s（通常表示 chassis_bridge 仍在运行）' % exc)
            return False

    def frame(self, field, value):
        x = float(self.get_parameter('linear_speed').value)
        scan = bool(self.get_parameter('scan_mode').value)
        sof = int(self.get_parameter('frame_sof').value)
        frame_id = int(self.get_parameter('frame_id').value)
        eof = int(self.get_parameter('frame_eof').value)
        order = str(self.get_parameter('byte_order').value)
        if field == 'w_speed':
            return encode_navigation_frame(
                x, 0.0, 0.0, value, scan,
                sof=sof, frame_id=frame_id, eof=eof, byte_order=order)
        if field == 'yaw_speed':
            return encode_navigation_frame(
                x, 0.0, value, 0.0, scan,
                sof=sof, frame_id=frame_id, eof=eof, byte_order=order)
        return encode_navigation_frame(
            x, 0.0, value, value, scan,
            sof=sof, frame_id=frame_id, eof=eof, byte_order=order)

    def send_zero(self, count=5):
        if self._serial is None:
            return
        zero = encode_navigation_frame(
            0.0, 0.0, 0.0, 0.0, bool(self.get_parameter('scan_mode').value),
            sof=int(self.get_parameter('frame_sof').value),
            frame_id=int(self.get_parameter('frame_id').value),
            eof=int(self.get_parameter('frame_eof').value),
            byte_order=str(self.get_parameter('byte_order').value))
        for _ in range(count):
            try:
                self._serial.write(zero)
                self._serial.flush()
            except Exception:
                break
            time.sleep(0.02)

    # ---------------------------------------------------------------- 测试
    def measure(self, field, value, duration):
        self._cum_yaw = 0.0
        self._max_rate = 0.0
        self._last_yaw = self._yaw

        data = self.frame(field, value)
        period = 1.0 / max(1.0, float(self.get_parameter('rate').value))
        deadline = time.monotonic() + max(0.1, duration)
        next_tx = time.monotonic()

        while time.monotonic() < deadline:
            now = time.monotonic()
            if now >= next_tx:
                try:
                    self._serial.write(data)
                    self._serial.flush()
                except Exception as exc:
                    self.get_logger().error('串口写入失败：%s' % exc)
                    break
                next_tx = now + period
            rclpy.spin_once(self, timeout_sec=0.01)

        # 只统计发速度帧期间的偏航，不把停止后的惯性/滑行算进去
        command_delta = self._cum_yaw
        self.send_zero()
        settle = float(self.get_parameter('settle_time').value)
        self.spin_for(settle)
        delta_deg = math.degrees(command_delta)
        rate = self._max_rate
        if rate < 1e-6 and duration > 0.0:
            rate = abs(math.radians(delta_deg)) / duration
        return delta_deg, rate

    def run(self):
        if not bool(self.get_parameter('force').value):
            publishers = self.count_publishers(
                str(self.get_parameter('cmd_vel_topic').value))
            if publishers > 0:
                self.get_logger().error(
                    '/cmd_vel 还有 %d 个发布者，先停止 bt_main/导航，'
                    '或设置 -p force:=true 强制测试。' % publishers)
                return False

        self.nav_cmd_pub.publish(String(data='cancel'))
        self.estop_pub.publish(Bool(data=False))
        self.spin_for(2.0)

        if self._odom_count == 0:
            self.get_logger().error(
                '没有收到 %s，无法测量真实偏航。先启动 FAST-LIO。'
                % str(self.get_parameter('odom_topic').value))
            return False
        self.get_logger().info(
            '起始 yaw=%.2f°，开始扫描（每项 %.1fs）'
            % (math.degrees(self._yaw),
               float(self.get_parameter('duration').value)))

        if not self.open_serial():
            return False

        fields = [f.strip() for f in
                  str(self.get_parameter('fields').value).split(',') if f.strip()]
        sign_mode = int(self.get_parameter('sign_mode').value)
        if sign_mode > 0:
            signs = [1.0]
        elif sign_mode < 0:
            signs = [-1.0]
        else:
            signs = [1.0, -1.0]
        if not fields:
            self.get_logger().error('fields 参数为空')
            return False

        speed = abs(float(self.get_parameter('angular_speed').value))
        linear = float(self.get_parameter('linear_speed').value)
        duration = float(self.get_parameter('duration').value)
        if abs(linear) > 1e-6:
            self.get_logger().info(
                '同时给前进速度 %.3f m/s（弧形测试）' % linear)
        results = []

        try:
            for field in fields:
                for sign in signs:
                    value = sign * speed
                    self.get_logger().info(
                        '测试 %s = %+.3f rad/s ...' % (field, value))
                    delta_deg, max_rate = self.measure(field, value, duration)
                    results.append((field, sign, delta_deg, max_rate))
                    self.get_logger().info(
                        '  -> delta_yaw=%+.2f° max_rate=%.3f rad/s'
                        % (delta_deg, max_rate))
        except KeyboardInterrupt:
            self.get_logger().warn('用户中断，发送零速度帧')
        finally:
            self.send_zero(10)
            if self._serial is not None:
                try:
                    self._serial.close()
                except Exception:
                    pass
                self._serial = None

        if not results:
            return False

        threshold = float(self.get_parameter('min_yaw_delta_deg').value)
        best = max(results, key=lambda r: abs(r[2]))
        self.get_logger().info('扫描结果：')
        for field, sign, delta_deg, max_rate in results:
            ok = abs(delta_deg) >= threshold
            self.get_logger().info(
                '  %-9s sign=%+d delta=%+7.2f° max_rate=%.3f %s'
                % (field, int(sign), delta_deg, max_rate,
                   'PASS' if ok else 'FAIL'))

        if abs(best[2]) < threshold:
            self.get_logger().error(
                '所有字段都没有产生足够旋转。MiniPC 很可能没有执行导航帧：'
                '检查 NAV/遥控模式、使能、串口波特率，或固件是否需要额外的'
                '模式帧/握手帧。')
            return False

        self.get_logger().info(
            '最佳字段：%s sign=%+d delta=%+.2f°'
            % (best[0], int(best[1]), best[2]))
        # 命令符号和实际偏航符号相反时，需要 invert_angular
        need_invert = (best[1] * best[2]) < 0.0
        self.get_logger().info(
            '建议：chassis_angular_target:=%s%s'
            % (best[0],
               '，并设置 invert_angular:=true' if need_invert else ''))

        self._write_csv(results)
        return True

    def _write_csv(self, results):
        path = os.path.expanduser(
            '~/.ros/chassis_rotation_sweep_%s.csv'
            % datetime.datetime.now().strftime('%Y%m%d_%H%M%S'))
        try:
            os.makedirs(os.path.dirname(path), exist_ok=True)
            with open(path, 'w', newline='') as handle:
                writer = csv.writer(handle)
                writer.writerow(['field', 'sign', 'delta_yaw_deg', 'max_rate'])
                for row in results:
                    writer.writerow(row)
            self.get_logger().info('结果已写入 %s' % path)
        except Exception as exc:
            self.get_logger().warn('写 CSV 失败：%s' % exc)


def main(args=None):
    rclpy.init(args=args)
    node = RotationSweep()
    ok = False
    try:
        ok = node.run()
    except KeyboardInterrupt:
        pass
    finally:
        try:
            node.estop_pub.publish(Bool(data=True))
            node.spin_for(0.2)
        except Exception:
            pass
        node.destroy_node()
        try:
            if rclpy.ok():
                rclpy.shutdown()
        except Exception:
            pass
    sys.exit(0 if ok else 1)


if __name__ == '__main__':
    main()
