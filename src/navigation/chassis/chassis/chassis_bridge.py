#!/usr/bin/env python3
"""底盘桥接节点：把 /cmd_vel 转换为 MiniPC 的二进制导航帧（或 ASCII 协议）。

二进制帧
--------
C++ 侧的结构体定义如下：

    struct __attribute__((packed)) VisionToGimbal {
        uint8_t SOF;        // 0x55
        uint8_t ID;         // 0x04 = kNavigationControlId
        float   x_speed;
        float   y_speed;
        float   yaw_speed;
        float   w_speed;
        bool    scan_mod_type;
        uint8_t Eof;        // 0xFF
    };

在小端（little-endian）主机上的紧凑内存布局（共 20 字节）：

    偏移    长度   字段
    0       1     SOF             0x55
    1       1     ID              0x04
    2       4     x_speed   (f32)
    6       4     y_speed   (f32)
    10      4     yaw_speed (f32)
    14      4     w_speed   (f32)
    18      1     scan_mod_type (bool, 0/1)
    19      1     Eof             0xFF

与 geometry_msgs/Twist 的映射关系
--------------------------------
    linear.x  -> x_speed
    linear.y  -> y_speed            （麦轮 / 全向底盘使用；差速底盘填 0）
    angular.z -> yaw_speed          （本车实测：只有 yaw_speed 能让底盘旋转）
                 或 w_speed / both   （由 angular_z_target 参数选择）
    scan_mod_type <- /scan_mode（std_msgs/Bool）或 scan_mode 参数

本车实测（2026-11-01）：
    - w_speed 不产生旋转；
    - yaw_speed 能旋转，但正命令产生负偏航，因此默认 invert_angular=True。

该帧按 publish_rate 周期性发送；若 cmd_timeout 时间内没有收到新的 /cmd_vel，
桥接节点会持续发送零速度帧（失效保护由 MCU 负责）。

ASCII 模式（protocol=ascii）为兼容旧版控制板而保留：
    PC -> MCU : $CMD,<vx_mm_s>,<wz_mrad_s>*XX\\n
                $STOP*XX\\n
    MCU -> PC : $ODOM,<vx_mm_s>,<wz_mrad_s>*XX\\n

话题
    订阅：/cmd_vel、/estop、/scan_mode
    发布：/wheel_odom、/chassis_status
"""

import glob
import math
import os
import struct
import threading
import time

import rclpy
from rclpy.executors import ExternalShutdownException
from rclpy.node import Node

from geometry_msgs.msg import Twist
from nav_msgs.msg import Odometry
from std_msgs.msg import Bool, String

try:
    import serial
except ImportError:      # 未安装 pyserial 时也要保证模块可导入
    serial = None


# SOF + ID + 4 个 float + bool + EOF。'<' 禁用所有字节对齐填充，以与 C++ 的
# __attribute__((packed)) 布局保持一致。
_BINARY_FORMATS = {
    'little': struct.Struct('<BBffff?B'),
    'big': struct.Struct('>BBffff?B'),
}


def encode_navigation_frame(x_speed, y_speed, yaw_speed, w_speed, scan_mode,
                            sof=0x55, frame_id=0x04, eof=0xFF,
                            byte_order='little'):
    """按 MiniPC 的 VisionToGimbal 结构体打包一帧数据（20 字节）。"""
    fmt = _BINARY_FORMATS.get(byte_order, _BINARY_FORMATS['little'])
    return fmt.pack(int(sof), int(frame_id),
                    float(x_speed), float(y_speed),
                    float(yaw_speed), float(w_speed),
                    bool(scan_mode), int(eof))


def checksum(payload):
    value = 0
    for ch in payload:
        value ^= ord(ch)
    return value


def clamp(value, limit):
    return max(-limit, min(limit, value))


class ChassisBridge(Node):
    def __init__(self):
        super().__init__('chassis_bridge')

        # ---------------------------------------------------------- 参数声明
        self.declare_parameter('use_serial', True)
        self.declare_parameter('fallback_to_fake', True)
        self.declare_parameter('port', '/dev/minipc')
        # 配置端口不存在时，自动从 by-id 里找 STM32 CDC，避免 ttyACM 编号变化
        self.declare_parameter('auto_detect_port', True)
        self.declare_parameter(
            'port_glob', '/dev/serial/by-id/*STM32_Virtual_ComPort*')
        self.declare_parameter('baud', 115200)
        self.declare_parameter('protocol', 'binary')      # 取值：binary（二进制帧）| ascii（ASCII 协议）
        self.declare_parameter('feedback_protocol', 'none')  # 反馈协议：none（不解析）| ascii

        # 二进制帧相关参数
        self.declare_parameter('frame_sof', 0x55)
        self.declare_parameter('frame_id', 0x04)
        self.declare_parameter('frame_eof', 0xFF)
        self.declare_parameter('byte_order', 'little')
        # 角速度写入哪个字段：w_speed | yaw_speed | both；本车实测 yaw_speed
        self.declare_parameter('angular_z_target', 'yaw_speed')
        self.declare_parameter('gimbal_yaw_speed', 0.0)
        self.declare_parameter('scan_mode', False)
        self.declare_parameter('scan_mode_topic', '/scan_mode')
        self.declare_parameter('protocol_max_speed', 2.0)

        # 限幅与时间参数
        self.declare_parameter('max_linear', 0.5)
        self.declare_parameter('max_lateral', 0.5)
        self.declare_parameter('max_angular', 1.5)
        # 本车实测：x_speed 正方向会让车倒着走，需要取反
        self.declare_parameter('invert_linear', True)
        self.declare_parameter('invert_lateral', False)
        # 本车实测：正的 yaw_speed 产生负偏航，需要取反
        self.declare_parameter('invert_angular', True)
        self.declare_parameter('cmd_timeout', 0.5)
        self.declare_parameter('publish_rate', 50.0)
        # 调试：周期性打印实际写出的 20 字节帧，用于和同学/厂商抓包对比
        self.declare_parameter('debug_frames', False)
        self.declare_parameter('debug_frame_period', 1.0)

        # 里程计参数
        self.declare_parameter('odom_frame', 'odom')
        self.declare_parameter('base_frame', 'base_link')
        self.declare_parameter('publish_wheel_tf', False)

        self.protocol = str(self.get_parameter('protocol').value).lower()
        if self.protocol not in ('binary', 'ascii'):
            self.protocol = 'binary'
        self.feedback = str(self.get_parameter('feedback_protocol').value).lower()
        self.byte_order = str(self.get_parameter('byte_order').value).lower()
        self.angular_z_target = str(
            self.get_parameter('angular_z_target').value).lower()

        self.max_v = float(self.get_parameter('max_linear').value)
        self.max_lat = float(self.get_parameter('max_lateral').value)
        self.max_w = float(self.get_parameter('max_angular').value)
        self.protocol_max = float(self.get_parameter('protocol_max_speed').value)
        self.timeout = float(self.get_parameter('cmd_timeout').value)
        self.odom_frame = str(self.get_parameter('odom_frame').value)
        self.base_frame = str(self.get_parameter('base_frame').value)
        self.scan_mode = bool(self.get_parameter('scan_mode').value)
        self.gimbal_yaw_speed = float(
            self.get_parameter('gimbal_yaw_speed').value)

        self._serial = None
        self._serial_lock = threading.Lock()
        self._reader_thread = None
        self._running = True
        self._fake = True
        self._status = 'init'

        self._rx_v = 0.0
        self._rx_w = 0.0
        self._rx_valid = False

        self._cmd_x = 0.0
        self._cmd_y = 0.0
        self._cmd_w = 0.0
        self._last_cmd_time = 0.0
        self._stopped = True
        self._estop = False
        self._timeout_logged = False
        self._last_debug_log = 0.0

        self._x = 0.0
        self._y = 0.0
        self._yaw = 0.0
        self._last_integrate = time.monotonic()

        if bool(self.get_parameter('use_serial').value):
            self._open_serial()

        self.create_subscription(Twist, '/cmd_vel', self._cmd_cb, 10)
        self.create_subscription(Bool, '/estop', self._estop_cb, 10)
        self.create_subscription(
            Bool, str(self.get_parameter('scan_mode_topic').value),
            self._scan_mode_cb, 10)

        self.odom_pub = self.create_publisher(Odometry, '/wheel_odom', 20)
        self.status_pub = self.create_publisher(String, '/chassis_status', 10)

        if bool(self.get_parameter('publish_wheel_tf').value):
            import tf2_ros
            from geometry_msgs.msg import TransformStamped
            self._tf = tf2_ros.TransformBroadcaster(self)
            self._TransformStamped = TransformStamped
        else:
            self._tf = None

        rate = max(10.0, float(self.get_parameter('publish_rate').value))
        self.create_timer(1.0 / rate, self._timer)
        self.create_timer(1.0, self._status_timer)

        self.get_logger().info(
            'Chassis bridge: protocol=%s frame=0x%02X id=0x%02X %s-endian '
            'angular.z->%s'
            % (self.protocol,
               int(self.get_parameter('frame_sof').value),
               int(self.get_parameter('frame_id').value),
               self.byte_order, self.angular_z_target))

    # ------------------------------------------------------------------ 串口操作
    def _resolve_port(self, port):
        """优先用配置端口；不存在时按 by-id 自动找 STM32 CDC。"""
        if os.path.exists(port):
            return port
        if not bool(self.get_parameter('auto_detect_port').value):
            return port
        pattern = str(self.get_parameter('port_glob').value)
        matches = sorted(glob.glob(pattern))
        if not matches:
            return port
        resolved = matches[0]
        if len(matches) > 1:
            self.get_logger().warn(
                'Found %d serial devices, using %s: %s'
                % (len(matches), resolved, ', '.join(matches)))
        else:
            self.get_logger().warn(
                'Configured port %s missing, auto-detected %s'
                % (port, resolved))
        return resolved

    def _open_serial(self):
        configured_port = str(self.get_parameter('port').value)
        port = self._resolve_port(configured_port)
        baud = int(self.get_parameter('baud').value)
        if serial is None:
            self.get_logger().error(
                'pyserial not installed (sudo apt install python3-serial)')
            self._use_fake_or_die()
            return
        try:
            self._serial = serial.Serial(port, baud, timeout=0.05)
            self._fake = False
            self._status = 'serial open %s' % port
            self.get_logger().info('Chassis serial opened: %s @ %d' % (port, baud))
            self._reader_thread = threading.Thread(
                target=self._read_loop, daemon=True)
            self._reader_thread.start()
        except Exception as exc:
            self.get_logger().error('Cannot open %s: %s' % (port, exc))
            self.get_logger().error(
                'ttyACM/ttyUSB 编号会变化，请用稳定路径：'
                'ls -l /dev/serial/by-id/')
            self._use_fake_or_die()

    def _use_fake_or_die(self):
        if bool(self.get_parameter('fallback_to_fake').value):
            self._fake = True
            self._status = 'FAKE (no serial)'
            self.get_logger().warn('Chassis falling back to simulation mode')
        else:
            raise RuntimeError('Chassis serial unavailable')

    def _read_loop(self):
        while self._running and rclpy.ok():
            ser = self._serial
            if ser is None:
                break
            try:
                if self.feedback == 'ascii':
                    line = ser.readline().decode(
                        'ascii', errors='ignore').strip()
                    if line:
                        self._parse_line(line)
                else:
                    # 非阻塞读：只把已有遥测读走，避免阻塞写循环
                    waiting = getattr(ser, 'in_waiting', 0)
                    if waiting:
                        ser.read(waiting)
                    else:
                        time.sleep(0.02)
            except Exception as exc:
                if not self._running:
                    break
                self.get_logger().warn('Serial read error: %s' % exc)
                time.sleep(0.2)

    def _parse_line(self, line):
        if not line.startswith('$') or '*' not in line:
            return
        payload, _, cs = line[1:].partition('*')
        try:
            if int(cs, 16) != checksum(payload):
                return
            fields = payload.split(',')
            if fields[0] == 'ODOM' and len(fields) >= 3:
                self._rx_v = int(fields[1]) / 1000.0
                self._rx_w = int(fields[2]) / 1000.0
                self._rx_valid = True
                self._status = 'odom'
            elif fields[0] == 'STAT':
                self._status = 'mcu:' + fields[1]
        except (ValueError, IndexError):
            return

    def _write(self, data):
        if self._serial is None:
            return
        try:
            with self._serial_lock:
                self._serial.write(data)
        except Exception as exc:
            self.get_logger().warn('Serial write error: %s' % exc)

    def _send_ascii(self, payload):
        frame = '$%s*%02X\n' % (payload, checksum(payload))
        self._write(frame.encode('ascii'))

    def _send_ascii_stop(self):
        self._send_ascii('STOP')
        self._stopped = True

    # ------------------------------------------------------------------ 数据帧
    def _frame_fields(self, now):
        """完成映射与安全限幅后返回 (x, y, yaw, w)。"""
        timed_out = (now - self._last_cmd_time) > self.timeout
        if self._estop or timed_out:
            return 0.0, 0.0, 0.0, 0.0

        vx = self._cmd_x
        vy = self._cmd_y
        ang = self._cmd_w

        # 运行时读取这两个参数，方便用 “ros2 param set” 动态切换映射方式
        target = str(self.get_parameter('angular_z_target').value).lower()
        yaw = float(self.get_parameter('gimbal_yaw_speed').value)
        w = 0.0
        if target == 'w_speed':
            w = ang
        elif target == 'yaw_speed':
            yaw += ang
        else:  # 'both'：仅适用于同时接受这两个字段的控制板
            yaw += ang
            w = ang

        vx = clamp(vx, min(self.max_v, self.protocol_max))
        vy = clamp(vy, min(self.max_lat, self.protocol_max))
        yaw = clamp(yaw, min(self.max_w, self.protocol_max))
        w = clamp(w, min(self.max_w, self.protocol_max))
        return vx, vy, yaw, w

    def _send_binary_frame(self, now):
        vx, vy, yaw, w = self._frame_fields(now)
        data = encode_navigation_frame(
            vx, vy, yaw, w, self.scan_mode,
            sof=int(self.get_parameter('frame_sof').value),
            frame_id=int(self.get_parameter('frame_id').value),
            eof=int(self.get_parameter('frame_eof').value),
            byte_order=self.byte_order)
        self._write(data)
        if bool(self.get_parameter('debug_frames').value):
            period = max(0.05, float(
                self.get_parameter('debug_frame_period').value))
            if now - self._last_debug_log >= period:
                self._last_debug_log = now
                self.get_logger().info(
                    'TX %s | x=%.3f y=%.3f yaw=%.3f w=%.3f'
                    % (data.hex(' '), vx, vy, yaw, w))
        return vx, vy, yaw, w

    # ------------------------------------------------------------------ 各类回调
    def _cmd_cb(self, msg):
        sx = -1.0 if bool(self.get_parameter('invert_linear').value) else 1.0
        sy = -1.0 if bool(self.get_parameter('invert_lateral').value) else 1.0
        sw = -1.0 if bool(self.get_parameter('invert_angular').value) else 1.0
        self._cmd_x = clamp(sx * msg.linear.x, self.max_v)
        self._cmd_y = clamp(sy * msg.linear.y, self.max_lat)
        self._cmd_w = clamp(sw * msg.angular.z, self.max_w)
        self._last_cmd_time = time.monotonic()
        self._timeout_logged = False

        if self._estop:
            return

        if self.protocol == 'ascii':
            if (abs(self._cmd_x) < 1e-3 and abs(self._cmd_w) < 1e-3):
                self._send_ascii_stop()
                self._cmd_x = self._cmd_y = self._cmd_w = 0.0
                return
            self._stopped = False
            self._send_ascii('CMD,%d,%d' % (int(round(self._cmd_x * 1000.0)),
                                            int(round(self._cmd_w * 1000.0))))
        else:
            self._stopped = False

    def _estop_cb(self, msg):
        self._estop = bool(msg.data)
        if self._estop:
            self._cmd_x = self._cmd_y = self._cmd_w = 0.0
            if self.protocol == 'ascii':
                self._send_ascii_stop()
            self.get_logger().warn('Chassis E-STOP')

    def _scan_mode_cb(self, msg):
        self.scan_mode = bool(msg.data)

    # ------------------------------------------------------------------ 定时循环
    def _timer(self):
        now = time.monotonic()
        timed_out = (now - self._last_cmd_time) > self.timeout

        if self.protocol == 'binary':
            # 每个周期都发一帧，零速度也发（由 MCU 做失效保护）
            self._send_binary_frame(now)
            if (self._estop or timed_out) and not self._timeout_logged:
                self.get_logger().warn(
                    'Chassis output zero speed (%s)'
                    % ('estop' if self._estop else 'cmd_vel timeout'))
                self._timeout_logged = True
            if self._estop or timed_out:
                self._stopped = True
        else:
            if self._estop:
                self._send_ascii_stop()
            elif not self._stopped and timed_out:
                self.get_logger().warn('cmd_vel timeout, stopping chassis')
                self._send_ascii_stop()
                self._cmd_x = self._cmd_y = self._cmd_w = 0.0

        # 里程计：有反馈就用反馈，否则对下发的速度指令做积分
        if (not self._fake) and self._rx_valid:
            vx, vy, w = self._rx_v, 0.0, self._rx_w
        else:
            if self._estop or timed_out:
                vx = vy = w = 0.0
            else:
                vx, vy, w = self._cmd_x, self._cmd_y, self._cmd_w

        dt = now - self._last_integrate
        self._last_integrate = now
        if dt <= 0.0 or dt > 0.5:
            dt = 0.0
        self._yaw += w * dt
        self._x += (vx * math.cos(self._yaw) - vy * math.sin(self._yaw)) * dt
        self._y += (vx * math.sin(self._yaw) + vy * math.cos(self._yaw)) * dt

        odom = Odometry()
        odom.header.stamp = self.get_clock().now().to_msg()
        odom.header.frame_id = self.odom_frame
        odom.child_frame_id = self.base_frame
        odom.pose.pose.position.x = self._x
        odom.pose.pose.position.y = self._y
        odom.pose.pose.orientation.z = math.sin(self._yaw * 0.5)
        odom.pose.pose.orientation.w = math.cos(self._yaw * 0.5)
        odom.twist.twist.linear.x = vx
        odom.twist.twist.linear.y = vy
        odom.twist.twist.angular.z = w
        self.odom_pub.publish(odom)

        if self._tf is not None:
            t = self._TransformStamped()
            t.header.stamp = odom.header.stamp
            t.header.frame_id = self.odom_frame
            t.child_frame_id = self.base_frame
            t.transform.translation.x = self._x
            t.transform.translation.y = self._y
            t.transform.rotation.z = odom.pose.pose.orientation.z
            t.transform.rotation.w = odom.pose.pose.orientation.w
            self._tf.sendTransform(t)

    def _status_timer(self):
        vx, vy, yaw, w = self._frame_fields(time.monotonic())
        msg = String()
        msg.data = ('protocol=%s mode=%s estop=%s x=%.2f y=%.2f yaw=%.2f '
                    'w=%.2f scan=%s feedback=%s serial=%s'
                    % (self.protocol, 'fake' if self._fake else 'serial',
                       self._estop, vx, vy, yaw, w, self.scan_mode,
                       self.feedback, self._status))
        self.status_pub.publish(msg)

    def destroy_node(self):
        try:
            if self.protocol == 'binary':
                self._write(encode_navigation_frame(
                    0.0, 0.0, 0.0, 0.0, self.scan_mode,
                    sof=int(self.get_parameter('frame_sof').value),
                    frame_id=int(self.get_parameter('frame_id').value),
                    eof=int(self.get_parameter('frame_eof').value),
                    byte_order=self.byte_order))
            elif self._serial is not None:
                self._send_ascii_stop()
        except Exception:
            pass

        self._running = False
        thread = self._reader_thread
        if thread is not None and thread.is_alive():
            thread.join(timeout=0.5)
        self._reader_thread = None

        try:
            with self._serial_lock:
                ser = self._serial
                self._serial = None
            if ser is not None:
                ser.close()
        except Exception:
            pass
        super().destroy_node()


def main(args=None):
    rclpy.init(args=args)
    node = ChassisBridge()
    try:
        rclpy.spin(node)
    except (KeyboardInterrupt, ExternalShutdownException):
        pass
    finally:
        try:
            node.destroy_node()
        except Exception:
            pass
        try:
            if rclpy.ok():
                rclpy.shutdown()
        except Exception:
            pass


if __name__ == '__main__':
    main()
