#!/usr/bin/env python3
"""LI-Init calibration workflow manager.

Services
--------
  ~/record_bag       (std_srvs/Trigger)  start `ros2 bag record` for lidar+imu
  ~/stop_recording   (std_srvs/Trigger)  stop and finalize the bag
  ~/apply_result     (std_srvs/Trigger)  parse the LI-Init result and write the
                                         calibrated FAST-LIO YAML

The heavy LI-Init optimization itself runs outside this node (ROS1 original or
a ROS2 port); this manager only automates bag recording and result application.
"""

import datetime
import os
import signal
import subprocess
import threading

import rclpy
from rclpy.node import Node
from std_msgs.msg import String
from std_srvs.srv import Trigger

from li_init_tools.apply_li_init import parse_result, write_config


class LiInitManager(Node):
    def __init__(self):
        super().__init__('li_init_manager')

        self.declare_parameter('bag_dir', '~/nav_system_ws/calib_bags')
        self.declare_parameter('bag_duration', 90.0)
        self.declare_parameter('lidar_topic', '/livox/lidar')
        self.declare_parameter('imu_topic', '/livox/imu')
        self.declare_parameter('result_file',
                               '~/nav_system_ws/calib_bags/li_init_result.txt')
        self.declare_parameter(
            'fastlio_input',
            '~/nav_system_ws/src/mapping/FAST_LIO/config/mid360.yaml')
        self.declare_parameter(
            'fastlio_output',
            '~/nav_system_ws/src/mapping/FAST_LIO/config/mid360_calibrated.yaml')
        self.declare_parameter('quat_order', 'xyzw')

        self.bag_dir = os.path.expanduser(
            str(self.get_parameter('bag_dir').value))
        self.duration = float(self.get_parameter('bag_duration').value)
        self.lidar_topic = str(self.get_parameter('lidar_topic').value)
        self.imu_topic = str(self.get_parameter('imu_topic').value)
        self.result_file = os.path.expanduser(
            str(self.get_parameter('result_file').value))
        self.fastlio_input = os.path.expanduser(
            str(self.get_parameter('fastlio_input').value))
        self.fastlio_output = os.path.expanduser(
            str(self.get_parameter('fastlio_output').value))
        self.quat_order = str(self.get_parameter('quat_order').value)

        self._proc = None
        self._bag_path = None
        self._start_time = 0.0
        self._status = 'idle'

        self.create_service(Trigger, '~/record_bag', self._record_srv)
        self.create_service(Trigger, '~/stop_recording', self._stop_srv)
        self.create_service(Trigger, '~/apply_result', self._apply_srv)
        self.status_pub = self.create_publisher(String, '/li_init_status', 10)
        self.create_timer(1.0, self._tick)

        self.get_logger().info(
            'LI-Init manager ready. Call ~/record_bag, run LI-Init, then '
            '~/apply_result. Result file: %s' % self.result_file)

    # ------------------------------------------------------------------ record
    def _record_srv(self, request, response):
        if self._proc is not None and self._proc.poll() is None:
            response.success = False
            response.message = 'already recording: %s' % self._bag_path
            return response
        os.makedirs(self.bag_dir, exist_ok=True)
        stamp = datetime.datetime.now().strftime('%Y%m%d_%H%M%S')
        self._bag_path = os.path.join(self.bag_dir, 'li_init_' + stamp)
        cmd = ['ros2', 'bag', 'record', '-o', self._bag_path,
               self.lidar_topic, self.imu_topic]
        try:
            self._proc = subprocess.Popen(
                cmd, stdout=subprocess.DEVNULL, stderr=subprocess.DEVNULL)
        except Exception as exc:
            response.success = False
            response.message = 'failed to start ros2 bag record: %s' % exc
            self._status = 'record failed'
            return response
        self._start_time = self.get_clock().now().nanoseconds / 1e9
        self._status = 'recording'
        response.success = True
        response.message = ('recording %s and %s for %.0f s -> %s'
                            % (self.lidar_topic, self.imu_topic,
                               self.duration, self._bag_path))
        self.get_logger().info(response.message)
        return response

    def _stop_recording(self):
        if self._proc is None:
            return 'not recording'
        if self._proc.poll() is None:
            try:
                self._proc.send_signal(signal.SIGINT)
                self._proc.wait(timeout=10.0)
            except Exception:
                try:
                    self._proc.kill()
                except Exception:
                    pass
        self._proc = None
        self._status = 'bag saved'
        msg = 'bag saved: %s' % self._bag_path
        self.get_logger().info(msg)
        return msg

    def _stop_srv(self, request, response):
        response.success = True
        response.message = self._stop_recording()
        return response

    def _tick(self):
        now = self.get_clock().now().nanoseconds / 1e9
        if self._proc is not None and self._proc.poll() is None:
            if now - self._start_time >= self.duration:
                # finalize in a thread so the timer is not blocked
                threading.Thread(target=self._stop_recording, daemon=True).start()
            else:
                self._status = 'recording %.0f/%.0f s' % (
                    now - self._start_time, self.duration)
        msg = String()
        msg.data = self._status
        self.status_pub.publish(msg)

    # ------------------------------------------------------------------ apply
    def _apply_srv(self, request, response):
        if not os.path.isfile(self.result_file):
            response.success = False
            response.message = 'result file not found: %s' % self.result_file
            return response
        try:
            R, T, dt = parse_result(self.result_file, self.quat_order)
            out = write_config(self.fastlio_input, self.fastlio_output, R, T, dt)
        except Exception as exc:
            response.success = False
            response.message = 'apply failed: %s' % exc
            return response
        response.success = True
        response.message = ('calibration applied -> %s (T=%s, dt=%.6f)'
                            % (out, ['%.4f' % v for v in T], dt))
        self.get_logger().info(response.message)
        return response

    def destroy_node(self):
        self._stop_recording()
        super().destroy_node()


def main(args=None):
    rclpy.init(args=args)
    node = LiInitManager()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
