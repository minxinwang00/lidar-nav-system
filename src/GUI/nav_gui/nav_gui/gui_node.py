#!/usr/bin/env python3
"""Qt5 操作控制台（第 4 层）。

发布话题：
  /goal_pose      (geometry_msgs/PoseStamped)
  /initialpose    (geometry_msgs/PoseWithCovarianceStamped, AMCL)
  /nav_command    (std_msgs/String: cancel | replan | estop | resume |
                   relocalize_on | relocalize_off)
  /estop          (std_msgs/Bool)

订阅话题：
  /nav_status     （bt_main 发出的 std_msgs/String 格式 JSON）
  /robot_pose     (geometry_msgs/PoseStamped)
  /amcl_pose      (geometry_msgs/PoseWithCovarianceStamped，优先采用的位姿)

调用的服务：
  /pointcloud_to_grid/save_map            (std_srvs/Trigger)
  /pointcloud_to_grid/start_mapping       (std_srvs/Trigger)
  /pointcloud_to_grid/stop_mapping        (std_srvs/Trigger)
  /map_server/load_map                    (nav2_msgs/LoadMap)
  /reinitialize_global_localization       (std_srvs/Empty)
"""

import json
import math
import os
import shutil
import signal
import subprocess
import sys
import threading
import time
from collections import deque

import rclpy
from rclpy.duration import Duration
from rclpy.executors import ExternalShutdownException
from rclpy.node import Node
from rclpy.time import Time

import tf2_ros

from ament_index_python.packages import get_package_share_directory
from geometry_msgs.msg import PoseStamped, PoseWithCovarianceStamped
from nav2_msgs.srv import LoadMap
from std_msgs.msg import Bool, String
from std_srvs.srv import Empty, Trigger

from PyQt5.QtCore import Qt, QTimer
from PyQt5.QtGui import QTextCursor
from PyQt5.QtWidgets import (QApplication, QCheckBox, QComboBox, QDialog,
                             QDoubleSpinBox, QFileDialog, QGridLayout,
                             QGroupBox, QHBoxLayout, QLabel, QMainWindow,
                             QMessageBox, QPlainTextEdit, QPushButton,
                             QScrollArea, QTextBrowser, QVBoxLayout, QWidget)

from nav_gui.help_content import HELP_HTML
from nav_gui.map_view import MapView, find_latest_map, find_maps


def yaw_to_quat(yaw):
    return 0.0, 0.0, math.sin(yaw * 0.5), math.cos(yaw * 0.5)


def _quat_yaw(q):
    return math.atan2(2.0 * (q.w * q.z + q.x * q.y),
                      1.0 - 2.0 * (q.y * q.y + q.z * q.z))


class GuiBridge(Node):
    TF_CHECKS = [
        ('map', 'odom', 'AMCL / 静态'),
        ('odom', 'base_link', 'lio_odom_bridge'),
        ('odom', 'camera_init', '里程计桥静态'),
        ('camera_init', 'body', 'FAST-LIO'),
        ('body', 'livox_frame', '雷达静态'),
        ('map', 'base_link', '组合位姿'),
    ]

    EXPECTED_NODES = [
        ('livox_lidar_publisher', '雷达驱动'),
        ('fastlio_mapping', 'FAST-LIO'),
        ('pointcloud_to_grid', '栅格/代价地图'),
        ('lio_odom_bridge', 'TF/里程计桥'),
        ('map_server', '地图服务'),
        ('amcl', 'AMCL 定位'),
        ('bt_main_node', '决策/控制'),
        ('chassis_bridge', '底盘串口桥'),
        ('nav_gui', 'GUI'),
        ('rviz2', 'RViz'),
    ]

    def __init__(self):
        super().__init__('nav_gui')
        self.declare_parameter('auto_start_rviz', False)
        self.declare_parameter(
            'map_dir', '~/nav_system_ws/src/mapping/maps')
        self.declare_parameter('initial_map', '')
        self.auto_start_rviz = bool(
            self.get_parameter('auto_start_rviz').value)
        self.map_dir = os.path.expanduser(
            str(self.get_parameter('map_dir').value))
        self.initial_map = os.path.expanduser(
            str(self.get_parameter('initial_map').value))

        self._lock = threading.Lock()
        self._status = {}
        self._pose = None
        self._pose_time = 0.0
        self._amcl_pose = None
        self._amcl_time = 0.0
        self._logs = deque(maxlen=200)
        # 长廊等弱特征场景可关闭 AMCL 全局重定位，避免粒子突然撒满整张地图
        self.relocalize_enabled = True
        # 刚由用户切换后，短时间内忽略旧的 /nav_status，避免按钮被回滚
        self._relocalize_override_until = 0.0

        # TF / 节点诊断
        self._diag_text = '正在采集 TF 与节点状态...'
        self._tf_buffer = tf2_ros.Buffer()
        self._tf_listener = tf2_ros.TransformListener(self._tf_buffer, self)

        self.goal_pub = self.create_publisher(PoseStamped, '/goal_pose', 10)
        self.initial_pub = self.create_publisher(
            PoseWithCovarianceStamped, '/initialpose', 10)
        self.cmd_pub = self.create_publisher(String, '/nav_command', 10)
        self.estop_pub = self.create_publisher(Bool, '/estop', 10)
        self.scan_pub = self.create_publisher(Bool, '/scan_mode', 10)
        self.save_client = self.create_client(
            Trigger, '/pointcloud_to_grid/save_map')
        self.start_mapping_client = self.create_client(
            Trigger, '/pointcloud_to_grid/start_mapping')
        self.stop_mapping_client = self.create_client(
            Trigger, '/pointcloud_to_grid/stop_mapping')
        self.load_map_client = self.create_client(
            LoadMap, '/map_server/load_map')
        self.relocalize_client = self.create_client(
            Empty, '/reinitialize_global_localization')

        self.create_subscription(String, '/nav_status', self._status_cb, 10)
        self.create_subscription(PoseStamped, '/robot_pose', self._pose_cb, 10)
        self.create_subscription(PoseWithCovarianceStamped, '/amcl_pose',
                                 self._amcl_cb, 10)
        self.create_timer(1.0, self._diag_timer)
        self.log('界面已启动')

    def _status_cb(self, msg):
        try:
            data = json.loads(msg.data)
        except Exception:
            return
        with self._lock:
            self._status = data

    def _pose_cb(self, msg):
        p = msg.pose.position
        q = msg.pose.orientation
        with self._lock:
            self._pose = (p.x, p.y, _quat_yaw(q))
            self._pose_time = time.monotonic()

    def _amcl_cb(self, msg):
        p = msg.pose.pose.position
        q = msg.pose.pose.orientation
        with self._lock:
            self._amcl_pose = (p.x, p.y, _quat_yaw(q))
            self._amcl_time = time.monotonic()

    def snapshot(self):
        with self._lock:
            pose = self._pose
            # AMCL 滤波后的位姿足够新时，优先采用它
            if self._amcl_pose is not None and \
                    time.monotonic() - self._amcl_time < 3.0:
                pose = self._amcl_pose
            return dict(self._status), pose

    def log(self, text):
        with self._lock:
            self._logs.append('%s  %s' % (time.strftime('%H:%M:%S'), text))

    def drain_logs(self):
        with self._lock:
            items = list(self._logs)
            self._logs.clear()
        return items

    # ------------------------------------------------------------------ 诊断
    def _diag_timer(self):
        lines = ['TF（目标 <- 源）:']
        for target, source, label in self.TF_CHECKS:
            try:
                tr = self._tf_buffer.lookup_transform(
                    target, source, Time(), Duration(seconds=0.05))
                stamp = Time.from_msg(tr.header.stamp)
                age = (self.get_clock().now() - stamp).nanoseconds / 1e9
                yaw = _quat_yaw(tr.transform.rotation)
                lines.append(
                    '  %-11s <- %-12s OK   age=%5.2fs x=%+.2f y=%+.2f '
                    'yaw=%+6.1f° [%s]'
                    % (target, source, max(0.0, age),
                       tr.transform.translation.x,
                       tr.transform.translation.y,
                       math.degrees(yaw), label))
            except Exception:
                lines.append('  %-11s <- %-12s FAIL [%s]'
                             % (target, source, label))

        lines.append('')
        lines.append('节点（OK=已拉起，MISSING=未启动）:')
        try:
            present = set()
            for name, namespace in self.get_node_names_and_namespaces():
                full = ('%s/%s' % (namespace.rstrip('/'), name)).lstrip('/')
                present.add(full)
        except Exception:
            present = set()
        expected = set()
        for node, label in self.EXPECTED_NODES:
            expected.add(node)
            ok = node in present
            suffix = '（可选）' if node == 'rviz2' else ''
            lines.append('  /%-24s %-7s %s%s'
                         % (node, 'OK' if ok else 'MISSING', label, suffix))
        lines.append('')
        lines.append('全部节点 (%d, 运行中):' % len(present))
        for node in sorted(present):
            lines.append('  /%-28s RUNNING' % node)

        with self._lock:
            self._diag_text = '\n'.join(lines)

    def diag_snapshot(self):
        with self._lock:
            return self._diag_text

    # ------------------------------------------------------------------ 命令发布
    def send_goal(self, x, y, yaw):
        msg = PoseStamped()
        msg.header.frame_id = 'map'
        msg.header.stamp = self.get_clock().now().to_msg()
        msg.pose.position.x = float(x)
        msg.pose.position.y = float(y)
        qx, qy, qz, qw = yaw_to_quat(float(yaw))
        msg.pose.orientation.x = qx
        msg.pose.orientation.y = qy
        msg.pose.orientation.z = qz
        msg.pose.orientation.w = qw
        self.goal_pub.publish(msg)
        self.log('发送目标 (%.2f, %.2f, %.2f)' % (x, y, yaw))

    def send_initial_pose(self, x, y, yaw):
        msg = PoseWithCovarianceStamped()
        msg.header.frame_id = 'map'
        msg.header.stamp = self.get_clock().now().to_msg()
        msg.pose.pose.position.x = float(x)
        msg.pose.pose.position.y = float(y)
        qx, qy, qz, qw = yaw_to_quat(float(yaw))
        msg.pose.pose.orientation.x = qx
        msg.pose.pose.orientation.y = qy
        msg.pose.pose.orientation.z = qz
        msg.pose.pose.orientation.w = qw
        msg.pose.covariance[0] = 0.25
        msg.pose.covariance[7] = 0.25
        msg.pose.covariance[35] = 0.0685
        self.initial_pub.publish(msg)
        self.log('设置初始位姿 (%.2f, %.2f, %.2f)' % (x, y, yaw))

    def set_relocalize_enabled(self, enabled):
        self.relocalize_enabled = bool(enabled)
        self._relocalize_override_until = time.monotonic() + 2.0
        self.send_command('relocalize_on' if self.relocalize_enabled
                          else 'relocalize_off')
        self.log('全局重定位：%s' % ('开启' if self.relocalize_enabled else '关闭'))

    def global_relocalize(self):
        if not self.relocalize_enabled:
            self.log('全局重定位已关闭，忽略本次请求')
            return
        if not self.relocalize_client.service_is_ready():
            self.log('全局重定位服务不可用：AMCL 未启动')
            return
        future = self.relocalize_client.call_async(Empty.Request())
        future.add_done_callback(self._relocalize_done)

    def _relocalize_done(self, future):
        try:
            future.result()
            self.log('AMCL 全局重定位已触发')
        except Exception as exc:
            self.log('全局重定位失败: %s' % exc)

    def send_command(self, cmd):
        msg = String()
        msg.data = cmd
        self.cmd_pub.publish(msg)
        self.log('命令: %s' % cmd)

    def set_estop(self, value):
        msg = Bool()
        msg.data = bool(value)
        self.estop_pub.publish(msg)

    def set_scan_mode(self, value):
        msg = Bool()
        msg.data = bool(value)
        self.scan_pub.publish(msg)
        self.log('扫描模式: %s' % ('开' if value else '关'))
        self.send_command('estop' if value else 'resume')

    def save_map(self):
        if not self.save_client.service_is_ready():
            self.log('保存失败：/pointcloud_to_grid/save_map 不可用')
            return
        future = self.save_client.call_async(Trigger.Request())
        future.add_done_callback(self._save_done)

    def _save_done(self, future):
        try:
            response = future.result()
            self.log(response.message)
        except Exception as exc:
            self.log('保存失败: %s' % exc)

    def start_mapping(self):
        self._call_trigger(self.start_mapping_client, '开始建图')

    def stop_mapping(self):
        self._call_trigger(self.stop_mapping_client, '结束建图并保存')

    def _call_trigger(self, client, action):
        if not client.service_is_ready():
            self.log('%s失败：服务未就绪（对应节点未启动）' % action)
            return
        future = client.call_async(Trigger.Request())
        future.add_done_callback(
            lambda fut: self._trigger_done(fut, action))

    def _trigger_done(self, future, action):
        try:
            response = future.result()
            if response.success:
                self.log('%s：%s' % (action, response.message))
            else:
                self.log('%s失败：%s' % (action, response.message))
        except Exception as exc:
            self.log('%s失败: %s' % (action, exc))

    def load_map(self, yaml_path):
        if not self.load_map_client.service_is_ready():
            self.log('加载地图失败：/map_server/load_map 不可用'
                     '（启动导航时需指定 map_yaml）')
            return
        request = LoadMap.Request()
        # Humble 的 nav2_map_server 在部分发行版中不接受 file:// 前缀，
        # 直接传绝对路径最稳妥。
        request.map_url = os.path.abspath(yaml_path)
        future = self.load_map_client.call_async(request)
        future.add_done_callback(self._load_map_done)

    def _load_map_done(self, future):
        try:
            response = future.result()
            if response.result == LoadMap.Response.RESULT_SUCCESS:
                info = response.map.info
                self.log('地图已加载：%dx%d @ %.3f m'
                         % (info.width, info.height, info.resolution))
                # 新地图加载后让 AMCL 重新定位；开关关闭时不触发全局重定位
                if (self.relocalize_enabled and
                        self.relocalize_client.service_is_ready()):
                    self.relocalize_client.call_async(Empty.Request())
            else:
                self.log('地图加载失败，错误码 %d' % response.result)
        except Exception as exc:
            self.log('地图加载失败: %s' % exc)

    def call_trigger_now(self, client, timeout=2.5):
        """同步调用 Trigger，用于关闭系统前保存地图。"""
        if not client.service_is_ready():
            return False, '服务未就绪'
        future = client.call_async(Trigger.Request())
        deadline = time.monotonic() + timeout
        while not future.done() and time.monotonic() < deadline:
            time.sleep(0.05)
        if not future.done():
            return False, '服务调用超时'
        try:
            response = future.result()
            if response is None:
                return False, '无响应'
            return bool(response.success), response.message
        except Exception as exc:
            return False, str(exc)


class MainWindow(QMainWindow):
    def __init__(self, bridge):
        super().__init__()
        self.bridge = bridge
        self._rviz_proc = None
        self._mapping_active = False
        self._closing = False
        self._last_map_path = ''
        self._last_goal = None
        self._last_diag = ''
        self.setWindowTitle('LiDAR Navigation Console')
        self.resize(1180, 780)
        self._build_ui()
        self.setStyleSheet(STYLE)

        self.timer = QTimer(self)
        self.timer.timeout.connect(self._refresh)
        self.timer.start(200)

        QTimer.singleShot(0, self._load_initial_map)
        if self.bridge.auto_start_rviz:
            QTimer.singleShot(0, self._open_rviz)

    def _build_ui(self):
        central = QWidget()
        root = QVBoxLayout(central)

        top = QHBoxLayout()
        title = QLabel('LiDAR 导航系统 · 命令界面')
        title.setObjectName('title')
        top.addWidget(title)
        top.addStretch(1)
        btn_help = QPushButton('帮助')
        btn_help.setObjectName('help')
        btn_help.clicked.connect(self._show_help)
        btn_terminal = QPushButton('打开终端')
        btn_terminal.clicked.connect(self._open_terminal)
        btn_close = QPushButton('关闭系统')
        btn_close.setObjectName('close')
        btn_close.clicked.connect(self._close_system)
        for button in (btn_help, btn_terminal, btn_close):
            top.addWidget(button)
        root.addLayout(top)

        body = QHBoxLayout()

        # ------------------------------------------------------ 左侧控制面板
        left_widget = QWidget()
        left = QVBoxLayout(left_widget)
        left.setContentsMargins(0, 0, 6, 0)

        box = QGroupBox('系统状态')
        grid = QGridLayout(box)
        self.lbl_state = QLabel('--')
        self.lbl_map = QLabel('--')
        self.lbl_pose = QLabel('--')
        self.lbl_goal = QLabel('--')
        self.lbl_dist = QLabel('--')
        self.lbl_estop = QLabel('--')
        self.lbl_mapping = QLabel('待机')
        rows = [('状态', self.lbl_state), ('地图', self.lbl_map),
                ('位姿', self.lbl_pose), ('目标', self.lbl_goal),
                ('距目标', self.lbl_dist), ('急停', self.lbl_estop),
                ('建图', self.lbl_mapping)]
        for i, (name, widget) in enumerate(rows):
            grid.addWidget(QLabel(name), i, 0)
            grid.addWidget(widget, i, 1)
        left.addWidget(box)

        # ------------------------------------------------------ TF / 节点诊断
        box = QGroupBox('TF 树 / 节点拉起状态')
        diag_layout = QVBoxLayout(box)
        self.diag_view = QPlainTextEdit()
        self.diag_view.setReadOnly(True)
        self.diag_view.setMaximumBlockCount(500)
        self.diag_view.setMinimumHeight(300)
        self.diag_view.setMaximumHeight(380)
        self.diag_view.setLineWrapMode(QPlainTextEdit.WidgetWidth)
        diag_layout.addWidget(self.diag_view)
        left.addWidget(box)

        # ------------------------------------------------------ 建图
        box = QGroupBox('建图')
        grid = QGridLayout(box)
        self.btn_start_mapping = QPushButton('开始建图')
        self.btn_start_mapping.setObjectName('mapping')
        self.btn_start_mapping.clicked.connect(self._start_mapping)
        self.btn_stop_mapping = QPushButton('结束建图并自动保存')
        self.btn_stop_mapping.setObjectName('stopMapping')
        self.btn_stop_mapping.clicked.connect(self._stop_mapping)
        grid.addWidget(self.btn_start_mapping, 0, 0)
        grid.addWidget(self.btn_stop_mapping, 0, 1)
        btn_save = QPushButton('仅保存当前地图')
        btn_save.clicked.connect(self.bridge.save_map)
        grid.addWidget(btn_save, 1, 0, 1, 2)
        left.addWidget(box)

        # ------------------------------------------------------ 位姿输入
        box = QGroupBox('位姿 / 目标输入 (x / y / yaw)')
        grid = QGridLayout(box)
        self.spin_x = self._spin(-100, 100)
        self.spin_y = self._spin(-100, 100)
        self.spin_yaw = self._spin(-3.14, 3.14)
        grid.addWidget(QLabel('x [m]'), 0, 0)
        grid.addWidget(self.spin_x, 0, 1)
        grid.addWidget(QLabel('y [m]'), 0, 2)
        grid.addWidget(self.spin_y, 0, 3)
        grid.addWidget(QLabel('yaw [rad]'), 1, 0)
        grid.addWidget(self.spin_yaw, 1, 1)
        send = QPushButton('发送目标')
        send.clicked.connect(self._send_goal)
        grid.addWidget(send, 1, 3)
        left.addWidget(box)

        # ------------------------------------------------------ 定位
        box = QGroupBox('定位 (AMCL)')
        grid = QGridLayout(box)
        btn_initial = QPushButton('设为初始位姿')
        btn_initial.clicked.connect(self._send_initial_pose)
        self.btn_reloc = QPushButton('全局重定位')
        self.btn_reloc.clicked.connect(self.bridge.global_relocalize)
        self.btn_reloc_toggle = QPushButton('关闭全局重定位')
        self.btn_reloc_toggle.clicked.connect(self._toggle_relocalize)
        btn_rviz = QPushButton('打开 RViz')
        btn_rviz.clicked.connect(self._open_rviz)
        grid.addWidget(btn_initial, 0, 0)
        grid.addWidget(self.btn_reloc, 0, 1)
        grid.addWidget(self.btn_reloc_toggle, 1, 0)
        grid.addWidget(btn_rviz, 1, 1)
        left.addWidget(box)

        # ------------------------------------------------------ 控制
        box = QGroupBox('控制')
        grid = QGridLayout(box)
        btn_cancel = QPushButton('取消目标')
        btn_cancel.clicked.connect(lambda: self.bridge.send_command('cancel'))
        btn_replan = QPushButton('重新规划')
        btn_replan.clicked.connect(lambda: self.bridge.send_command('replan'))
        btn_estop = QPushButton('急停')
        btn_estop.setObjectName('estop')
        btn_estop.clicked.connect(lambda: self.bridge.set_estop(True))
        btn_resume = QPushButton('解除急停')
        btn_resume.clicked.connect(lambda: self.bridge.set_estop(False))
        self.chk_scan = QCheckBox('扫描模式')
        self.chk_scan.toggled.connect(self.bridge.set_scan_mode)
        grid.addWidget(btn_cancel, 0, 0)
        grid.addWidget(btn_replan, 0, 1)
        grid.addWidget(btn_estop, 1, 0)
        grid.addWidget(btn_resume, 1, 1)
        grid.addWidget(self.chk_scan, 2, 0, 1, 2)
        left.addWidget(box)

        hint = QLabel('提示：在右侧地图上单击可直接选择目标（默认立即发送）；'
                      '滚轮缩放、按住左键拖动。建图结束后点“结束建图并自动保存”。')
        hint.setWordWrap(True)
        hint.setObjectName('hint')
        left.addWidget(hint)
        left.addStretch(1)

        scroll = QScrollArea()
        scroll.setWidgetResizable(True)
        scroll.setWidget(left_widget)
        scroll.setMinimumWidth(400)
        scroll.setMaximumWidth(470)
        scroll.setHorizontalScrollBarPolicy(Qt.ScrollBarAlwaysOff)
        body.addWidget(scroll)

        # ------------------------------------------------------ 右侧地图 + 日志
        right = QVBoxLayout()

        map_box = QGroupBox('栅格地图 / 目标点')
        map_layout = QVBoxLayout(map_box)

        map_row = QHBoxLayout()
        map_row.addWidget(QLabel('地图:'))
        self.combo_map = QComboBox()
        self.combo_map.setMinimumWidth(260)
        self.combo_map.currentIndexChanged.connect(self._on_map_selected)
        map_row.addWidget(self.combo_map, 1)
        btn_refresh_map = QPushButton('刷新')
        btn_refresh_map.clicked.connect(self._refresh_map_list)
        btn_import_map = QPushButton('导入地图...')
        btn_import_map.clicked.connect(self._import_map)
        map_row.addWidget(btn_refresh_map)
        map_row.addWidget(btn_import_map)
        map_layout.addLayout(map_row)

        map_actions = QHBoxLayout()
        btn_load_map = QPushButton('加载到导航')
        btn_load_map.setObjectName('mapping')
        btn_load_map.clicked.connect(self._load_selected_map)
        btn_fit = QPushButton('适应窗口')
        btn_fit.clicked.connect(lambda: self.map_view.fit_map())
        btn_zoom_in = QPushButton('放大 +')
        btn_zoom_in.clicked.connect(lambda: self.map_view.zoom_in())
        btn_zoom_out = QPushButton('缩小 -')
        btn_zoom_out.clicked.connect(lambda: self.map_view.zoom_out())
        self.chk_click_sends = QCheckBox('单击地图立即发送目标')
        self.chk_click_sends.setChecked(True)
        self.chk_click_sends.toggled.connect(self._on_click_sends_changed)
        self.chk_grid = QCheckBox('显示 1 m 栅格线')
        self.chk_grid.setChecked(True)
        self.chk_grid.toggled.connect(self._on_grid_toggled)
        for widget in (btn_load_map, btn_fit, btn_zoom_in, btn_zoom_out,
                       self.chk_click_sends, self.chk_grid):
            map_actions.addWidget(widget)
        map_actions.addStretch(1)
        map_layout.addLayout(map_actions)

        self.lbl_map_info = QLabel('未加载地图')
        self.lbl_map_info.setObjectName('hint')
        self.lbl_map_info.setWordWrap(True)
        map_layout.addWidget(self.lbl_map_info)

        self.map_view = MapView()
        self.map_view.pointSelected.connect(self._on_map_clicked)
        map_layout.addWidget(self.map_view, 1)
        right.addWidget(map_box, 3)

        log_box = QGroupBox('运行日志')
        log_layout = QVBoxLayout(log_box)
        self.log_view = QPlainTextEdit()
        self.log_view.setReadOnly(True)
        self.log_view.setMaximumBlockCount(300)
        log_layout.addWidget(self.log_view)
        right.addWidget(log_box, 2)

        body.addLayout(right, 1)
        root.addLayout(body, 1)
        self.setCentralWidget(central)

    # ------------------------------------------------------------------ 地图
    def _load_initial_map(self):
        preferred = self.bridge.initial_map
        if preferred and not os.path.isfile(preferred):
            self.bridge.log('指定的初始地图不存在: %s' % preferred)
            preferred = ''
        self._refresh_map_list(select_path=preferred)
        target = preferred or find_latest_map(self.bridge.map_dir)
        if target:
            self._preview_map(target, log=True)
        else:
            self.bridge.log('地图目录中没有 .yaml 地图: %s'
                            % self.bridge.map_dir)

    def _refresh_map_list(self, select_path=None):
        current = select_path or self._last_map_path
        maps = find_maps(self.bridge.map_dir)
        if current and os.path.isfile(current) and \
                os.path.abspath(current) not in \
                [os.path.abspath(p) for p in maps]:
            maps.insert(0, os.path.abspath(current))
        self.combo_map.blockSignals(True)
        self.combo_map.clear()
        selected_index = -1
        for index, path in enumerate(maps):
            stamp = time.strftime('%m-%d %H:%M',
                                  time.localtime(os.path.getmtime(path)))
            self.combo_map.addItem('%s   (%s)' % (os.path.basename(path),
                                                  stamp), path)
            if current and os.path.abspath(path) == os.path.abspath(current):
                selected_index = index
        if selected_index < 0 and maps:
            selected_index = 0
        if selected_index >= 0:
            self.combo_map.setCurrentIndex(selected_index)
        self.combo_map.blockSignals(False)
        if selected_index >= 0:
            self._preview_map(self.combo_map.itemData(selected_index),
                              log=False)
        else:
            self.lbl_map_info.setText('未找到地图，请先建图或导入 .yaml')
        return maps

    def _on_map_selected(self, index):
        if index < 0:
            return
        path = self.combo_map.itemData(index)
        if path:
            self._preview_map(path, log=False)

    def _preview_map(self, yaml_path, log=False):
        try:
            meta = self.map_view.load_map(yaml_path)
        except Exception as exc:
            self.lbl_map_info.setText('地图预览失败: %s' % exc)
            self.bridge.log('地图预览失败: %s' % exc)
            return
        self._last_map_path = meta['yaml_path']
        self.lbl_map_info.setText(
            '%s | %dx%d | %.3f m/格 | origin (%.2f, %.2f)\n'
            '黑=障碍  灰=未知  白=空闲；蓝线为 1 m 网格'
            % (os.path.basename(meta['yaml_path']), meta['width'],
               meta['height'], meta['resolution'], meta['origin_x'],
               meta['origin_y']))
        if log:
            self.bridge.log('已载入地图预览: %s' % meta['yaml_path'])

    def _import_map(self):
        path, _ = QFileDialog.getOpenFileName(
            self, '选择栅格地图 YAML', self.bridge.map_dir,
            'ROS 地图 (*.yaml *.yml);;所有文件 (*)')
        if not path:
            return
        if not path.lower().endswith(('.yaml', '.yml')):
            QMessageBox.warning(
                self, '地图格式错误',
                '请选择 nav2 栅格地图的 .yaml 文件，而不是 .pgm/.png 图片。\n\n'
                '栅格地图由 YAML + PGM 组成：YAML 提供分辨率、原点和阈值，'
                '单独一张图片不能直接用于导航。')
            return
        path = os.path.abspath(path)
        found = False
        for index in range(self.combo_map.count()):
            if os.path.abspath(self.combo_map.itemData(index)) == path:
                self.combo_map.setCurrentIndex(index)
                found = True
                break
        if not found:
            self.combo_map.blockSignals(True)
            self.combo_map.addItem(os.path.basename(path), path)
            self.combo_map.setCurrentIndex(self.combo_map.count() - 1)
            self.combo_map.blockSignals(False)
        self._preview_map(path, log=True)
        self.bridge.log('地图已导入选择，点“加载到导航”后 map_server 生效')

    def _load_selected_map(self):
        index = self.combo_map.currentIndex()
        if index < 0:
            self.bridge.log('没有可加载的地图')
            return
        path = self.combo_map.itemData(index)
        if not path:
            return
        self._preview_map(path, log=False)
        self.bridge.load_map(path)

    def _on_click_sends_changed(self, checked):
        self.map_view.set_click_sends(checked)

    def _on_grid_toggled(self, checked):
        self.map_view.set_grid_visible(checked)

    def _on_map_clicked(self, x, y, send):
        self.spin_x.setValue(x)
        self.spin_y.setValue(y)
        self._last_goal = (x, y)
        if send:
            self.bridge.send_goal(x, y, self.spin_yaw.value())
        else:
            self.bridge.log('地图已选点 (%.2f, %.2f)，点“发送目标”生效'
                            % (x, y))

    # ------------------------------------------------------------------ 建图
    def _start_mapping(self):
        ok, message = self.bridge.call_trigger_now(
            self.bridge.start_mapping_client, timeout=3.0)
        self.bridge.log('开始建图: %s' % message)
        if ok:
            self._mapping_active = True
            self.lbl_mapping.setText('建图中')
            self.lbl_mapping.setStyleSheet('color:#38d67a;font-weight:bold')
            self.map_view.clear_goal()

    def _stop_mapping(self):
        ok, message = self.bridge.call_trigger_now(
            self.bridge.stop_mapping_client, timeout=15.0)
        self.bridge.log('结束建图: %s' % message)
        if ok or 'Mapping stopped' in message:
            self._mapping_active = False
            self.lbl_mapping.setText('已保存' if ok else '停止（保存失败）')
            self.lbl_mapping.setStyleSheet('')
            latest = find_latest_map(self.bridge.map_dir)
            self._refresh_map_list(select_path=latest)
            if ok:
                self.bridge.log('新地图已生成并选为默认地图，'
                                '点“加载到导航”后 map_server 生效')

    # ------------------------------------------------------------------ 帮助 / 终端 / 关闭
    def _show_help(self):
        dialog = QDialog(self)
        dialog.setWindowTitle('使用帮助 · 常见问题与终端命令')
        dialog.resize(860, 700)
        layout = QVBoxLayout(dialog)
        browser = QTextBrowser()
        browser.setOpenExternalLinks(False)
        browser.setHtml(HELP_HTML)
        layout.addWidget(browser, 1)
        row = QHBoxLayout()
        btn_copy = QPushButton('复制帮助内容')
        btn_copy.clicked.connect(
            lambda: QApplication.clipboard().setText(browser.toPlainText()))
        btn_close = QPushButton('关闭')
        btn_close.clicked.connect(dialog.accept)
        row.addStretch(1)
        row.addWidget(btn_copy)
        row.addWidget(btn_close)
        layout.addLayout(row)
        dialog.exec_()

    def _open_terminal(self):
        workspace = os.path.expanduser('~/nav_system_ws')
        command = ('source /opt/ros/humble/setup.bash; '
                   'if [ -f install/setup.bash ]; then '
                   'source install/setup.bash; fi; '
                   'echo "ROS 2 终端已就绪"; exec bash')
        candidates = [
            ('gnome-terminal', ['gnome-terminal',
                                '--working-directory=' + workspace,
                                '--', 'bash', '-lc', command]),
            ('x-terminal-emulator', ['x-terminal-emulator', '-e',
                                     'bash', '-lc', command]),
            ('konsole', ['konsole', '--workdir', workspace, '-e',
                         'bash', '-lc', command]),
            ('xfce4-terminal', ['xfce4-terminal',
                                '--working-directory=' + workspace,
                                '-e', 'bash -lc "%s"' % command.replace('"', '\\"')]),
            ('xterm', ['xterm', '-e', 'bash', '-lc', command]),
        ]
        for name, argv in candidates:
            if shutil.which(name):
                try:
                    subprocess.Popen(argv, cwd=workspace)
                    self.bridge.log('已打开终端: %s' % name)
                    return
                except Exception as exc:
                    self.bridge.log('打开 %s 失败: %s' % (name, exc))
        self.bridge.log('未找到可用终端程序（gnome-terminal/xterm 等）')

    def _find_launch_pid(self):
        pid = os.getppid()
        for _ in range(8):
            if pid <= 1:
                break
            try:
                with open('/proc/%d/cmdline' % pid, 'rb') as stream:
                    cmd = stream.read().replace(b'\x00', b' ').decode(
                        'utf-8', 'replace')
            except Exception:
                break
            if 'launch' in cmd and ('bringup' in cmd or 'nav_sim' in cmd):
                return pid
            try:
                with open('/proc/%d/stat' % pid, 'r') as stream:
                    pid = int(stream.read().split()[3])
            except Exception:
                break
        return None

    def _close_system(self):
        if self._closing:
            return
        reply = QMessageBox.question(
            self, '关闭导航系统',
            '将停止所有导航/建图节点并关闭界面。\n'
            '如果正在建图，会先保存地图。\n\n确定关闭吗？',
            QMessageBox.Yes | QMessageBox.No, QMessageBox.No)
        if reply != QMessageBox.Yes:
            return

        self._closing = True
        if self._mapping_active:
            ok, message = self.bridge.call_trigger_now(
                self.bridge.stop_mapping_client, timeout=15.0)
            self.bridge.log('关闭前保存地图: %s' % message)
            self._mapping_active = False

        pid = self._find_launch_pid()
        if pid is not None:
            self.bridge.log('向 launch 进程 %d 发送 SIGINT' % pid)
            try:
                os.kill(pid, signal.SIGINT)
            except Exception as exc:
                self.bridge.log('终止 launch 失败: %s' % exc)
        else:
            self.bridge.log('未找到父 launch，尝试停止 bringup/nav_sim 进程')
            try:
                subprocess.Popen([
                    'bash', '-lc',
                    "pkill -INT -f 'bringup.*(navigation|mapping)\\.launch\\.py'"
                    " || true"])
            except Exception as exc:
                self.bridge.log('停止进程失败: %s' % exc)

        QTimer.singleShot(1200, self.close)

    @staticmethod
    def _spin(lo, hi):
        spin = QDoubleSpinBox()
        spin.setRange(lo, hi)
        spin.setDecimals(2)
        spin.setSingleStep(0.1)
        spin.setMinimumWidth(90)
        return spin

    def _send_goal(self):
        x = self.spin_x.value()
        y = self.spin_y.value()
        self._last_goal = (x, y)
        self.map_view.set_goal_pose(x, y)
        self.bridge.send_goal(x, y, self.spin_yaw.value())

    def _send_initial_pose(self):
        self.bridge.send_initial_pose(self.spin_x.value(),
                                      self.spin_y.value(),
                                      self.spin_yaw.value())

    def _toggle_relocalize(self):
        enabled = not self.bridge.relocalize_enabled
        self.bridge.set_relocalize_enabled(enabled)
        self._apply_relocalize_state(enabled)

    def _apply_relocalize_state(self, enabled):
        enabled = bool(enabled)
        self.btn_reloc.setEnabled(enabled)
        self.btn_reloc_toggle.setText(
            '关闭全局重定位' if enabled else '启用全局重定位')

    # ------------------------------------------------------------------ rviz 控制
    def _rviz_running(self):
        if self._rviz_proc is not None and self._rviz_proc.poll() is None:
            return True
        # 同时检测由 bringup launch 启动的 RViz 进程
        try:
            return subprocess.call(
                ['pgrep', '-f', 'rviz2'],
                stdout=subprocess.DEVNULL, stderr=subprocess.DEVNULL) == 0
        except Exception:
            return False

    def _open_rviz(self):
        if self._rviz_running():
            self.bridge.log('RViz 已在运行')
            return
        try:
            share = get_package_share_directory('bringup')
            cfg = os.path.join(share, 'rviz', 'nav.rviz')
        except Exception as exc:
            self.bridge.log('找不到 bringup/nav.rviz: %s' % exc)
            return
        try:
            self._rviz_proc = subprocess.Popen(['rviz2', '-d', cfg])
            self.bridge.log('已启动 RViz: %s' % cfg)
        except Exception as exc:
            self.bridge.log('启动 RViz 失败: %s' % exc)

    def closeEvent(self, event):
        if self._rviz_proc is not None and self._rviz_proc.poll() is None:
            self._rviz_proc.terminate()
        super().closeEvent(event)

    def _refresh(self):
        status, pose = self.bridge.snapshot()
        if self._rviz_proc is not None and self._rviz_proc.poll() is not None:
            self._rviz_proc = None
            self.bridge.log('RViz 已退出')
        self.lbl_state.setText(str(status.get('state', '--')))
        reloc = status.get('relocalize_enabled')
        if (reloc is not None and
                time.monotonic() > self.bridge._relocalize_override_until and
                bool(reloc) != self.bridge.relocalize_enabled):
            self.bridge.relocalize_enabled = bool(reloc)
            self._apply_relocalize_state(bool(reloc))
        self.lbl_map.setText('已就绪' if status.get('map_ready') else '等待 /map')
        if pose and len(pose) >= 2:
            if len(pose) >= 3:
                self.lbl_pose.setText('x=%.2f  y=%.2f  yaw=%.2f'
                                      % (pose[0], pose[1], pose[2]))
                self.map_view.set_robot_pose(pose[0], pose[1], pose[2])
            else:
                self.lbl_pose.setText('x=%.2f  y=%.2f' % (pose[0], pose[1]))
                self.map_view.set_robot_pose(pose[0], pose[1], 0.0)
        else:
            self.lbl_pose.setText('无定位')
        goal = status.get('goal')
        if goal:
            self.lbl_goal.setText('(%.2f, %.2f)' % (goal[0], goal[1]))
            self._last_goal = (goal[0], goal[1])
            self.map_view.set_goal_pose(goal[0], goal[1])
        else:
            self.lbl_goal.setText('无')
        dist = status.get('distance_to_goal')
        self.lbl_dist.setText('%.2f m' % dist if dist is not None else '--')
        self.lbl_estop.setText('已急停' if status.get('estop') else '正常')
        self.lbl_estop.setStyleSheet(
            'color:#ff6b6b;font-weight:bold' if status.get('estop') else '')
        if self._mapping_active:
            self.lbl_mapping.setText('建图中')
            self.lbl_mapping.setStyleSheet(
                'color:#38d67a;font-weight:bold')

        diag = self.bridge.diag_snapshot()
        if diag != self._last_diag:
            self._last_diag = diag
            scroll = self.diag_view.verticalScrollBar().value()
            self.diag_view.setPlainText(diag)
            self.diag_view.verticalScrollBar().setValue(scroll)

        for line in self.bridge.drain_logs():
            self.log_view.appendPlainText(line)
            cursor = self.log_view.textCursor()
            cursor.movePosition(QTextCursor.End)
            self.log_view.setTextCursor(cursor)


STYLE = """
QWidget { background-color:#1e1f22; color:#e6e6e6; font-size:13px; }
QLabel#title { font-size:18px; font-weight:600; padding:6px 2px; color:#7fd1ff; }
QLabel#hint { color:#9aa0a6; font-size:12px; }
QGroupBox { border:1px solid #3a3d42; border-radius:6px; margin-top:10px; padding:8px; }
QGroupBox::title { subcontrol-origin:margin; left:10px; padding:0 4px; color:#9aa0a6; }
QPushButton { background-color:#2f80ed; border:none; border-radius:4px; padding:6px 10px; }
QPushButton:hover { background-color:#3f8ff0; }
QPushButton:pressed { background-color:#2a6fd0; }
QPushButton#estop { background-color:#c0392b; }
QPushButton#estop:hover { background-color:#d64a3a; }
QPushButton#close { background-color:#c0392b; }
QPushButton#close:hover { background-color:#d64a3a; }
QPushButton#help { background-color:#3a3d42; }
QPushButton#help:hover { background-color:#4a4e55; }
QPushButton#mapping { background-color:#2e9e5b; }
QPushButton#mapping:hover { background-color:#38b86b; }
QPushButton#stopMapping { background-color:#b9770e; }
QPushButton#stopMapping:hover { background-color:#d18a12; }
QDoubleSpinBox, QComboBox { background-color:#2a2c30; border:1px solid #3a3d42;
                            border-radius:4px; padding:3px; }
QComboBox QAbstractItemView { background-color:#2a2c30; color:#e6e6e6;
                              selection-background-color:#2f80ed; }
QPlainTextEdit { background-color:#151619; border:1px solid #3a3d42;
                 border-radius:4px; font-family:monospace; font-size:12px; }
QTextBrowser { background-color:#151619; border:1px solid #3a3d42;
               border-radius:4px; }
QGraphicsView#mapView { border:1px solid #3a3d42; border-radius:4px; }
QScrollArea { border:none; }
QCheckBox { spacing:6px; }
"""


def _spin_node(node):
    try:
        rclpy.spin(node)
    except ExternalShutdownException:
        pass
    except Exception:
        pass


def main(args=None):
    rclpy.init(args=args)
    bridge = GuiBridge()
    spin_thread = threading.Thread(target=_spin_node, args=(bridge,), daemon=True)
    spin_thread.start()

    app = QApplication(sys.argv)
    window = MainWindow(bridge)
    window.show()
    try:
        app.exec_()
    finally:
        try:
            bridge.destroy_node()
        except Exception:
            pass
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == '__main__':
    main()
