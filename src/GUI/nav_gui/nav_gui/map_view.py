#!/usr/bin/env python3
"""栅格地图显示控件：支持缩放、拖动、单击选择目标点。"""

import glob
import math
import os
import time

import yaml

from PyQt5.QtCore import QPointF, QRectF, Qt, pyqtSignal
from PyQt5.QtGui import QColor, QImage, QPainter, QPen, QPixmap
from PyQt5.QtWidgets import (QGraphicsEllipseItem, QGraphicsItemGroup,
                             QGraphicsLineItem, QGraphicsScene,
                             QGraphicsView)


def find_maps(map_dir):
    """返回目录下所有 yaml 地图，按修改时间从新到旧排序。"""
    if not map_dir:
        return []
    patterns = [os.path.join(map_dir, '*.yaml'),
                os.path.join(map_dir, '*.yml')]
    files = []
    for pattern in patterns:
        files.extend(glob.glob(pattern))
    files = [p for p in files if os.path.isfile(p)]
    return sorted(files, key=os.path.getmtime, reverse=True)


def find_latest_map(map_dir):
    maps = find_maps(map_dir)
    return maps[0] if maps else ''


def read_map_yaml(yaml_path):
    """读取 nav2 风格的地图 yaml，返回显示所需元数据。"""
    yaml_path = os.path.abspath(os.path.expanduser(yaml_path))
    with open(yaml_path, 'r', encoding='utf-8') as stream:
        data = yaml.safe_load(stream) or {}

    image = str(data.get('image', '')).strip()
    if not image:
        raise ValueError('地图 yaml 缺少 image 字段')

    if image.startswith('file://'):
        image_path = image[len('file://'):]
    elif image.startswith('package://'):
        from ament_index_python.packages import get_package_share_directory
        rest = image[len('package://'):]
        pkg, _, rel = rest.partition('/')
        image_path = os.path.join(get_package_share_directory(pkg), rel)
    elif os.path.isabs(image):
        image_path = image
    else:
        image_path = os.path.join(os.path.dirname(yaml_path), image)

    resolution = float(data.get('resolution', 0.05))
    if resolution <= 0.0:
        raise ValueError('地图 resolution 必须大于 0')
    origin = data.get('origin', [0.0, 0.0, 0.0])
    origin = [float(v) for v in origin] + [0.0, 0.0, 0.0]
    return {
        'yaml_path': yaml_path,
        'image_path': os.path.abspath(image_path),
        'resolution': resolution,
        'origin_x': origin[0],
        'origin_y': origin[1],
        'origin_yaw': origin[2],
        'negate': bool(int(data.get('negate', 0))),
    }


class MapView(QGraphicsView):
    """在地图上点选目标，坐标使用地图 yaml 的 world 坐标系。"""

    pointSelected = pyqtSignal(float, float, bool)  # x, y, force_send

    def __init__(self, parent=None):
        super().__init__(parent)
        self._scene = QGraphicsScene(self)
        self.setScene(self._scene)
        self.setRenderHints(QPainter.Antialiasing |
                            QPainter.SmoothPixmapTransform)
        self.setDragMode(QGraphicsView.ScrollHandDrag)
        self.setTransformationAnchor(QGraphicsView.AnchorUnderMouse)
        self.setResizeAnchor(QGraphicsView.AnchorViewCenter)
        self.setBackgroundBrush(QColor('#151619'))
        self.setMinimumSize(420, 320)
        self.setObjectName('mapView')

        self._meta = None
        self._click_sends = True
        self._grid_visible = True
        self._grid_spacing_m = 1.0
        self._press_pos = None
        self._last_click = 0.0
        self._robot_item = None
        self._goal_item = None

    # ------------------------------------------------------------------ 地图
    def load_map(self, yaml_path):
        meta = read_map_yaml(yaml_path)
        image = QImage(meta['image_path'])
        if image.isNull():
            raise RuntimeError('无法读取地图图片: %s' % meta['image_path'])
        if meta['negate']:
            image.invertPixels()
        pixmap = QPixmap.fromImage(image)
        if pixmap.isNull():
            raise RuntimeError('无法转换为 QPixmap: %s' % meta['image_path'])

        self._scene.clear()
        self._robot_item = None
        self._goal_item = None
        item = self._scene.addPixmap(pixmap)
        item.setZValue(-10)
        self._scene.setSceneRect(QRectF(0.0, 0.0,
                                        float(pixmap.width()),
                                        float(pixmap.height())))
        meta['width'] = pixmap.width()
        meta['height'] = pixmap.height()
        self._meta = meta
        self.fit_map()
        return meta

    def meta(self):
        return dict(self._meta) if self._meta else None

    def has_map(self):
        return self._meta is not None

    def fit_map(self):
        if self._meta is None:
            return
        self.resetTransform()
        self.fitInView(self._scene.sceneRect(), Qt.KeepAspectRatio)

    def zoom_in(self):
        self.scale(1.25, 1.25)

    def zoom_out(self):
        self.scale(0.8, 0.8)

    def wheelEvent(self, event):
        if self._meta is None:
            return
        factor = 1.25 if event.angleDelta().y() > 0 else 0.8
        self.scale(factor, factor)
        event.accept()

    def set_click_sends(self, enabled):
        self._click_sends = bool(enabled)

    def set_grid_visible(self, visible):
        self._grid_visible = bool(visible)
        self.viewport().update()

    def set_grid_spacing(self, meters):
        self._grid_spacing_m = max(0.1, float(meters))
        self.viewport().update()

    # ------------------------------------------------------------------ 栅格线
    def drawForeground(self, painter, rect):
        """在图像上叠加 world 坐标的 1 m 网格线，方便确认栅格比例。"""
        super().drawForeground(painter, rect)
        if not self._grid_visible or self._meta is None:
            return
        res = self._meta['resolution']
        width = self._meta['width']
        height = self._meta['height']
        step = self._grid_spacing_m / res
        if step < 5.0:
            return
        painter.save()
        pen = QPen(QColor(127, 209, 255, 80))
        pen.setWidth(0)  # cosmetic pen: 不随缩放变粗
        painter.setPen(pen)
        left = max(0.0, rect.left())
        right = min(float(width), rect.right())
        top = max(0.0, rect.top())
        bottom = min(float(height), rect.bottom())
        start_x = math.floor(left / step) * step
        start_y = math.floor(top / step) * step
        x = start_x
        while x <= right:
            painter.drawLine(QPointF(x, top), QPointF(x, bottom))
            x += step
        y = start_y
        while y <= bottom:
            painter.drawLine(QPointF(left, y), QPointF(right, y))
            y += step
        painter.restore()

    # ------------------------------------------------------------------ 坐标换算
    def _world_to_scene(self, x, y):
        meta = self._meta
        col = (x - meta['origin_x']) / meta['resolution']
        row = meta['height'] - (y - meta['origin_y']) / meta['resolution']
        return QPointF(col, row)

    def _scene_to_world(self, point):
        meta = self._meta
        x = meta['origin_x'] + point.x() * meta['resolution']
        y = meta['origin_y'] + \
            (meta['height'] - point.y()) * meta['resolution']
        return x, y

    def _scene_point_in_map(self, point):
        return (self._meta is not None and
                0.0 <= point.x() <= self._meta['width'] and
                0.0 <= point.y() <= self._meta['height'])

    # ------------------------------------------------------------------ 标记
    def _marker_pen(self, color):
        pen = QPen(QColor(color))
        pen.setWidthF(2.0)
        return pen

    def set_robot_pose(self, x, y, yaw=0.0):
        if self._meta is None:
            return
        if self._robot_item is None:
            group = QGraphicsItemGroup()
            radius = max(4.0, 0.18 / self._meta['resolution'])
            ellipse = QGraphicsEllipseItem(-radius, -radius,
                                           radius * 2.0, radius * 2.0)
            ellipse.setPen(self._marker_pen('#38d67a'))
            ellipse.setBrush(QColor(56, 214, 122, 90))
            group.addToGroup(ellipse)
            line = QGraphicsLineItem(0.0, 0.0, radius * 1.8, 0.0)
            line.setPen(self._marker_pen('#38d67a'))
            group.addToGroup(line)
            group.setZValue(20)
            self._scene.addItem(group)
            self._robot_item = group
        self._robot_item.setPos(self._world_to_scene(x, y))
        # 世界 yaw 逆时针，而场景 y 轴向下，因此取负角度
        self._robot_item.setRotation(-math.degrees(yaw))

    def set_goal_pose(self, x, y):
        if self._meta is None:
            return
        if self._goal_item is None:
            group = QGraphicsItemGroup()
            size = max(5.0, 0.30 / self._meta['resolution'])
            for x1, y1, x2, y2 in ((-size, 0.0, size, 0.0),
                                   (0.0, -size, 0.0, size)):
                line = QGraphicsLineItem(x1, y1, x2, y2)
                line.setPen(self._marker_pen('#ffb020'))
                group.addToGroup(line)
            group.setZValue(21)
            self._scene.addItem(group)
            self._goal_item = group
        self._goal_item.setPos(self._world_to_scene(x, y))
        self._goal_item.setVisible(True)

    def clear_goal(self):
        if self._goal_item is not None:
            self._goal_item.setVisible(False)

    # ------------------------------------------------------------------ 鼠标事件
    def mousePressEvent(self, event):
        if event.button() == Qt.LeftButton:
            self._press_pos = event.pos()
        super().mousePressEvent(event)

    def _click_to_world(self, event):
        if self._meta is None:
            return None
        point = self.mapToScene(event.pos())
        if not self._scene_point_in_map(point):
            return None
        return self._scene_to_world(point)

    def mouseReleaseEvent(self, event):
        moved = False
        if self._press_pos is not None:
            moved = (event.pos() - self._press_pos).manhattanLength() > 4
        super().mouseReleaseEvent(event)
        if event.button() != Qt.LeftButton or moved:
            return
        now = time.monotonic()
        if now - self._last_click < 0.30:
            return
        world = self._click_to_world(event)
        if world is None:
            return
        self._last_click = now
        self.set_goal_pose(*world)
        self.pointSelected.emit(world[0], world[1], self._click_sends)

    def mouseDoubleClickEvent(self, event):
        super().mouseDoubleClickEvent(event)
        if event.button() != Qt.LeftButton:
            return
        now = time.monotonic()
        # 单击已经发过目标时，双击不再重复发送
        if self._click_sends and now - self._last_click < 0.30:
            return
        world = self._click_to_world(event)
        if world is None:
            return
        self._last_click = now
        self.set_goal_pose(*world)
        self.pointSelected.emit(world[0], world[1], True)
