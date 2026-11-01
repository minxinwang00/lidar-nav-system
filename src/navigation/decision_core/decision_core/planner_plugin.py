#!/usr/bin/env python3
"""全局规划器：在（膨胀后的）占据栅格地图上做 A* 搜索。

规划器按地图版本号缓存膨胀后的栅格，并可叠加局部代价地图（local costmap），
使得*全局*路径也能避开动态障碍物。
"""

import heapq
import math
import threading
import time

import numpy as np
from scipy import ndimage

from geometry_msgs.msg import PoseStamped, Quaternion
from nav_msgs.msg import Path


def yaw_to_quaternion(yaw):
    return Quaternion(x=0.0, y=0.0,
                      z=math.sin(yaw * 0.5), w=math.cos(yaw * 0.5))


class GridPlanner:
    def __init__(self, robot_radius=0.25, inflation_radius=0.45,
                 plan_resolution=0.10, allow_unknown=False,
                 clearance_weight=1.5, max_expansions=300000,
                 time_budget=2.0, local_obstacle_cost=30.0,
                 smoothing_clearance=0.35):
        self.robot_radius = float(robot_radius)
        self.inflation_radius = max(float(inflation_radius), self.robot_radius + 1e-3)
        self.plan_resolution = float(plan_resolution)
        self.allow_unknown = bool(allow_unknown)
        self.clearance_weight = float(clearance_weight)
        self.smoothing_clearance = float(smoothing_clearance)
        self.max_expansions = int(max_expansions)
        self.time_budget = float(time_budget)
        self.local_obstacle_cost = float(local_obstacle_cost)

        self._lock = threading.Lock()
        self._version = -1
        self._res = self.plan_resolution
        self._origin_x = 0.0
        self._origin_y = 0.0
        self._occ = None          # 原始障碍物，已换算到规划分辨率
        self._unknown = None
        self._dist = None         # 到最近障碍物的距离 [m]
        self._blocked = None      # 膨胀后的障碍物 + 未知区域

        self.last_error = ''

    # ------------------------------------------------------------------ 地图缓存
    def set_map(self, msg, version):
        if version == self._version:
            return True
        with self._lock:
            if version == self._version:
                return True
            info = msg.info
            try:
                data = np.asarray(msg.data, dtype=np.int8).reshape(
                    (info.height, info.width))
            except Exception as exc:
                self.last_error = 'bad map data: %s' % exc
                return False

            factor = max(1, int(round(self.plan_resolution / info.resolution)))
            if factor > 1:
                h = info.height // factor * factor
                w = info.width // factor * factor
                if h == 0 or w == 0:
                    self.last_error = 'map smaller than planning resolution'
                    return False
                blocks = data[:h, :w].reshape(h // factor, factor,
                                             w // factor, factor)
                occ = blocks.max(axis=(1, 3)) >= 50
                unknown = (blocks == -1).any(axis=(1, 3))
            else:
                occ = data >= 50
                unknown = data == -1

            res = info.resolution * factor
            dist = ndimage.distance_transform_edt(~occ) * res
            blocked = dist < self.robot_radius
            if not self.allow_unknown:
                blocked |= unknown

            self._res = res
            self._origin_x = info.origin.position.x
            self._origin_y = info.origin.position.y
            self._occ = occ
            self._unknown = unknown
            self._dist = dist.astype(np.float32)
            self._blocked = blocked
            self._version = version
            self.last_error = ''
            return True

    # ------------------------------------------------------------------ 辅助方法
    def _world_to_cell(self, x, y):
        ix = int(math.floor((x - self._origin_x) / self._res))
        iy = int(math.floor((y - self._origin_y) / self._res))
        return ix, iy

    def _cell_to_world(self, ix, iy):
        return (self._origin_x + (ix + 0.5) * self._res,
                self._origin_y + (iy + 0.5) * self._res)

    def _merge_local(self, blocked, local_grid):
        """返回 `blocked` 的副本，并把 local_grid 中的动态障碍物叠加进去。"""
        if local_grid is None:
            return blocked
        try:
            info = local_grid.info
            arr = np.asarray(local_grid.data, dtype=np.int8).reshape(
                (info.height, info.width))
        except Exception:
            return blocked
        iy, ix = np.nonzero(arr >= self.local_obstacle_cost)
        if ix.size == 0:
            return blocked
        wx = info.origin.position.x + (ix + 0.5) * info.resolution
        wy = info.origin.position.y + (iy + 0.5) * info.resolution
        gx = np.floor((wx - self._origin_x) / self._res).astype(np.int32)
        gy = np.floor((wy - self._origin_y) / self._res).astype(np.int32)
        h, w = blocked.shape
        ok = (gx >= 0) & (gx < w) & (gy >= 0) & (gy < h)
        merged = blocked.copy()
        merged[gy[ok], gx[ok]] = True
        return merged

    @staticmethod
    def _nearest_free(blocked, ix, iy, max_radius=12):
        h, w = blocked.shape
        if 0 <= ix < w and 0 <= iy < h and not blocked[iy, ix]:
            return ix, iy
        for r in range(1, max_radius + 1):
            for dy in range(-r, r + 1):
                for dx in range(-r, r + 1):
                    if max(abs(dx), abs(dy)) != r:
                        continue
                    nx, ny = ix + dx, iy + dy
                    if 0 <= nx < w and 0 <= ny < h and not blocked[ny, nx]:
                        return nx, ny
        return None

    def _segment_free(self, p0, p1, blocked, dist=None, clearance=0.0):
        ix0, iy0 = self._world_to_cell(*p0)
        ix1, iy1 = self._world_to_cell(*p1)
        h, w = blocked.shape
        dx, dy = abs(ix1 - ix0), abs(iy1 - iy0)
        sx = 1 if ix0 < ix1 else -1
        sy = 1 if iy0 < iy1 else -1
        err = dx - dy
        while True:
            if not (0 <= ix0 < w and 0 <= iy0 < h):
                return False
            if blocked[iy0, ix0]:
                return False
            if dist is not None and clearance > 0.0:
                if dist[iy0, ix0] < clearance:
                    return False
            if ix0 == ix1 and iy0 == iy1:
                return True
            e2 = 2 * err
            if e2 > -dy:
                err -= dy
                ix0 += sx
            if e2 < dx:
                err += dx
                iy0 += sy

    def _smooth_path(self, points, blocked, dist, iterations=2):
        """局部移动平均圆角，并保证平滑后每段仍满足安全距离。"""
        if len(points) < 3:
            return points
        current = list(points)
        for _ in range(max(0, int(iterations))):
            smoothed = list(current)
            for i in range(1, len(current) - 1):
                p0 = current[i - 1]
                p1 = current[i]
                p2 = current[i + 1]
                cand = (0.25 * p0[0] + 0.5 * p1[0] + 0.25 * p2[0],
                        0.25 * p0[1] + 0.5 * p1[1] + 0.25 * p2[1])
                if (self._segment_free(p0, cand, blocked, dist,
                                       self.smoothing_clearance) and
                        self._segment_free(cand, p2, blocked, dist,
                                           self.smoothing_clearance)):
                    smoothed[i] = cand
            current = smoothed
        return current

    # ------------------------------------------------------------------ 路径规划
    def plan(self, start, goal, local_grid=None):
        """start / goal 为 (x, y) 坐标。返回 nav_msgs/Path，失败时返回 None。"""
        if self._blocked is None:
            self.last_error = 'no map'
            return None
        with self._lock:
            blocked = self._merge_local(self._blocked, local_grid)
            dist = self._dist
            return self._plan_locked(start, goal, blocked, dist)

    def _plan_locked(self, start, goal, blocked, dist):
        h, w = blocked.shape
        sx, sy = self._world_to_cell(*start)
        gx, gy = self._world_to_cell(*goal)
        if not (0 <= sx < w and 0 <= sy < h and 0 <= gx < w and 0 <= gy < h):
            self.last_error = 'start/goal outside map'
            return None
        if blocked[sy, sx]:
            free = self._nearest_free(blocked, sx, sy, 20)
            if free is None:
                self.last_error = 'start in collision'
                return None
            sx, sy = free
        if blocked[gy, gx]:
            free = self._nearest_free(blocked, gx, gy, 30)
            if free is None:
                self.last_error = 'goal in collision'
                return None
            gx, gy = free

        # 在起点和终点周围裁剪搜索区域，保证 A* 足够快
        margin = int(round(5.0 / self._res))
        x0 = max(0, min(sx, gx) - margin)
        y0 = max(0, min(sy, gy) - margin)
        x1 = min(w - 1, max(sx, gx) + margin)
        y1 = min(h - 1, max(sy, gy) + margin)
        sub = blocked[y0:y1 + 1, x0:x1 + 1]
        sub_dist = dist[y0:y1 + 1, x0:x1 + 1]
        sh, sw = sub.shape
        s_idx = (sy - y0) * sw + (sx - x0)
        g_idx = (gy - y0) * sw + (gx - x0)

        moves = [(-1, 0, 1.0), (1, 0, 1.0), (0, -1, 1.0), (0, 1, 1.0),
                 (-1, -1, 1.41421356), (-1, 1, 1.41421356),
                 (1, -1, 1.41421356), (1, 1, 1.41421356)]

        gscore = np.full(sh * sw, np.inf, dtype=np.float64)
        parent = np.full(sh * sw, -1, dtype=np.int32)
        closed = np.zeros(sh * sw, dtype=bool)
        gscore[s_idx] = 0.0

        def heuristic(idx):
            cx, cy = idx % sw, idx // sw
            dx, dy = abs(cx - (gx - x0)), abs(cy - (gy - y0))
            return (dx + dy + (1.41421356 - 2.0) * min(dx, dy)) * self._res

        heap = [(heuristic(s_idx), 0.0, s_idx)]
        infl = self.inflation_radius
        cw = self.clearance_weight
        found = False
        expansions = 0
        deadline = time.monotonic() + self.time_budget
        self.last_error = ''

        while heap:
            _, g, idx = heapq.heappop(heap)
            if closed[idx]:
                continue
            if idx == g_idx:
                found = True
                break
            closed[idx] = True
            expansions += 1
            if expansions > self.max_expansions:
                self.last_error = 'max expansions reached'
                break
            if (expansions & 2047) == 0 and time.monotonic() > deadline:
                self.last_error = 'planning timeout'
                break

            cx, cy = idx % sw, idx // sw
            for dx, dy, step in moves:
                nx, ny = cx + dx, cy + dy
                if nx < 0 or nx >= sw or ny < 0 or ny >= sh:
                    continue
                nidx = ny * sw + nx
                if closed[nidx] or sub[ny, nx]:
                    continue
                if dx != 0 and dy != 0:
                    if sub[cy, nx] or sub[ny, cx]:   # 禁止斜穿墙角
                        continue
                d = sub_dist[ny, nx]
                mult = 1.0 + cw * max(0.0, 1.0 - d / infl)
                ng = g + step * self._res * mult
                if ng + 1e-9 < gscore[nidx]:
                    gscore[nidx] = ng
                    parent[nidx] = idx
                    heapq.heappush(heap, (ng + heuristic(nidx), ng, nidx))

        if not found:
            if not self.last_error:
                self.last_error = 'no path'
            return None

        cells = []
        idx = g_idx
        while idx != -1:
            cells.append((x0 + idx % sw, y0 + idx // sw))
            idx = parent[idx]
        cells.reverse()

        pts = [self._cell_to_world(ix, iy) for ix, iy in cells]

        # 视线可达性平滑（line-of-sight smoothing）
        kept = [0]
        i = 0
        while i < len(pts) - 1:
            j = len(pts) - 1
            while j > i + 1 and not self._segment_free(
                    pts[i], pts[j], blocked, dist, self.smoothing_clearance):
                j -= 1
            kept.append(j)
            i = j

        # 插值加密到约 0.1 m 的间距
        dense = []
        for a, b in zip(kept[:-1], kept[1:]):
            pa, pb = pts[a], pts[b]
            d = math.hypot(pb[0] - pa[0], pb[1] - pa[1])
            n = max(1, int(math.ceil(d / 0.10)))
            for k in range(n):
                t = k / float(n)
                dense.append((pa[0] + (pb[0] - pa[0]) * t,
                              pa[1] + (pb[1] - pa[1]) * t))
        dense.append(pts[-1])
        dense = self._smooth_path(dense, blocked, dist, iterations=2)

        path = Path()
        path.header.frame_id = 'map'
        for i, (x, y) in enumerate(dense):
            yaw = 0.0
            if i + 1 < len(dense):
                yaw = math.atan2(dense[i + 1][1] - y, dense[i + 1][0] - x)
            elif i > 0:
                yaw = math.atan2(y - dense[i - 1][1], x - dense[i - 1][0])
            pose = PoseStamped()
            pose.header.frame_id = 'map'
            pose.pose.position.x = float(x)
            pose.pose.position.y = float(y)
            pose.pose.orientation = yaw_to_quaternion(yaw)
            path.poses.append(pose)

        self.last_error = ''
        return path
