#!/usr/bin/env python3
"""Local controller: DWA sampling + a local A* detour planner.

When the global path is blocked by a dynamic obstacle inside the rolling local
costmap, this controller plans a short local A* path around the obstacle and
tracks that path with DWA.  The final goal always comes from the global path.
"""

import heapq
import math
import time

import numpy as np
from geometry_msgs.msg import Twist


def wrap_angle(a):
    while a > math.pi:
        a -= 2.0 * math.pi
    while a < -math.pi:
        a += 2.0 * math.pi
    return a


class DwaController:
    def __init__(self, max_linear=0.35, max_angular=1.0, lookahead=0.6,
                 goal_tolerance=0.15, goal_yaw_tolerance=0.35,
                 goal_hold_time=1.0, goal_slow_radius=0.6,
                 rotate_in_place_error=0.8, rotate_in_place_gain=1.0,
                 path_heading_weight=0.15, curvature_weight=0.05,
                 sim_time=1.2, sim_dt=0.1,
                 v_samples=5, w_samples=13, max_dv=0.15, max_dw=0.6,
                 heading_weight=0.45, clearance_weight=0.25,
                 path_weight=0.20, speed_weight=0.10,
                 unknown_cost=10.0, collision_cost=30.0,
                 local_plan_distance=2.5, local_replan_period=0.4,
                 local_blocked_cost=30.0):
        self.max_linear = float(max_linear)
        self.max_angular = float(max_angular)
        self.lookahead = float(lookahead)
        self.goal_tolerance = float(goal_tolerance)
        self.goal_yaw_tolerance = float(goal_yaw_tolerance)
        self.goal_hold_time = max(0.0, float(goal_hold_time))
        self.goal_slow_radius = max(0.1, float(goal_slow_radius))
        self.rotate_in_place_error = float(rotate_in_place_error)
        self.rotate_in_place_gain = float(rotate_in_place_gain)
        self.path_heading_weight = float(path_heading_weight)
        self.curvature_weight = float(curvature_weight)
        self.sim_time = float(sim_time)
        self.sim_dt = float(sim_dt)
        self.v_samples = max(2, int(v_samples))
        self.w_samples = max(3, int(w_samples))
        self.max_dv = float(max_dv)
        self.max_dw = float(max_dw)
        self.heading_weight = float(heading_weight)
        self.clearance_weight = float(clearance_weight)
        self.path_weight = float(path_weight)
        self.speed_weight = float(speed_weight)
        self.unknown_cost = float(unknown_cost)
        self.collision_cost = float(collision_cost)
        self.local_plan_distance = float(local_plan_distance)
        self.local_replan_period = float(local_replan_period)
        self.local_blocked_cost = float(local_blocked_cost)

        self._global_path = np.zeros((0, 2), dtype=np.float64)
        self._local_path = np.zeros((0, 2), dtype=np.float64)
        self._local_stamp = -1e9
        self._last_v = 0.0
        self._last_w = 0.0
        self._goal_yaw = None
        self._arrival_since = None
        self.debug_goal_distance = None
        self.debug_goal_yaw_error = None
        self.debug_local_points = 0
        self.debug_active_local = False
        self.debug_blocked = False
        self.debug_candidates = 0
        self.debug_plan_attempts = 0
        self.debug_plan_ok = False

    # ------------------------------------------------------------------ path
    def set_path(self, path_msg, goal_yaw=None):
        pts = [(p.pose.position.x, p.pose.position.y) for p in path_msg.poses]
        self._global_path = (np.asarray(pts, dtype=np.float64)
                             if pts else np.zeros((0, 2)))
        self._local_path = np.zeros((0, 2), dtype=np.float64)
        self._local_stamp = -1e9
        self._last_v = 0.0
        self._last_w = 0.0
        self._goal_yaw = (wrap_angle(float(goal_yaw))
                          if goal_yaw is not None else None)
        self._arrival_since = None
        self.debug_goal_distance = None
        self.debug_goal_yaw_error = None

    def reset(self):
        self._global_path = np.zeros((0, 2), dtype=np.float64)
        self._local_path = np.zeros((0, 2), dtype=np.float64)
        self._local_stamp = -1e9
        self._last_v = 0.0
        self._last_w = 0.0
        self._goal_yaw = None
        self._arrival_since = None
        self.debug_goal_distance = None
        self.debug_goal_yaw_error = None

    def has_path(self):
        return self._global_path.shape[0] > 0

    def _carrot(self, path, x, y):
        d = np.hypot(path[:, 0] - x, path[:, 1] - y)
        idx = int(np.argmin(d))
        for i in range(idx, path.shape[0]):
            if math.hypot(path[i, 0] - x, path[i, 1] - y) >= self.lookahead:
                return path[i]
        return path[-1]

    def _pure_pursuit(self, x, y, yaw, carrot):
        dx = carrot[0] - x
        dy = carrot[1] - y
        dist = math.hypot(dx, dy)
        err = wrap_angle(math.atan2(dy, dx) - yaw)
        cmd = Twist()
        if abs(err) > 0.5:
            cmd.linear.x = 0.0
            cmd.angular.z = self.max_angular if err > 0 else -self.max_angular
        else:
            cmd.linear.x = min(self.max_linear, max(0.05, dist))
            cmd.angular.z = max(-self.max_angular, min(self.max_angular, 1.8 * err))
        return cmd

    def _smooth(self, v, w):
        dv = max(-self.max_dv, min(self.max_dv, v - self._last_v))
        dw = max(-self.max_dw, min(self.max_dw, w - self._last_w))
        self._last_v += dv
        self._last_w += dw
        return self._last_v, self._last_w

    # ------------------------------------------------------------------ local grid
    @staticmethod
    def _cell(x, y, info):
        return (int(math.floor((x - info.origin.position.x) /
                               info.resolution)),
                int(math.floor((y - info.origin.position.y) /
                               info.resolution)))

    def _cell_cost(self, raw, info, x, y):
        ix, iy = self._cell(x, y, info)
        if ix < 0 or ix >= info.width or iy < 0 or iy >= info.height:
            return None
        v = int(raw[iy, ix])
        return 10.0 if v < 0 else float(v)

    def _nearest_free(self, blocked, ix, iy, max_radius=15):
        h, w = blocked.shape
        for r in range(0, max_radius + 1):
            for dy in range(-r, r + 1):
                for dx in range(-r, r + 1):
                    if max(abs(dx), abs(dy)) != r:
                        continue
                    nx, ny = ix + dx, iy + dy
                    if 0 <= nx < w and 0 <= ny < h and not blocked[ny, nx]:
                        return nx, ny
        return None

    def _plan_local_grid(self, raw, info, start, goal):
        """A* on the local costmap. Returns world points (N,2) or None."""
        res = info.resolution
        h, w = raw.shape
        blocked = raw >= self.local_blocked_cost

        sx, sy = self._cell(start[0], start[1], info)
        gx, gy = self._cell(goal[0], goal[1], info)
        sx = max(0, min(w - 1, sx))
        sy = max(0, min(h - 1, sy))
        gx = max(0, min(w - 1, gx))
        gy = max(0, min(h - 1, gy))

        if blocked[sy, sx]:
            free = self._nearest_free(blocked, sx, sy, 20)
            if free is None:
                return None
            sx, sy = free
        if blocked[gy, gx]:
            free = self._nearest_free(blocked, gx, gy, 30)
            if free is None:
                return None
            gx, gy = free

        start_idx = sy * w + sx
        goal_idx = gy * w + gx
        gscore = np.full(h * w, np.inf, dtype=np.float64)
        parent = np.full(h * w, -1, dtype=np.int32)
        closed = np.zeros(h * w, dtype=bool)
        gscore[start_idx] = 0.0

        def hcost(idx):
            cx, cy = idx % w, idx // w
            dx, dy = abs(cx - gx), abs(cy - gy)
            return (dx + dy + (1.41421356 - 2.0) * min(dx, dy)) * res

        moves = [(-1, 0, 1.0), (1, 0, 1.0), (0, -1, 1.0), (0, 1, 1.0),
                 (-1, -1, 1.41421356), (-1, 1, 1.41421356),
                 (1, -1, 1.41421356), (1, 1, 1.41421356)]
        heap = [(hcost(start_idx), 0.0, start_idx)]
        found = False
        expansions = 0

        while heap:
            _, g, idx = heapq.heappop(heap)
            if closed[idx]:
                continue
            if idx == goal_idx:
                found = True
                break
            closed[idx] = True
            expansions += 1
            if expansions > 40000:
                return None
            cx, cy = idx % w, idx // w
            for dx, dy, step in moves:
                nx, ny = cx + dx, cy + dy
                if nx < 0 or nx >= w or ny < 0 or ny >= h:
                    continue
                nidx = ny * w + nx
                if closed[nidx] or blocked[ny, nx]:
                    continue
                if dx != 0 and dy != 0 and (blocked[cy, nx] or blocked[ny, cx]):
                    continue
                cost = max(0.0, float(raw[ny, nx]))
                ng = g + step * res * (1.0 + 2.0 * cost / 100.0)
                if ng + 1e-9 < gscore[nidx]:
                    gscore[nidx] = ng
                    parent[nidx] = idx
                    heapq.heappush(heap, (ng + hcost(nidx), ng, nidx))

        if not found:
            return None

        cells = []
        idx = goal_idx
        while idx != -1:
            cells.append((idx % w, idx // w))
            idx = parent[idx]
        cells.reverse()

        points = []
        for a, b in zip(cells[:-1], cells[1:]):
            pa = (info.origin.position.x + (a[0] + 0.5) * res,
                  info.origin.position.y + (a[1] + 0.5) * res)
            pb = (info.origin.position.x + (b[0] + 0.5) * res,
                  info.origin.position.y + (b[1] + 0.5) * res)
            dist = math.hypot(pb[0] - pa[0], pb[1] - pa[1])
            n = max(1, int(math.ceil(dist / 0.10)))
            for k in range(n):
                t = k / float(n)
                points.append((pa[0] + (pb[0] - pa[0]) * t,
                               pa[1] + (pb[1] - pa[1]) * t))
        points.append((info.origin.position.x + (cells[-1][0] + 0.5) * res,
                       info.origin.position.y + (cells[-1][1] + 0.5) * res))
        return np.asarray(points, dtype=np.float64)

    def _maybe_local_path(self, x, y, raw, info):
        now = time.monotonic()
        if (self._local_path.shape[0] >= 2 and
                now - self._local_stamp < self.local_replan_period):
            return self._local_path

        d = np.hypot(self._global_path[:, 0] - x,
                     self._global_path[:, 1] - y)
        idx = int(np.argmin(d))
        blocked = False
        detour_goal = None
        after_block = False
        candidates = 0
        for i in range(idx, self._global_path.shape[0]):
            p = self._global_path[i]
            dist = math.hypot(p[0] - x, p[1] - y)
            if dist > self.local_plan_distance:
                break
            cost = self._cell_cost(raw, info, p[0], p[1])
            if cost is None:
                continue
            if cost >= self.local_blocked_cost:
                blocked = True
                after_block = True
                continue
            if after_block:
                # 障碍后方的第一个/最远空闲点作为局部 A* 目标，
                # 这样局部路径才会真正绕过障碍，而不是停在障碍前。
                detour_goal = p
                candidates += 1

        self.debug_blocked = blocked
        self.debug_candidates = candidates
        if not blocked:
            self._local_path = np.zeros((0, 2), dtype=np.float64)
            return None
        if detour_goal is None:
            # 障碍后方在局部窗口内没有空闲点，交给恢复行为处理。
            self._local_path = np.zeros((0, 2), dtype=np.float64)
            return None
        if math.hypot(detour_goal[0] - x, detour_goal[1] - y) < 0.6:
            return None

        self.debug_plan_attempts += 1
        pts = self._plan_local_grid(raw, info, (x, y), detour_goal)
        self.debug_plan_ok = pts is not None
        if pts is None or pts.shape[0] < 2:
            return None
        self._local_path = pts
        self._local_stamp = now
        return pts

    # ------------------------------------------------------------------ control
    def compute(self, x, y, yaw, local_grid=None):
        """Return (Twist, arrived, blocked)."""
        zero = Twist()
        if self._global_path.shape[0] < 2:
            return zero, False, True

        goal = self._global_path[-1]
        d_goal = math.hypot(goal[0] - x, goal[1] - y)
        self.debug_goal_distance = d_goal
        yaw_err = None
        if self._goal_yaw is not None:
            yaw_err = wrap_angle(self._goal_yaw - yaw)
        self.debug_goal_yaw_error = yaw_err

        if d_goal < self.goal_tolerance:
            # 进入终点半径后还要满足朝向，并保持一小段时间；
            # 这样不会因为左右摆动瞬间扫过目标就误判到达。
            if (self.goal_yaw_tolerance > 0.0 and yaw_err is not None and
                    abs(yaw_err) > self.goal_yaw_tolerance):
                self._arrival_since = None
                # 到点后只原地转，不再保留残余前进速度，否则会绕目标画小圈
                self._last_v = 0.0
                w = max(-self.max_angular,
                        min(self.max_angular, 0.8 * yaw_err))
                _, w = self._smooth(0.0, w)
                cmd = Twist()
                cmd.linear.x = 0.0
                cmd.angular.z = w
                return cmd, False, False

            now = time.monotonic()
            if self._arrival_since is None:
                self._arrival_since = now
            if now - self._arrival_since >= self.goal_hold_time:
                self._arrival_since = None
                self._last_v = self._last_w = 0.0
                return zero, True, False
            self._last_v = self._last_w = 0.0
            return zero, False, False

        self._arrival_since = None

        raw = None
        info = None
        active = self._global_path
        if local_grid is not None and local_grid.data:
            info = local_grid.info
            try:
                raw = np.asarray(local_grid.data, dtype=np.int8).reshape(
                    (info.height, info.width))
            except Exception:
                raw = None
        if raw is not None:
            local = self._maybe_local_path(x, y, raw, info)
            if local is not None:
                active = local

        self.debug_local_points = int(self._local_path.shape[0])
        self.debug_active_local = bool(raw is not None and
                                       self._local_path.shape[0] >= 2 and
                                       active is self._local_path)
        carrot = self._carrot(active, x, y)

        # 大角度偏差时先原地转。DWA 只采样前进速度，如果不先转，
        # 它会用一条前进弧线去“绕”到路径方向，轨迹就会变成一串 C。
        carrot_err = wrap_angle(
            math.atan2(carrot[1] - y, carrot[0] - x) - yaw)
        if abs(carrot_err) > self.rotate_in_place_error:
            self._last_v = 0.0
            w = max(-self.max_angular,
                    min(self.max_angular,
                        self.rotate_in_place_gain * carrot_err))
            _, w = self._smooth(0.0, w)
            cmd = Twist()
            cmd.linear.x = 0.0
            cmd.angular.z = w
            return cmd, False, False

        if raw is None:
            cmd = self._pure_pursuit(x, y, yaw, carrot)
            v, w = self._smooth(cmd.linear.x, cmd.angular.z)
            cmd.linear.x, cmd.angular.z = v, w
            return cmd, False, False

        res = info.resolution
        ox = info.origin.position.x
        oy = info.origin.position.y
        arr = np.where(raw < 0, self.unknown_cost, raw).astype(np.float64)

        ci, cj = int((x - ox) / res), int((y - oy) / res)
        if 0 <= ci < info.width and 0 <= cj < info.height:
            if arr[cj, ci] >= 99.0:
                self._last_v = self._last_w = 0.0
                return zero, False, True

        max_v = self.max_linear
        if d_goal < self.goal_slow_radius:
            # 终点前线性降速，减少冲过目标后来回修正
            max_v = max(0.04, self.max_linear * d_goal / self.goal_slow_radius)
        v_list = np.linspace(0.0, max_v, self.v_samples)
        w_list = np.linspace(-self.max_angular, self.max_angular, self.w_samples)
        V = np.repeat(v_list, len(w_list))
        W = np.tile(w_list, len(v_list))
        S = V.size
        N = max(2, int(round(self.sim_time / self.sim_dt)))

        xs = np.zeros((S, N))
        ys = np.zeros((S, N))
        ths = np.zeros((S, N))
        xs[:, 0], ys[:, 0], ths[:, 0] = x, y, yaw
        for k in range(1, N):
            ths[:, k] = ths[:, k - 1] + W * self.sim_dt
            xs[:, k] = xs[:, k - 1] + V * np.cos(ths[:, k]) * self.sim_dt
            ys[:, k] = ys[:, k - 1] + V * np.sin(ths[:, k]) * self.sim_dt

        ix = np.floor((xs - ox) / res).astype(np.int32)
        iy = np.floor((ys - oy) / res).astype(np.int32)
        valid = (ix >= 0) & (ix < info.width) & (iy >= 0) & (iy < info.height)
        cost = np.full(xs.shape, self.unknown_cost, dtype=np.float64)
        cost[valid] = arr[iy[valid], ix[valid]]

        collided = (cost >= self.collision_cost).any(axis=1)
        clearance = 1.0 - np.minimum(cost.max(axis=1), 100.0) / 100.0

        target_yaw = np.arctan2(carrot[1] - ys[:, -1], carrot[0] - xs[:, -1])
        heading = 1.0 - np.abs(np.arctan2(np.sin(target_yaw - ths[:, -1]),
                                          np.cos(target_yaw - ths[:, -1]))) / math.pi

        d_all = np.hypot(active[None, :, 0] - xs[:, -1, None],
                         active[None, :, 1] - ys[:, -1, None])
        nearest = np.argmin(d_all, axis=1)
        d_path = d_all[np.arange(S), nearest]
        path_score = 1.0 / (1.0 + d_path)

        # 轨迹末端还要与路径切向一致，避免“贴着路径但车头朝外”的 C 形轨迹
        nxt = np.minimum(nearest + 1, active.shape[0] - 1)
        prv = np.maximum(nearest - 1, 0)
        tx = active[nxt, 0] - active[prv, 0]
        ty = active[nxt, 1] - active[prv, 1]
        path_yaw = np.arctan2(ty, tx)
        path_align = 1.0 - np.abs(
            np.arctan2(np.sin(path_yaw - ths[:, -1]),
                       np.cos(path_yaw - ths[:, -1]))) / math.pi
        # 路径点重合时没有有效切向，不做切向惩罚
        path_align[np.hypot(tx, ty) < 1e-9] = 1.0

        # 同样速度下少选大角速度候选，抑制左右来回画弧
        curvature = np.abs(W) / np.maximum(V, 0.10)

        speed_score = V / max(self.max_linear, 1e-3)
        score = (self.heading_weight * heading +
                 self.clearance_weight * clearance +
                 self.path_weight * path_score +
                 self.path_heading_weight * path_align +
                 self.speed_weight * speed_score -
                 self.curvature_weight * np.minimum(curvature, 1.0))
        score[collided] = -1.0
        score[(V < 1e-6) & (np.abs(W) < 1e-6)] = -1.0

        best = int(np.argmax(score))
        if score[best] < 0.0:
            self._last_v = self._last_w = 0.0
            return zero, False, True

        v, w = self._smooth(float(V[best]), float(W[best]))
        if abs(v) < 1e-3 and abs(w) < 1e-3:
            return zero, False, True

        cmd = Twist()
        cmd.linear.x = v
        cmd.angular.z = w
        return cmd, False, False
