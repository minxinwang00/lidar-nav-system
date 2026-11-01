#!/usr/bin/env python3
"""导航决策层使用的行为树（behaviour tree）叶子节点集合。"""

import math
import threading

import numpy as np
import py_trees
from geometry_msgs.msg import Twist
from std_srvs.srv import Empty


# ---------------------------------------------------------------------- 辅助函数
def _grid_data(node):
    with node.ctx.lock:
        grid = node.ctx.local_grid
    if grid is None or not grid.data:
        return None
    try:
        arr = np.asarray(grid.data, dtype=np.int8).reshape(
            (grid.info.height, grid.info.width))
        return (arr, grid.info.resolution,
                grid.info.origin.position.x, grid.info.origin.position.y)
    except Exception:
        return None


def _costs_at(node, pose, offsets, sign=1.0):
    """取机器人前方（+1）或后方（-1）若干偏移位置上的局部代价最大值。"""
    data = _grid_data(node)
    if data is None or pose is None:
        return None
    arr, res, ox, oy = data
    x, y, yaw = pose
    values = []
    for d in offsets:
        px = x + sign * d * math.cos(yaw)
        py = y + sign * d * math.sin(yaw)
        ix = int(math.floor((px - ox) / res))
        iy = int(math.floor((py - oy) / res))
        if 0 <= ix < arr.shape[1] and 0 <= iy < arr.shape[0]:
            v = int(arr[iy, ix])
            values.append(60 if v < 0 else v)   # 未知区域按保守值处理
    return values or None


class MapReady(py_trees.behaviour.Behaviour):
    """在收到占据栅格地图（occupancy grid）之前一直返回 RUNNING。"""

    def __init__(self, name, node):
        super().__init__(name)
        self.node = node
        self._warned = False

    def update(self):
        with self.node.ctx.lock:
            ready = self.node.ctx.map_msg is not None
        if ready:
            return py_trees.common.Status.SUCCESS
        if not self._warned:
            self.node.get_logger().info('Waiting for /map ...')
            self._warned = True
        return py_trees.common.Status.RUNNING


class GoalAvailable(py_trees.behaviour.Behaviour):
    """没有目标点时返回 RUNNING（空闲状态）。"""

    def __init__(self, name, node):
        super().__init__(name)
        self.node = node

    def update(self):
        with self.node.ctx.lock:
            has_goal = self.node.ctx.goal is not None
            if not has_goal:
                self.node.ctx.state = 'IDLE'
        return (py_trees.common.Status.SUCCESS if has_goal
                else py_trees.common.Status.RUNNING)


class PlanPath(py_trees.behaviour.Behaviour):
    """在工作线程中运行 A*，保证 10 Hz 的控制循环不被阻塞。"""

    def __init__(self, name, node, planner):
        super().__init__(name)
        self.node = node
        self.planner = planner
        self._gen = 0
        self._thread = None
        self._result = None

    def initialise(self):
        self._gen += 1
        self._thread = None
        self._result = None
        with self.node.ctx.lock:
            self.node.ctx.state = 'PLANNING'

    def _worker(self, gen, start, goal, map_msg, version, local_grid):
        try:
            if not self.planner.set_map(map_msg, version):
                self._result = (gen, None, self.planner.last_error)
                return
            path = self.planner.plan(start, goal, local_grid)
            self._result = (gen, path, self.planner.last_error)
        except Exception as exc:  # 规划器内部出错也不能影响整棵行为树
            self._result = (gen, None, 'planner exception: %s' % exc)

    def update(self):
        ctx = self.node.ctx
        with ctx.lock:
            goal = ctx.goal
            map_msg = ctx.map_msg
            version = ctx.map_version
            local_grid = ctx.local_grid
            pose = ctx.robot_pose

        if goal is None:
            return py_trees.common.Status.FAILURE
        if map_msg is None:
            return py_trees.common.Status.RUNNING
        if pose is None:
            return py_trees.common.Status.RUNNING

        if self._thread is None:
            self._thread = threading.Thread(
                target=self._worker,
                args=(self._gen, (pose[0], pose[1]), (goal[0], goal[1]),
                      map_msg, version, local_grid),
                daemon=True)
            self._thread.start()
            return py_trees.common.Status.RUNNING

        if self._thread.is_alive():
            return py_trees.common.Status.RUNNING

        result = self._result
        if result is None:
            return py_trees.common.Status.FAILURE
        gen, path, err = result
        if gen != self._gen:
            # 这是更早一次请求的结果，继续等待当前请求
            return py_trees.common.Status.RUNNING
        if path is None:
            with ctx.lock:
                ctx.last_error = err
                ctx.state = 'PLAN_FAILED'
            self.node.get_logger().warn('Planning failed: %s' % err)
            return py_trees.common.Status.FAILURE

        path.header.stamp = self.node.get_clock().now().to_msg()
        with ctx.lock:
            if ctx.goal is None:
                return py_trees.common.Status.RUNNING
            if (abs(ctx.goal[0] - goal[0]) > 1e-3 or
                    abs(ctx.goal[1] - goal[1]) > 1e-3):
                return py_trees.common.Status.RUNNING
            ctx.path = path
            ctx.path_stamp = self.node.get_clock().now()
            ctx.blocked = False
            ctx.state = 'TRACKING'
        self.node.path_pub.publish(path)
        self.node.get_logger().info('Path planned: %d poses' % len(path.poses))
        return py_trees.common.Status.SUCCESS


class FollowPath(py_trees.behaviour.Behaviour):
    """用 DWA 控制器跟踪当前路径。"""

    def __init__(self, name, node, controller, blocked_timeout=0.6):
        super().__init__(name)
        self.node = node
        self.controller = controller
        self.blocked_timeout = float(blocked_timeout)
        self._blocked_since = None

    def initialise(self):
        ctx = self.node.ctx
        self._blocked_since = None
        with ctx.lock:
            path = ctx.path
            goal = ctx.goal
            ctx.state = 'TRACKING'
            ctx.blocked = False
        if path is not None:
            goal_yaw = goal[2] if goal is not None else None
            self.controller.set_path(path, goal_yaw=goal_yaw)
        else:
            self.controller.reset()

    def update(self):
        ctx = self.node.ctx
        with ctx.lock:
            if ctx.estop:
                return py_trees.common.Status.RUNNING
            pose = ctx.robot_pose
            local_grid = ctx.local_grid
            has_path = ctx.path is not None

        if not has_path or not self.controller.has_path():
            return py_trees.common.Status.FAILURE
        if pose is None:
            return py_trees.common.Status.RUNNING

        cmd, arrived, blocked = self.controller.compute(
            pose[0], pose[1], pose[2], local_grid)

        now = self.node.get_clock().now()
        if arrived:
            self.node.cmd_pub.publish(Twist())
            dist = getattr(self.controller, 'debug_goal_distance', None)
            yaw_err = getattr(self.controller, 'debug_goal_yaw_error', None)
            self.node.get_logger().info(
                'Goal reached: dist=%s m yaw_err=%s rad'
                % ('%.2f' % dist if dist is not None else '?',
                   '%.2f' % yaw_err if yaw_err is not None else '?'))
            return py_trees.common.Status.SUCCESS

        if blocked:
            with ctx.lock:
                ctx.blocked = True
                ctx.state = 'BLOCKED'
            if self._blocked_since is None:
                self._blocked_since = now
                self.node.get_logger().warn('Path blocked, waiting for clearance')
            elapsed = (now - self._blocked_since).nanoseconds / 1e9
            if elapsed > self.blocked_timeout:
                self.node.cmd_pub.publish(Twist())
                self.node.get_logger().warn('Path blocked, triggering recovery')
                return py_trees.common.Status.FAILURE
            self.node.cmd_pub.publish(Twist())
            return py_trees.common.Status.RUNNING

        self._blocked_since = None
        with ctx.lock:
            ctx.blocked = False
            ctx.state = 'TRACKING'
        self.node.cmd_pub.publish(cmd)
        return py_trees.common.Status.RUNNING


class ClearPath(py_trees.behaviour.Behaviour):
    """清除当前路径，使下一次 PlanPath 重新计算。"""

    def __init__(self, name, node):
        super().__init__(name)
        self.node = node

    def update(self):
        ctx = self.node.ctx
        with ctx.lock:
            ctx.path = None
            ctx.blocked = False
            ctx.state = 'RECOVERY'
        return py_trees.common.Status.SUCCESS


class WaitForClearance(py_trees.behaviour.Behaviour):
    """停车等待动态障碍物离开。

    一旦前方通道重新空出就返回 FAILURE：这会中止恢复链的后续步骤，让 Retry
    装饰器立即重新规划。只有完整等待时间结束后才返回 SUCCESS，说明障碍物是
    持续存在的。
    """

    def __init__(self, name, node, duration=1.2, clear_cost=50):
        super().__init__(name)
        self.node = node
        self.duration = float(duration)
        self.clear_cost = int(clear_cost)
        self._start = None

    def initialise(self):
        self._start = self.node.get_clock().now()
        self.node.get_logger().warn('Recovery: waiting for obstacle clearance')

    def update(self):
        self.node.cmd_pub.publish(Twist())
        elapsed = (self.node.get_clock().now() - self._start).nanoseconds / 1e9
        with self.node.ctx.lock:
            pose = self.node.ctx.robot_pose
        costs = _costs_at(self.node, pose, [0.4, 0.8, 1.2])
        if costs is not None and max(costs) < self.clear_cost and elapsed > 0.3:
            self.node.get_logger().info('Recovery: corridor clear, replanning')
            return py_trees.common.Status.FAILURE
        if elapsed >= self.duration:
            self.node.get_logger().warn('Recovery: obstacle still present')
            return py_trees.common.Status.SUCCESS
        return py_trees.common.Status.RUNNING


class BackUp(py_trees.behaviour.Behaviour):
    """向后退离障碍物，并对后方空间做安全检查。"""

    def __init__(self, name, node, distance=0.35, speed=0.12, timeout=4.0):
        super().__init__(name)
        self.node = node
        self.distance = float(distance)
        self.speed = float(speed)
        self.timeout = float(timeout)
        self._start = None
        self._start_time = None
        self._unsafe = False

    def initialise(self):
        with self.node.ctx.lock:
            self._start = self.node.ctx.robot_pose
        self._start_time = self.node.get_clock().now()
        rear = _costs_at(self.node, self._start, [-0.3, -0.5, -0.7])
        self._unsafe = rear is not None and max(rear) >= 70
        if self._unsafe:
            self.node.get_logger().warn('Recovery: rear blocked, skip backup')
        else:
            self.node.get_logger().warn('Recovery: backing up %.2f m' % self.distance)

    def update(self):
        now = self.node.get_clock().now()
        elapsed = (now - self._start_time).nanoseconds / 1e9
        if self._unsafe:
            self.node.cmd_pub.publish(Twist())
            return py_trees.common.Status.FAILURE

        with self.node.ctx.lock:
            pose = self.node.ctx.robot_pose
        if self._start is not None and pose is not None:
            travelled = math.hypot(pose[0] - self._start[0],
                                   pose[1] - self._start[1])
        else:
            travelled = elapsed * self.speed

        if travelled >= self.distance or elapsed > self.timeout:
            self.node.cmd_pub.publish(Twist())
            return py_trees.common.Status.SUCCESS

        rear = _costs_at(self.node, pose, [-0.25, -0.4, -0.6])
        if rear is not None and max(rear) >= 80:
            self.node.get_logger().warn('Recovery: rear blocked during backup')
            self.node.cmd_pub.publish(Twist())
            return py_trees.common.Status.FAILURE

        cmd = Twist()
        cmd.linear.x = -abs(self.speed)
        self.node.cmd_pub.publish(cmd)
        return py_trees.common.Status.RUNNING

    def terminate(self, new_status):
        self.node.cmd_pub.publish(Twist())


class RecoverySpin(py_trees.behaviour.Behaviour):
    """原地旋转，直到前方通道空出或计时结束。"""

    def __init__(self, name, node, duration=1.8, angular_speed=0.6,
                 min_time=0.5, clear_cost=50):
        super().__init__(name)
        self.node = node
        self.duration = float(duration)
        self.angular_speed = float(angular_speed)
        self.min_time = float(min_time)
        self.clear_cost = int(clear_cost)
        self._start = None
        self._direction = 1.0

    def initialise(self):
        self._direction = -self._direction
        self._start = self.node.get_clock().now()
        self.node.get_logger().warn('Recovery: rotating in place')

    def update(self):
        elapsed = (self.node.get_clock().now() - self._start).nanoseconds / 1e9
        if elapsed >= self.duration:
            self.node.cmd_pub.publish(Twist())
            return py_trees.common.Status.SUCCESS

        if elapsed > self.min_time:
            with self.node.ctx.lock:
                pose = self.node.ctx.robot_pose
            costs = _costs_at(self.node, pose, [0.4, 0.8, 1.0])
            if costs is not None and max(costs) < self.clear_cost:
                self.node.get_logger().info('Recovery: rotated to a free heading')
                self.node.cmd_pub.publish(Twist())
                return py_trees.common.Status.SUCCESS

        cmd = Twist()
        cmd.angular.z = self._direction * self.angular_speed
        self.node.cmd_pub.publish(cmd)
        return py_trees.common.Status.RUNNING

    def terminate(self, new_status):
        self.node.cmd_pub.publish(Twist())


class GlobalRelocalize(py_trees.behaviour.Behaviour):
    """请求 AMCL 在整个地图范围内重新撒播粒子（绑架恢复 kidnapped recovery）。"""

    def __init__(self, name, node,
                 service='/reinitialize_global_localization', timeout=2.5):
        super().__init__(name)
        self.node = node
        self.timeout = float(timeout)
        self.client = node.create_client(Empty, service)
        self._future = None
        self._start = None

    def initialise(self):
        self._future = None
        self._start = None

    def update(self):
        if not getattr(self.node, 'relocalize_enabled', True):
            self.node.get_logger().info(
                'Recovery: global relocalization disabled; skip')
            return py_trees.common.Status.SUCCESS
        if not self.client.service_is_ready():
            self.node.get_logger().warn(
                'Recovery: relocalization service unavailable')
            return py_trees.common.Status.FAILURE
        now = self.node.get_clock().now()
        if self._future is None:
            self._future = self.client.call_async(Empty.Request())
            self._start = now
            self.node.get_logger().warn('Recovery: global relocalization requested')
            return py_trees.common.Status.RUNNING
        if self._future.done():
            try:
                self._future.result()
                return py_trees.common.Status.SUCCESS
            except Exception as exc:
                self.node.get_logger().warn('Relocalization failed: %s' % exc)
                return py_trees.common.Status.FAILURE
        if (now - self._start).nanoseconds / 1e9 > self.timeout:
            return py_trees.common.Status.FAILURE
        return py_trees.common.Status.RUNNING


class FailureIsSuccess(py_trees.decorators.Decorator):
    """把可选恢复步骤的失败转换成 SUCCESS，即“跳过该步骤”。"""

    def __init__(self, name, child):
        super().__init__(name=name, child=child)

    def update(self):
        if self.decorated.status == py_trees.common.Status.FAILURE:
            return py_trees.common.Status.SUCCESS
        return self.decorated.status
