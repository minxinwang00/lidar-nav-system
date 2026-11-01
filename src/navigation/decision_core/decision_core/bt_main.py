#!/usr/bin/env python3
"""导航决策层（第 3 层）。

职责
  * 持有行为树（behaviour tree），决定何时执行路径规划 / 路径跟踪 / 恢复行为
  * 运行全局规划器（A*）与局部 DWA 控制器
  * 为 Qt 图形界面（第 4 层）提供简洁的命令 / 状态接口

订阅：/map、/local_costmap、/goal_pose、/nav_command、/estop
发布：/cmd_vel、/plan、/nav_status
"""

import json
import math

import py_trees
import rclpy
from rclpy.duration import Duration
from rclpy.node import Node
from rclpy.qos import DurabilityPolicy, HistoryPolicy, QoSProfile, ReliabilityPolicy
from rclpy.time import Time

import tf2_ros
from geometry_msgs.msg import PoseStamped, Twist
from nav_msgs.msg import OccupancyGrid, Path
from std_msgs.msg import Bool, String

from decision_core.bt_nodes import (BackUp, ClearPath, FailureIsSuccess,
                                    FollowPath, GlobalRelocalize, GoalAvailable,
                                    MapReady, PlanPath, RecoverySpin,
                                    WaitForClearance)
from decision_core.controller_plugin import DwaController
from decision_core.nav_context import NavContext
from decision_core.planner_plugin import GridPlanner


def quat_to_matrix(q):
    x, y, z, w = q.x, q.y, q.z, q.w
    return [[1 - 2 * (y * y + z * z), 2 * (x * y - z * w), 2 * (x * z + y * w)],
            [2 * (x * y + z * w), 1 - 2 * (x * x + z * z), 2 * (y * z - x * w)],
            [2 * (x * z - y * w), 2 * (y * z + x * w), 1 - 2 * (x * x + y * y)]]


def yaw_from_quat(q):
    return math.atan2(2.0 * (q.w * q.z + q.x * q.y),
                      1.0 - 2.0 * (q.y * q.y + q.z * q.z))


class BTMainNode(Node):
    def __init__(self):
        super().__init__('bt_main_node')

        # ------------------------------------------------------------ 参数声明
        self.declare_parameter('map_topic', '/map')
        self.declare_parameter('local_costmap_topic', '/local_costmap')
        self.declare_parameter('goal_topic', '/goal_pose')
        self.declare_parameter('cmd_vel_topic', '/cmd_vel')
        self.declare_parameter('path_topic', '/plan')
        self.declare_parameter('status_topic', '/nav_status')
        self.declare_parameter('command_topic', '/nav_command')
        self.declare_parameter('map_frame', 'map')
        self.declare_parameter('odom_frame', 'odom')
        self.declare_parameter('base_frame', 'base_link')
        self.declare_parameter('control_rate', 10.0)
        self.declare_parameter('status_rate', 5.0)

        self.declare_parameter('robot_radius', 0.25)
        self.declare_parameter('inflation_radius', 0.45)
        self.declare_parameter('plan_resolution', 0.10)
        self.declare_parameter('allow_unknown', False)
        self.declare_parameter('clearance_weight', 1.5)
        self.declare_parameter('smoothing_clearance', 0.35)
        self.declare_parameter('max_expansions', 300000)
        self.declare_parameter('plan_time_budget', 2.0)
        self.declare_parameter('local_obstacle_cost', 30.0)

        self.declare_parameter('max_linear', 0.35)
        self.declare_parameter('max_angular', 1.0)
        self.declare_parameter('lookahead_dist', 0.6)
        self.declare_parameter('goal_tolerance', 0.12)
        self.declare_parameter('goal_yaw_tolerance', 0.35)
        self.declare_parameter('goal_hold_time', 1.0)
        self.declare_parameter('goal_slow_radius', 0.6)
        self.declare_parameter('rotate_in_place_error', 0.8)
        self.declare_parameter('rotate_in_place_gain', 1.0)
        self.declare_parameter('path_heading_weight', 0.15)
        self.declare_parameter('curvature_weight', 0.05)
        self.declare_parameter('sim_time', 1.2)
        self.declare_parameter('max_dv', 0.15)
        self.declare_parameter('max_dw', 0.6)
        self.declare_parameter('dwa_heading_weight', 0.50)
        self.declare_parameter('dwa_clearance_weight', 0.40)
        self.declare_parameter('dwa_path_weight', 0.25)
        self.declare_parameter('dwa_speed_weight', 0.10)
        self.declare_parameter('collision_cost', 30.0)
        self.declare_parameter('unknown_cost', 10.0)
        self.declare_parameter('local_plan_distance', 2.5)
        self.declare_parameter('local_replan_period', 0.4)
        self.declare_parameter('local_blocked_cost', 30.0)
        self.declare_parameter('blocked_timeout', 0.6)
        self.declare_parameter('max_navigation_attempts', 3)
        self.declare_parameter('recovery_wait', 1.2)
        self.declare_parameter('recovery_backup_distance', 0.35)
        self.declare_parameter('recovery_backup_speed', 0.12)
        self.declare_parameter('recovery_spin_duration', 1.8)
        self.declare_parameter('recovery_spin_speed', 0.6)
        self.declare_parameter('recovery_relocalize', True)
        self.declare_parameter('relocalize_service',
                               '/reinitialize_global_localization')

        p = lambda n: self.get_parameter(n).value
        self.map_frame = str(p('map_frame'))
        self.odom_frame = str(p('odom_frame'))
        self.base_frame = str(p('base_frame'))
        # 可由 GUI 的“关闭全局重定位”按钮在运行时切换
        self.relocalize_enabled = bool(p('recovery_relocalize'))

        self.ctx = NavContext()
        self.planner = GridPlanner(
            robot_radius=p('robot_radius'),
            inflation_radius=p('inflation_radius'),
            plan_resolution=p('plan_resolution'),
            allow_unknown=p('allow_unknown'),
            clearance_weight=p('clearance_weight'),
            smoothing_clearance=p('smoothing_clearance'),
            max_expansions=p('max_expansions'),
            time_budget=p('plan_time_budget'),
            local_obstacle_cost=p('local_obstacle_cost'))
        self.controller = DwaController(
            max_linear=p('max_linear'),
            max_angular=p('max_angular'),
            lookahead=p('lookahead_dist'),
            goal_tolerance=p('goal_tolerance'),
            goal_yaw_tolerance=p('goal_yaw_tolerance'),
            goal_hold_time=p('goal_hold_time'),
            goal_slow_radius=p('goal_slow_radius'),
            rotate_in_place_error=p('rotate_in_place_error'),
            rotate_in_place_gain=p('rotate_in_place_gain'),
            path_heading_weight=p('path_heading_weight'),
            curvature_weight=p('curvature_weight'),
            sim_time=p('sim_time'),
            max_dv=p('max_dv'),
            max_dw=p('max_dw'),
            heading_weight=p('dwa_heading_weight'),
            clearance_weight=p('dwa_clearance_weight'),
            path_weight=p('dwa_path_weight'),
            speed_weight=p('dwa_speed_weight'),
            collision_cost=p('collision_cost'),
            unknown_cost=p('unknown_cost'),
            local_plan_distance=p('local_plan_distance'),
            local_replan_period=p('local_replan_period'),
            local_blocked_cost=p('local_blocked_cost'))

        # ------------------------------------------------------------ 话题接口
        self.cmd_pub = self.create_publisher(Twist, str(p('cmd_vel_topic')), 10)
        self.path_pub = self.create_publisher(Path, str(p('path_topic')), 5)
        self.status_pub = self.create_publisher(String, str(p('status_topic')), 10)

        map_qos = QoSProfile(depth=1,
                             reliability=ReliabilityPolicy.RELIABLE,
                             durability=DurabilityPolicy.TRANSIENT_LOCAL,
                             history=HistoryPolicy.KEEP_LAST)
        self.create_subscription(OccupancyGrid, str(p('map_topic')),
                                 self._map_cb, map_qos)
        self.create_subscription(OccupancyGrid, str(p('local_costmap_topic')),
                                 self._local_cb, 5)
        self.create_subscription(PoseStamped, str(p('goal_topic')),
                                 self._goal_cb, 10)
        self.create_subscription(String, str(p('command_topic')),
                                 self._command_cb, 10)
        self.create_subscription(Bool, '/estop', self._estop_cb, 10)

        self.tf_buffer = tf2_ros.Buffer()
        self.tf_listener = tf2_ros.TransformListener(self.tf_buffer, self)

        # ------------------------------------------------------------ 行为树搭建
        map_ready = MapReady('MapReady', self)
        goal_ready = GoalAvailable('GoalAvailable', self)
        plan = PlanPath('PlanPath', self, self.planner)
        follow = FollowPath('FollowPath', self, self.controller,
                            blocked_timeout=p('blocked_timeout'))
        navigate = py_trees.composites.Sequence('Navigate', memory=True)
        navigate.add_children([plan, follow])

        recovery_children = [
            ClearPath('ClearPath', self),
            WaitForClearance('WaitForClearance', self,
                             duration=p('recovery_wait')),
            FailureIsSuccess('SafeBackUp', BackUp(
                'BackUp', self,
                distance=p('recovery_backup_distance'),
                speed=p('recovery_backup_speed'))),
            FailureIsSuccess('SafeSpin', RecoverySpin(
                'RecoverySpin', self,
                duration=p('recovery_spin_duration'),
                angular_speed=p('recovery_spin_speed'))),
            # 始终挂上该节点；是否真正调用服务由 relocalize_enabled 运行时决定，
            # 这样 GUI 的“关闭/启用全局重定位”可以立即生效。
            FailureIsSuccess('SafeRelocalize',
                             GlobalRelocalize(
                                 'GlobalRelocalize', self,
                                 service=str(p('relocalize_service')))),
        ]
        recovery_actions = py_trees.composites.Sequence('RecoveryActions', memory=True)
        recovery_actions.add_children(recovery_children)
        # py_trees composites implement tick() directly and do not call
        # update(); use the standard decorator so a completed recovery chain
        # becomes FAILURE and Retry runs Navigate again.
        recovery = py_trees.decorators.SuccessIsFailure('Recovery', recovery_actions)

        fallback = py_trees.composites.Selector('NavigateOrRecover', memory=True)
        fallback.add_children([navigate, recovery])
        retry = py_trees.decorators.Retry(
            'RetryNavigation', fallback,
            num_failures=max(1, int(p('max_navigation_attempts'))))

        root = py_trees.composites.Sequence('Root', memory=True)
        root.add_children([map_ready, goal_ready, retry])
        root.setup_with_descendants()
        self.root = root

        self._reset_requested = False
        self._last_tick = self.get_clock().now()

        rate = max(1.0, float(p('control_rate')))
        self.create_timer(1.0 / rate, self._tick_tree)
        srate = max(1.0, float(p('status_rate')))
        self.create_timer(1.0 / srate, self._publish_status)
        self.create_timer(0.05, self._estop_timer)

        self.get_logger().info('Navigation decision layer ready (bt_main)')

    # ------------------------------------------------------------------ 各种回调
    def _map_cb(self, msg):
        with self.ctx.lock:
            self.ctx.map_msg = msg
            self.ctx.map_version += 1

    def _local_cb(self, msg):
        with self.ctx.lock:
            self.ctx.local_grid = msg

    def _goal_cb(self, msg):
        goal = self._goal_to_map(msg)
        if goal is None:
            return
        with self.ctx.lock:
            self.ctx.estop = False
            self.ctx.set_goal(goal)
            self.ctx.last_error = ''
        self._reset_requested = True
        self.get_logger().info('New goal: x=%.2f y=%.2f yaw=%.2f'
                               % (goal[0], goal[1], goal[2]))

    def _command_cb(self, msg):
        cmd = msg.data.strip().lower()
        if cmd in ('cancel', 'stop', 'abort'):
            with self.ctx.lock:
                self.ctx.clear_goal()
                self.ctx.state = 'IDLE'
            self._publish_stop()
            self._reset_requested = True
            self.get_logger().info('Goal cancelled by command')
        elif cmd in ('estop', 'e_stop', 'emergency'):
            with self.ctx.lock:
                self.ctx.estop = True
                self.ctx.clear_goal()
                self.ctx.state = 'ESTOP'
            self._publish_stop()
            self._reset_requested = True
            self.get_logger().warn('E-STOP engaged')
        elif cmd in ('resume', 'estop_clear', 'clear_estop'):
            with self.ctx.lock:
                self.ctx.estop = False
                self.ctx.state = 'IDLE'
            self._reset_requested = True
            self.get_logger().info('E-STOP cleared')
        elif cmd == 'replan':
            with self.ctx.lock:
                self.ctx.path = None
            self._reset_requested = True
            self.get_logger().info('Replan requested')
        elif cmd in ('relocalize_off', 'disable_relocalize'):
            self.relocalize_enabled = False
            self.get_logger().warn('Global relocalization disabled by GUI')
        elif cmd in ('relocalize_on', 'enable_relocalize'):
            self.relocalize_enabled = True
            self.get_logger().info('Global relocalization enabled by GUI')
        else:
            self.get_logger().warn('Unknown nav command: %s' % cmd)

    def _estop_cb(self, msg):
        with self.ctx.lock:
            self.ctx.estop = bool(msg.data)
            if self.ctx.estop:
                self.ctx.clear_goal()
                self.ctx.state = 'ESTOP'
        self._reset_requested = True

    # ------------------------------------------------------------------ 辅助方法
    def _goal_to_map(self, msg):
        frame = msg.header.frame_id or self.map_frame
        p = msg.pose.position
        yaw = yaw_from_quat(msg.pose.orientation)
        if frame == self.map_frame:
            return (p.x, p.y, yaw)
        try:
            tr = self.tf_buffer.lookup_transform(
                self.map_frame, frame, Time(), Duration(seconds=0.2))
        except Exception as exc:
            self.get_logger().warn('Cannot transform goal from %s: %s' % (frame, exc))
            return None
        R = quat_to_matrix(tr.transform.rotation)
        t = tr.transform.translation
        x = R[0][0] * p.x + R[0][1] * p.y + R[0][2] * p.z + t.x
        y = R[1][0] * p.x + R[1][1] * p.y + R[1][2] * p.z + t.y
        return (x, y, yaw + yaw_from_quat(tr.transform.rotation))

    def get_robot_pose(self):
        for target in (self.map_frame, self.odom_frame):
            try:
                tr = self.tf_buffer.lookup_transform(
                    target, self.base_frame, Time(), Duration(seconds=0.05))
            except Exception:
                continue
            t = tr.transform.translation
            return (t.x, t.y, yaw_from_quat(tr.transform.rotation))
        return None

    def _publish_stop(self):
        self.cmd_pub.publish(Twist())

    # ------------------------------------------------------------------ 主循环
    def _tick_tree(self):
        now = self.get_clock().now()
        dt = (now - self._last_tick).nanoseconds / 1e9
        self._last_tick = now
        with self.ctx.lock:
            self.ctx.robot_pose = self.get_robot_pose()
            estop = self.ctx.estop

        if estop:
            self._publish_stop()
            return

        if self._reset_requested:
            self.root.stop(py_trees.common.Status.INVALID)
            self._reset_requested = False
            return

        try:
            self.root.tick_once()
            status = self.root.status
        except Exception as exc:
            self.get_logger().error('Behaviour tree exception: %s' % exc)
            self._publish_stop()
            return

        if status == py_trees.common.Status.SUCCESS:
            with self.ctx.lock:
                self.ctx.state = 'SUCCEEDED'
                self.ctx.goal = None
                self.ctx.path = None
                self.ctx.blocked = False
            self._publish_stop()
            self.get_logger().info('Navigation finished successfully')
            self._reset_requested = True
        elif status == py_trees.common.Status.FAILURE:
            with self.ctx.lock:
                self.ctx.state = 'FAILED'
                self.ctx.goal = None
                self.ctx.path = None
                self.ctx.blocked = False
                self.ctx.last_error = 'navigation failed'
            self._publish_stop()
            self.get_logger().error('Navigation failed, goal dropped')
            self._reset_requested = True

    def _estop_timer(self):
        with self.ctx.lock:
            estop = self.ctx.estop
        if estop:
            self._publish_stop()

    def _publish_status(self):
        snap = self.ctx.snapshot()
        dist = None
        if snap['goal'] is not None and snap['pose'] is not None:
            dist = math.hypot(snap['goal'][0] - snap['pose'][0],
                              snap['goal'][1] - snap['pose'][1])
        data = {
            'state': snap['state'],
            'map_ready': snap['map_ready'],
            'local_costmap_ready': snap['local_ready'],
            'estop': snap['estop'],
            'blocked': snap['blocked'],
            'goal': list(snap['goal']) if snap['goal'] else None,
            'pose': list(snap['pose']) if snap['pose'] else None,
            'path_points': len(snap['path'].poses) if snap['path'] else 0,
            'local_path_points': int(getattr(self.controller, 'debug_local_points', 0)),
            'active_local_path': bool(getattr(self.controller,
                                              'debug_active_local', False)),
            'local_blocked': bool(getattr(self.controller, 'debug_blocked', False)),
            'local_candidates': int(getattr(self.controller,
                                            'debug_candidates', 0)),
            'local_attempts': int(getattr(self.controller,
                                          'debug_plan_attempts', 0)),
            'local_plan_ok': bool(getattr(self.controller,
                                          'debug_plan_ok', False)),
            'distance_to_goal': dist,
            'relocalize_enabled': bool(self.relocalize_enabled),
            'error': snap['error'],
        }
        msg = String()
        msg.data = json.dumps(data)
        self.status_pub.publish(msg)

    def destroy_node(self):
        try:
            self._publish_stop()
        except Exception:
            pass
        super().destroy_node()


def main(args=None):
    rclpy.init(args=args)
    node = BTMainNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
