#!/usr/bin/env python3
"""ROS 节点与行为树（behaviour tree）叶子节点之间共享的状态容器。"""

import threading


class NavContext:
    def __init__(self):
        self.lock = threading.RLock()

        self.map_msg = None
        self.map_version = 0
        self.local_grid = None

        self.goal = None            # 目标点 (x, y, 航向角 yaw)
        self.path = None            # 全局路径，类型为 nav_msgs/Path
        self.path_stamp = None

        self.robot_pose = None      # 机器人位姿 (x, y, 航向角 yaw)
        self.estop = False
        self.blocked = False
        self.state = 'IDLE'
        self.last_error = ''

    # ------------------------------------------------------------------ 辅助方法
    def set_goal(self, xy_yaw):
        with self.lock:
            self.goal = xy_yaw
            self.path = None
            self.path_stamp = None
            self.blocked = False
            self.state = 'PLANNING'

    def clear_goal(self):
        with self.lock:
            self.goal = None
            self.path = None
            self.path_stamp = None
            self.blocked = False
            if not self.estop:
                self.state = 'IDLE'

    def snapshot(self):
        with self.lock:
            return {
                'goal': self.goal,
                'path': self.path,
                'map_ready': self.map_msg is not None,
                'local_ready': self.local_grid is not None,
                'estop': self.estop,
                'blocked': self.blocked,
                'state': self.state,
                'pose': self.robot_pose,
                'error': self.last_error,
            }
