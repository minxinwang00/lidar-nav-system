#!/usr/bin/env python3
"""控制台“帮助”页内容：常见问题与常用终端命令。"""

HELP_HTML = """
<html><head><style>
body { color:#e6e6e6; font-size:13px; }
h2 { color:#7fd1ff; font-size:16px; margin-top:14px; }
h3 { color:#ffb020; font-size:14px; margin-top:12px; }
code { background:#2a2c30; color:#9fe3a0; padding:1px 4px; }
pre { background:#151619; color:#9fe3a0; padding:8px; border:1px solid #3a3d42; }
li { margin:4px 0; }
.note { color:#9aa0a6; }
</style></head><body>

<h2>LiDAR 导航系统 · 使用帮助</h2>
<p class="note">工作空间：<code>/home/mxw/nav_system_ws</code></p>

<h3>1. 编译与环境</h3>
<pre>cd /home/mxw/nav_system_ws
source /opt/ros/humble/setup.bash
colcon build --cmake-args -DCMAKE_BUILD_TYPE=Release
source install/setup.bash</pre>

<h3>2. 启动建图</h3>
<pre>ros2 launch bringup mapping.launch.py
# 或从本界面点击“开始建图”。建图结束后点“结束建图并保存”。</pre>

<h3>3. 启动导航（默认加载最新地图）</h3>
<pre># 实车：必须 use_chassis:=true，并保持遥控器开机 + NAV 模式
ros2 launch bringup navigation.launch.py \\
  use_gui:=true use_chassis:=true
# 指定地图：
ros2 launch bringup navigation.launch.py \\
  map_yaml:=$HOME/nav_system_ws/src/mapping/maps/occupancy_map.yaml \\
  use_gui:=true use_chassis:=true</pre>

<h3>4. 仿真（没有雷达/底盘时）</h3>
<pre>ros2 launch nav_sim sim_navigation.launch.py
ros2 param set /sim_lio obstacle_enable true</pre>

<h3>5. 保存地图 / 导入地图</h3>
<pre>ros2 service call /pointcloud_to_grid/save_map std_srvs/srv/Trigger {}
# 地图文件默认目录：
ls -lt ~/nav_system_ws/src/mapping/maps/*.yaml
# 导航中导入其它地图（map_server 运行时可调用）：
ros2 service call /map_server/load_map nav2_msgs/srv/LoadMap \\
  "{map_url: '/home/mxw/nav_system_ws/src/mapping/maps/occupancy_map.yaml'}"</pre>

<h3>6. 定位与目标点</h3>
<pre># 初始位姿（GUI“设为初始位姿”也会发这个）
ros2 topic pub --once /initialpose \\
  geometry_msgs/PoseWithCovarianceStamped \\
  "{header: {frame_id: map}, pose: {pose: {position: {x: 0.0, y: 0.0}, \\
  orientation: {w: 1.0}}}}"

# 发送目标
ros2 topic pub --once /goal_pose geometry_msgs/PoseStamped \\
  "{header: {frame_id: map}, pose: {position: {x: 1.5, y: 2.5}, \\
  orientation: {w: 1.0}}}"</pre>

<h3>7. 底盘与急停</h3>
<pre># 检查 20 字节二进制帧是否发出
ros2 topic echo /chassis_status
ros2 topic hz /cmd_vel

# 急停 / 解除
ros2 topic pub --once /estop std_msgs/Bool "{data: true}"
ros2 topic pub --once /estop std_msgs/Bool "{data: false}"</pre>

<h3>8. LI-Init 外参标定</h3>
<pre>ros2 launch li_init_tools calibrate.launch.py
# 机器人做旋转+平移激励后：
ros2 service call /li_init_manager/record_bag std_srvs/srv/Trigger {}
# 用 LI-Init 处理 bag，结果保存为：
# ~/nav_system_ws/calib_bags/li_init_result.txt
ros2 service call /li_init_manager/apply_result std_srvs/srv/Trigger {}</pre>

<h2>常见问题</h2>

<h3>Q1：界面显示“等待 /map”，导航不动</h3>
<ul>
<li>确认 <code>map_yaml</code> 路径存在，并且 <code>map_server</code> 已启动。</li>
<li>检查：<code>ros2 lifecycle get /map_server</code>，应为 active。</li>
<li>没有地图时先建图：<code>ros2 launch bringup mapping.launch.py</code>。</li>
</ul>

<h3>Q2：发送目标后没有 /cmd_vel</h3>
<ul>
<li>先设初始位姿，AMCL 才有 <code>map -&gt; odom</code>。</li>
<li><code>ros2 topic echo /nav_status</code> 查看是否卡在 MapReady/GoalAvailable。</li>
<li>检查规划器日志：<code>ros2 topic echo /plan --once</code>。</li>
</ul>

<h3>Q3：底盘不动</h3>
<ul>
<li><b>遥控器必须开机并切到 NAV/自动模式</b>，否则串口帧会被忽略。</li>
<li>确认启动参数 <code>use_chassis:=true</code>，串口存在：
<code>ls -l /dev/serial/by-id/</code>。</li>
<li>确认协议：<code>chassis_protocol:=binary</code>，
<code>chassis_angular_target:=yaw_speed</code>，且
<code>invert_angular: true</code>。</li>
<li>检查 <code>/estop</code> 是否为 true，以及 <code>/chassis_status</code> 的
<code>mode=serial</code>、<code>x/w</code>。</li>
<li>串口权限：<code>sudo usermod -aG dialout $USER</code>，重新登录。</li>
</ul>

<h3>Q4：地图不更新 / 动态障碍不避让</h3>
<ul>
<li>确认 <code>/cloud_registered</code>、<code>/Odometry</code> 有数据。</li>
<li><code>ros2 topic hz /local_costmap</code> 约 10 Hz。</li>
<li>建图中点“开始建图”后才会累积全局地图；导航阶段请确认局部代价地图正常。</li>
<li>动态障碍需要局部代价地图里有新点云；检查 <code>z_min/z_max</code> 滤波范围。</li>
</ul>

<h3>Q5：AMCL 定位漂移 / 重定位失败</h3>
<ul>
<li>先在地图上用“设为初始位姿”给出大致位置和朝向。</li>
<li>点“全局重定位”，并在环境中缓慢移动。</li>
<li>长廊等弱特征场景全局重定位容易跳变，可点“关闭全局重定位”禁用它；
  之后只用“设为初始位姿”做局部定位。</li>
<li>检查 <code>/scan</code> 是否有数据：<code>ros2 topic hz /scan</code>。</li>
<li>地图与实际环境差异过大时，AMCL 无法可靠定位，需要重新建图。</li>
</ul>

<h3>Q6：RViz 打不开 / 黑屏</h3>
<ul>
<li>确认 <code>DISPLAY</code> 已设置，且在桌面会话中运行。</li>
<li>手动启动：<code>rviz2 -d $(ros2 pkg prefix bringup)/share/bringup/rviz/nav.rviz</code>。</li>
<li>Fixed Frame 设为 <code>map</code>，并检查 TF 是否有 <code>map -&gt; odom -&gt; base_link</code>。</li>
</ul>

<h3>Q7：地图能显示但点选目标坐标不对</h3>
<ul>
<li>本界面按地图 yaml 的 <code>resolution/origin</code> 换算，不要在 RViz 里再次缩放地图。</li>
<li>确认 yaml 中 <code>image</code> 路径正确；不要手工修改 origin 后仍用旧图片。</li>
<li>点“适应窗口”后再点选，避免点到地图外的空白区域。</li>
</ul>

<h3>Q8：LI-Init 无法运行</h3>
<ul>
<li>本机没有网络/ROS1 时，先使用 FAST-LIO 在线外参估计：<code>extrinsic_est_en: true</code>。</li>
<li>联网后运行 <code>src/calibration/li_init_tools/scripts/install_li_init.sh</code>。</li>
<li>上游 LI-Init 是 ROS1 工程，需要 ROS1 Noetic 容器/工作区，或使用 ROS2 移植版本。</li>
</ul>

<h3>Q9：如何完全关闭系统</h3>
<ul>
<li>点击右上角“关闭系统”，会再次确认，并尝试保存地图后停止 launch。</li>
<li>也可以直接 <code>Ctrl+C</code> 结束 <code>ros2 launch</code> 终端。</li>
<li>残留进程检查：<code>ros2 node list</code>、<code>ros2 topic list</code>。</li>
</ul>

</body></html>
"""
