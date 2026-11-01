"""四层导航系统的统一启动文件。

第 1 层  livox_ros_driver2          -> /livox/lidar、/livox/imu
第 2 层  fast_lio + grid_mapping    -> /Odometry、/cloud_registered、/local_costmap
第 3 层  nav_localization + decision_core -> /plan、/cmd_vel、/nav_status
第 4 层  nav_gui（可选的 Qt 控制台）

用法示例
  # 只建图：启动 Qt 控制台，可用“开始建图 / 结束建图并自动保存”
  ros2 launch bringup mapping.launch.py

  # 导航：map_yaml 留空时自动加载 maps/ 下最新生成的地图
  ros2 launch bringup navigation.launch.py use_gui:=true

  # 指定地图
  ros2 launch bringup navigation.launch.py \
      map_yaml:=$HOME/nav_system_ws/src/mapping/maps/occupancy_map.yaml \
      use_gui:=true

Qt 控制台支持：地图缩放/点选目标、导入地图、一键启停建图、帮助、打开终端、
确认后关闭整个系统。
"""

import glob
import os

from launch import LaunchDescription
from launch.actions import (DeclareLaunchArgument, GroupAction, LogInfo,
                            SetEnvironmentVariable)
from launch.conditions import IfCondition
from launch.substitutions import (LaunchConfiguration, PathJoinSubstitution,
                                  PythonExpression)
from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterValue
from launch_ros.substitutions import FindPackageShare


MAP_DIR = os.path.expanduser('~/nav_system_ws/src/mapping/maps')


def _latest_map():
    """默认加载最新生成的栅格地图；没有地图时返回空串。"""
    maps = glob.glob(os.path.join(MAP_DIR, '*.yaml'))
    maps += glob.glob(os.path.join(MAP_DIR, '*.yml'))
    maps = [path for path in maps if os.path.isfile(path)]
    if not maps:
        return ''
    return max(maps, key=os.path.getmtime)


def generate_launch_description():
    use_sim_time = LaunchConfiguration('use_sim_time')
    use_driver = LaunchConfiguration('use_driver')
    use_lio = LaunchConfiguration('use_lio')
    use_grid = LaunchConfiguration('use_grid')
    use_localization = LaunchConfiguration('use_localization')
    use_chassis = LaunchConfiguration('use_chassis')
    use_gui = LaunchConfiguration('use_gui')
    use_rviz = LaunchConfiguration('use_rviz')
    use_amcl = LaunchConfiguration('use_amcl')
    auto_save = LaunchConfiguration('auto_save')
    map_yaml = LaunchConfiguration('map_yaml')
    lio_config = LaunchConfiguration('lio_config')
    chassis_port = LaunchConfiguration('chassis_port')
    chassis_protocol = LaunchConfiguration('chassis_protocol')
    chassis_angular_target = LaunchConfiguration('chassis_angular_target')
    chassis_invert_linear = LaunchConfiguration('chassis_invert_linear')
    base_to_body_yaw = LaunchConfiguration('base_to_body_yaw')

    use_map = PythonExpression(["'", map_yaml, "' != ''"])
    # AMCL 只有在有地图时才有意义；没有地图时，桥接节点继续发布
    # 静态的 map->odom 变换。
    use_amcl_map = PythonExpression(
        ["'", use_amcl, "' == 'true' and '", map_yaml, "' != ''"])
    publish_map_to_odom = PythonExpression(
        ["'", use_amcl, "' != 'true' or '", map_yaml, "' == ''"])

    sim_time_param = {'use_sim_time': ParameterValue(use_sim_time, value_type=bool)}

    driver_params = PathJoinSubstitution(
        [FindPackageShare('livox_ros_driver2'), 'config', 'MID360_config.json'])
    grid_params = PathJoinSubstitution(
        [FindPackageShare('grid_mapping'), 'config', 'grid_params.yaml'])
    chassis_params = PathJoinSubstitution(
        [FindPackageShare('chassis'), 'config', 'chassis.yaml'])
    nav_params = PathJoinSubstitution(
        [FindPackageShare('bringup'), 'config', 'nav_params.yaml'])
    amcl_params = PathJoinSubstitution(
        [FindPackageShare('bringup'), 'config', 'amcl_params.yaml'])
    rviz_cfg = PathJoinSubstitution(
        [FindPackageShare('bringup'), 'rviz', 'nav.rviz'])

    declared = [
        DeclareLaunchArgument('use_sim_time', default_value='false'),
        DeclareLaunchArgument('use_driver', default_value='true',
                              description='启动 Livox MID-360 驱动'),
        DeclareLaunchArgument('use_lio', default_value='true',
                              description='启动 FAST-LIO（里程计 + 点云地图）'),
        DeclareLaunchArgument('use_grid', default_value='true',
                              description='启动 3D->2D 栅格投影'),
        DeclareLaunchArgument('use_localization', default_value='true',
                              description='启动 map/odom 的 TF 桥接'),
        DeclareLaunchArgument('use_amcl', default_value='true',
                              description='启动 AMCL 重定位（需要 map_yaml）'),
        DeclareLaunchArgument('use_chassis', default_value='false',
                              description='启动底盘串口桥接'),
        DeclareLaunchArgument('use_gui', default_value='false',
                              description='启动 Qt5 控制台'),
        DeclareLaunchArgument('use_rviz', default_value=LaunchConfiguration('use_gui'),
                              description='启动 RViz（默认跟随 use_gui）'),
        DeclareLaunchArgument('auto_save', default_value='false',
                              description='周期性保存累积的栅格地图'),
        DeclareLaunchArgument(
            'map_yaml', default_value=_latest_map(),
            description='占据栅格地图 yaml；默认自动使用 maps/ 下最新生成的地图'),
        DeclareLaunchArgument(
            'lio_config',
            default_value=PathJoinSubstitution(
                [FindPackageShare('fast_lio'), 'config', 'mid360.yaml'])),
        DeclareLaunchArgument(
            'chassis_port', default_value='/dev/minipc',
            description='底盘串口；优先用 udev 固定别名 /dev/minipc，'
                        '不存在时 chassis_bridge 会自动按 by-id 查找'),
        DeclareLaunchArgument('chassis_protocol', default_value='binary',
                              description='binary（MiniPC 二进制帧）或 ascii'),
        DeclareLaunchArgument('chassis_angular_target', default_value='yaw_speed',
                              description='w_speed | yaw_speed | both；'
                                          '本车实测只有 yaw_speed 有效'),
        DeclareLaunchArgument(
            'chassis_invert_linear', default_value='true',
            description='本车实测 x_speed 正方向与车体前向相反，默认取反'),
        DeclareLaunchArgument(
            'base_to_body_yaw', default_value='3.141592653589793',
            description='雷达/IMU 相对车体前向的安装 yaw 偏移 [rad]；'
                        '当前装车实测为 180°，用于对齐车头与路径方向'),
    ]

    nodes = []

    nodes.append(Node(
        package='livox_ros_driver2',
        executable='livox_ros_driver2_node',
        name='livox_lidar_publisher',
        output='screen',
        condition=IfCondition(use_driver),
        parameters=[{
            'xfer_format': 1,          # 1 = livox 自定义消息（FAST-LIO 需要）
            'multi_topic': 0,
            'data_src': 0,
            'publish_freq': 10.0,
            'output_data_type': 0,
            'frame_id': 'livox_frame',
            'lvx_file_path': '',
            'user_config_path': driver_params,
            'cmdline_input_bd_code': 'livox0000000001',
        }]))

    nodes.append(Node(
        package='tf2_ros',
        executable='static_transform_publisher',
        name='body_to_livox_frame',
        arguments=['0', '0', '0', '0', '0', '0', 'body', 'livox_frame'],
        condition=IfCondition(use_driver)))

    nodes.append(GroupAction([
        # MVS 相机 SDK 的 /opt/MVS/lib/64 带有旧 libusb，会让 libpcl_io 找不到
        # libusb_set_option。预加载系统 libusb，不修改 LD_LIBRARY_PATH，
        # 否则会破坏工作空间自身的 ROS 库搜索路径。
        SetEnvironmentVariable(
            'LD_PRELOAD', '/lib/x86_64-linux-gnu/libusb-1.0.so.0'),
        Node(package='fast_lio',
             executable='fastlio_mapping',
             name='fastlio_mapping',
             output='screen',
             condition=IfCondition(use_lio),
             parameters=[lio_config, sim_time_param]),
    ]))

    nodes.append(Node(
        package='grid_mapping',
        executable='pointcloud_to_grid',
        name='pointcloud_to_grid',
        output='screen',
        condition=IfCondition(use_grid),
        parameters=[grid_params,
                    {'auto_save_enable': ParameterValue(auto_save, value_type=bool)},
                    sim_time_param]))

    nodes.append(Node(
        package='nav_localization',
        executable='lio_odom_bridge',
        name='lio_odom_bridge',
        output='screen',
        condition=IfCondition(use_localization),
        parameters=[{
            'map_frame': 'map',
            'odom_frame': 'odom',
            'camera_init_parent': 'odom',
            'publish_map_to_odom': ParameterValue(publish_map_to_odom,
                                                  value_type=bool),
            'base_frame': 'base_link',
            # MID-360 安装在车体中心前方 10 cm：base = IMU - 0.10 m(前向)
            'base_to_body_x': -0.10,
            'base_to_body_yaw': ParameterValue(base_to_body_yaw,
                                               value_type=float),
            'publish_camera_init_tf': True,
            'map_to_odom_x': 0.0,
            'map_to_odom_y': 0.0,
            'map_to_odom_yaw': 0.0,
        }, sim_time_param]))

    nodes.append(Node(
        package='nav2_map_server',
        executable='map_server',
        name='map_server',
        output='screen',
        condition=IfCondition(use_map),
        parameters=[{'yaml_filename': map_yaml}, sim_time_param]))

    nodes.append(Node(
        package='nav2_lifecycle_manager',
        executable='lifecycle_manager',
        name='lifecycle_manager_map',
        output='screen',
        condition=IfCondition(use_map),
        parameters=[{'autostart': True, 'node_names': ['map_server']},
                    sim_time_param]))

    # ---- AMCL 重定位：3D 点云 -> /scan -> nav2_amcl -> map->odom ----
    nodes.append(Node(
        package='nav_localization',
        executable='cloud_to_scan',
        name='cloud_to_scan',
        output='screen',
        condition=IfCondition(use_amcl_map),
        parameters=[{
            'cloud_topic': '/cloud_registered',
            'scan_topic': '/scan',
            'target_frame': 'base_link',
            'z_min': -0.20,
            'z_max': 0.80,
            'range_min': 0.20,
            'range_max': 12.0,
            'num_bins': 720,
            'max_points': 6000,
        }, sim_time_param]))

    nodes.append(Node(
        package='nav2_amcl',
        executable='amcl',
        name='amcl',
        output='screen',
        condition=IfCondition(use_amcl_map),
        parameters=[amcl_params, sim_time_param]))

    nodes.append(Node(
        package='nav2_lifecycle_manager',
        executable='lifecycle_manager',
        name='lifecycle_manager_amcl',
        output='screen',
        condition=IfCondition(use_amcl_map),
        parameters=[{'autostart': True, 'node_names': ['amcl']},
                    sim_time_param]))

    nodes.append(Node(
        package='decision_core',
        executable='bt_main',
        name='bt_main_node',
        output='screen',
        parameters=[nav_params, sim_time_param]))

    nodes.append(Node(
        package='chassis',
        executable='chassis_bridge',
        name='chassis_bridge',
        output='screen',
        condition=IfCondition(use_chassis),
        parameters=[chassis_params,
                    {'port': chassis_port,
                     'protocol': chassis_protocol,
                     'angular_z_target': chassis_angular_target,
                     'invert_linear': ParameterValue(
                         chassis_invert_linear, value_type=bool)},
                    sim_time_param]))

    nodes.append(Node(
        package='nav_gui',
        executable='nav_gui',
        name='nav_gui',
        output='screen',
        condition=IfCondition(use_gui),
        parameters=[{'map_dir': MAP_DIR,
                     'initial_map': map_yaml}]))

    nodes.append(Node(
        package='rviz2',
        executable='rviz2',
        name='rviz2',
        arguments=['-d', rviz_cfg],
        condition=IfCondition(use_rviz)))

    hints = [
        LogInfo(msg='[bringup] driver: /livox/lidar + /livox/imu -> fast_lio -> '
                    '/Odometry + /cloud_registered -> grid_mapping -> /local_costmap'),
        LogInfo(msg='[bringup] AMCL: /cloud_registered -> /scan -> amcl -> '
                    'map->odom; set the initial pose from the GUI or RViz '
                    '"2D Pose Estimate".'),
        LogInfo(msg='[bringup] map_yaml empty -> no /map and no AMCL; the decision '
                    'layer will wait. Set map_yaml:=/path/to/occupancy_map.yaml'),
        LogInfo(msg='[bringup] publish a goal with: ros2 topic pub --once '
                    '/goal_pose geometry_msgs/PoseStamped '
                    '"{header: {frame_id: map}, pose: {position: {x: 1.0, y: 0.0}, '
                    'orientation: {w: 1.0}}}"'),
        LogInfo(msg='[bringup] CHASSIS: keep the RC powered on and switched to '
                    'NAV/auto mode, otherwise serial frames are ignored.'),
    ]

    return LaunchDescription(declared + hints + nodes)
