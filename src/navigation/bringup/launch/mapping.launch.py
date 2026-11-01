"""建图启动文件：驱动 + FAST-LIO + 3D->2D 投影。

建图完成后，可以在地图保存的两种方式中选择其一：Qt 控制台上的“保存地图”
按钮，或者以下命令：

    ros2 service call /pointcloud_to_grid/save_map std_srvs/srv/Trigger {}

生成的文件位于 src/mapping/maps/occupancy_map.{pgm,yaml}，之后可通过
navigation.launch.py 的 map_yaml 参数加载使用。
"""

from launch import LaunchDescription
from launch.actions import (DeclareLaunchArgument, GroupAction, LogInfo,
                            SetEnvironmentVariable)
from launch.conditions import IfCondition
from launch.substitutions import (LaunchConfiguration, PathJoinSubstitution)
from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterValue
from launch_ros.substitutions import FindPackageShare


def generate_launch_description():
    use_sim_time = LaunchConfiguration('use_sim_time')
    use_driver = LaunchConfiguration('use_driver')
    use_rviz = LaunchConfiguration('use_rviz')
    use_gui = LaunchConfiguration('use_gui')
    lio_config = LaunchConfiguration('lio_config')
    auto_save = LaunchConfiguration('auto_save')
    base_to_body_yaw = LaunchConfiguration('base_to_body_yaw')

    sim_time_param = {'use_sim_time': ParameterValue(use_sim_time, value_type=bool)}
    driver_params = PathJoinSubstitution(
        [FindPackageShare('livox_ros_driver2'), 'config', 'MID360_config.json'])
    grid_params = PathJoinSubstitution(
        [FindPackageShare('grid_mapping'), 'config', 'grid_params.yaml'])
    rviz_cfg = PathJoinSubstitution(
        [FindPackageShare('fast_lio'), 'rviz', 'fastlio.rviz'])

    declared = [
        DeclareLaunchArgument('use_sim_time', default_value='false'),
        DeclareLaunchArgument('use_driver', default_value='true'),
        DeclareLaunchArgument('use_rviz', default_value='true'),
        DeclareLaunchArgument('use_gui', default_value='true',
                              description='启动 Qt 控制台（一键开始/结束建图）'),
        DeclareLaunchArgument('auto_save', default_value='true',
                              description='建图过程中自动保存 pgm/yaml'),
        DeclareLaunchArgument(
            'lio_config',
            default_value=PathJoinSubstitution(
                [FindPackageShare('fast_lio'), 'config', 'mid360.yaml'])),
        DeclareLaunchArgument(
            'base_to_body_yaw', default_value='3.141592653589793',
            description='雷达/IMU 相对车体前向的安装 yaw 偏移 [rad]；'
                        '当前装车实测为 180°'),
    ]

    nodes = [
        Node(package='livox_ros_driver2',
             executable='livox_ros_driver2_node',
             name='livox_lidar_publisher',
             output='screen',
             condition=IfCondition(use_driver),
             parameters=[{
                 'xfer_format': 1,
                 'multi_topic': 0,
                 'data_src': 0,
                 'publish_freq': 10.0,
                 'output_data_type': 0,
                 'frame_id': 'livox_frame',
                 'lvx_file_path': '',
                 'user_config_path': driver_params,
                 'cmdline_input_bd_code': 'livox0000000001',
             }]),
        Node(package='tf2_ros',
             executable='static_transform_publisher',
             name='body_to_livox_frame',
             arguments=['0', '0', '0', '0', '0', '0', 'body', 'livox_frame'],
             condition=IfCondition(use_driver)),
        GroupAction([
            # MVS 相机 SDK 的 /opt/MVS/lib/64 带有旧 libusb，会让 libpcl_io
            # 找不到 libusb_set_option。预加载系统 libusb，不修改
            # LD_LIBRARY_PATH，否则会破坏工作空间自身的 ROS 库搜索路径。
            SetEnvironmentVariable(
                'LD_PRELOAD', '/lib/x86_64-linux-gnu/libusb-1.0.so.0'),
            Node(package='fast_lio',
                 executable='fastlio_mapping',
                 name='fastlio_mapping',
                 output='screen',
                 parameters=[lio_config, sim_time_param]),
        ]),
        Node(package='nav_localization',
             executable='lio_odom_bridge',
             name='lio_odom_bridge',
             output='screen',
             parameters=[{'map_frame': 'map',
                          'odom_frame': 'odom',
                          'base_frame': 'base_link',
                          # MID-360 安装在车体中心前方 10 cm
                          'base_to_body_x': -0.10,
                          'base_to_body_yaw': ParameterValue(
                              base_to_body_yaw, value_type=float),
                          'publish_camera_init_tf': True},
                         sim_time_param]),
        Node(package='grid_mapping',
             executable='pointcloud_to_grid',
             name='pointcloud_to_grid',
             output='screen',
             parameters=[grid_params,
                         {'auto_save_enable': ParameterValue(auto_save,
                                                             value_type=bool)},
                         sim_time_param]),
        Node(package='nav_gui',
             executable='nav_gui',
             name='nav_gui',
             output='screen',
             condition=IfCondition(use_gui)),
        Node(package='rviz2',
             executable='rviz2',
             name='rviz2',
             arguments=['-d', rviz_cfg],
             condition=IfCondition(use_rviz)),
    ]

    hints = [
        LogInfo(msg='[mapping] move the robot slowly; watch /cloud_registered in RViz'),
        LogInfo(msg='[mapping] save with: ros2 service call '
                    '/pointcloud_to_grid/save_map std_srvs/srv/Trigger {}'),
    ]
    return LaunchDescription(declared + hints + nodes)
