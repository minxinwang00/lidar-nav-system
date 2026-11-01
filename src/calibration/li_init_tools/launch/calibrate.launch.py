"""LI-Init calibration bringup.

Starts the LiDAR driver + calibration manager.  The manager records a rosbag2
of /livox/lidar and /livox/imu; LI-Init itself is run separately (ROS1 original
or a ROS2 port), then the result is applied to the FAST-LIO YAML.

Workflow:
  1. ros2 launch li_init_tools calibrate.launch.py
  2. drive/rotate the robot for the whole recording
  3. ros2 service call /li_init_manager/record_bag std_srvs/srv/Trigger {}
  4. run LI-Init on ~/nav_system_ws/calib_bags/li_init_xxx
  5. save its output as ~/nav_system_ws/calib_bags/li_init_result.txt
  6. ros2 service call /li_init_manager/apply_result std_srvs/srv/Trigger {}
  7. ros2 launch bringup mapping.launch.py \\
       lio_config:=$HOME/nav_system_ws/src/mapping/FAST_LIO/config/mid360_calibrated.yaml
"""

import os

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, LogInfo
from launch.conditions import IfCondition
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterValue
from launch_ros.substitutions import FindPackageShare


def generate_launch_description():
    use_sim_time = LaunchConfiguration('use_sim_time')
    use_driver = LaunchConfiguration('use_driver')
    use_chassis = LaunchConfiguration('use_chassis')
    use_rviz = LaunchConfiguration('use_rviz')
    bag_duration = LaunchConfiguration('bag_duration')

    sim_time = {'use_sim_time': ParameterValue(use_sim_time, value_type=bool)}
    driver_params = PathJoinSubstitution(
        [FindPackageShare('livox_ros_driver2'), 'config', 'MID360_config.json'])
    manager_params = PathJoinSubstitution(
        [FindPackageShare('li_init_tools'), 'config', 'li_init_params.yaml'])
    chassis_params = PathJoinSubstitution(
        [FindPackageShare('chassis'), 'config', 'chassis.yaml'])
    rviz_cfg = PathJoinSubstitution(
        [FindPackageShare('fast_lio'), 'rviz', 'fastlio.rviz'])

    declared = [
        DeclareLaunchArgument('use_sim_time', default_value='false'),
        DeclareLaunchArgument('use_driver', default_value='true'),
        DeclareLaunchArgument('use_chassis', default_value='false',
                              description='Start chassis bridge for manual motion'),
        DeclareLaunchArgument('use_rviz', default_value='false'),
        DeclareLaunchArgument('bag_duration', default_value='90.0'),
        DeclareLaunchArgument(
            'lio_input',
            default_value=os.path.expanduser(
                '~/nav_system_ws/src/mapping/FAST_LIO/config/mid360.yaml')),
        DeclareLaunchArgument(
            'lio_output',
            default_value=os.path.expanduser(
                '~/nav_system_ws/src/mapping/FAST_LIO/config/mid360_calibrated.yaml')),
        DeclareLaunchArgument(
            'result_file',
            default_value=os.path.expanduser(
                '~/nav_system_ws/calib_bags/li_init_result.txt')),
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
        Node(package='li_init_tools',
             executable='li_init_manager',
             name='li_init_manager',
             output='screen',
             parameters=[manager_params,
                         {'bag_duration': ParameterValue(bag_duration,
                                                         value_type=float),
                          'fastlio_input': LaunchConfiguration('lio_input'),
                          'fastlio_output': LaunchConfiguration('lio_output'),
                          'result_file': LaunchConfiguration('result_file')},
                         sim_time]),
        Node(package='chassis',
             executable='chassis_bridge',
             name='chassis_bridge',
             output='screen',
             condition=IfCondition(use_chassis),
             parameters=[chassis_params, sim_time]),
        Node(package='rviz2',
             executable='rviz2',
             name='rviz2',
             arguments=['-d', rviz_cfg],
             condition=IfCondition(use_rviz)),
    ]

    hints = [
        LogInfo(msg='[li_init] drive the robot with rotation + translation '
                    'excitation; keep the LiDAR data and IMU data streaming.'),
        LogInfo(msg='[li_init] start recording: ros2 service call '
                    '/li_init_manager/record_bag std_srvs/srv/Trigger {}'),
        LogInfo(msg='[li_init] apply result: ros2 service call '
                    '/li_init_manager/apply_result std_srvs/srv/Trigger {}'),
        LogInfo(msg='[li_init] then launch FAST-LIO with '
                    'lio_config:=$HOME/nav_system_ws/src/mapping/FAST_LIO/'
                    'config/mid360_calibrated.yaml'),
    ]

    return LaunchDescription(declared + hints + nodes)
