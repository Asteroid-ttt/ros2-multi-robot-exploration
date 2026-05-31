"""
car_bringup_launch.py — 启动单个小车的 car_interface 节点

用于将真实串口小车接入 ROS2 多机器人系统。
此 launch 文件可被主 launch 文件包含（IncludeLaunchDescription），
每次包含时传入不同的 namespace 和 port 参数即可启动多台小车。

使用示例:

  # 单台小车:
  ros2 launch car_interface car_bringup_launch.py port:=/dev/ttyUSB0 robot_name:=tb1

  # 在父级 launch 中包含多台小车:
  IncludeLaunchDescription(
      PythonLaunchDescriptionSource(
          os.path.join(pkg_share, 'launch', 'car_bringup_launch.py')
      ),
      launch_arguments={
          'port': '/dev/ttyUSB0',
          'robot_name': 'tb1',
      }.items(),
  )
"""

import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description():
    pkg_share = get_package_share_directory('car_interface')

    # ---- 声明启动参数 ----
    port = LaunchConfiguration('port')
    declare_port = DeclareLaunchArgument(
        'port',
        default_value='/dev/ttyUSB0',
        description='串口设备路径，例如 /dev/ttyUSB0',
    )

    robot_name = LaunchConfiguration('robot_name')
    declare_robot_name = DeclareLaunchArgument(
        'robot_name',
        default_value='tb1',
        description='机器人命名空间，用于话题隔离，例如 tb1 → /tb1/odom',
    )

    baudrate = LaunchConfiguration('baudrate')
    declare_baudrate = DeclareLaunchArgument(
        'baudrate',
        default_value='115200',
        description='串口波特率',
    )

    status_rate = LaunchConfiguration('status_rate')
    declare_status_rate = DeclareLaunchArgument(
        'status_rate',
        default_value='20.0',
        description='状态查询频率 (Hz)',
    )

    cmd_vel_timeout = LaunchConfiguration('cmd_vel_timeout')
    declare_cmd_vel_timeout = DeclareLaunchArgument(
        'cmd_vel_timeout',
        default_value='0.5',
        description='cmd_vel 超时自动停车时间 (秒)',
    )

    max_linear_mms = LaunchConfiguration('max_linear_mms')
    declare_max_linear_mms = DeclareLaunchArgument(
        'max_linear_mms',
        default_value='500.0',
        description='最大线速度 (mm/s)',
    )

    max_angular_degs = LaunchConfiguration('max_angular_degs')
    declare_max_angular_degs = DeclareLaunchArgument(
        'max_angular_degs',
        default_value='180.0',
        description='最大角速度 (deg/s)',
    )

    use_sim_time = LaunchConfiguration('use_sim_time')
    declare_use_sim_time = DeclareLaunchArgument(
        'use_sim_time',
        default_value='false',
        description='是否使用仿真时间（真实小车应为 false）',
    )

    # ---- 节点定义 ----
    car_interface_node = Node(
        package='car_interface',
        executable='car_interface_node',
        namespace=[robot_name],       # 命名空间隔离，如 /tb1
        name='car_interface_node',
        output='screen',
        parameters=[{
            'port':             port,
            'baudrate':         baudrate,
            'status_rate':      status_rate,
            'cmd_vel_timeout':  cmd_vel_timeout,
            'max_linear_mms':   max_linear_mms,
            'max_angular_degs': max_angular_degs,
            'use_sim_time':     use_sim_time,
            # odom_frame 和 base_frame 保持默认值
        }],
        # 如果机器人模型使用不同的 frame 名称，可以在此覆盖:
        # remappings=[
        #     ('odom', 'odom'),
        #     ('cmd_vel', 'cmd_vel'),
        # ],
    )

    # ---- 组装 LaunchDescription ----
    ld = LaunchDescription()
    ld.add_action(declare_port)
    ld.add_action(declare_robot_name)
    ld.add_action(declare_baudrate)
    ld.add_action(declare_status_rate)
    ld.add_action(declare_cmd_vel_timeout)
    ld.add_action(declare_max_linear_mms)
    ld.add_action(declare_max_angular_degs)
    ld.add_action(declare_use_sim_time)
    ld.add_action(car_interface_node)

    return ld
