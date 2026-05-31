"""
car_interface_node.py — ROS2 主节点: 串口小车 ↔ Nav2 导航栈 桥接器

功能概述:
  1. 订阅 /cmd_vel (geometry_msgs/Twist) → 转换为 car cruise 指令发送到串口
  2. 周期性查询 car status → 推算里程计
  3. 发布 /odom (nav_msgs/Odometry) 供 Nav2 定位使用
  4. 发布 tf 变换 odom → base_footprint 供 TF 树使用

参数 (ROS2 Parameter):
  port             - 串口设备路径 (默认: /dev/ttyUSB0)
  baudrate         - 串口波特率 (默认: 115200)
  odom_frame       - 里程计坐标系 ID (默认: odom)
  base_frame       - 机器人本体坐标系 ID (默认: base_footprint)
  status_rate      - 状态查询频率 Hz (默认: 20.0)
  max_linear_mms   - 线速度上限 mm/s (默认: 500.0)
  max_angular_degs - 角速度上限 deg/s (默认: 180.0)
"""

import math
import time

import rclpy
from rclpy.node import Node
from rclpy.time import Time

from geometry_msgs.msg import Twist, TransformStamped, Quaternion
from nav_msgs.msg import Odometry
from tf2_ros import TransformBroadcaster

from .car_driver import CarDriver
from .odometry_tracker import OdometryTracker
from .serial_protocol import twist_to_cruise


def quaternion_from_yaw(yaw: float) -> Quaternion:
    """从偏航角创建表示绕 Z 轴旋转的四元数。

    Args:
        yaw: 偏航角 (rad)

    Returns:
        geometry_msgs/Quaternion 消息。
    """
    q = Quaternion()
    q.x = 0.0
    q.y = 0.0
    q.z = math.sin(yaw * 0.5)
    q.w = math.cos(yaw * 0.5)
    return q


class CarInterfaceNode(Node):
    """ROS2 节点: 将串口控制的小车接入 Nav2 导航栈。

    输入:  /cmd_vel (Twist)
    输出:  /odom (Odometry), /tf (odom→base_footprint)
    """

    def __init__(self):
        super().__init__('car_interface_node')

        # =====================================================================
        # 声明参数（所有可配置参数集中在此）
        # =====================================================================
        self.declare_parameter('port',             '/dev/ttyACM0')
        self.declare_parameter('baudrate',         115200)
        self.declare_parameter('odom_frame',       'odom')
        self.declare_parameter('base_frame',       'base_footprint')
        self.declare_parameter('status_rate',      20.0)
        self.declare_parameter('cmd_vel_timeout',  0.5)
        self.declare_parameter('max_linear_mms',   500.0)
        self.declare_parameter('max_angular_degs', 180.0)

        # 读取参数值
        port              = self.get_parameter('port').get_parameter_value().string_value
        baudrate          = self.get_parameter('baudrate').get_parameter_value().integer_value
        self._odom_frame  = self.get_parameter('odom_frame').get_parameter_value().string_value
        self._base_frame  = self.get_parameter('base_frame').get_parameter_value().string_value
        status_rate       = self.get_parameter('status_rate').get_parameter_value().double_value
        self._max_lin     = self.get_parameter('max_linear_mms').get_parameter_value().double_value
        self._max_ang     = self.get_parameter('max_angular_degs').get_parameter_value().double_value

        # =====================================================================
        # 核心组件初始化
        # =====================================================================
        # 串口 I/O 驱动器（内部启动独立线程）
        self._driver = CarDriver(port=port, baudrate=baudrate)
        self._driver.start()

        # 里程计推算器
        self._odom_tracker = OdometryTracker()

        # TF 广播器
        self._tf_broadcaster = TransformBroadcaster(self)

        # =====================================================================
        # ROS2 通信接口
        # =====================================================================
        # 发布: 里程计
        self._odom_pub = self.create_publisher(
            Odometry,
            'odom',
            10,
        )

        # 订阅: 速度指令
        self._cmd_vel_sub = self.create_subscription(
            Twist,
            'cmd_vel',
            self._cmd_vel_callback,
            10,
        )

        # =====================================================================
        # 定时器：周期性发布里程计和 TF
        # =====================================================================
        period = 1.0 / status_rate
        self._timer = self.create_timer(period, self._timer_callback)

        # =====================================================================
        # 状态追踪
        # =====================================================================
        # 上一次发布 odom 时的 seq 序号
        self._odom_seq: int = 0

        self.get_logger().info(
            f'CarInterfaceNode 已启动: port={port}, baudrate={baudrate}, '
            f'namespace={self.get_namespace() or "/"}'
        )

    # =========================================================================
    # 析构
    # =========================================================================

    def destroy_node(self):
        """节点销毁时确保串口线程正确停止。"""
        self._driver.stop()
        super().destroy_node()

    # =========================================================================
    # cmd_vel 回调
    # =========================================================================

    def _cmd_vel_callback(self, msg: Twist):
        """接收 Nav2 控制器的速度指令，转换为 cruise 指令发送。

        在 ROS 线程中运行，不直接操作串口，仅写入 driver 的共享变量。

        Args:
            msg: geometry_msgs/Twist 速度指令。
        """
        # --- 单位转换 ---
        linear_mm_s, angular_deg_s = twist_to_cruise(
            msg.linear.x,
            msg.angular.z,
        )

        # --- 限幅保护：防止超过小车物理能力上限 ---
        linear_mm_s   = self._clamp(linear_mm_s,   -self._max_lin, self._max_lin)
        angular_deg_s = self._clamp(angular_deg_s, -self._max_ang, self._max_ang)

        # --- 写入 driver（线程安全） ---
        self._driver.set_cruise(linear_mm_s, angular_deg_s)

    # =========================================================================
    # 定时器回调（周期性里程计发布）
    # =========================================================================

    def _timer_callback(self):
        """周期性执行（由 ROS Timer 驱动，在主线程中运行）:

        1. 从 driver 读取最新状态
        2. 更新里程计
        3. 发布 /odom 和 /tf
        """
        # --- 1. 读取小车状态 ---
        status = self._driver.get_status()
        if status is None:
            # 尚未收到有效状态，发布一次连接状态日志
            if not self._driver.connected:
                self.get_logger().warn(
                    '串口未连接，等待重连...',
                    throttle_duration_sec=5.0,
                )
            else:
                self.get_logger().warn(
                    '串口已连接但尚未收到小车状态数据...',
                    throttle_duration_sec=3.0,
                )
            return

        # --- 周期性展示小车当前状态（2s 节流） ---
        stop_str = '已停车' if status.stop else '运行中'
        self.get_logger().info(
            f'小车状态 [{stop_str}] '
            f'Yaw={status.yaw_deg:.1f}° '
            f'V_Lin={status.v_lin_cur:.1f}/{status.v_lin_tgt:.1f} mm/s '
            f'V_Ang={status.v_ang_cur:.1f}/{status.v_ang_tgt:.1f} deg/s',
            throttle_duration_sec=2.0,
        )

        # --- 2. 更新里程计 ---
        now = self.get_clock().now()
        now_sec = now.nanoseconds / 1e9

        self._odom_tracker.update(
            v_lin_mms  = status.v_lin_cur,
            v_ang_degs = status.v_ang_cur,
            yaw_deg    = status.yaw_deg,
            current_time = now_sec,
        )

        # --- 3. 发布里程计消息 ---
        self._publish_odometry(now, status)

        # --- 4. 发布 TF 变换 ---
        self._publish_tf(now)

    # =========================================================================
    # 里程计发布
    # =========================================================================

    def _publish_odometry(self, now: Time, status):
        """构造并发布 nav_msgs/Odometry 消息。

        Args:
            now:    当前 ROS 时间
            status: 小车状态快照
        """
        odom = Odometry()

        # --- 消息头 ---
        odom.header.stamp    = now.to_msg()
        odom.header.frame_id = self._odom_frame
        odom.child_frame_id  = self._base_frame

        # --- 位姿 ---
        odom.pose.pose.position.x   = self._odom_tracker.x
        odom.pose.pose.position.y   = self._odom_tracker.y
        odom.pose.pose.position.z   = 0.0
        odom.pose.pose.orientation  = quaternion_from_yaw(self._odom_tracker.yaw)

        # --- 位姿协方差 ---
        # 简化的协方差矩阵：x, y 方向方差较大（因为积分会累积误差），
        # yaw 方差较小（因为直接使用小车上报的绝对值）
        cov_pose = [0.0] * 36
        cov_pose[0]  = 0.01   # x 方差
        cov_pose[7]  = 0.01   # y 方差
        cov_pose[35] = 0.005  # yaw 方差（较小，信任小车的偏航角传感器）
        odom.pose.covariance = cov_pose

        # --- 速度 ---
        odom.twist.twist.linear.x  = self._odom_tracker.v_lin
        odom.twist.twist.linear.y  = 0.0
        odom.twist.twist.linear.z  = 0.0
        odom.twist.twist.angular.x = 0.0
        odom.twist.twist.angular.y = 0.0
        odom.twist.twist.angular.z = self._odom_tracker.v_ang

        # --- 速度协方差 ---
        cov_twist = [0.0] * 36
        cov_twist[0]  = 0.01   # v_lin 方差
        cov_twist[35] = 0.01   # v_ang 方差
        odom.twist.covariance = cov_twist

        # --- 发布 ---
        self._odom_pub.publish(odom)

    # =========================================================================
    # TF 发布
    # =========================================================================

    def _publish_tf(self, now: Time):
        """发布 odom → base_footprint 的 TF 变换。"""
        t = TransformStamped()

        t.header.stamp    = now.to_msg()
        t.header.frame_id = self._odom_frame
        t.child_frame_id  = self._base_frame

        t.transform.translation.x = self._odom_tracker.x
        t.transform.translation.y = self._odom_tracker.y
        t.transform.translation.z = 0.0

        t.transform.rotation = quaternion_from_yaw(self._odom_tracker.yaw)

        self._tf_broadcaster.sendTransform(t)

    # =========================================================================
    # 工具
    # =========================================================================

    @staticmethod
    def _clamp(value: float, lo: float, hi: float) -> float:
        """将数值限制在 [lo, hi] 范围内。"""
        return max(lo, min(hi, value))


# =============================================================================
# 入口点
# =============================================================================

def main(args=None):
    rclpy.init(args=args)
    node = CarInterfaceNode()

    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        node.get_logger().info('收到中断信号，正在关闭...')
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
