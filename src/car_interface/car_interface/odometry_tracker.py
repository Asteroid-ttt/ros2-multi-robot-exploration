"""
odometry_tracker.py — 里程计推算

根据小车周期性上报的 CarStatus（偏航角 + 当前实际速度），
通过积分线速度推算小车在 odom 坐标系中的位姿 (x, y, θ)。

推算策略:
  - 偏航角 θ:   直接使用小车上报的 Yaw 绝对值（来自其自带传感器，无漂移累积）
  - 位置 (x, y): 对实际线速度在偏航角方向上积分得到
  - 首次收到的 Yaw 作为零偏，保证 odom 从 (0, 0, 0) 起始

注意:
  这种方式依赖小车上报的偏航角精度。如果小车传感器存在固定偏差，
  可通过 yaw_offset 参数进行补偿。
"""

import math
import time


class OdometryTracker:
    """基于小车状态反馈的里程计推算器。

    用法:
        tracker = OdometryTracker()
        # 每次收到 CarStatus 时调用:
        tracker.update(v_lin_mms, v_ang_degs, yaw_deg, current_time)
        # 读取当前位姿:
        x, y, yaw = tracker.pose
    """

    def __init__(self):
        # ---- 位姿状态 ----
        self._x: float     = 0.0       # 位置 X (m)，odom 坐标系
        self._y: float     = 0.0       # 位置 Y (m)，odom 坐标系
        self._yaw: float   = 0.0       # 偏航角 (rad)，范围 (-π, π]

        # ---- 速度状态 ----
        self._v_lin: float = 0.0       # 当前线速度 (m/s)
        self._v_ang: float = 0.0       # 当前角速度 (rad/s)

        # ---- 推算中间量 ----
        self._initial_yaw: float | None = None  # 首次收到的原始偏航角 (rad)，作为零偏
        self._last_time: float | None   = None  # 上一次更新的时间戳 (秒)

    # =========================================================================
    # 公开 API
    # =========================================================================

    def update(
        self,
        v_lin_mms: float,
        v_ang_degs: float,
        yaw_deg: float,
        current_time: float,
    ):
        """使用新收到的状态数据更新里程计。

        应在每次收到并解析 CarStatus 后调用。

        Args:
            v_lin_mms:    当前实际线速度 (mm/s)，来自 CarStatus.v_lin_cur
            v_ang_degs:   当前实际角速度 (deg/s)，来自 CarStatus.v_ang_cur
            yaw_deg:      当前偏航角 (度)，来自 CarStatus.yaw_deg
            current_time: 当前时间戳 (秒)，如 time.time() 或 rclpy 时间
        """
        # --- 单位转换 ---
        v_lin_ms  = v_lin_mms / 1000.0              # mm/s  → m/s
        v_ang_rads = math.radians(v_ang_degs)        # deg/s → rad/s
        yaw_raw_rad = math.radians(yaw_deg)          # deg   → rad

        # --- 初始化偏航零偏 ---
        if self._initial_yaw is None:
            self._initial_yaw = yaw_raw_rad
            self._yaw = 0.0
        else:
            # 计算相对于初始偏航的朝向
            self._yaw = self._normalize_angle(yaw_raw_rad - self._initial_yaw)

        # --- 积分位置 ---
        if self._last_time is not None:
            dt = current_time - self._last_time
            if dt > 0 and dt < 1.0:  # 合理的 dt 范围（防止跳变大导致积分发散）
                # 使用上一周期的偏航角进行积分（欧拉法）
                self._x += v_lin_ms * math.cos(self._yaw) * dt
                self._y += v_lin_ms * math.sin(self._yaw) * dt

        # --- 更新速度和时戳 ---
        self._v_lin = v_lin_ms
        self._v_ang = v_ang_rads
        self._last_time = current_time

    def reset(self):
        """重置里程计，清除所有累积状态。"""
        self._x = 0.0
        self._y = 0.0
        self._yaw = 0.0
        self._v_lin = 0.0
        self._v_ang = 0.0
        self._initial_yaw = None
        self._last_time = None

    # =========================================================================
    # 属性访问器
    # =========================================================================

    @property
    def x(self) -> float:
        """当前位置 X 坐标 (m)"""
        return self._x

    @property
    def y(self) -> float:
        """当前位置 Y 坐标 (m)"""
        return self._y

    @property
    def yaw(self) -> float:
        """当前偏航角 (rad)，范围 (-π, π]"""
        return self._yaw

    @property
    def v_lin(self) -> float:
        """当前线速度 (m/s)"""
        return self._v_lin

    @property
    def v_ang(self) -> float:
        """当前角速度 (rad/s)"""
        return self._v_ang

    @property
    def pose(self) -> tuple[float, float, float]:
        """返回 (x, y, yaw) 元组。"""
        return (self._x, self._y, self._yaw)

    # =========================================================================
    # 内部工具
    # =========================================================================

    @staticmethod
    def _normalize_angle(angle: float) -> float:
        """将角度规范化到 (-π, π] 范围。

        Args:
            angle: 任意弧度值

        Returns:
            规范化后的弧度值。
        """
        angle = angle % (2.0 * math.pi)
        if angle > math.pi:
            angle -= 2.0 * math.pi
        return angle
