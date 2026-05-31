"""
serial_protocol.py — 小车串口协议解析与命令构造

协议说明:
  上位机 → 小车:  "car cruise <mm/s> <deg/s>"  控制指令
  上位机 → 小车:  "car status"                   状态查询
  小车 → 上位机:  返回对应格式的文本响应 (以 \\r\\n 结尾)

小车返回 status 格式示例:
  --- Car Status ---
  Yaw:   45.3 deg (total 45.3, circles 0)
  V_Lin: 198.2 / 200.0 mm/s  (cur / tgt)
  V_Ang: 28.7 / 30.0 deg/s (cur / tgt)
  Stop:  NO

小车返回 cruise 确认格式示例:
  Cruise: 200.0 mm/s, turn 30.0 deg/s
"""

import re
import math
from dataclasses import dataclass, field


# =============================================================================
# 数据结构
# =============================================================================

@dataclass
class CarStatus:
    """从 'car status' 响应中解析出的小车状态。

    Attributes:
        yaw_deg:         当前偏航角，单位 度 (deg)
        yaw_total_deg:   累计偏航角（无符号），单位 度
        yaw_circles:     完整圈数
        v_lin_cur:       当前实际线速度，单位 mm/s
        v_lin_tgt:       目标线速度，单位 mm/s
        v_ang_cur:       当前实际角速度，单位 deg/s
        v_ang_tgt:       目标角速度，单位 deg/s
        stop:            是否处于急停状态，True = 已急停
        timestamp:       解析时的时间戳（由调用方填入，单位: 秒）
    """
    yaw_deg:       float = 0.0
    yaw_total_deg: float = 0.0
    yaw_circles:   int   = 0
    v_lin_cur:     float = 0.0
    v_lin_tgt:     float = 0.0
    v_ang_cur:     float = 0.0
    v_ang_tgt:     float = 0.0
    stop:          bool  = False
    timestamp:     float = 0.0


# =============================================================================
# 正则表达式 — 用于解析 status 响应中的各字段
# =============================================================================

# 匹配 Yaw 行，例如: "Yaw:   45.3 deg (total 45.3, circles 0)"
_RE_YAW   = re.compile(
    r'Yaw:\s+([-\d.]+)\s+deg\s+\(total\s+([-\d.]+),\s+circles\s+([-\d]+)\)'
)

# 匹配 V_Lin 行，例如: "V_Lin: 198.2 / 200.0 mm/s  (cur / tgt)"
_RE_VLIN  = re.compile(
    r'V_Lin:\s+([-\d.]+)\s*/\s*([-\d.]+)\s+mm/s'
)

# 匹配 V_Ang 行，例如: "V_Ang: 28.7 / 30.0 deg/s (cur / tgt)"
_RE_VANG  = re.compile(
    r'V_Ang:\s+([-\d.]+)\s*/\s*([-\d.]+)\s+deg/s'
)

# 匹配 Stop 行，例如: "Stop:  NO" 或 "Stop:  YES"
_RE_STOP  = re.compile(
    r'Stop:\s+(YES|NO)'
)


# =============================================================================
# 协议解析函数
# =============================================================================

def parse_status(raw_text: str) -> CarStatus | None:
    """解析 'car status' 的原始响应文本，提取小车状态。

    Args:
        raw_text: 从串口读取到的原始响应字符串。

    Returns:
        解析成功返回 CarStatus 对象，失败返回 None。
        注意: timestamp 字段需要由调用方在解析后手动填入。
    """
    if not raw_text or '--- Car Status ---' not in raw_text:
        return None

    status = CarStatus()

    # 按行搜索各字段，只要匹配任意一行即可
    found_yaw  = False
    found_vlin = False
    found_vang = False
    found_stop = False

    for line in raw_text.split('\n'):
        line = line.strip()

        # --- 解析 Yaw ---
        m = _RE_YAW.search(line)
        if m:
            status.yaw_deg       = float(m.group(1))
            status.yaw_total_deg = float(m.group(2))
            status.yaw_circles   = int(m.group(3))
            found_yaw = True
            continue

        # --- 解析 V_Lin ---
        m = _RE_VLIN.search(line)
        if m:
            status.v_lin_cur = float(m.group(1))
            status.v_lin_tgt = float(m.group(2))
            found_vlin = True
            continue

        # --- 解析 V_Ang ---
        m = _RE_VANG.search(line)
        if m:
            status.v_ang_cur = float(m.group(1))
            status.v_ang_tgt = float(m.group(2))
            found_vang = True
            continue

        # --- 解析 Stop ---
        m = _RE_STOP.search(line)
        if m:
            status.stop = (m.group(1) == 'YES')
            found_stop = True
            continue

    # 只有至少匹配到 Yaw 和 Stop 才算有效状态（V_Lin/V_Ang 可选）
    if not (found_yaw and found_stop):
        return None

    # V_Lin/V_Ang 未匹配到时保持默认值 0.0（例如刚启动时可能没有这些字段）
    return status


# =============================================================================
# 命令构造函数
# =============================================================================

def build_cruise_command(linear_mm_s: float, angular_deg_s: float) -> bytes:
    """构造 'car cruise' 控制指令。

    Args:
        linear_mm_s:   目标线速度，单位 mm/s。
                       正值 = 前进，负值 = 后退。
        angular_deg_s: 目标角速度，单位 deg/s。
                       正值 = 逆时针旋转。

    Returns:
        编码后的串口命令字节串，例如 b'car cruise 200.0 30.0\\r\\n'。
    """
    # 保留 1 位小数，去除无意义的尾随零
    lin_str  = f"{linear_mm_s:.1f}"
    ang_str  = f"{angular_deg_s:.1f}"
    return f"car cruise {lin_str} {ang_str}\r\n".encode('ascii')


def build_status_command() -> bytes:
    """构造 'car status' 状态查询指令。

    Returns:
        编码后的串口命令字节串: b'car status\\r\\n'。
    """
    return b"car status\r\n"


# =============================================================================
# 单位转换工具函数
# =============================================================================

def twist_to_cruise(linear_ms: float, angular_rads: float) -> tuple[float, float]:
    """将 ROS Twist 消息中的速度转换为小车 cruise 指令参数。

    Args:
        linear_ms:    线速度，单位 m/s (来自 Twist.linear.x)
        angular_rads: 角速度，单位 rad/s (来自 Twist.angular.z)

    Returns:
        (linear_mm_s, angular_deg_s) 元组。
    """
    linear_mm_s   = linear_ms * 1000.0       # m/s  → mm/s
    angular_deg_s = angular_rads * 180.0 / math.pi  # rad/s → deg/s
    return linear_mm_s, angular_deg_s


def status_vel_to_ros(v_lin_mms: float, v_ang_degs: float) -> tuple[float, float]:
    """将小车状态中的速度转换为 ROS 标准单位。

    Args:
        v_lin_mms:  线速度，单位 mm/s
        v_ang_degs: 角速度，单位 deg/s

    Returns:
        (linear_ms, angular_rads) 元组。
    """
    linear_ms    = v_lin_mms / 1000.0             # mm/s   → m/s
    angular_rads = v_ang_degs * math.pi / 180.0   # deg/s  → rad/s
    return linear_ms, angular_rads


def yaw_deg_to_rad(yaw_deg: float) -> float:
    """将偏航角从度转换为弧度。

    Args:
        yaw_deg: 偏航角，单位 度

    Returns:
        偏航角，单位 弧度，范围 (-π, π]
    """
    return math.radians(yaw_deg) % (2 * math.pi)
