"""
car_driver.py — 串口 I/O 管理线程

在独立线程中运行一个固定频率的 I/O 循环，统一管理串口的读写操作，
避免 ROS 回调线程与串口读写之间的竞态条件。

核心设计:
  - I/O 循环固定频率运行（默认 50Hz），每次循环执行以下操作之一:
      1. 如果有待发送的 cruise 指令 → 发送 cruise，读取确认，紧接着查询一次状态
      2. 否则 → 发送 car status，读取并解析状态
  - 所有对串口的操作都封装在 I/O 线程内部
  - cmd_vel 回调只负责将速度指令写入线程安全的队列，不直接操作串口
  - 状态更新通过线程安全的 getter/setter 暴露给 ROS 节点线程

通信协议细节:
  - 下位机返回以 \\r\\n 结尾的文本行
  - status 响应以 "Stop: YES/NO" 行结束，之后下位机输出 \\r\\n>  提示符
  - cruise 响应为单行 "Cruise: xxx mm/s, turn xxx deg/s\\r\\n"，之后同样跟 \\r\\n>
  - 检测到 "> " 提示符表示响应结束
"""

import threading
import time
import logging

import serial

from .serial_protocol import (
    CarStatus,
    parse_status,
    build_cruise_command,
    build_status_command,
)

logger = logging.getLogger(__name__)

# 确保 INFO 级别日志输出到 stderr（ROS2 会捕获并显示）
logging.basicConfig(
    level=logging.INFO,
    format='[%(levelname)s] [%(name)s] %(message)s',
)


class CarDriver:
    """小车串口通信驱动器。

    在后台线程中运行 I/O 循环，负责:
      - 串口连接管理（打开、关闭、自动重连）
      - 周期性地查询小车状态（car status）
      - 按需发送巡航控制指令（car cruise）
      - 缓冲和解析串口响应
    """

    # 默认参数
    DEFAULT_BAUDRATE   = 115200      # 串口波特率（USB虚拟串口可任意设置）
    DEFAULT_CYCLE_RATE = 50.0        # I/O 循环频率 (Hz)
    DEFAULT_SERIAL_TIMEOUT = 0.1     # 串口读取超时 (秒)，防止 readline 永久阻塞
    DEFAULT_RECONNECT_DELAY = 2.0    # 断线重连初始间隔 (秒)
    MAX_RECONNECT_DELAY = 10.0       # 断线重连最大间隔 (秒)，指数退避上限

    def __init__(self, port: str, baudrate: int | None = None):
        """初始化串口驱动器。

        Args:
            port:     串口设备路径，例如 '/dev/ttyUSB0'。
            baudrate: 波特率，默认 115200（USB 虚拟串口对波特率不敏感）。
        """
        self._port     = port
        self._baudrate = baudrate if baudrate is not None else self.DEFAULT_BAUDRATE

        # 串口对象（仅在 I/O 线程中访问）
        self._ser: serial.Serial | None = None

        # ---- 线程安全共享状态 ----
        self._lock = threading.Lock()

        # 待发送的 cruise 指令: (linear_mm_s, angular_deg_s)，None 表示无待发指令
        self._pending_cruise: tuple[float, float] | None = None

        # 上次已实际发送的 cruise 值，用于去重（仅在 I/O 线程中访问），
        # 避免高频重复发送相同指令导致小车固件频繁重新处理而产生卡顿
        self._last_sent_cruise: tuple[float, float] | None = None

        # 最新解析的状态快照
        self._latest_status: CarStatus | None = None

        # 是否处于连接状态（由 I/O 线程更新）
        self._connected: bool = False

        # ---- I/O 线程控制 ----
        self._thread: threading.Thread | None = None
        self._running = False
        self._cycle_time = 1.0 / self.DEFAULT_CYCLE_RATE

    # =========================================================================
    # 公开 API（线程安全）
    # =========================================================================

    def start(self) -> bool:
        """启动 I/O 线程并打开串口。

        Returns:
            True 表示线程已启动，False 表示已在运行。
        """
        if self._running:
            return False

        self._running = True
        self._thread = threading.Thread(
            target=self._io_loop,
            name=f"CarDriver-{self._port}",
            daemon=True,
        )
        self._thread.start()
        logger.info(f"[CarDriver] I/O 线程已启动, 串口: {self._port}")
        return True

    def stop(self):
        """停止 I/O 线程并关闭串口。"""
        self._running = False
        if self._thread is not None:
            self._thread.join(timeout=3.0)
            self._thread = None
        self._close_serial()
        logger.info(f"[CarDriver] I/O 线程已停止, 串口: {self._port}")

    def set_cruise(self, linear_mm_s: float, angular_deg_s: float):
        """设置待发送的 cruise 指令（线程安全，由 cmd_vel 回调调用）。

        Args:
            linear_mm_s:   目标线速度 (mm/s)
            angular_deg_s: 目标角速度 (deg/s)
        """
        with self._lock:
            self._pending_cruise = (linear_mm_s, angular_deg_s)

    def get_status(self) -> CarStatus | None:
        """获取最新的小车状态快照（线程安全）。

        Returns:
            CarStatus 或 None（尚未收到任何有效状态）。
        """
        with self._lock:
            return self._latest_status

    @property
    def connected(self) -> bool:
        """返回当前串口连接状态（线程安全）。"""
        with self._lock:
            return self._connected

    # =========================================================================
    # 串口管理（仅在 I/O 线程中调用）
    # =========================================================================

    def _open_serial(self) -> bool:
        """尝试打开串口。

        Returns:
            True 表示打开成功。
        """
        try:
            self._ser = serial.Serial(
                port=self._port,
                baudrate=self._baudrate,
                timeout=self.DEFAULT_SERIAL_TIMEOUT,
                write_timeout=self.DEFAULT_SERIAL_TIMEOUT,
            )
            # 清空收发缓冲区
            self._ser.reset_input_buffer()
            self._ser.reset_output_buffer()
            logger.info(f"[CarDriver] 串口已打开: {self._port} @ {self._baudrate}")
            return True
        except (serial.SerialException, OSError) as e:
            logger.error(f"[CarDriver] 无法打开串口 {self._port}: {e}")
            self._ser = None
            return False

    def _close_serial(self):
        """关闭串口连接。"""
        if self._ser is not None and self._ser.is_open:
            try:
                self._ser.close()
            except Exception as e:
                logger.warning(f"[CarDriver] 关闭串口时出错: {e}")
        self._ser = None
        with self._lock:
            self._connected = False

    # =========================================================================
    # 串口读写（仅在 I/O 线程中调用）
    # =========================================================================

    def _read_response(self, timeout: float = 0.3) -> str:
        """从串口逐行读取，直到检测到 CLI 提示符 '>' 或超时。

        下位机每次响应结束后会打印 \\r\\n>  作为提示符。
        检测到行内容为 ">" 或以 ">" 开头即认为响应结束。

        Args:
            timeout: 最大等待时间 (秒)

        Returns:
            读取到的全部文本（包含 \\r\\n 换行符）。
        """
        if self._ser is None or not self._ser.is_open:
            return ""

        lines = []
        start = time.time()

        while time.time() - start < timeout:
            try:
                line_bytes = self._ser.readline()
            except serial.SerialException as e:
                logger.warning(f"[CarDriver] 串口读取异常: {e}")
                break

            if line_bytes:
                try:
                    line = line_bytes.decode('utf-8', errors='replace')
                except UnicodeDecodeError:
                    line = line_bytes.decode('latin-1', errors='replace')

                # 检测 CLI 提示符 (">" 或 "> ")
                stripped = line.strip()
                if stripped == '>' or stripped.startswith('>'):
                    if lines:
                        # 已有实质内容，这是响应结束标志
                        break
                    # 还没有任何内容，这是上一条命令残留的提示符，丢弃并继续等待
                    continue

                lines.append(line)
            else:
                # readline 超时返回空字节 — 如果已经收到了一些数据，
                # 可能响应已经结束（例如没有提示符的异常情况）
                if lines:
                    break
                # 没有数据则短暂休眠避免忙等
                time.sleep(0.001)

        return ''.join(lines)

    def _send_command(self, command: bytes) -> bool:
        """发送一条命令到串口。

        Args:
            command: 已编码的命令字节串

        Returns:
            True 表示发送成功。
        """
        if self._ser is None or not self._ser.is_open:
            return False
        try:
            self._ser.write(command)
            self._ser.flush()
            return True
        except serial.SerialException as e:
            logger.error(f"[CarDriver] 串口写入失败: {e}")
            self._close_serial()
            return False

    def _query_status(self) -> CarStatus | None:
        """发送 'car status' 查询，读取并解析响应。

        Returns:
            解析后的 CarStatus，失败返回 None。
        """
        if not self._send_command(build_status_command()):
            return None

        raw = self._read_response(timeout=0.3)

        if not raw:
            logger.warning("[CarDriver] car status 无响应（串口读取超时）")
            return None

        status = parse_status(raw)
        if status is None:
            logger.warning(f"[CarDriver] car status 解析失败，原始数据: {raw[:200]!r}")
        return status

    def _send_cruise(self, linear_mm_s: float, angular_deg_s: float) -> bool:
        """发送 'car cruise' 指令并消费确认响应。

        Args:
            linear_mm_s:   线速度 (mm/s)
            angular_deg_s: 角速度 (deg/s)

        Returns:
            True 表示指令已发送并收到确认。
        """
        cmd = build_cruise_command(linear_mm_s, angular_deg_s)
        if not self._send_command(cmd):
            return False

        # 消费 cruise 的确认响应 "Cruise: xxx mm/s, turn xxx deg/s\r\n"
        # 以及随后的 \r\n>  提示符。不需要解析内容，只需清空接收缓冲区。
        self._read_response(timeout=0.2)
        return True

    # =========================================================================
    # I/O 主循环
    # =========================================================================

    def _io_loop(self):
        """I/O 主循环 — 在独立线程中运行。

        每个循环周期:
          1. 检查是否有待发送的 cruise 指令
          2. 发送 cruise 或查询 status
          3. 更新共享状态
          4. 休眠以维持固定循环频率
        """
        reconnect_delay = self.DEFAULT_RECONNECT_DELAY

        while self._running:
            # --- 确保串口已连接 ---
            if self._ser is None or not self._ser.is_open:
                if self._open_serial():
                    with self._lock:
                        self._connected = True
                    reconnect_delay = self.DEFAULT_RECONNECT_DELAY
                    # 重连后清空上次发送记录，确保第一条指令一定会发送
                    self._last_sent_cruise = None
                else:
                    with self._lock:
                        self._connected = False
                    # 指数退避重连
                    logger.info(
                        f"[CarDriver] {reconnect_delay:.0f}s 后重试连接..."
                    )
                    time.sleep(reconnect_delay)
                    reconnect_delay = min(
                        reconnect_delay * 1.5,
                        self.MAX_RECONNECT_DELAY,
                    )
                    continue

            cycle_start = time.time()

            try:
                # --- 检查是否有待发送的 cruise 指令 ---
                with self._lock:
                    cruise = self._pending_cruise
                    self._pending_cruise = None  # 消费指令

                if cruise is not None:
                    # --- 去重：只有速度值真正变化时才发送 ---
                    # 避免高频重复发送相同 cruise 指令导致小车固件反复处理而产生卡顿
                    if self._last_sent_cruise is None or cruise != self._last_sent_cruise:
                        self._send_cruise(cruise[0], cruise[1])
                        self._last_sent_cruise = cruise
                        logger.info(
                            f'[CarDriver] 发送控制指令: {cruise[0]:.1f} mm/s, '
                            f'{cruise[1]:.1f} deg/s'
                        )
                        # 短暂停顿确保下位机处理完毕
                        time.sleep(0.005)
                    # 无论是否发送 cruise，都查询一次状态获取反馈
                    status = self._query_status()
                else:
                    # 没有待发指令，正常查询状态
                    status = self._query_status()

                # --- 更新共享状态 ---
                if status is not None:
                    status.timestamp = time.time()
                    with self._lock:
                        self._latest_status = status

            except serial.SerialException as e:
                logger.error(f"[CarDriver] 串口通信异常: {e}")
                self._close_serial()
            except Exception as e:
                logger.error(f"[CarDriver] I/O 循环未预期的异常: {e}", exc_info=True)

            # --- 维持固定循环频率 ---
            elapsed = time.time() - cycle_start
            sleep_time = self._cycle_time - elapsed
            if sleep_time > 0:
                time.sleep(sleep_time)

        # 线程退出时清理
        self._close_serial()
        logger.info("[CarDriver] I/O 循环已退出")
