# 故障排查 (Troubleshooting)

## 构建问题

### 错误：`ModuleNotFoundError: No module named 'multi_robot_exploration'`

**含义：** Python 找不到 multi_robot_exploration 包。

**可能原因：**
1. 没有构建或构建失败
2. 没有 source `install/setup.bash`
3. 构建缓存问题

**修复步骤：**
```bash
# 1. 确保在正确目录
cd ~/ros2-multi-robot-automap

# 2. 重新构建
colcon build --cmake-clean-cache --symlink-install --packages-select multi_robot_exploration

# 3. 重新 source
source install/setup.bash

# 4. 验证
ros2 run multi_robot_exploration control --ros-args -p robot_count:=1
```

---

### 错误：`ImportError: No module named 'rclpy'`

**含义：** ROS 2 Python 客户端库未安装或未 source。

**修复：**
```bash
source /opt/ros/humble/setup.bash
```

---

## 运行时问题

### 症状：控制节点启动但没有反应

**排查步骤：**
```bash
# 1. 确认节点在运行
ros2 node list | grep headquarters_control

# 2. 检查机器人数量参数
ros2 param get /headquarters_control robot_count

# 3. 检查是否有 merge_map 数据发布
ros2 topic echo /merge_map --once

# 4. 检查机器人 odom 是否发布
ros2 topic echo /tb1/odom --once

# 5. 查看节点日志
ros2 run rqt_console rqt_console
```

---

### 症状：机器人分配了目标但不动

**可能原因：**
1. Nav2 navigation_to_pose Action 服务器未就绪
2. 目标点不可达（被障碍物包围）
3. 目标点在世界坐标中但正确的 map→odom 变换未建立

**排查步骤：**
```bash
# 检查 Nav2 action server 状态
ros2 action list | grep navigate_to_pose

# 查看目标点的具体坐标
# 在控制节点日志中查找 "Target for tbX is ..."

# 在 RViz 中手动发送该坐标，看机器人是否能导航过去
```

---

### 症状：探索很快停止（没有探索完就保存地图）

**可能原因：**

1. **expansion_size 太大：** 默认 7 个格子。如果分辨率是 0.05m/grid，膨胀了 0.35m。在大房间中没问题，在狭窄走廊中可能把所有空闲格子都膨胀为障碍物。

**检查方法：**
- 减小 `expansion_size` (control.py:21)
- 在 RViz 中观察膨胀后的代价地图发布（如果有的话）

2. **地图分辨率问题：** `data * resolution` (control.py:106) 将数值乘以分辨率后，out-of-range 值可能破坏 frontier 检测。

---

### 症状：多个机器人总是去同一个地方

**原因：** 当前的多机器人协调策略很简单——每个机器人独立计算前沿，`VISITED` 列表去重。

**说明：** 这不是 bug，而是设计的局限（见 `TUTORIAL.md` 第 8.3 节）。两个线程可能几乎同时选择同一个目标并都加入 `VISITED`。

**改进方向：** 见 `EXERCISES.md` 的 M4 练习。

---

### 症状：`save_map` 找不到文件路径

**日志：** `[HATA] Failed to save the map: ...` 或 `[HATA] Unexpected error during map saving: ...`

**可能原因：** `get_package_share_directory('multi_robot')` 返回的路径中，拼接的 `../../../../src/saved_map/` 目录不存在。

**检查方法：**
```bash
# 查找 multi_robot 包的 share 目录
ros2 pkg prefix multi_robot

# 手动创建保存目录（如果不存在）
mkdir -p ~/ros2-multi-robot-automap/src/saved_map/
```

**临时修复：** 修改 `control.py:300-302` 中的路径为绝对路径：
```python
'-f', '/home/your_user/ros2-multi-robot-automap/src/saved_map/my_map'
```

---

## 调试技巧

### 1. 启用详细日志

在启动命令中添加日志级别参数：
```bash
ros2 run multi_robot_exploration control --ros-args -p robot_count:=2 --log-level DEBUG
```

`control.py:249` 中的 `self.get_logger().debug()` 语句将在 DEBUG 级别输出。

### 2. 使用 rqt 监视

```bash
# 查看 ROS 图
rqt_graph

# 查看话题数据
rqt_plot /tb1/odom/pose/pose/position/x

# 查看日志
rqt_console
```

### 3. 添加打印调试

在 `exploration()` 函数（control.py:126）中添加 print 语句来查看中间结果：
```python
print(f"[DEBUG] groups count: {len(groups)}")
print(f"[DEBUG] chosen coordinate: {coordinate}")
```

### 4. 单机器人测试

调试多机器人问题时，先用单机器人确认算法正确：
```bash
ros2 run multi_robot_exploration control --ros-args -p robot_count:=1
```

### 5. 离线测试

将 map_callback 中接收到的地图保存为 numpy 文件，然后离线运行 `exploration()` 函数，不依赖真实的机器人仿真。这比每次启动 Gazebo 快得多。

```python
import numpy as np
# 在 map_callback 中添加：
np.save('/tmp/debug_map.npy', self.map_data)
# 然后在独立脚本中加载：
map_data = np.load('/tmp/debug_map.npy')
```

---

## 常见概念混淆

| 混淆 | 澄清 |
|------|------|
| 栅格坐标 vs 世界坐标 | 栅格坐标是矩阵行列号（整数），世界坐标是物理位置（浮点米）。`exploration()` 输入栅格坐标，`findClosestGroup()` 输出世界坐标。 |
| frontierB 的 4 邻域 vs DFS 的 8 邻域 | frontierB 用 4 方向检测边界，DFS 用 8 方向分组。这是有意为之——检测只需要知道紧邻的未知格子，分组需要捕获对角线连接。 |
| VISITED 全局变量 | VISITED 在主线程中的 `exploration()` 函数内被 append，但 `exploration()` 被多个线程各自调用。这就产生了竞态条件风险。 |
| "idle" vs "active" | idle = 机器人空闲，可以接收新目标；active = 正在导航中。目标完成或失败后总是回到 idle。 |

---

## 需要帮助时

如果自学遇到困难，可以：
1. 重新阅读 `TUTORIAL.md` 中相关章节
2. 在 `control.py` 中用 `# TODO` 标记不理解的地方
3. 绘制数据流图帮助理解
4. 用最简单的情况（1 个机器人，5×5 的小地图）手动模拟算法
5. 在 ROS 2 社区（Robotics Stack Exchange, ROS Discourse）提问
