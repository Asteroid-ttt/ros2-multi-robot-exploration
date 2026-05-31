# 词汇表 (Glossary)

## ROS 2 通用术语

### Node（节点）
一个独立的 ROS 2 可执行程序，能够通过 Topics、Services、Actions 与其他节点通信。
**本项目中的节点：** `HeadquartersControl` (control.py:160)

### Topic（话题）
命名的数据通道，节点通过它发布和订阅消息。一个节点发布，多个节点可以订阅。
**本项目中的 Topics：** `/merge_map`（合并地图）、`/tb1/odom`（机器人里程计）

### Message（消息）
通过 Topic 传输的数据结构。定义在 `.msg` 文件中。
**本项目中的消息类型：** `OccupancyGrid`, `Odometry`, `Twist`

### Action（动作）
类似 Service 但用于长时间运行的任务。客户端发送目标，服务器期间反馈进度，完成后返回结果。异步非阻塞。
**本项目中的 Action：** `NavigateToPose` — 导航到指定位姿

### ActionClient（动作客户端）
发送 Action 目标的一方。
**位置：** control.py:182

### Callback（回调函数）
当有新消息到达或事件发生时被调用的函数。ROS 2 的 `spin()` 驱动回调执行。
**示例：** `map_callback` (control.py:205), `goal_result_callback` (control.py:272)

### Spin（事件循环）
`rclpy.spin()` 让节点持续运行，等待并处理收到的消息。

## 导航相关术语

### OccupancyGrid（占用栅格地图）
一种网格地图，每个格子存储一个值表示该位置的状态。
- **0：** 空闲 (free) — 确定没有障碍物
- **100：** 占据 (occupied) — 确定有障碍物
- **-1：** 未知 (unknown) — 没有被探测过

**位置：** 由 `map_callback` 解析 (control.py:205)

### Odometry（里程计）
机器人对自身位姿（位置 + 朝向）的估计。
**位置：** 由 `robot_odom_callback` 处理 (control.py:213)

### Nav2 (Navigation 2)
ROS 2 的导航框架。提供路径规划、运动控制、定位等功能。
**本项目用途：** 使用其 `NavigateToPose` Action 将机器人导航到探索目标点。

### NavigateToPose
Nav2 的 Action 接口，接受目标位姿，驱动机器人自主导航到达。

### SLAM (Simultaneous Localization and Mapping)
同时定位与建图：机器人在未知环境中一边定位自己，一边构建环境地图。
**本项目：** 使用 slam_toolbox 实现

### Map Merging（地图融合）
将多个机器人各自构建的地图合并为一张全局地图。
**本项目：** 由 `merge_map` 包负责

## 探索算法术语

### Frontier（前沿）
已知空闲区域与未知区域的边界。用于指导自主探索——机器人应前往前沿以获取新信息。
**位置：** `frontierB` 函数 (control.py:28)

### 代价地图膨胀 (Costmap Inflation)
将障碍物周围一定范围内的格子也标记为障碍物，增加安全缓冲距离。
**位置：** `costmap` 函数 (control.py:94)
**参数：** `expansion_size = 7` (control.py:21)

### DFS (Depth-First Search，深度优先搜索)
一种图遍历算法。从起点出发，沿一个方向探索到底，然后回溯，继续探索其他方向。
**本项目用途：** 将相互连接的前沿格子分组 (control.py:51)

### 连通分量 (Connected Component)
图中任意两个节点之间都存在路径的最大子图。
**本项目用途：** 一组相互连接的（8 邻域）前沿格子构成了一个候选探索目标。

### 8 邻域 (8-Connectivity)
在二维网格中，每个格子有 8 个邻居：上下左右 + 四个对角线。
**位置：** `dfs` 函数 (control.py:61-68)

### Centroid（质心）
一组点的几何中心（平均值）。
**位置：** `calculate_centroid` 函数 (control.py:76)

### 评分函数 (Scoring Function)
用于在多个候选项中做出选择的数学公式。本项目中：
```
score = group_size / (distance + 1e-6)
```
**位置：** `findClosestGroup` (control.py:120)

## Python 编程术语

### Thread（线程）
程序中的一条独立执行路径。多线程允许同时执行多个任务。
**本项目用途：** 每个机器人有一个独立的探索线程 (control.py:200-201)

### Daemon Thread（守护线程）
当主程序退出时自动终止的线程。非守护线程会阻止程序退出。
**位置：** `daemon=True` (control.py:201)

### Global Variable（全局变量）
在模块级别定义，所有函数可访问的变量。
**本项目：** `VISITED` 列表 (control.py:25) 和配置参数 (control.py:19-23)

### Race Condition（竞态条件）
多个线程同时访问共享数据而未正确同步时产生的不可预测行为。
**本项目潜在位置：** `VISITED` 全局列表被多个探索线程同时修改。

### Lambda（匿名函数）
不需要名称的小型函数，通常用于简单的回调。
**位置：** control.py:180 (odom 回调), control.py:265 (goal 结果回调)

### NumPy
Python 的科学计算库，特别适合处理多维数组。
**本项目用途：** 地图数据表示为二维 NumPy 数组。

### Subprocess
Python 标准库，用于在 Python 中启动外部进程。
**本项目用途：** 调用 `ros2 run nav2_map_server map_saver_cli` 保存地图 (control.py:298)

## 项目特定缩写

| 缩写 | 全称 | 位置 |
|------|------|------|
| `tb` | TurtleBot | 机器人命名前缀：tb1, tb2, ... |
| `B` 在 `frontierB` | 可能是 "boundary" 的缩写 | control.py:28 |
| `fGroups` | Filter Groups（筛选组） | control.py:71 |
| `BILGI` / `HATA` | 土耳其语 "信息" / "错误" 的意思 | control.py:153, 310 |

## 坐标系术语

### 栅格坐标 (Grid Coordinate)
地图矩阵中的行列索引，单位是"格子"。
- `row` / `grid_y`：矩阵行号 → 世界坐标 y
- `column` / `grid_x`：矩阵列号 → 世界坐标 x

### 世界坐标 (World Coordinate)
物理空间中的位置，单位是"米"。
- `originX, originY`：地图原点的世界坐标

### 坐标转换公式
```python
# 世界 → 栅格（在 exploration 中）
grid_x = (world_x - originX) / resolution
grid_y = (world_y - originY) / resolution

# 栅格 → 世界（在 findClosestGroup 中）
world_x = grid_y * resolution + originX
world_y = grid_x * resolution + originY
```

注意 x/y 的交叉对应关系：栅格的**列**对应世界的 **x**，栅格的**行**对应世界的 **y**。
