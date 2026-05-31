# merge_map 包完全教程：从原理到实践

---

## 目录

1. [概述：这个包是做什么的？](#1-概述这个包是做什么的)
2. [项目结构一览](#2-项目结构一览)
3. [前置知识：我们要合并的地图是什么？](#3-前置知识我们要合并的地图是什么)
4. [核心算法：空间并集与坐标变换](#4-核心算法空间并集与坐标变换)
5. [合并冲突策略：谁说了算？](#5-合并冲突策略谁说了算)
6. [源码逐行分析：在线合并 `merge_map.py`](#6-源码逐行分析在线合并-merge_mappy)
7. [源码逐行分析：离线合并 `offline_merge_map.py`](#7-源码逐行分析离线合并-offline_merge_mappy)
8. [启动文件：如何组织系统](#8-启动文件如何组织系统)
9. [RViz 可视化配置](#9-rviz-可视化配置)
10. [完整数据流图](#10-完整数据流图)
11. [总结与改进建议](#11-总结与改进建议)

---

## 1. 概述：这个包是做什么的？

这是一个 **ROS2 多机器人地图合并** 包。在多机器人协同建图场景中，每个机器人在各自区域进行 SLAM（同步定位与建图），各自生成一张局部的占据栅格地图（OccupancyGrid）。本包的功能是：**将多张局部地图拼合成一张全局地图**。

本包支持两种工作模式：

| 模式 | 说明 | 入口 |
|------|------|------|
| **在线合并** | 在 SLAM 运行过程中实时订阅各机器人的 `/map` 话题，即时合成 | `merge_map.py` |
| **离线合并** | 读取磁盘上保存的 `.yaml` 地图文件，通过 `map_server` 发布后再合成 | `offline_merge_map.py` |

---

## 2. 项目结构一览

```
merge_map/
├── config/                          # RViz2 配置文件
│   ├── map_merge_tb1_tb2.rviz       #   在线合并可视化 (tb1/tb2)
│   ├── merge_map.rviz               #   在线合并可视化 (tb3 + 无人机)
│   └── offline_merge_map.rviz       #   离线合并可视化 (4个机器人)
├── launch/                          # ROS2 launch 文件
│   ├── merge_map_launch.py          #   启动在线合并
│   └── offline_merge_map_launch.py  #   启动离线合并
├── merge_map/                       # Python 源码
│   ├── __init__.py                  #   空文件，标记为 Python 包
│   ├── merge_map.py                 # ★ 在线合并节点
│   └── offline_merge_map.py         # ★ 离线合并节点
├── resource/
│   └── merge_map                    #   ament 索引标记文件
├── test/                            # 测试文件（代码风格检查）
├── LICENSE
├── package.xml                      # ROS2 包清单
├── setup.cfg
└── setup.py                         # 安装配置
```

这是一个 **纯 Python 的 ament_python 包**，没有 C++ 代码、没有自定义的消息/服务/动作定义，完全依赖标准 ROS2 消息类型。

---

## 3. 前置知识：我们要合并的地图是什么？

### 3.1 OccupancyGrid（占据栅格地图）

包中使用的是 ROS2 标准消息 `nav_msgs/OccupancyGrid`，它表示一张**二维栅格地图**。

一条 OccupancyGrid 消息包含以下核心字段：

```
OccupancyGrid:
  Header header
    stamp:      时间戳
    frame_id:   坐标系名称 (如 "map")
  MapMetaData info
    resolution: 每个栅格的物理尺寸 (米/格)，如 0.05 表示一格 = 5cm
    width:      栅格宽度（列数）
    height:     栅格高度（行数）
    origin:     地图左下角在世界坐标系中的位置 (geometry_msgs/Pose)
      position.x, position.y, position.z
      orientation.x, orientation.y, orientation.z, orientation.w
  int8[] data:  栅格数据（一维数组，按行优先排列，长度 = width * height）
```

### 3.2 栅格数据的含义

`data` 中每个格子（cell）的值有三种状态：

| 值 | 含义 |
|----|------|
| `-1` | 未知（unknown）：传感器没有覆盖到的区域 |
| `0` | 空闲（free）：确认没有障碍物的区域 |
| `100` | 占据（occupied）：确认有障碍物的区域 |

> 注：实际上 `data` 是 int8 类型，值域为 0~100，-1 用于标记未知。0 和 100 对应概率的极端值。

### 3.3 地图在世界坐标系中的定位

地图不是凭空存在的，它有确定的空间位置。通过 `origin` 字段，可以知道地图左下角格子的中心在世界坐标系中的坐标。

```
世界坐标系原点 (0,0)
    │
    │       地图 origin (ox, oy)
    │          ┌────────────────────┐
    │          │                    │
    │          │      地图本体      │ height × resolution
    │          │                    │
    │          └────────────────────┘
    │            width × resolution
    │
    └─────────────────────────────────► X
```

> 每个格子的中心坐标为：`world_x = ox + (col + 0.5) * resolution`

这是地图合并时做坐标换算的基础。

---

## 4. 核心算法：空间并集与坐标变换

两个文件中 `merge_maps()` 函数的合并算法前 4 步完全一致。我们以 `merge_map.py` 的在线版本为例进行详解。

### 步骤 1：计算所有地图的包围盒

```python
min_x = min(m.info.origin.position.x for m in maps)
min_y = min(m.info.origin.position.y for m in maps)
max_x = max(m.info.origin.position.x + m.info.width  * m.info.resolution for m in maps)
max_y = max(m.info.origin.position.y + m.info.height * m.info.resolution for m in maps)
```

**原理：** 找出所有输入地图在空间上的最小和最大范围。合并后的地图需要覆盖所有局部地图所覆盖的区域，所以取它们的**空间并集包围盒**。

```
地图A: origin=(0,0),   宽2m, 高2m  →  覆盖 (0,0) 到 (2,2)
地图B: origin=(1.5,1), 宽2m, 高2m  →  覆盖 (1.5,1) 到 (3.5,3)

包围盒: min_x=0, min_y=0, max_x=3.5, max_y=3
```

### 步骤 2：创建合并地图的输出属性

```python
merged_map = OccupancyGrid()
merged_map.header = maps[0].header           # 复用第一个地图的时间戳
merged_map.header.frame_id = frame_id         # 设置输出坐标系
merged_map.info.resolution = min(m.info.resolution for m in maps)  # 取最精细分辨率
merged_map.info.origin.position.x = min_x
merged_map.info.origin.position.y = min_y
```

**原理：**
- 输出地图的 `origin` 设为包围盒左下角，保证包容所有局部地图
- 分辨率取**最小值（最精细）**，避免丢失细节。如果地图A是 0.05m/格、地图B是 0.1m/格，合并后使用 0.05m/格

### 步骤 3：计算合并地图的尺寸

```python
merged_map.info.width  = int(np.ceil((max_x - min_x) / merged_map.info.resolution))
merged_map.info.height = int(np.ceil((max_y - min_y) / merged_map.info.resolution))
merged_map.data = np.full(
    merged_map.info.width * merged_map.info.height, -1, dtype=np.int8
)
```

**原理：**

- 用包围盒尺寸除以每个格子的物理尺寸，得到栅格数量
- `np.ceil` 向上取整，确保不丢失边缘区域
- 所有格子**先初始化为 -1（未知）**

### 步骤 4：将每个局部地图的数据投影到合并网格上

```python
for map_msg in maps:
    for y in range(map_msg.info.height):
        for x in range(map_msg.info.width):
```

对每个输入地图的每个格子，执行坐标转换：

```python
world_x = map_msg.info.origin.position.x + (x + 0.5) * map_msg.info.resolution
world_y = map_msg.info.origin.position.y + (y + 0.5) * map_msg.info.resolution
```

**原理：** 计算该栅格中心在世界坐标系中的位置。`(x + 0.5)` 是因为 `x` 是栅格索引（从 0 开始），栅格中心在 `(x + 0.5) * resolution` 处。

```python
merged_x = int(np.floor((world_x - min_x) / merged_res))
merged_y = int(np.floor((world_y - min_y) / merged_res))
merged_idx = merged_y * merged_w + merged_x
```

**原理：** 将世界坐标映射回合并在网格中的索引。这里用的是 `floor`（向下取整），与前面的 `(x + 0.5)` 的 center-based 计算配合，确保空间对应关系正确。

```
示意图：两个地图投影到合并网格

地图A 格子 → 世界坐标 → 合并网格索引
地图B 格子 → 世界坐标 → 合并网格索引

例如地图B origin(1.5, 1)，格子(0,0) 中心 → world(1.525, 1.025)
→ merged_x = floor((1.525 - 0) / 0.05) = floor(30.5) = 30
```

---

## 5. 合并冲突策略：谁说了算？

当两张地图有重叠区域时，同一个合并网格可能被两张地图的格子"争夺"。这是整个算法的核心差异所在。

### 5.1 在线版本：先到先得（First-Writer-Wins）

在线 `merge_map.py` 中的逻辑：

```python
if merged_map.data[merged_idx] == -1:
    merged_map.data[merged_idx] = map_msg.data[idx]
```

**含义：** 哪个地图的数据先被处理，哪个地图的值就被写入。一旦被写过（不再是 -1），后续地图就不能覆盖它。

**危险场景：** 假设地图A和地图B有重叠区域，地图A标记该区域为 100（障碍物），地图B标记为 0（空闲）。如果 ROS 消息回调中**地图B先到达**，则合并结果中该区域会是 0（空闲）——障碍物被抹掉了！这在导航中可能导致碰撞。

**为什么在线版本会这样做？** 通常在线合并的假设是各机器人建图结果一致，且对重叠区域的处理需求简单。但事实上这是一个设计缺陷。

### 5.2 离线版本：障碍物优先（Obstacle-Priority）

离线 `offline_merge_map.py` 中的逻辑：

```python
if map_msg.data[idx] == 100:                    # ★ 障碍物无条件覆盖
    merged_map.data[merged_idx] = 100
elif merged_map.data[merged_idx] == -1:         # 只有目标格是 -1 才写入
    merged_map.data[merged_idx] = map_msg.data[idx]
```

**含义：** 障碍物（100）**永远赢**——它会覆盖掉已有的任何值。如果当前格子是空闲（0）或未知（-1），且目标格子尚未被障碍物占据（仍为 -1 或 0），才会写入。

这是**安全的合并策略**：宁可多一些障碍物（保守），也绝不丢失已检测到的障碍物。

| 场景 | 地图A | 地图B | 在线结果 | 离线结果 |
|------|-------|-------|---------|----------|
| 都未知 | -1 | -1 | -1 | -1 |
| 空闲 vs 空闲 | 0 | 0 | 0 | 0 |
| 障碍物 vs 空闲 | 100 | 0 | **不确定** (取决于谁的callback先到) | **100** (障碍物安全) |
| 空闲 vs 未知 | 0 | -1 | 0 (如果0先到) | 0 |
| 障碍物 vs 未知 | 100 | -1 | 100 (如果100先到) | 100 |

---

## 6. 源码逐行分析：在线合并 `merge_map.py`

### 6.1 导入

```python
import rclpy                              # ROS2 Python 客户端库
from rclpy.node import Node               # ROS2 节点基类
from nav_msgs.msg import OccupancyGrid    # 占据栅格地图消息
import numpy as np                        # 数值计算
```

### 6.2 `merge_maps(maps, frame_id)` 函数

这是前面第 4 节和第 5.1 节讲述的核心合并函数。

```python
def merge_maps(maps, frame_id):
    if not maps:
        return None
    # ... 包围盒计算 ...
    # ... 创建输出地图 ...
    # ... 先到先得投影 ...
    return merged_map
```

**输入：** `maps` 是一个 `OccupancyGrid` 对象的列表，`frame_id` 是输出地图的坐标系名称。

**输出：** 一张合并后的 `OccupancyGrid`。

### 6.3 `MergeMapNode(Node)` 类

#### __init__ 构造函数

```python
class MergeMapNode(Node):
    def __init__(self):
        super().__init__('merge_map_node')
```

节点名称硬编码为 `merge_map_node`。

```python
        self.declare_parameter('frame_id', 'merge_map')
        self.declare_parameter('robot_count', 3)
        self.frame_id = self.get_parameter('frame_id').get_parameter_value().string_value
        self.robot_count = self.get_parameter('robot_count').get_parameter_value().integer_value
```

从 ROS2 参数服务器读取配置参数。`robot_count` 决定了要监听几个机器人的话题。

```python
        self.maps = [None] * self.robot_count
```

`self.maps` 是一个列表，用于暂存每个机器人最新收到的地图。初始全是 `None`。

#### QoS 配置

```python
        qos = QoSProfile(depth=10, durability=DurabilityPolicy.TRANSIENT_LOCAL,
                         reliability=ReliabilityPolicy.RELIABLE)
```

- **depth=10：** 最多缓存 10 条历史消息
- **TRANSIENT_LOCAL：** 关键！这意味着新订阅者加入时，发布者会自动重发最后一条消息。这保证了 merge node 启动后能立即获得各机器人的地图，不需要等到下一次发布。
- **RELIABLE：** 确保消息不会丢失

#### 发布者

```python
        self.publisher = self.create_publisher(OccupancyGrid, '/' + self.frame_id, qos)
```

输出合并后的地图的话题名是 `/{frame_id}`，默认即 `/merge_map`。同样使用 TRANSIENT_LOCAL，确保新加入的订阅者（如 RViz）能立即看到最新的合并地图。

#### 订阅者

```python
        for i in range(self.robot_count):
            self.create_subscription(OccupancyGrid, f'/tb{i+1}/map',
                lambda msg, idx=i: self.map_callback(msg, idx), qos)
```

订阅话题名为动态生成：`/tb1/map`, `/tb2/map`, `/tb3/map` ...

**注意 lambda 中的 `idx=i` 闭包陷阱：** 如果写成 `lambda msg: self.map_callback(msg, i)`，由于 Python 闭包是延迟绑定的，所有回调最终都会使用循环结束后的 `i` 值。`idx=i` 这个默认参数技巧将当前值"冻结"进了闭包中。

#### map_callback 和 try_merge_and_publish

```python
    def map_callback(self, msg, index):
        self.maps[index] = msg
        self.try_merge_and_publish()
```

每收到一个机器人发来的地图消息，就存入对应位置。

```python
    def try_merge_and_publish(self):
        if all(m is not None for m in self.maps):
            merged = merge_maps(self.maps, self.frame_id)
            self.publisher.publish(merged)
```

**只有当所有机器人的地图都已到达时（all not None），才执行合并并发布。** 如果有一个机器人掉线，合并会被阻塞。

#### main 函数

```python
def main(args=None):
    rclpy.init(args=args)
    node = MergeMapNode()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()
```

标准的 ROS2 节点启动流程。

---

## 7. 源码逐行分析：离线合并 `offline_merge_map.py`

离线版本的结构与在线版本高度镜像，只有两个关键差异：

### 7.1 障碍物优先的 merge_maps

前面第 5.2 节已详细分析。核心区别在这里：

```python
# 离线版本
if map_msg.data[idx] == 100:
    merged_map.data[merged_idx] = 100          # 障碍物永远赢
elif merged_map.data[merged_idx] == -1:
    merged_map.data[merged_idx] = map_msg.data[idx]

# 在线版本
if merged_map.data[merged_idx] == -1:
    merged_map.data[merged_idx] = map_msg.data[idx]   # 先到先得
```

### 7.2 MergeMapNode 完全相同

离线版本的 `MergeMapNode` 类与在线版本**完全一致**——订阅 `/tb{n}/map`、等待全部到达后合并发布。代码重复了。

---

## 8. 启动文件：如何组织系统

### 8.1 在线启动：`merge_map_launch.py`

```python
from launch import LaunchDescription
from launch_ros.actions import Node
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
```

这是 ROS2 launch 系统的标准写法。

```python
def generate_launch_description():
    return LaunchDescription([
        DeclareLaunchArgument('frame_id',    default_value='merge_map'),
        DeclareLaunchArgument('robot_count', default_value='3'),
```

声明启动参数，用户可以通过命令行覆盖：`ros2 launch merge_map merge_map_launch.py robot_count:=4`

```python
        Node(package='rviz2', executable='rviz2',
             arguments=['-d', config_path],
             parameters=[{'use_sim_time': True}]),
```

启动 RViz2 可视化，加载指定的 `.rviz` 配置文件。

```python
        Node(package='merge_map', executable='merge_map',
             parameters=[{
                 'frame_id':    LaunchConfiguration('frame_id'),
                 'robot_count': LaunchConfiguration('robot_count'),
                 'use_sim_time': True,
             }]),
```

启动合并节点，传入参数。

### 8.2 离线启动：`offline_merge_map_launch.py`

离线启动更复杂，因为它需要先通过 `nav2_map_server` 将磁盘上的 YAML 文件加载为 ROS2 话题。

#### 验证输入

```python
    if not os.path.isdir(maps_directory):
        raise RuntimeError(f"MAPS DIRECTORY NOT FOUND")
    map_files = sorted(glob.glob(os.path.join(maps_directory, "*.yaml")))
    if len(map_files) != robot_count:
        raise RuntimeError(f"Expected {robot_count} ...")
```

确保目录存在，且 YAML 文件数量与机器人数量匹配。

#### 为每个地图创建 map_server

```python
    for i, map_file in enumerate(map_files):
        map_server = Node(
            package='nav2_map_server',
            executable='map_server',
            name=f'map_server_{i+1}',
            parameters=[{'yaml_filename': map_file, 'frame_id': frame_id}],
        )
        actions.append(map_server)
```

**`nav2_map_server` 是 Navigation2 提供的标准节点，它会：**
1. 读取 YAML 文件（其中指定了 `.pgm` 图片路径、分辨率、原点等）
2. 将地图数据反序列化为 `OccupancyGrid` 消息
3. 发布到 `/map` 话题

#### 生命周期管理

`map_server` 是生命周期节点（Lifecycle Node），必须经过 `configure` → `activate` 流程才能开始发布地图。Launch 文件中通过定时器命令来实现：

```python
        actions.append(TimerAction(period=2.0, actions=[
            ExecuteProcess(cmd=['ros2', 'lifecycle', 'set', f'map_server_{i+1}', 'configure'])
        ]))
        actions.append(TimerAction(period=5.0, actions=[
            ExecuteProcess(cmd=['ros2', 'lifecycle', 'set', f'map_server_{i+1}', 'activate'])
        ]))
```

- **2 秒后：** 执行 `configure` 状态转换（加载地图数据）
- **5 秒后：** 执行 `activate` 状态转换（开始发布地图话题）

这个延迟是必要的，因为 launcher 创建进程本身需要时间。

#### 启动合并节点

```python
    offline_merge_node = Node(
        package='merge_map',
        executable='offline_merge_map',
        parameters=[{'frame_id': frame_id, 'robot_count': robot_count}],
    )
```

同样等待所有地图到达后合并发布。

---

## 9. RViz 可视化配置

以 `offline_merge_map.rviz` 为例说明。

### 固定参考系

```yaml
Fixed Frame: map
```

RViz 中所有显示的物体都相对于这个坐标系。

### Map 显示 (5 个 Map 图层)

```yaml
- Class: rviz_default_plugins/Map
  Topic: /map                           # 合并后的地图
  Durability Policy: Transient Local     # 与发布者的 QoS 匹配

- Class: rviz_default_plugins/Map
  Topic: /tb1/map                       # 机器人 1 的原始地图
  ...
- Class: rviz_default_plugins/Map
  Topic: /tb2/map                       # 机器人 2 的原始地图
  ...
# ... tb3, tb4 同理
```

通过同时显示各机器人的原始地图和合并后的地图，可以直观地验证合并效果。

### Path 显示（机器人轨迹）

```yaml
- Class: rviz_default_plugins/Path
  Topic: /tb1/visual_path
  Color: yellow

- Class: rviz_default_plugins/Path
  Topic: /tb2/visual_path
  Color: green
```

显示各机器人的历史运动路径，便于分析覆盖范围。

---

## 10. 完整数据流图

### 在线模式

```
                    ┌──────────┐
  机器人1 SLAM ───► │ /tb1/map │ ───────────────────────┐
                    └──────────┘                        │
                    ┌──────────┐                        │   ┌──────────────┐     ┌───────────────┐
  机器人2 SLAM ───► │ /tb2/map │ ───────────────────────┤──►│ MergeMapNode │────►│ /merge_map    │──► RViz2
                    └──────────┘                        │   │              │     │ (合并地图)    │
                    ┌──────────┐                        │   │ merge_maps() │     └───────────────┘
  机器人3 SLAM ───► │ /tb3/map │ ───────────────────────┘   └──────────────┘
                    └──────────┘
                    TRANSIENT_LOCAL QoS （全部话题）
```

### 离线模式

```
                    ┌───────────────────────────────────────────────────────┐
                    │                offline_merge_map_launch.py            │
                    │                                                       │
  map_1.yaml ──► map_server_1 ──► /tb1/map ──┐                             │
                                              │   ┌──────────────────┐     │
  map_2.yaml ──► map_server_2 ──► /tb2/map ──┼──►│ MergeMapNode      │    │
                                              │   │ (offline version) │    │
  map_3.yaml ──► map_server_3 ──► /tb3/map ──┤   └────────┬─────────┘    │
                                              │            │               │
  map_4.yaml ──► map_server_4 ──► /tb4/map ──┘            ▼               │
                                             /map ──────► RViz2            │
                                                                    │
                    ┌──────────┐                                    │
                    │ 生命周期  │  2s: configure                     │
                    │ 管理      │  5s: activate                      │
                    └──────────┘                                    │
                    └──────────────────────────────────────────────────────┘
```

---

## 11. 总结与改进建议

### 核心要点回顾

1. **本质：** 多张占据栅格地图在共享坐标系下的空间拼接
2. **算法：** 包围盒 → 统一分辨率 → 坐标变换 → 冲突解决
3. **在线 vs 离线：** 唯一本质区别在于冲突策略——先到先得 vs 障碍物优先
4. **关键前提：** 所有机器人必须在**同一个世界坐标系**中进行 SLAM（共享 map 原点）

### 潜在改进方向

| 问题 | 说明 | 改进方向 |
|------|------|----------|
| 代码重复 | `MergeMapNode` 在两个文件中完全相同 | 抽取基类或共享模块 |
| 在线合并不安全 | 先到先得可能丢失障碍物 | 统一使用障碍物优先策略 |
| 前提假设强 | 要求机器人共享同一个 map 原点 | 引入 TF2，支持不同坐标系间的地图变换 |
| 无超时/降级 | 一个机器人掉线则永久阻塞 | 增加超时机制，支持部分合并 |
| 无地图保存 | 合并完成后不会自动保存结果 | 集成 nav2_map_saver 自动保存合并后的地图 |

---

> 作者注：本教程基于 `merge_map` 包（MIT License, CHAN JIAN LE）的源码分析编写，旨在帮助理解多机器人地图合并的基本原理与实现方式。
