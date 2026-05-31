# 项目地图 (Project Map)

## 目录结构

```text
multi_robot_exploration/
├── multi_robot_exploration/       # Python 源代码
│   ├── __init__.py                # 空文件，标记为 Python 包
│   └── control.py                 # ★ 核心文件：探索算法 + ROS 2 控制节点
│
├── resource/                      # ROS 2 ament 索引标记
│   └── multi_robot_exploration    # 空标记文件
│
├── test/                          # 自动生成的测试脚手架
│   ├── test_copyright.py          # 版权检查测试（已跳过）
│   ├── test_flake8.py             # 代码风格检查测试
│   └── test_pep257.py             # 文档字符串检查测试
│
├── package.xml                    # ROS 2 包清单（依赖、元数据）
├── setup.py                       # Python 打包和入口点定义
├── setup.cfg                      # setuptools 配置
└── LICENSE                        # MIT 许可证
```

## 文件重要性分级

### ★★★ 必须读懂
- `multi_robot_exploration/control.py` — 全部代码（338 行），包含探索算法和 ROS 2 节点

### ★★☆ 需要了解
- `setup.py` — 定义 ROS 2 命令行入口 `control`
- `package.xml` — 了解依赖关系

### ★☆☆ 可以跳过
- `test/` 目录下的文件 — 自动生成的 linter 测试，代码简单
- `resource/multi_robot_exploration` — ament 索引标记，无实际内容
- `setup.cfg` — 标准 setuptools 配置
- `__init__.py` — 空文件

## 入口点

### 命令行入口

在 `setup.py:21-24` 中定义：

```python
entry_points={
    'console_scripts': [
        'control = multi_robot_exploration.control:main',
    ],
},
```

- **命令：** `ros2 run multi_robot_exploration control`
- **函数：** `main()` at `control.py:314`
- **流程：** main() → 创建临时 Node 读取参数 → 创建 HeadquartersControl → rclpy.spin()

### 启动配置

在外部启动文件 `multi_robot/launch/gazebo_multirobot_mapping_with_nav2.launch.py` 中调用：

```bash
ros2 run multi_robot_exploration control --ros-args -p robot_count:=4
```

参数 `robot_count` 在 `control.py:318` 被读取。

## 数据流概览

```
/merge_map (OccupancyGrid) ──→ map_callback ──→ self.map_data (numpy 数组)
                                                      │
/tb<i>/odom (Odometry) ──→ robot_odom_callback ──→ self.robot_positions
                                                      │
                                                      ▼
                                              start_exploration (循环线程)
                                                      │
                                                      ▼
                                              exploration() 算法
                                                      │
                                                      ▼
                                              send_goal() ──→ /tb<i>/navigate_to_pose (Action)
                                                      │
                                                      ▼
                                              goal_result_callback ──→ check_exploration_completion
                                                      │
                                                      ▼
                                              save_map() ──→ 磁盘文件
```

## 关键数据结构

| 变量 | 类型 | 位置 | 含义 |
|------|------|------|------|
| `self.map_data` | `numpy.ndarray` (2D) | control.py:207 | 合并后的占用栅格地图 |
| `self.robot_positions` | `dict[str, tuple]` | control.py:215 | 机器人名 → (x, y) |
| `self.robot_states` | `dict[str, str]` | control.py:188 | 机器人名 → "idle"/"active" |
| `self.robot_nav_clients` | `dict[str, ActionClient]` | control.py:182 | 机器人名 → Nav2 导航客户端 |
| `VISITED` (全局) | `list[tuple]` | control.py:25 | 已访问的前沿目标世界坐标 |

## 核心算法函数

| 函数 | 行号 | 作用 |
|------|------|------|
| `frontierB(matrix)` | 28-40 | 检测前沿单元（已知空闲与未知区域的边界） |
| `assign_groups(matrix)` | 42-49 | 启动 DFS 分组 |
| `dfs(matrix, i, j, group, groups)` | 51-69 | 深度优先搜索连通分量（8 邻域） |
| `fGroups(groups)` | 71-74 | 筛选前 5 个最大的前沿组 |
| `calculate_centroid(x, y)` | 76-83 | 计算点集的质心 |
| `costmap(data, width, height, resolution)` | 94-107 | 代价地图：膨胀墙壁增加安全距离 |
| `findClosestGroup(...)` | 109-124 | 选择最优前沿目标（大小/距离加权） |
| `exploration(...)` | 126-148 | 主探索管道：串联上述所有步骤 |
| `visitedControl(targetP)` | 85-92 | 检查目标是否已被访问过 |

## 全局配置参数

| 参数 | 默认值 | 行号 | 含义 |
|------|--------|------|------|
| `lookahead_distance` | 0.5 m | 19 | 前瞻距离（当前未使用） |
| `speed` | 0.4 m/s | 20 | 最大速度（当前未使用） |
| `expansion_size` | 7 cells | 21 | 墙壁膨胀半径（格子数） |
| `target_error` | 0.15 m | 22 | 目标点容差（当前未使用） |
| `max_distance` | 150.0 | 23 | 最大探索距离（排除太远的前沿） |

## 外部依赖

| 包 | 用途 |
|----|------|
| `rclpy` | ROS 2 Python 客户端库 |
| `nav_msgs` | OccupancyGrid, Odometry 消息类型 |
| `geometry_msgs` | PoseStamped, Twist, Pose 消息类型 |
| `nav2_msgs` | NavigateToPose Action 定义 |
| `std_msgs` | Float32MultiArray（导入但未使用） |
| `numpy` | 数组运算、地图处理 |
| `scipy.interpolate` | 导入但未使用 |
| `heapq` | 导入但未使用 |
| `ament_index_python` | 查找包共享目录路径 |

> 注意：`lookahead_distance`, `speed`, `target_error`, `scipy`, `heapq`, `Float32MultiArray` 在代码中均未实际使用，可能是预留或遗留导入。
