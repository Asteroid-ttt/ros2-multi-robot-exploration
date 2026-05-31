# Multi-Robot Exploration 教程

## 前言

本教程带你从零开始理解 `multi_robot_exploration` 包——一个实现**基于前沿（frontier-based）的多机器人自主探索**的 ROS 2 (Humble) Python 包。

### 这个项目解决什么问题？

在地图未知的环境中，让一个或多个机器人**自主地、高效地**探索并构建完整地图。

**一个类比：** 想象你在黑暗中用火把探索一个陌生的房间。墙壁是"obstacle"（障碍物），你看得到的地方是"free space"（已知空闲区），你看不到的地方是"unknown"（未知区）。你自然会走向**已知和未知的边界**（frontier，前沿）去照亮新的区域。这个项目就是用代码实现了这个思路——并且协调多个"火把"（机器人）同时探索。

### 这个包在整个系统中的角色

在完整的 `ros2-multi-robot-automap` 工程中：

```
[SLAM Toolbox]  →  每个机器人各自建图 (/tb<i>/map)
        ↓
[merge_map]     →  合并为一张全局地图 (/merge_map)
        ↓
[multi_robot_exploration] ← 你正在学的包！
        |   读取合并地图
        |   检测前沿
        |   为每个机器人分配合适的目标
        |   通过 Nav2 发送导航指令
        ↓
[Nav2]          →  驱动机器人移动到目标点
```

---

## 第 1 章：环境搭建与运行

### 1.1 前置条件

- Ubuntu 22.04
- ROS 2 Humble
- Gazebo
- TurtleBot3 包

### 1.2 构建

```bash
cd ~/ros2-multi-robot-automap
colcon build --cmake-clean-cache --symlink-install --packages-select multi_robot_exploration
source install/setup.bash
```

### 1.3 运行

需要完整的系统环境才能运行。启动整条管线：

```bash
ros2 launch multi_robot gazebo_multirobot_mapping_with_nav2.launch.py robot_count:=2
```

这会启动 Gazebo、SLAM、Nav2、merge_map 和本包的控制节点。控制节点通过以下命令启动（定义在启动文件中）：

```bash
ros2 run multi_robot_exploration control --ros-args -p robot_count:=2
```

### 1.4 只看这个包的入口

如果你只想看源代码是如何启动的，从 `control.py:314` 的 `main()` 函数开始：

```python
def main(args=None):
    rclpy.init(args=args)                          # 初始化 ROS 2
    node = Node("control_node")                     # 创建临时节点读取参数
    node.declare_parameter('robot_count', 2)        # 声明参数
    robot_count = node.get_parameter('robot_count').get_parameter_value().integer_value
    control = HeadquartersControl(robot_count)      # 创建核心控制节点
    rclpy.spin(control)                             # 阻塞运行，处理回调
```

**这段代码做了什么：**

1. 初始化 ROS 2 通信系统
2. 创建一个临时 Node 读取 `robot_count` 参数（默认 2）
3. 用机器人数量创建 `HeadquartersControl` 实例——这是我们的核心节点
4. `rclpy.spin()` 让节点持续运行，处理所有回调

### 1.5 阅读练习

1. 在 `setup.py:21-24` 找到将 `main()` 注册为命令行入口的代码。你能解释 `console_scripts` 的作用吗？
2. 如果你传入 `robot_count:=4`，`HeadquartersControl` 会为哪些机器人创建订阅者？

---

## 第 2 章：前沿检测 — `frontierB`

### 2.1 什么是前沿（Frontier）？

在占用栅格地图中，每个格子有三种状态：
- **-1（未知）：** 还没被传感器探索过的区域
- **0（空闲）：** 确定没有障碍物的区域
- **100（障碍物）：** 被探测到的墙壁、物体

**前沿单元 =** 一个值为 0（空闲）的格子，它的上下左右某个邻居是 -1（未知）。

这是已知空间和未知空间的"前线"——机器人应该去这些地方探索。

### 2.2 代码详解

`control.py:28-40`:

```python
def frontierB(matrix):
    for i in range(len(matrix)):
        for j in range(len(matrix[i])):
            if matrix[i][j] == 0.0:                 # ① 只检查空闲格子
                if i > 0 and matrix[i-1][j] < 0:    # ② 上邻接未知？
                    matrix[i][j] = 2
                elif i < len(matrix)-1 and matrix[i+1][j] < 0:  # 下
                    matrix[i][j] = 2
                elif j > 0 and matrix[i][j-1] < 0:  # 左
                    matrix[i][j] = 2
                elif j < len(matrix[i])-1 and matrix[i][j+1] < 0:  # 右
                    matrix[i][j] = 2
    return matrix
```

**逐行解释：**

| 行 | 含义 |
|----|------|
| ① | 只检查值为 0 的格子（已知空闲区） |
| ② | `matrix[i-1][j] < 0`：上方的格子是未知（-1）的吗？是就把当前格子标记为 2（前沿） |
| ③-⑤ | 同理检查下、左、右三个方向 |

**为什么标记为 2？** 在后续处理中，0 和 2 可以区分"普通空闲区"和"前沿区域"。这是一个简单的状态编码技巧。

### 2.3 可视化示例

假设 3×3 的小地图：

```
输入：
[-1, -1, -1]
[ 0,  0, -1]
[ 0,  100, -1]

处理：
位置 (1,0) 值为 0，上方 (0,0) 是 -1 → 标记为 2
位置 (1,1) 值为 0，上方 (0,1) 是 -1 → 标记为 2
位置 (2,0) 值为 0，上方 (1,0) 不是 -1，但右边 (2,1) 是 100 → 不标记
位置 (2,0) 上方不是 -1 → 不标记

输出：
[-1, -1, -1]
[ 2,  2, -1]
[ 0,  100, -1]
```

### 2.4 练习

1. 在手机上打开 Python 解释器，手动创建一个 3×3 的列表，调用 `frontierB()` 验证你的理解。
2. 为什么函数只检查上下左右四个方向，不检查对角线？这会漏掉什么？属于 bug 还是有意为之？

---

## 第 3 章：前沿分组 — DFS 连通分量

### 3.1 为什么需要分组？

`frontierB` 找到了所有前沿格子，但它们可能分散在地图的不同角落。如果每个格子都是一个候选目标，那就太碎片化了。

更好的做法是：**把连在一起的前沿格子视为一组（group）**，整个组作为一个候选探索目标。

### 3.2 深度优先搜索 (DFS)

`control.py:51-69`:

```python
def dfs(matrix, i, j, group, groups):
    if i < 0 or i >= len(matrix) or j < 0 or j >= len(matrix[0]):
        return group                                    # ① 越界保护
    if matrix[i][j] != 2:
        return group                                    # ② 不是前沿，跳过
    if group in groups:
        groups[group].append((i, j))                    # ③ 记录该格子属于当前组
    else:
        groups[group] = [(i, j)]
    matrix[i][j] = 0                                    # ④ 标记为已访问
    dfs(matrix, i + 1, j, group, groups)                # ⑤ 递归探索 8 个方向
    dfs(matrix, i - 1, j, group, groups)
    dfs(matrix, i, j + 1, group, groups)
    dfs(matrix, i, j - 1, group, groups)
    dfs(matrix, i + 1, j + 1, group, groups)            # 右下对角线
    dfs(matrix, i - 1, j - 1, group, groups)            # 左上对角线
    dfs(matrix, i - 1, j + 1, group, groups)            # 右上对角线
    dfs(matrix, i + 1, j - 1, group, groups)            # 左下对角线
    return group + 1                                    # ⑥ 返回下一组编号
```

**关键是 8 邻域搜索：** 上下左右 + 四个对角线 = 8 方向。而 `frontierB` 只检查 4 方向。这是合理的——前沿检测用 4 方向就够，但分组需要 8 方向来确保真正连接的前沿不被断开。

### 3.3 入口函数

`control.py:42-49`:

```python
def assign_groups(matrix):
    group = 1
    groups = {}
    for i in range(len(matrix)):
        for j in range(len(matrix[0])):
            if matrix[i][j] == 2:
                group = dfs(matrix, i, j, group, groups)  # 对每个前沿启动 DFS
    return matrix, groups
```

**工作流程：**

1. 遍历整个地图
2. 遇到值为 2（前沿）的格子时，启动 DFS
3. DFS 会**吃掉**所有连通的 2，标记为 0
4. 每吃完一组，`group += 1`，准备编号下一组
5. `groups` 字典：`{1: [(x1,y1), (x2,y2), ...], 2: [...], ...}`

### 3.4 过滤大组

`control.py:71-74`:

```python
def fGroups(groups):
    sorted_groups = sorted(groups.items(), key=lambda x: len(x[1]), reverse=True)
    top_five_groups = [g for g in sorted_groups[:5] if len(g[1]) > 2]
    return top_five_groups
```

- 按大小降序排序
- 取前 5 组
- 排除太小（≤2 个格子）的组——可能是噪声

### 3.5 练习

1. 手动绘制一个 4×4 的地图矩阵，找出手动执行 `assign_groups` 后有哪些分组。
2. 为什么 DFS 使用 8 邻域而 `frontierB` 只用 4 邻域？你能想到什么场景下 4 邻域分组会导致问题？
3. `fGroups` 中 `len(g[1]) > 2` 的含义是什么？如果改成 `> 10` 会有什么效果？

---

## 第 4 章：目标选择 — 哪个前沿最值得去？

### 4.1 核心思想

有了 5 个（或更少）候选前沿组，我们需要选一个最优的。判断标准：

1. **组大小：** 越大的前沿组，意味着越多的未知区域等待探索
2. **距离：** 离机器人越近，移动成本越低
3. **是否去过：** 避免重复探索同一个区域

### 4.2 评分函数

`control.py:109-124`:

```python
def findClosestGroup(matrix, groups, current, resolution, originX, originY):
    best_centroid = None
    best_score = float('-inf')
    for i in range(len(groups)):
        middle = calculate_centroid(
            [p[0] for p in groups[i][1]],
            [p[1] for p in groups[i][1]]
        )                                                     # ① 计算组质心
        centroid_world = (
            middle[1] * resolution + originX,
            middle[0] * resolution + originY
        )                                                     # ② 转为世界坐标
        distance = math.sqrt(
            (current[0] - middle[0]) ** 2 +
            (current[1] - middle[1]) ** 2
        )                                                     # ③ 计算距离
        if distance > max_distance:
            continue                                           # ④ 太远，跳过
        if visitedControl(centroid_world) == False:
            group_size = len(groups[i][1])
            score = group_size / (distance + 1e-6)            # ⑤ 评分公式
            if score > best_score:
                best_score = score
                best_centroid = centroid_world
    return best_centroid
```

**评分公式：**

```
score = group_size / (distance + 1e-6)
```

- `group_size`：前沿组大小（越大越好）
- `distance`：格数距离（越小越好）
- `1e-6`：防止除以零
- **直觉：** group_size / distance → 单位距离的"探索收益"。越近越大的组得分越高。

> [!important]
>
> 这计算的是前沿组的质心，但是质心是否符合条件，让小车完成探索，这有待考量。

### 4.3 坐标转换

一个重要细节：栅格坐标 ↔ 世界坐标的转换。

```python
# 栅格 → 世界
world_x = grid_y * resolution + originX
world_y = grid_x * resolution + originY

# 世界 → 栅格 (在 exploration 中)
grid_x = (world_x - originX) / resolution
grid_y = (world_y - originY) / resolution
```

注意：**栅格的 x 对应世界的 y，栅格的 y 对应世界的 x**。这是因为矩阵的行列（i, j）和笛卡尔坐标（x, y）是转置关系。

### 4.4 已访问去重

`control.py:85-92`:

```python
def visitedControl(targetP):
    global VISITED
    for i in range(len(VISITED)):
        k = VISITED[i]
        d = math.sqrt((k[0] - targetP[0]) ** 2 + (k[1] - targetP[1]) ** 2)
        if d < 0.2:
            return True
    return False
```

全局列表 `VISITED` 记录所有已发送过的目标。如果新目标距离已经访问过的某个点在 0.2 米以内，则认为"已访问"，跳过。

> [!important]
>
> 可以添加一个无法访问的标签，用于标记小车无法访问的位置。

### 4.5 练习

1. 如果改成 `score = group_size / (distance ** 2)`，探索行为会发生什么变化？
2. 为什么 `visitedControl` 使用 0.2 米作为阈值？如果改成 5.0 米会怎样？
3. 当前算法没有考虑多个机器人之间的协调（可能两个机器人去同一个前沿）。你认为可以怎么改进？

> [!important]
>
> - 最简单：全局“已分配（ASSIGNED）”集合 + 锁（线程安全）。分配时把目标的网格坐标加入 ASSIGNED，任务完成或超时后移除。其他机器人分配前检查 ASSIGNED。适合快速防冲突，低实现成本。
> - 区域划分（Voronoi/栅格分区）：按机器人当前位置对地图分区，每台机器人只在自己分区内挑选前沿，天然避免重叠。
> - 中央化分配（推荐）：总部节点为所有机器人计算一张代价矩阵（robots × frontiers），代价可为 travel_time / utility，然后用 Hungarian 算法或拍卖算法分配最优匹配，保证全局最优或近似最优。
> - 分布式拍卖：机器人彼此竞价，具备鲁棒性与扩展性，适合去中心化需求。

---

## 第 5 章：探索管道 — `exploration()` 函数

### 5.1 完整管道

`control.py:126-148`:

```python
def exploration(data, width, height, resolution, column, row, originX, originY, choice, last_target=None):
    global VISITED
    f = 1
    data = costmap(data, width, height, resolution)         # ① 膨胀墙壁
    data[row][column] = 0                                   # ② 清除机器人所在格
    data[data > 5] = 1                                      # ③ 障碍物标记为 1
    data = frontierB(data)                                   # ④ 前沿检测
    data, groups = assign_groups(data)                       # ⑤ 前沿分组
    groups = fGroups(groups)                                 # ⑥ 筛选大组

    if len(groups) == 0:                                     # ⑦ 无可用前沿
        f = -1
    else:
        data[data < 0] = 1
        coordinate = findClosestGroup(
            data, groups, (row, column),
            resolution, originX, originY
        )
        if coordinate is None or (
            last_target and
            math.sqrt((coordinate[0] - last_target[0]) ** 2 +
                      (coordinate[1] - last_target[1]) ** 2) < 0.5
        ):                                                    # ⑧ 目标过近或无目标
            f = -1
        else:
            VISITED.append(coordinate)                        # ⑨ 记录已访问
    if f == -1:
        return None                                           # ⑩ 探索完成
    else:
        return coordinate                                     # ⑪ 返回世界坐标目标
```

**步骤详解：**

| 步骤 | 函数 | 作用 |
|------|------|------|
| ① | `costmap` | 膨胀墙壁，增加安全距离 |
| ② | 手动赋值 | 把机器人当前位置强制设为 0（防止被误标记） |
| ③ | 条件赋值 | 把 >5 的值（主要是 100，障碍物）统一标记为 1 |
| ④ | `frontierB` | 检测前沿（0→2） |
| ⑤ | `assign_groups` | DFS 分组 |
| ⑥ | `fGroups` | 取前 5 大组 |
| ⑦ | 检查 | 无前沿 → 返回 None |
| ⑧ | 检查 | 目标不存在 或 和上次目标太近（<0.5m）→ 返回 None |
| ⑨ | 记录 | 将目标加入 VISITED 列表 |
| ⑩ | 完成 | 返回 None 告诉调用方：没有新目标了 |
| ⑪ | 返回 | 返回目标世界坐标 |

> [!important]
>
> 机器人所在格肯定不是一个格子，因此需要将机器人周围一定范围内的格子全部手动设定为 0 。

### 5.2 代价地图膨胀

`control.py:94-107`:

```python
def costmap(data, width, height, resolution):
    data = np.array(data).reshape(height, width)
    wall = np.where(data == 100)
    for i in range(-expansion_size, expansion_size + 1):
        for j in range(-expansion_size, expansion_size + 1):
            if i == 0 and j == 0:
                continue
            x = wall[0] + i
            y = wall[1] + j
            x = np.clip(x, 0, height - 1)
            y = np.clip(y, 0, width - 1)
            data[x, y] = 100
    data = data * resolution
    return data
```

**工作原理：** 找到所有值为 100（障碍物）的格子，把它们周围 `expansion_size`（默认 7）格以内的所有格子也标记为障碍物。这类似于给墙壁"加粗"，让机器人保持安全距离。

### 5.3 练习

1. `data[data > 5] = 1` 这行代码的作用是什么？为什么是 5 而不是别的数？
2. 追踪 `f` 变量的值变化——它在什么条件下保持为 1，什么条件下变成 -1？
3. `data[row][column] = 0` 这步是做什么的？如果删掉，会发生什么？

---

## 第 6 章：ROS 2 节点 — `HeadquartersControl`

### 6.1 节点初始化

`control.py:160-203`:

```python
class HeadquartersControl(Node):
    def __init__(self, num_robots):
        super().__init__("headquarters_control")
        self.shutdown_initiated = False
        self.robots = {}                                    # 未使用

        self.map_sub = self.create_subscription(
            OccupancyGrid, "merge_map", self.map_callback, 10
        )                                                    # ① 订阅合并地图

        self.robot_odom_subs = {}
        self.robot_nav_clients = {}
        self.robot_positions = {}
        self.subscription_cmd_vel = {}

        for i in range(self.num_robots):
            robot_name = f"tb{i + 1}"                        # tb1, tb2, ...
            # ② 订阅每个机器人的里程计
            self.robot_odom_subs[robot_name] = self.create_subscription(
                Odometry, f"{robot_name}/odom",
                lambda msg, r=robot_name: self.robot_odom_callback(msg, r), 10
            )
            # ③ 为每个机器人创建 Nav2 Action 客户端
            self.robot_nav_clients[robot_name] = ActionClient(
                self, NavigateToPose, f"{robot_name}/navigate_to_pose"
            )
            # ④ 订阅 cmd_vel（用于状态监控，当前未使用）
            self.subscription_cmd_vel[robot_name] = self.create_subscription(
                Twist, f"{robot_name}/cmd_vel",
                lambda msg, r=robot_name: self.robot_status_control(msg, r), 4
            )
            self.robot_positions[robot_name] = (0.0, 0.0)

        self.robot_states = {
            f"tb{i+1}": "idle" for i in range(self.num_robots)
        }                                                     # ⑤ 初始状态：全部 idle

        # ⑥ 为每个机器人启动一个独立的探索线程
        for i in range(self.num_robots):
            threading.Thread(
                target=self.start_exploration, args=(i + 1,), daemon=True
            ).start()
```

**ROSTopic / Action 接口一览：**

| 接口 | 方向 | Topic/Action | 用途 |
|------|------|-------------|------|
| `map_sub` | 订阅 | `/merge_map` | 接收合并地图 |
| `robot_odom_subs[tbN]` | 订阅 | `/tbN/odom` | 获取机器人位置 |
| `subscription_cmd_vel[tbN]` | 订阅 | `/tbN/cmd_vel` | 获取速度命令 |
| `robot_nav_clients[tbN]` | Action | `/tbN/navigate_to_pose` | 发送导航目标 |

### 6.2 为什么用多线程？

每个机器人有自己的 `start_exploration` 线程（`control.py:200-201`）。这意味着 N 个机器人有 N 个线程**同时**运行探索逻辑。

多线程是必要的，因为 `start_exploration` 中有 `time.sleep(8)`（每 8 秒检查一次）。如果只有一个线程，机器人必须排队等待——效率极低。

### 6.3 练习

1. 为什么 `robot_odom_callback` 使用 `lambda` 来传递 `robot_name`？不用 lambda 会有什么问题？
2. `daemon=True` 的含义是什么？为什么探索线程被设为守护线程？
3. 在这个设计中，如果一个机器人的 Nav2 Action 服务器还没启动，会发生什么？

---

## 第 7 章：探索循环与目标发送

### 7.1 探索循环

`control.py:221-252`:

```python
def start_exploration(self, robot_number):
    robot_name = f"tb{robot_number}"
    last_target = None
    while rclpy.ok():                                         # ① ROS 2 运行时循环
        if self.map_data is not None:
            robot_position = self.robot_positions[robot_name]
            if self.robot_states[robot_name] == "idle":        # ② 机器人空闲？
                target = exploration(
                    self.map_data,
                    self.map_width, self.map_height,
                    self.resolution,
                    int((robot_position[0] - self.origin[0]) / self.resolution),  # ③ 世界→栅格
                    int((robot_position[1] - self.origin[1]) / self.resolution),
                    self.origin[0], self.origin[1],
                    robot_name,
                    last_target=last_target
                )
                if target:
                    last_target = target
                    self.robot_states[robot_name] = "active"   # ④ 标记为活跃
                    self.send_goal(robot_name, target)          # ⑤ 发送导航目标
        time.sleep(8)                                           # ⑥ 等待 8 秒再检查
    self.check_exploration_completion()                         # ⑦ 循环结束后检查
```

**这是一个状态机：**
```
[idle] ──(有目标)──→ [active] ──(到达目标)──→ [idle]
   ↑                                              │
   └──────────────────────────────────────────────┘
               (失败/取消 也回到 idle)
```

### 7.2 发送导航目标

`control.py:254-265`:

```python
def send_goal(self, robot_name, target):
    if robot_name in self.robot_nav_clients and \
       self.robot_nav_clients[robot_name].wait_for_server(timeout_sec=2.0):
        goal = NavigateToPose.Goal()
        goal.pose.pose.position.x = target[0]                # 目标 x
        goal.pose.pose.position.y = target[1]                # 目标 y
        goal.pose.header.frame_id = "map"                    # 坐标系
        goal.pose.header.stamp = self.get_clock().now().to_msg()  # 时间戳

        future = self.robot_nav_clients[robot_name].send_goal_async(
            goal, feedback_callback=self.feedback_callback
        )
        future.add_done_callback(
            lambda f: self.goal_result_callback(robot_name, f.result())
        )
```

**关键点：**
- `wait_for_server(timeout_sec=2.0)` 等待 Nav2 Action 服务器就绪
- `frame_id = "map"` 表示目标坐标在 map 坐标系中
- 异步发送目标：`send_goal_async` 不阻塞
- 结果回调：机器人到达（或失败）后触发 `goal_result_callback`

### 7.3 结果处理

`control.py:272-281`:

```python
def goal_result_callback(self, robot_name, result):
    if result.status == GoalStatus.STATUS_SUCCEEDED:
        self.get_logger().info(f"{robot_name} reached its goal!")
    self.robot_states[robot_name] = "idle"    # 不管成功失败都回到 idle
    self.check_exploration_completion()
```

**设计决策：** 无论导航成功还是失败，都重新标记为 idle，让探索循环安排下一个目标。这样即使某个目标无法到达，探索也不会因此卡死。

### 7.4 练习

1. `time.sleep(8)` 为什么是 8 秒？如果改成 1 秒或 60 秒会有什么影响？
2. 如果机器人正在前往一个目标时，探索算法算出了一个新的更好的目标，当前设计会怎么处理？
3. `last_target` 参数的作用是什么？它在哪个函数中被使用？

---

## 第 8 章：探索完成与地图保存

### 8.1 完成检测

`control.py:283-291`:

```python
def check_exploration_completion(self):
    if all(state == "idle" for state in self.robot_states.values()) and not self.map_saved:
        self.get_logger().info("All robots are idle. Saving the map.")
        threading.Thread(target=self.save_map).start()
        self.map_saved = True
    elif all(state == "idle" for state in self.robot_states.values()) and self.map_saved:
        self.map_saved = False
```

**触发条件：** 所有机器人状态都是 idle + 还没保存过地图。

### 8.2 保存地图

`control.py:294-311`:

```python
def save_map(self):
    time.sleep(20)                              # 等待稳定
    subprocess.run([
        'ros2', 'run', 'nav2_map_server', 'map_saver_cli',
        '-f', os.path.join(
            get_package_share_directory('multi_robot'),
            '../../../../src/saved_map/multi_robot_autonomous_mapping_of_'
            + str(self.num_robots) + '_robots'
        ),
        '--ros-args',
        '-p', 'map_subscribe_transient_local:=true',
        '-r', '/map:=/merge_map'
    ], check=True)
```

通过调用 ROS 2 的 `map_saver_cli` 工具，把 `/merge_map` topic 上的合并地图保存到文件。

### 8.3 当前策略的局限性

注意：当前的多机器人协调策略非常简单：

1. 所有机器人共享同一张合并地图
2. 每个机器人独立运行 `exploration()` 算法
3. 算法给每个机器人推荐相同的"最优"前沿
4. 通过 `VISITED` 列表避免完全重复——但不同线程之间可能有竞争条件（race condition）
5. 机器人之间没有显式的任务分配（如市场拍卖、匈牙利算法等）

**这是一个验证概念的原型系统**，而不是工业级的协调方案。理解了这一点很重要。

### 8.4 练习

1. 为什么保存前要 `time.sleep(20)`？
2. 如果探索完成了（返回 None），但 `start_exploration` 循环仍在运行——调用方 `exploration()` 返回 None 后，当前线程代码中做了什么？找到具体的处理逻辑。
3. 指出 `VISITED` 列表在多线程环境下的潜在问题。

---

## 第 9 章：完整数据流回顾

### 9.1 从地图到目标的全过程

```
/merge_map (OccupancyGrid 消息)
  │
  ▼
map_callback()                                    [control.py:205]
  │  解析消息 → self.map_data, self.resolution, self.origin
  │
  ▼
start_exploration() 线程循环                       [control.py:221]
  │  每 8 秒检查一次
  │
  ▼
exploration()                                     [control.py:126]
  │  ① costmap()          膨胀墙壁
  │  ② frontierB()        检测前沿
  │  ③ assign_groups()    DFS 分组
  │  ④ fGroups()          筛选大组
  │  ⑤ findClosestGroup() 选择最优目标
  │
  ▼
send_goal()                                       [control.py:254]
  │  通过 NavigateToPose Action 发送目标
  │
  ▼
Nav2 导航栈 驱动机器人移动
  │
  ▼
goal_result_callback()                            [control.py:272]
  │  重置状态 → idle
  │
  ▼
check_exploration_completion()                    [control.py:283]
  │  全部 idle → save_map()
```

### 9.2 时间线

```
t=0s    启动 HeadquartersControl，为每个机器人创建线程
t=0s    开始接收 /merge_map
t=0s    开始接收各机器人 /odom
t=0s    线程 1 进入 start_exploration 循环
t=8s    线程第一次执行 exploration()，如果没有地图数据则跳过
t=...   有地图数据后，开始分配目标
t=...   所有前沿被探索，返回 None
t=...   所有机器人陆续 idle
t=...   触发 save_map，等待 20 秒后保存
```

---

## 第 10 章：重建小版本（迷你项目）

### 阶段 5a：纯 Python 前沿检测（不依赖 ROS 2）

**目标：** 用纯 Python（NumPy）复现 frontier 检测和分组逻辑。

创建文件 `mini_explorer.py`：

```python
import numpy as np

def frontierB(matrix):
    """检测前沿：值为 0 且邻接 -1 的格子标记为 2"""
    result = matrix.copy()
    for i in range(len(result)):
        for j in range(len(result[i])):
            if result[i][j] == 0.0:
                neighbors = []
                if i > 0: neighbors.append(result[i-1][j])
                if i < len(result)-1: neighbors.append(result[i+1][j])
                if j > 0: neighbors.append(result[i][j-1])
                if j < len(result[i])-1: neighbors.append(result[i][j+1])
                if any(n < 0 for n in neighbors):
                    result[i][j] = 2
    return result

# 测试地图
test_map = np.array([
    [-1, -1, -1, -1, -1],
    [-1,  0,  0, -1, -1],
    [-1,  0,100,  0, -1],
    [-1, -1,  0,  0, -1],
    [-1, -1, -1, -1, -1],
])

frontiers = frontierB(test_map)
print("原始地图:")
print(test_map)
print("\n前沿检测结果 (2 = 前沿):")
print(frontiers)
```

### 阶段 5b：添加 DFS 分组

在同一个文件中添加 `dfs` 和 `assign_groups` 函数，参考 `control.py:42-69`。

### 阶段 5c：添加简单目标选择

不依赖 ROS 2 的坐标转换，直接在栅格坐标上实现：选择最大的前沿组的质心作为目标。

### 阶段 5d：集成 ROS 2

创建一个最小 ROS 2 节点：
1. 订阅一个 OccupancyGrid topic
2. 在回调中调用 exploration 管道
3. 打印计算结果

### 阶段 5e：集成 Nav2

使用 ActionClient 发送导航目标。

### 阶段 5f：多机器人扩展

创建多个独立的探索线程，复用同一个地图数据。

---

## 总结

通过本教程，你学习了：

1. **前沿检测** — 找到已知与未知的边界
2. **DFS 连通分量** — 将前沿分组
3. **目标评分** — 按大小/距离权重选择最优目标
4. **代价地图膨胀** — 增加安全距离
5. **ROS 2 集成** — 订阅地图、发送导航目标
6. **多机器人协调** — 多线程、状态机、完成检测

你已经有能力修改这个包，添加新的协调策略，或者从零重建一个类似的系统。