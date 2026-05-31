# 练习 (Exercises)

## 阅读练习

### E1: 追踪一个变量的生命周期

**任务：** 追踪 `self.map_data` 变量从"未定义"到"被填充"的完整生命周期。

**步骤：**
1. 在 `__init__` 中找到它的初始值 (control.py:192)
2. 在 `map_callback` 中看它如何被赋值 (control.py:207)
3. 在 `start_exploration` 中找到它如何被使用 (control.py:230)
4. 画出数据流图

**进阶：** 如果多个机器人同时读取 `self.map_data`，会有线程安全问题吗？为什么？

---

### E2: 理解数据预处理

**任务：** 解释 `exploration()` 中 `data[data > 5] = 1` 这行代码的作用。

**步骤：**
1. 查出 OccupancyGrid 中 100 代表什么
2. 思考为什么是 > 5 而不是 == 100
3. 在 NumPy 中试试：`np.array([0, -1, 100, 50]) > 5` 的结果是什么？

**答案方向：** 在地图处理过程中，数值可能因乘法（`data * resolution`）而不再是精确的 100，> 5 提供了一个容错范围。

---

### E3: 分析参数影响

**任务：** 如果 `expansion_size` 从 7 改成 2，探索行为会发生什么变化？

**思考清单：**
- costmap 函数会膨胀多少层？
- 这对 safety（安全性）有什么影响？
- 这对 frontier 检测有什么影响？（膨胀后某些区域可能不再有空闲格子）
- 什么时候可能需要更大的 expansion_size？

---

## 修改练习

### M1: 修改探索频率

**当前行为：** `start_exploration` 每 8 秒检查一次是否需要新目标。

**任务：** 将 `time.sleep(8)` 改为 `time.sleep(2)`，描述可能的效果：
- 正面效果：
- 负面效果：
- CPU 影响：

**验证：** 如果实际上不改代码，你能解释为什么 8 秒是被选中的吗？（提示：考虑机器人从收到目标到开始移动的时间）

---

### M2: 添加探索状态日志

**任务：** 在 `start_exploration` 的 while 循环中添加日志记录，每 10 次循环输出一次：
- 当前机器人名称
- 当前状态（idle/active）
- 当前地图尺寸（width × height）
- 已发现的 frontier 组数量

**提示：**
1. 添加一个计数器 `loop_count`
2. 使用 `self.get_logger().info()` 输出（参考 control.py:249）
3. 尝试运行并观察日志输出

---

### M3: 修改评分函数

**当前评分：** `score = group_size / (distance + 1e-6)`

**任务 1：** 修改为 `score = group_size / (distance ** 2 + 1e-6)`，使距离权重更高。预测：机器人会倾向于探索更近（更小）的前沿还是更远（更大）的前沿？

**任务 2：** 修改为 `score = group_size * group_size / (distance + 1e-6)`，使大小权重更高。预测行为变化。

---

### M4: 实现简单的多机器人协调

**当前问题：** 两个机器人可能被分配到同一个前沿区域。

**任务：** 修改 `exploration()` 或 `start_exploration()`，使机器人 A 收到某个目标后，该目标区域被"预订"（reserve），机器人 B 在选择目标时排除被预订的区域。

**实现思路：**
1. 添加全局 `RESERVED = {}` 字典：`{robot_name: target_coordinate}`
2. 在 `findClosestGroup` 中添加检查：如果目标距离某个已预订目标 < 阈值，则跳过
3. 在 `goal_result_callback` 中清除预订

**设计问题：**
- 谁来管理 `RESERVED`？需要锁吗？
- 如果机器人 A 预订后失败，如何释放？
- 此方案的局限性是什么？

---

### M5: 过滤已探索的前沿

**当前问题：** 如果探索完所有前沿，`exploration()` 返回 None，但 `start_exploration` 线程继续循环。

**任务：** 在 `start_exploration` 中添加逻辑：如果连续 3 次（24 秒）都没产生目标，触发 `check_exploration_completion()` 并退出循环。

---

## 解释练习

### X1: 解释探索终止条件

用自己的话解释：探索在什么条件下终止？请引用 `control.py` 中具体的行号和代码来支持你的回答。

应该解释的路径：
1. exploration() 返回 None (line 146)
2. start_exploration 如何响应 None (line 243-247)
3. goal_result_callback 如何重置状态 (line 276-278)
4. check_exploration_completion 如何触发 save_map (line 283-288)

---

### X2: 解释为什么这个设计适用/不适用于真实机器人

**提示：** 考虑以下方面：
- 线程安全（全局 VISITED 列表）
- 时间假设（sleep(8)、sleep(20)）
- 无路径 cost 考虑（简单欧氏距离）
- 无动态障碍物处理
- 无机器人优先级

---

## 重建练习

### R1: 单机器人探索器（脱离 ROS 2）

**目标：** 创建一个纯 Python 脚本，从文件中加载地图，模拟多步探索过程。

**要求：**
1. 加载一个 OccupancyGrid 数据（用 NumPy 生成模拟地图）
2. 实现 frontierB、DFS、fGroups、findClosestGroup
3. 模拟机器人移动：每选择一个目标，将目标周围 5×5 区域标记为"已探索"
4. 循环直到无前沿可用
5. 输出每一步的目标和地图变化

**文件：** 创建 `exercise_r1.py`

---

### R2: 单机器人 ROS 2 探索器

**目标：** 用 ROS 2 和 Nav2 实现一个单机器人版本的探索节点。

**要求：**
1. 创建 ROS 2 节点，订阅 `/map` topic
2. 实现 exploration 管道
3. 创建 ActionClient 连接到 `navigate_to_pose`
4. 循环分配目标直到完成

**测试：** 在 Gazebo 中启动一个 TurtleBot3 和 SLAM，运行你的节点观察机器人是否自主探索。

---

### R3: 多机器人探索器（最终）

**目标：** 从零重建 `HeadquartersControl`，但添加任务分配的改进。

**要求：**
1. 支持 N 个机器人（从参数读取）
2. 实现任务分配：使用匈牙利算法或拍卖机制避免重复分配
3. 添加 map_saved 后的自动 shutdown
4. 编写基本测试验证 frontierB 和 DFS 的正确性

**文件：** 创建 `exercise_r3.py`
