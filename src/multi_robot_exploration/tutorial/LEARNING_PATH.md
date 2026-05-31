# 学习路径 (Learning Path)

## 概述

本文档为 `multi_robot_exploration` 包提供分阶段的学习计划。学完本路径后，你将能够：
- 理解基于前沿（frontier）的自主探索算法
- 解释多机器人探索协调的工作原理
- 运行、修改并重建一个小型版本

---

## 阶段 0：预备知识（先学再看代码）

**预计时间：** 1-2 小时

在阅读本项目的任何代码之前，请确保你理解以下概念：

| 概念 | 为什么需要 | 学习资源 |
|------|-----------|---------|
| ROS 2 基础（Node, Topic, Action） | 整个项目运行在 ROS 2 上 | [ROS 2 Humble Tutorials](https://docs.ros.org/en/humble/Tutorials.html) |
| 占用栅格地图 (OccupancyGrid) | 核心数据结构，地图由此表示 | ROS 2 nav_msgs 文档 |
| Nav2 NavigateToPose Action | 机器人通过此接口移动到目标点 | [Nav2 文档](https://navigation.ros.org/) |
| Python 面向对象 | 控制节点用类实现 | Python 官方教程 |
| NumPy 基础 | 地图数据用 NumPy 数组处理 | NumPy 入门教程 |

**验证你是否准备好了：**
- [ ] 能在终端运行 `ros2 node list` 看到节点列表
- [ ] 理解 Topic 发布/订阅和 Action 客户端/服务器的区别
- [ ] 能解释 OccupancyGrid 中 0, 100, -1 分别代表什么
- [ ] 能用 NumPy 创建和操作二维数组

---

## 阶段 1：项目概览（30 分钟）

**目标：** 了解项目整体结构和它在多机器人系统中的角色。

**阅读清单：**
1. `PROJECT_MAP.md` — 项目结构和文件职责
2. `package.xml` — ROS 2 包清单
3. `setup.py` — 入口点定义

**检查点：** 能画出 multi_robot_exploration 在整个系统中的位置（它接收什么数据、产出什么结果）。

---

## 阶段 2：核心算法（主教程 第 1-4 章）

**预计时间：** 2-3 小时

按顺序学习 `control.py` 中的算法函数：

| 章节 | 内容 | 难度 | 预计时间 |
|------|------|------|---------|
| 第 1 章 | 环境搭建与运行 | ★☆☆☆☆ | 30 min |
| 第 2 章 | 前沿检测 `frontierB` | ★★★☆☆ | 45 min |
| 第 3 章 | 前沿分组 `DFS` + `assign_groups` | ★★★☆☆ | 45 min |
| 第 4 章 | 目标选择 `findClosestGroup` | ★★★☆☆ | 30 min |

**每一步的练习：** 见 `EXERCISES.md` 对应章节。

**检查点：** 能用一个简单的 NumPy 数组模拟地图，手动调用 `exploration()` 函数，观察输出的目标坐标。

---

## 阶段 3：ROS 2 集成（主教程 第 5-7 章）

**预计时间：** 2-3 小时

| 章节 | 内容 | 难度 | 预计时间 |
|------|------|------|---------|
| 第 5 章 | `HeadquartersControl` 节点初始化 | ★★★★☆ | 45 min |
| 第 6 章 | 探索循环 `start_exploration` | ★★★★☆ | 45 min |
| 第 7 章 | 探索完成检测与地图保存 | ★★★☆☆ | 30 min |

**检查点：** 理解从 map_callback → exploration → send_goal → goal_result_callback 的完整数据流。

---

## 阶段 4：进阶主题

**预计时间：** 1-2 小时

| 内容 | 难度 |
|------|------|
| 代价地图膨胀 `costmap` | ★★☆☆☆ |
| 已访问点去重 `visitedControl` | ★★☆☆☆ |
| 多机器人协调策略 | ★★★★☆ |
| 启动文件中的集成配置 | ★★★☆☆ |

---

## 阶段 5：重建小版本（迷你项目）

**预计时间：** 2-4 小时

按照 `EXERCISES.md` 中的"重建路径"，从零开始构建一个简化的单机器人探索系统。

| 阶段 | 目标 |
|------|------|
| 5a | 纯 Python 版 frontier 检测（脱离 ROS 2） |
| 5b | 添加 DFS 连通分量分组 |
| 5c | 添加简单的目标选择逻辑 |
| 5d | 集成 ROS 2 节点订阅地图 Topic |
| 5e | 集成 Nav2 Action 发送导航目标 |
| 5f | 扩展为多机器人版本 |

---

## 完成标准

当你做到以下所有点时，学习结束：

- [ ] 能用自己的话解释 frontier-based exploration 的完整流程
- [ ] 在 code review 中能指出 `control.py` 中每行的作用
- [ ] 能修改探索参数（如 `expansion_size`、`lookahead_distance`）并预测效果
- [ ] 修复了一个由你自己发现的 bug
- [ ] 完成了迷你项目 5f（多机器人探索）
