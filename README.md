
# 🤖 Leo Rover Autonomous Navigation Simulation

本项目是一个基于 **ROS 2 Jazzy** 和 **Gazebo Harmonic** 的移动机器人全栈仿真工作空间。核心模型基于 Leo Rover（四轮滑移转向底盘），深度集成了 SLAM Toolbox（实时建图）、Nav2（自主导航）以及基于状态机的导航控制微服务，实现了在室内环境中的自主探索、目标抓取与放置的闭环控制。

## 🛠️ 环境依赖 (Dependencies)

本项目在以下环境中开发与测试：
* **OS:** Ubuntu 24.04 LTS
* **ROS 2:** Jazzy Jalisco
* **Gazebo:** Harmonic (Ignition v8)

请确保系统已安装以下 ROS 2 官方核心组件：
```bash
sudo apt update
sudo apt install ros-jazzy-xacro ros-jazzy-ros-gz
sudo apt install ros-jazzy-navigation2 ros-jazzy-nav2-bringup
sudo apt install ros-jazzy-slam-toolbox
sudo apt install ros-jazzy-teleop-twist-keyboard

```

## 📁 工作空间结构 (Workspace Structure)

```text
Leo_Simulation/
├── README.md
└── src/
    ├── leo_description/        # 机器人物理与视觉描述包
    │   ├── models/             # 3D Mesh 模型文件
    │   └── urdf/
    │       ├── macros.xacro    # 核心宏定义 (包含高频雷达与底盘虚拟轴距补偿)
    │       └── leo_sim.urdf.xacro # 仿真总入口模型
    │
    └── my_robot_sim/           # 仿真控制、导航与启动包
        ├── launch/
        │   └── leo_sim_nav.launch.py # 一键启动文件 (含 Gazebo, RViz, SLAM, Nav2)
        ├── worlds/
        │   └── simple_room.sdf # 自定义 4x4m 室内环境
        ├── config/
        │   ├── nav2_params.yaml             # Nav2 参数 (RPP 曲线优化/AEB配置)
        │   └── mapper_params_online_async.yaml # SLAM 优化参数
       

```

## 🚀 快速启动 (Quick Start)

### 1. 编译工作空间

```bash
cd ~/Leo_Simulation
colcon build --symlink-install
source install/setup.bash

```

### 2. 启动核心联合仿真

一键拉起物理环境、机器人、建图与导航栈：

```bash
ros2 launch my_robot_sim leo_sim_nav.launch.py

```

## 🌟 核心特性与优化总结

本项目针对四轮滑移转向（Skid-Steering）机器人和 Gazebo 仿真环境进行了深度排雷与定制优化：

1. **彻底解决地图漂移与重影**：通过在 URDF 中增大底盘等效轮距、提升雷达扫描频率（20Hz / 720 samples），并压榨 SLAM Toolbox 的最小更新阈值（`0.05 rad`），消除了原地打转带来的累计误差。
2. **RPP 规划器“丝滑过弯”**：关闭了 Nav2 RPP 局部规划器的 `use_rotate_to_heading`，强迫小车采用平滑弧线过弯，避免了原地搓地导致的 TF 树震荡。
3. **消除雷达贴墙盲区**：将 GPU Lidar 的 `min_range` 从默认的 0.35m 下调至 0.10m，防止近距离贴墙时代价地图被意外擦除导致的车体撞击。
4. **统一 Base Frame 高度**：强制将 SLAM 与 Nav2 的参考坐标系设为 `base_footprint` (Z=0)，并设定生成高度为 `Z=0.02`，完美解决 RViz 中地图生成在车顶的“高空抛物”问题。


## m-explore 探索节点：逻辑与本仓库改动

探索节点源码位于 `src/m-explore-ros2/explore/src/explore.cpp`。相对上游 **explore_lite**（m-explore ROS2），本仓库在**结束条件、防卡死、黑名单与目标切换**上做了定制，便于在仿真/实车中与 Nav2 和上层 FSM 协同。实现细节以源码为准；修改 `explore.cpp` 时请同步更新该文件顶部的中文说明块。

### 总体流程

- 节点启动后连接 Nav2 的 `navigate_to_pose` Action，按 `planner_frequency` 定时调用 `makePlan()`。
- 在全局代价地图上搜索 **frontier**，选取代价最优且不在黑名单的目标发给 Nav2；到达或失败后由 `reachedGoal()` 再次触发规划（与原版思路一致）。
- **`explore/status`**（`explore_lite_msgs/ExploreStatus`，QoS 为 `transient_local`）：启动即 `EXPLORATION_STARTED`；正常扫完或熔断结束为 `EXPLORATION_COMPLETE`；手动 `stop()` 为 `EXPLORATION_PAUSED`；`resume()` 为 `EXPLORATION_IN_PROGRESS` 等；若开启回原点，另有 `RETURNING_TO_ORIGIN` / `RETURNED_TO_ORIGIN`。
- **`explore/resume`**（`std_msgs/Bool`）：`true` 调用 `resume()`，`false` 调用 `stop()`（与 README 下文 `/nav/cmd_explore` 联动时，上层通常会向该话题发指令）。

### 探索“进度”指标（非 SLAM 官方覆盖率）

在全局代价地图栅格上统计：`ratio = free_cells / (free_cells + unknown_cells)`（自由格与未知格之和为分母）。

- **启动后约 30 s** 视为预热：若分母为 0 仅打日志，避免空图误触发结束。
- 预热期内会节流打印“探索预热中…”；预热后约每 **5 s** 打印一次当前 `ratio`（百分比）。

### 面积 / 停滞熔断（`makePlan` 前半段）

- 预热后若 **ratio 几乎不变**（相邻周期变化绝对值 ≤ **0.0001**）持续过久：  
  - `ratio < 85%` 时阈值 **40 s**；  
  - `ratio ≥ 85%` 时阈值 **15 s**。  
  超时视为扩图停滞 → 发布 `EXPLORATION_COMPLETE` 并 `stop(true)`。
- **`ratio > 95%`** → 视为探索率达标 → 同样 `EXPLORATION_COMPLETE` 并 `stop(true)`。

### 物理防卡死看门狗（取得机器人位姿之后）

- 若 **10 s** 内位移不足 **0.15 m**，且此前确实发过目标（`prev_goal_` 非全零）：判定顶墙/卡住 → 当前目标入黑名单、`async_cancel_all_goals()`、**清空 `prev_goal_` 与 `prev_goal_cost_`**（避免与决策逻辑冲突），本周期直接 `return`；下一周期会改选其它 frontier。  
- 源码注释中若出现“20 秒”等字样，以 **10.0 s / 0.15 m** 为准。

### 黑名单与“扫盲”重试

- **`frontier_blacklist_`**：导航 **ABORT**、**progress_timeout** 无进展、看门狗拉黑等会加入。
- **`goalOnBlacklist()`**：在代价地图分辨率下，以 **5 格**（`tolerace`，变量名如此拼写）为容差判断目标是否与黑名单点近邻。
- **无 frontier**：若黑名单非空，最多 **清空黑名单重试 3 次**；若所有 frontier 都在黑名单，最多再清空 **1 次**。成功找到可用 frontier 后重试计数归零。

### 决策粘性（减少路口反复换目标）

若新最优 frontier 与上一目标不同，但代价与 `prev_goal_cost_` 相差不到约 **25%**，本周期**不发新 goal**，让 Nav2 继续执行原目标。

### 与原版一致的进展超时

- 若离当前目标更近会刷新 `last_progress_`；若超过参数 **`progress_timeout`**（默认 30 s）仍无进展且非 `resuming_`，将当前目标拉黑并**递归** `makePlan()`。
- 若本周期判定与上一目标 **same_point**（约 **1 cm** 内），直接 `return`，不打断 Nav2。

### 结束与回原点

- **`stop(finished_exploring)`**：`finished_exploring == true` 时不发 `PAUSED`（仅取消定时器与 goal）。
- 若参数 **`return_to_init`** 为 true 且探索**正常结束**（`finished_exploring`），会再发 `RETURNING_TO_ORIGIN` 并导航回启动时记录的位姿。

### 常用参数（`explore` 节点）

| 参数 | 含义 |
|------|------|
| `planner_frequency` | 规划频率（Hz），默认 1.0 |
| `progress_timeout` | 向目标无进展超时（秒），默认 30.0 |
| `return_to_init` | 探索结束后是否回启动位姿 |
| `min_frontier_size`、`potential_scale`、`gain_scale`、`orientation_scale` | frontier 搜索与代价权重（与上游一致） |
| `visualize` | 是否在 RViz 发布 frontier 标记 |

---


## 调试前提准备

在开始单独测试 `nav_node` 之前，请确保以下基础组件已经运行：
1. **Nav2 与 地图服务器**：保证 `/map` 话题有数据，且 Nav2 的 Action Server (`Maps_to_pose`) 已就绪。
2. **TF 树**：存在 `map` -> `base_footprint` 的变换。
3. **启动被测节点**：
```bash
ros2 launch robot_control_system nav2_explore.launch.py
   
```
## 模拟状态机切换：开启 / 停止探索
FSM 的 SEARCH 状态通过 /nav/cmd_explore 话题控制底层探索的启停。

* **开启探索 (模拟进入 SEARCH 状态)**
发送 True，节点将重置周期状态为 OBJECT，并向 m-explore 发送允许探索指令（或在 m-explore 不活跃时自动下发随机坐标点）。

```bash
ros2 topic pub -1 /nav/cmd_explore std_msgs/msg/Bool "{data: true}"
   
```
预期现象：终端打印“cmd_explore: True”，底盘开始随机游走或交由 m-explore 控制。

* **停止探索 / 紧急刹车**
发送 False，节点将立即取消当前的导航目标，底盘急刹车，并持续压制 m-explore。
```bash
ros2 topic pub -1 /nav/cmd_explore std_msgs/msg/Bool "{data: false}"
   
```
## 模拟抓取导航 (GOTO_OBJECT 阶段)
假设视觉系统发现了目标，FSM 会先停止探索，然后下发物体坐标。由于上一步开启探索时节点已将阶段重置为 OBJECT，此次发点将触发物体避让导航逻辑。

* **下发物体坐标 (模拟 MOVE_TO_OBJECT 状态)**
假设物体在地图坐标 (x: 2.0, y: 1.0)：

```bash
ros2 topic pub -1 /nav/goal_point geometry_msgs/msg/PointStamped "{header: {frame_id: 'map'}, point: {x: 2.0, y: 1.0, z: 0.0}}"
   
```
预期现象：

终端打印“【前往抓取】：计算距离目标 370mm 处的停靠点”。

机器人不会开到 (2.0, 1.0)，而是开到距离该点 370mm 的位置，并且车头正对该点。

到达后，监听终端会收到 /nav/goal_reached: True。

内部状态变化：节点自动将追踪器翻转为 BOX 阶段。

* **模拟机械臂反馈循环 (GRASP / DROP 阶段)**
在执行完 GOTO 任务后，nav_node 会自动锁定探索功能，等待机械臂完成动作。我们需要模拟机械臂的“抓取”和“复位”反馈。

模拟机械臂动作完成
向 /manipulator_feedback 连续发送 2 次 True（代表机械臂完成了一组动作）。
```bash
ros2 topic pub -1 /manipulator_feedback std_msgs/msg/Bool "{data: true}"
   
```
连发两次后预期现象：
终端打印“机械臂一轮完成 -> 恢复 m-explore.”（前提是 /nav/cmd_explore 依然为 True，否则会等它变为 True 才恢复）。

##  模拟放置导航 (GOTO_BOX 分段阶段)
由于刚才的抓取导航已经让内部状态变为了 BOX 阶段，此时再发送坐标，将触发分段放置逻辑（先回原点，再去箱子前方 370mm 处）。

* **下发箱子坐标 (模拟 MOVE_TO_BOX 状态)**
假设箱子在地图坐标 (x: -1.0, y: 3.0)：
```bash
ros2 topic pub -1 /nav/goal_point geometry_msgs/msg/PointStamped "{header: {frame_id: 'map'}, point: {x: -1.0, y: 3.0, z: 0.0}}"
   
```
预期现象：

第一阶段：终端打印“【前往放置-阶段1】：先返回(0,0)...”。机器人无视传入的坐标，直接向坐标原点 (0.0, 0.0) 行驶，到达后朝向与初始方向相反。

第二阶段：到达原点后，终端打印“【前往放置-阶段2】：已到达(0,0)，现在前往箱子前 370mm 的避让点”。机器人重新规划路线，前往箱子 (-1.0, 3.0) 前方 370mm 处，车头正对箱子。

完成：到达最终避让点后，监听终端再次收到 /nav/goal_reached: True。内部状态重置。

## 调试进阶：监听内部状态交互
如果你想确保 nav_node 完美地压制或唤醒了 m-explore，可以新开一个终端监听恢复话题：
```bash
ros2 topic echo /explore/resume
   
```
当你在步骤 2 (下发坐标) 时，你应该能看到此话题迅速发布了一次 False（打断探索）。

当你在步骤 3 (模拟两次机械臂反馈) 后，如果宏观探索开关是开着的，你应该能看到此话题发布了 True（恢复探索）。
