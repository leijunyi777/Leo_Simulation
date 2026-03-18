
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

*💡 注：Launch 文件内嵌了自动唤醒机制（启动 10 秒后注入 1mm/s 的微小速度），完美解决了 Gazebo 差速插件静止时不发布 Odom 导致的地图死锁问题。*

### 3. 启动导航执行引擎 (微服务)

在新终端中运行你编写的非阻塞 Action Client 节点：

```bash
source install/setup.bash
ros2 run robot_control_system nav_controller_node

```

## 🎮 导航微服务 API 接口说明

`nav_controller_node` 是一个独立的空间执行引擎，完全通过 ROS 2 Topic 接收外部状态机（大脑）的指令并反馈进度。你可以通过以下命令在终端进行接口联调：

* **1. 启动随机探索 (EXPLORING)**
```bash
ros2 topic pub -1 /status/moving std_msgs/msg/Bool "{data: true}"

```


* **2. 发现目标物体，下发抓取坐标 (APPROACHING_OBJ)**
```bash
ros2 topic pub -1 /target_object_pose geometry_msgs/msg/PoseStamped "{header: {frame_id: 'map'}, pose: {position: {x: 1.5, y: 0.5, z: 0.0}, orientation: {w: 1.0}}}"

```


* **3. 抓取完成，前往放置盒 (APPROACHING_BOX)**
```bash
ros2 topic pub -1 /status/going_to_box std_msgs/msg/Bool "{data: true}"
ros2 topic pub -1 /target_box_pose geometry_msgs/msg/PoseStamped "{header: {frame_id: 'map'}, pose: {position: {x: -1.0, y: -2.0, z: 0.0}, orientation: {w: 1.0}}}"

```


* **4. 紧急制动 / 中止任务 (HALT)**
```bash
ros2 topic pub -1 /status/moving std_msgs/msg/Bool "{data: false}"

```


* **5. 监听到达反馈 (Feedback)**
```bash
ros2 topic echo /move_feedback

```



## 🌟 核心特性与优化总结

本项目针对四轮滑移转向（Skid-Steering）机器人和 Gazebo 仿真环境进行了深度排雷与定制优化：

1. **彻底解决地图漂移与重影**：通过在 URDF 中增大底盘等效轮距、提升雷达扫描频率（20Hz / 720 samples），并压榨 SLAM Toolbox 的最小更新阈值（`0.05 rad`），消除了原地打转带来的累计误差。
2. **RPP 规划器“丝滑过弯”**：关闭了 Nav2 RPP 局部规划器的 `use_rotate_to_heading`，强迫小车采用平滑弧线过弯，避免了原地搓地导致的 TF 树震荡。
3. **消除雷达贴墙盲区**：将 GPU Lidar 的 `min_range` 从默认的 0.35m 下调至 0.10m，防止近距离贴墙时代价地图被意外擦除导致的车体撞击。
4. **统一 Base Frame 高度**：强制将 SLAM 与 Nav2 的参考坐标系设为 `base_footprint` (Z=0)，并设定生成高度为 `Z=0.02`，完美解决 RViz 中地图生成在车顶的“高空抛物”问题。

```

```