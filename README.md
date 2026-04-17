# 🤖 Leo Rover Autonomous Navigation Simulation

本项目是一个基于 **ROS 2 Jazzy** 和 **Gazebo Harmonic** 的移动机器人全栈仿真工作空间。

该项目以 Leo Rover 为底盘模型，搭载 **MyCobot 280** 机械臂及自适应夹爪。系统深度集成了 YOLO 视觉识别、Nav2 自主导航以及基于状态机的任务协调器，实现了在复杂室内环境（SDF 建模）中的“搜索-识别-抓取-转运-放置”全闭环控制。


## 🛠️ 环境依赖 (Dependencies)

### 1. 核心物理仿真与桥梁 (Core Simulation & Bridge)
用于连接 ROS 2 和 Gazebo Harmonic 的关键插件：
```bash
sudo apt install ros-jazzy-ros-gz-sim \
                 ros-jazzy-ros-gz-bridge \
                 ros-jazzy-ros-gz-image \
                 ros-jazzy-ros-gz-interfaces
```

### 2. 导航、建图与定位 (Nav2 & SLAM)
```bash
sudo apt install ros-jazzy-navigation2 \
                 ros-jazzy-nav2-bringup \
                 ros-jazzy-slam-toolbox \
                 ros-jazzy-robot-localization \
                 ros-jazzy-joint-state-publisher \
                 ros-jazzy-robot-state-publisher
```

### 3. 视觉处理与 AI 推理 (Vision & AI)
YOLO 推理及其与 ROS 图像格式转换所需的依赖：
```bash
# 系统级依赖
sudo apt install ros-jazzy-cv-bridge \
                 ros-jazzy-tf2-geometry-msgs \
                 python3-opencv

# AI 推理 (YOLO)
pip install ultralytics
```

### 4. 机械臂控制接口 (Manipulator Support)
用于解析和控制 MyCobot 280 及其夹爪：
```bash
sudo apt install ros-jazzy-xacro \
                 ros-jazzy-joint-state-publisher-gui
```

### 一键安装所有依赖 (The Rosdep Way)
如果你想确保没有遗漏任何 `package.xml` 中定义的细碎依赖，建议在工作空间根目录下使用 `rosdep`：

```bash
cd ~/Leo_Simulation
sudo rosdep init
rosdep update
rosdep install --from-paths src --ignore-src -y
```

## 📁 工作空间结构 (Structure)

```text
Leo_Simulation/src/
├── leo_description/          # 机器人模型包 (包含底盘、MyCobot280臂、D435相机、夹爪)
├── leo_gz_plugins/           # 自定义 Gazebo 差速系统插件
├── my_robot_interfaces/      # 自定义通信接口 (含 ObjectTarget, TESTControlColorStatus.srv等)
├── my_robot_sim/             # 仿真配置中心 (环境 SDF, Nav2/EKF/SLAM 配置文件)
├── robot_control_system/     # 核心控制算法 (YOLO推理、Nav调度、FSM状态机主节点)
└── m-explore-ros2/           # 前沿探索自主建图包 (Explore Lite)
```

## 🚀 快速启动 (Quick Start)

1. **编译工作空间**
   ```bash
   colcon build --symlink-install
   source install/setup.bash
   ```

2. **启动仿真底层 (Gazebo + Nav2 + SLAM)**
   ```bash
   ros2 launch my_robot_sim leo_sim_nav.launch.py
   ```

3. **启动控制节点 (FSM + Vision + Arm Control)**
   ```bash
   ros2 launch robot_control_system system_control.launch.py
   ```

## 🧠 测试逻辑与流程验证 (Testing Logic)

为了高效验证“Pick & Place”逻辑而不依赖随机探索，系统内置了**坐标注入 (Inject)** 模式。

### 1. FSM 状态流转
系统通过 `robot_fsm.py` 维护以下状态：
* **INIT (0)**: 硬件心跳检查。
* **SEARCH (1)**: 开启探索。
* **MOVE_TO_OBJECT (2)**: 导航至物体坐标（注入或视觉发现）。
* **GRASP (3)**: 机械臂执行抓取动作（含视觉闭环校验）。
* **MOVE_TO_BOX (4)**: 携带物体导航至对应颜色的目标盒。
* **DROP (5)**: 放置物体并复位，进入下一循环。

### 2. 注入测试步骤 (Manual Injection Flow)

通过自定义 Service 直接向 FSM 注入上帝视角坐标（SDF 真实坐标），强制触发任务流转：

#### **步骤 A：注入目标坐标（以黄色为例）**
在终端发送注入命令，小车将立即结束搜索并开向 `yellow_object`：
```bash
ros2 service call /test_update_color_status my_robot_interfaces/srv/TESTControlColorStatus "{color: 'yellow', status: 'inject'}"
```
```bash
ros2 service call /test_update_color_status my_robot_interfaces/srv/TESTControlColorStatus "{color: 'purple', status: 'inject'}"
```
```bash
ros2 service call /test_update_color_status my_robot_interfaces/srv/TESTControlColorStatus "{color: 'red', status: 'inject'}"
```
#### **步骤 B：模拟导航到达**
若想跳过等待小车行驶的过程，可手动触发导航到达反馈：
```bash
ros2 topic pub --once /nav/goal_reached std_msgs/msg/Bool "{data: true}"
```

#### **步骤 C：模拟抓取闭环（关键验证）**
当 FSM 终端显示 `Waiting 10 seconds for arm...` 时，利用 Gazebo 服务在物理世界中删除物体，模拟“物体已被抓走”的视觉反馈：
```bash
gz service -s /world/testing_world/remove --reqtype gz.msgs.Entity --reptype gz.msgs.Boolean --timeout 2000 --req 'name: "yellow_object", type: MODEL'
```
```bash
gz service -s /world/testing_world/remove --reqtype gz.msgs.Entity --reptype gz.msgs.Boolean --timeout 2000 --req 'name: "purple_object", type: MODEL'
```
```bash
gz service -s /world/testing_world/remove --reqtype gz.msgs.Entity --reptype gz.msgs.Boolean --timeout 2000 --req 'name: "red_object", type: MODEL'
```
*注：FSM 识别到物体消失后，将自动跳转至 `MOVE_TO_BOX` 状态。*

#### **步骤 D：完成循环**
再次发布 `/nav/goal_reached` 模拟到达彩盒，FSM 将自动完成放置动作并返回 `SEARCH` 状态，`Cycle Count` 加 1。


**附：注入坐标参考 (Testing World Coords)**
* **Red**: Object (-2.0, -1.0) | Box (2.5, 0.5)
* **Yellow**: Object (1.0, -2.0) | Box (3.8, 2.3)
* **Purple**: Object (-3.0, 2.0) | Box (-4.0, -2.4)