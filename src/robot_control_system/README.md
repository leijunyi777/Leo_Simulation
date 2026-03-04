# README

This project demonstrates an integrated ROS 2 solution for autonomous mobile manipulation. By decoupling perception, navigation, and state logic, the system achieves a robust search-and-sort capability with high reliability, utilizing modern computer vision and semantic spatial memory to minimize task execution time.

## 1. System Architecture

The system is built as a set of decoupled microservices that communicate via an event-driven signal architecture.

| Node (Entry Point) | Runtime Name | Responsibility |
| :--- | :--- | :--- |
| `robot_fsm` | `robot_control_node` | **The Brain.** Manages the FSM, hardware self-check, and task synchronization. |
| `camera_node` | `camera_node` | **The Eyes.** YOLOv8 inference, 3D coordinate calculation, and grasp verification. |
| `manipulator_node` | `manipulator_node` | **The Arm.** Executes IK-based pick and place. Uses vision feedback for verification. |
| `nav_node` | `nav_node` | **The Legs.** Interfaces with Nav2. Handles random exploration and precision approach. |
| `tf_sim_node` | `tf_sim_node` | **The Stage.** Publishes the TF tree (`map` → `odom` → `base_link` → `camera_link`). |

## 2. Finite State Machine (FSM) Flow

The mission follows a strict event-driven sequence. The robot repeats this cycle **3 times** before automatic shutdown.

### State Transitions

* **INIT**: FSM polls for the presence of Camera, Nav, and Arm node publishers.
* **SEARCH**: Nav node performs random exploration while Vision node looks for a matching Object-Box pair.
* **MOVE_TO_OBJ**: Once a pair is found, Nav node navigates to the locked object coordinates.
* **GRASP**: Manipulator picks the object. It confirms success only if the Vision node reports the object has disappeared (`/vision_grasp_status`).
* **MOVE_TO_BOX**: Nav node navigates to the previously memorized box coordinates.
* **DROP**: Manipulator places the object. Upon completion, a `/status/reset_vision` signal is sent to clear all node caches for the next cycle.

## 3. Communication API (Topic Map)

### Control Signals (FSM Outbound)

| Topic Name | Type | Description |
| :--- | :--- | :--- |
| `/status/moving` | `std_msgs/Bool` | Enables/Disables Navigation movements. |
| `/status/going_to_box`| `std_msgs/Bool` | Toggles destination (False: Object, True: Box). |
| `/status/arm_active` | `std_msgs/Bool` | Triggers the physical Arm sequence. |
| `/status/reset_vision`| `std_msgs/Bool` | Resets memory in Vision/Nav nodes for a new cycle. |

### Feedback & Perception (Inbound)

| Topic Name | Type | Description |
| :--- | :--- | :--- |
| `/detected_pair` | `std_msgs/Bool` | True when Vision finds a matching color pair. |
| `/target_object_pose` | `geometry_msgs/PoseStamped` | Locked global coordinates of the target object. |
| `/target_box_pose` | `geometry_msgs/PoseStamped` | Locked global coordinates of the target box. |
| `/vision_grasp_status`| `std_msgs/Bool` | True if the object is no longer seen (Grasp Success). |
| `/move_feedback` | `std_msgs/Bool` | Navigation arrival confirmation. |
| `/manipulator_feedback`| `std_msgs/Bool` | Arm sequence completion confirmation. |

## 4. Installation & Setup

### Prerequisites

* **ROS 2 Jazzy** (Ubuntu 24.04 recommended)
* **Python libraries**: `ultralytics` (YOLOv8), `numpy`, `opencv-python`
* **Custom Interfaces**: Build `my_robot_interfaces` first.

### Build Instructions

```bash
# Clone and build
cd ~/ros2_ws
colcon build --packages-select my_robot_interfaces robot_control_system
source install/setup.bash
```

## 5. Launching & Testing

### Launching the System

```bash
ros2 launch robot_control_system automated_mission.launch.py
```

### Manual Verification (Simulation)

If hardware is not available, you can simulate signals via terminal to verify the FSM transitions:

```bash
# 1. Simulate Object Found (Triggers MOVE_TO_OBJ)
ros2 topic pub /detected_pair std_msgs/msg/Bool "{data: true}" --once

# 2. Simulate Navigation Arrival (Triggers GRASP or DROP)
ros2 topic pub /move_feedback std_msgs/msg/Bool "{data: true}" --once

# 3. Simulate Grasp Verification (Confirms object is picked)
ros2 topic pub /vision_grasp_status std_msgs/msg/Bool "{data: true}" --once
ros2 topic pub /manipulator_feedback std_msgs/msg/Bool "{data: true}" --once
```

## 6. Project Structure

```text
.
├── my_robot_interfaces          # Custom ROS 2 interfaces
│   ├── msg/
│   │   └── ErrorStatus.msg      # Emergency/System error definitions
│   └── CMakeLists.txt
└── robot_control_system         # Main logic package
    ├── robot_control_system/    # Python source files
    │   ├── best.pt              # YOLOv8 Weights (Trained)
    │   ├── camera_node.py       # Perception & Verification
    │   ├── robot_fsm.py         # State machine control (Brain)
    │   ├── nav_node.py          # Nav2 interface & memory
    │   ├── manipulator_node.py  # Arm execution logic
    │   └── tf_sim_node.py       # TF Tree Simulator
    ├── launch/
    │   └── automated_mission.launch.py
    ├── setup.py
    └── package.xml
```

## 7. Troubleshooting

* **TF Lookup Failures**: Ensure `tf_sim_node` is running and `base_link` to `camera_link` transforms are broadcasted.
* **YOLO Latency**: If the frame rate is low on CPU, consider using the `yolov8n.pt` model for faster inference.
* **Grasp Verification**: If the arm gets stuck after the pick action, verify the camera node is publishing correctly to `/vision_grasp_status`.
* **Nav2 Initialization**: Ensure the robot is localized in the map before the FSM enters the `SEARCH` state.
