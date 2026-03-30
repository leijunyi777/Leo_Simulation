# README

## Understand the Master Control Code in 10 Minutes: ROS 2 FSM Design & Communication

>This document breaks down the `robot_control_node`. 

### 1. System Architecture & Communication Mechanisms

The Master Control Node acts as the "Brain" (control flow), while other modules act as sensors or actuators (data flow). They communicate entirely asynchronously via **ROS 2 Topics**.

#### 1.1 Node Responsibilities

| Node | Role | Core Responsibility |
| --- | --- | --- |
| **`robot_control_node`** | **Brain** | Runs the core FSM, manages color-based semantic memory, and dispatches global/local coordinates to actuators. |
| **`camera_node`** | **Eyes** | Processes camera frames (e.g., YOLOv8) and outputs 3D coordinates (in the `camera_link` frame) with color and type. |
| **`nav_node`** | **Legs** | Receives `map` coordinates from the Master, plans paths, and navigates the chassis to a safe standoff distance. |
| **`manipulator_node`** | **Arm** | Receives grasp/place coordinates (in `camera_link`) and gripper control signals to execute physical movements. |

#### 1.2 Communication Interfaces (API)

| Topic | Msg Type | Direction | Purpose |
| --- | --- | --- | --- |
| **`/nav/goal_point`** | `PointStamped` | **Out** (Pub) | Sends global target points (`map` frame) to Navigation. |
| **`/nav/cmd_explore`** | `Bool` | **Out** (Pub) | Toggles the chassis random exploration mode (True/False). |
| **`/arm/grasp_pose`** | `CamArmPose` | **Out** (Pub) | Sends relative target points (`camera_link` frame) to the Arm. |
| **`/arm/grasp_status`** | `GripperState` | **Out** (Pub) | Controls the gripper: Open (`True`) or Close (`False`). |
| **`/arm/initial_position`** | `GripperState` | **Out** (Pub) | Commands the arm to retreat to a safe, initial waiting pose. |
| **`/detection_state`** | `Bool` | **In** (Sub) | Vision node heartbeat; confirms the camera is alive and streaming. |
| **`/detected_object`** | `ObjectTarget` | **In** (Sub) | Receives real-time 3D object coordinates from the Vision node. |
| **`/nav/goal_reached`** | `Bool` | **In** (Sub) | Receives success feedback when Navigation reaches the target. |


### 2. Core Variables & Data Structures

Before understanding the logic, you must understand how the robot "remembers" its environment.

#### 2.1 Semantic Memory: `self.map_hash`

This is the robot's core memory bank. It groups data by **Color** and stores the poses for both the "object" and the "box", along with their current FSM status.

| Memory Component | Description |
| --- | --- |
| **`cam_pose`** | Raw coordinates (`x, y, z`) in the local `camera_link` frame. Overwritten continuously. |
| **`map_pose`** | Global coordinates transformed via TF into the `map` frame. |
| **`status`** | `notfound`: Incomplete pair (missing object or box).<br>`paired`: Both found! Ready for action.<br>`deferred`: Grasp failed 5 times; temporarily skipped to prioritize other colors.<br>`solved`: Object successfully dropped in the box. Cycle complete. |

#### 2.2 FSM Tracking Variables
> **Note on State Lifecycle:** To prevent FSM logic collapse caused by "dirty states", these tracking variables are strictly reset to their default values every time the robot transitions to a new state (`transition_to_state`) or completes a full pick-and-place cycle.


| Variable | Purpose |
| --- | --- |
| **`self.camera_working`** | Boolean flag indicating if the vision frame has been received. |
| **`self.vision_msg_tick`** | Global counter. Increments +1 for every vision message received (acts as a system clock). |
| **`self.last_obj_update_tick`** | Records the exact "tick" when the active target object was last seen. |
| **`self.grasp_attempts`** | Tracks consecutive failed grasp attempts for the current object (max 5). |
| **`self.arm_action_done`** | Flag indicating the physical arm movement and delays are fully complete. |

> **Why are these variables necessary?** <br>In an asynchronous ROS 2 environment, the main control loop, sensor callbacks, and hardware timers operate independently. These tracking variables serve as a **shared blackboard** to synchronize these components without blocking the main thread.<br>Instead of using traditional wall-clock timeouts (like `time.sleep()`, which would freeze the FSM), the controller uses `vision_msg_tick` and `last_obj_update_tick` as an **event-driven clock**. <br>By calculating the difference between them, the robot can dynamically verify grasp/drop success based on visual feedback. Meanwhile, `arm_action_done` and `grasp_attempts` act as critical safety guards—ensuring the FSM strictly waits for physical arm movements to finish, while preventing infinite retry loops.


### 3. Finite State Machine (FSM) & Function Logic

The Master Node is driven by a high-frequency timer function (`execute_control_loop` running at 10Hz) that evaluates the current state and triggers transitions.

#### FSM State Breakdown

| FSM State | Triggered Functions | Core Logic & Verification | Transition Condition |
| --- | --- | --- | --- |
| **`[0] INIT`** | `check_hardware_readiness()` | **Hardware Probe:** Verifies if the camera heartbeat is active and if Nav/Arm have active subscribers. | If all ready → `[1] SEARCH` |
| **`[1] SEARCH`** | `execute_control_loop()`<br>`vision_target_callback()` | **Global Patrol:** Enables Nav exploration. Vision callback continually saves `map_pose`. The main loop prioritizes `paired` colors, and only targets `deferred` colors if all others are `solved`. | Found target → `[2] MOVE_TO_OBJECT` |
| **`[2] MOVE_TO_OBJECT`** | `execute_control_loop()`<br>`move_feedback_callback()` | **Approach:** Publishes the target object's `map` coordinates to Navigation continuously. | Nav goal reached → `[3] GRASP` |
| **`[3] GRASP`** | `trigger_arm_action()`<br>`_after_gripper_wait()`<br>`execute_control_loop()` | **Pick Sequence:** <br>1. Open gripper & send `cam_pose`.<br>2. Wait 10s (non-blocking).<br>3. Close gripper & reset arm.<br>**Verification:** <br>If object disappears from vision for 5 frames → Success. <br>If still visible → Retry (max 5 times, then mark `deferred`). | Success → `[4] MOVE_TO_BOX`<br>Fail x5 → `[1] SEARCH` |
| **`[4] MOVE_TO_BOX`** | `execute_control_loop()`<br>`move_feedback_callback()` | **Approach:** Publishes the target box's `map` coordinates to Navigation continuously. | Nav goal reached → `[5] DROP` |
| **`[5] DROP`** | `trigger_arm_action()`<br>`_after_gripper_wait()`<br>`_final_drop_cleanup()` | **Place Sequence:** <br>1. Send `cam_pose` & wait 10s.<br>2. Open gripper to drop & reset arm.<br>3. Wait 3s, then close gripper.<br>**Verification:** <br>Disappears for 5 frames → Success. <br>Mark `solved`, increment `cycle_count`. | Cycle < 3 → `[1] SEARCH`<br>Cycle = 3 → **Shutdown** |
