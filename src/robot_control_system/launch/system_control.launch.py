import os
from launch import LaunchDescription
from launch_ros.actions import Node


def generate_launch_description():
    """
    Launch file for the fully automated robot mission.

    Strategy: "Parallel Startup with FSM Handshake"
    -------------------------------------------------
    1. All nodes are started simultaneously (in parallel).
    2. We do NOT use TimerAction delays (e.g., waiting 5 seconds).
    3. The 'robot_fsm' node will start in STATE_INIT and actively
       poll for the availability of other nodes (camera, nav, arm).
    4. The mission triggers automatically strictly when all hardware
       is reported ready by the FSM.
    """

    # 1. Define the package name variable to avoid hardcoding strings repeatedly.
    #    Ensure this matches the package name in your package.xml and setup.py.
    package_name = "robot_control_system"
    # [MODIFIED] All nodes use the same time source for consistency.
    common_params = [{"use_sim_time": True}]

    return LaunchDescription(
        [
            # ========================================================================
            # Node 1: TF Simulator (Environment / Infrastructure)
            # ========================================================================
            # This node broadcasts the static and dynamic coordinate transformations
            # (Map -> Odom -> Base -> Camera/Arm).
            # It is the "stage" upon which other nodes operate.
            Node(
                package=package_name,  # The package containing the executable
                executable="tf_sim_node",  # The entry point defined in setup.py
                name="tf_simulator",  # Renames the node for runtime clarity
                parameters=common_params,  # Use common parameters for consistency
                output="screen",  # Prints logs to the terminal/console
                emulate_tty=True,
            ),
            # ========================================================================
            # Node 2: Vision System (Perception Layer)
            # ========================================================================
            # Responsible for:
            # - capturing images
            # - detecting objects (Apple/Banana etc.)
            # - broadcasting the 'target_object' TF frame
            Node(
                package=package_name,
                executable="camera_node",
                name="vision_system",
                parameters=common_params,
                output="screen",
                emulate_tty=True,
            ),
            # ========================================================================
            # Node 3: Manipulator Controller (Action Layer - Arm)
            # ========================================================================
            # Responsible for:
            # - Inverse Kinematics (calculating joint angles)
            # - Executing Grasp and Drop sequences
            # - Publishing '/manipulator_feedback' when ready/done
            Node(
                package=package_name,
                executable="manipulator_node",
                name="manipulator_controller",
                parameters=common_params,
                output="screen",
                emulate_tty=True,
            ),
            # ========================================================================
            # Node 4: Navigation Controller (Action Layer - Base)
            # ========================================================================
            # Responsible for:
            # - Interfacing with Nav2 (Navigation Stack)
            # - Moving the chassis to coordinates
            # - Publishing '/move_feedback' when ready/done
            Node(
                package=package_name,
                executable="nav_node",
                name="navigation_controller",
                parameters=common_params,
                output="screen",
                emulate_tty=True,
            ),
            # ========================================================================
            # Node 5: Finite State Machine (Decision Layer / Brain)
            # ========================================================================
            # This is the "Main" node.
            # NOTE: Even if this node starts first, it will stay in STATE_INIT
            # until it detects the publishers from Nodes 2, 3, and 4.
            # This guarantees a safe start without race conditions.
            Node(
                package=package_name,
                executable="robot_fsm",
                name="main_fsm",
                parameters=common_params,
                output="screen",
                emulate_tty=True,
            ),
        ]
    )
