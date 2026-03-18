import os
from launch import LaunchDescription
from launch_ros.actions import Node

def generate_launch_description():
    """
    Launch file for the fully automated robot mission (Real Robot / NUC Version).

    Strategy: "Parallel Startup with FSM Handshake"
    -------------------------------------------------
    1. All 4 core NUC nodes are started simultaneously.
    2. The TF tree (map -> odom -> base_link -> camera_link) is expected to be 
       provided externally by the Leo Rover's robot_state_publisher.
    3. The 'robot_fsm' (Main Control) will start in STATE_INIT and actively
       poll for the availability of the other 3 hardware/actuator nodes.
    4. The mission triggers automatically strictly when all systems are online.
    """

    # Package name from setup.py
    package_name = "robot_control_system"
    
    # [CRITICAL UPDATE] Set to False for real robot execution on the NUC.
    # Forces all nodes to use the real hardware system clock.
    common_params = [{"use_sim_time": False}]

    return LaunchDescription(
        [
            # ========================================================================
            # Node 1: Vision System (Perception Layer)
            # ========================================================================
            # Matches 'camera_node' in setup.py console_scripts
            Node(
                package=package_name,
                executable="camera_node",  
                name="camera_node",
                parameters=common_params,
                output="screen",
                emulate_tty=True,
            ),
            # ========================================================================
            # Node 2: Manipulator Controller (Action Layer - Arm)
            # ========================================================================
            # Matches 'manipulator_node' in setup.py console_scripts
            Node(
                package=package_name,
                executable="manipulator_node", 
                name="manipulator_node",
                parameters=common_params,
                output="screen",
                emulate_tty=True,
            ),
            # ========================================================================
            # Node 3: Navigation Wrapper (Action Layer - Base)
            # ========================================================================
            # Matches 'nav_node' in setup.py console_scripts
            Node(
                package=package_name,
                executable="nav_node",  
                name="nav_node",
                parameters=common_params,
                output="screen",
                emulate_tty=True,
            ),
            # ========================================================================
            # Node 4: Main Controller Node (Decision Layer / Brain FSM)
            # ========================================================================
            # Matches 'robot_fsm' in setup.py console_scripts
            Node(
                package=package_name,
                executable="robot_fsm",  
                name="robot_fsm",
                parameters=common_params,
                output="screen",
                emulate_tty=True,
            ),
        ]
    )