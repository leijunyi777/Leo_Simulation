import os
from launch import LaunchDescription
from launch.actions import TimerAction
from launch_ros.actions import Node
from ament_index_python.packages import get_package_share_directory

def generate_launch_description():
    package_name = "robot_control_system"
    use_sim_time = True
    common_params = [{"use_sim_time": use_sim_time}]

    pkg_dir = get_package_share_directory(package_name)
    explore_params_file = os.path.join(pkg_dir, "config", "explore_params.yaml")

    # --- 1. 相机节点 ---
    camera_node = Node(
        package=package_name,
        executable="camera_node",
        name="camera_node",
        parameters=common_params,
        output="screen"
    )

    # --- 2. 虚拟机械臂节点 (用于骗过 FSM 的初始化检查) ---
    # 确保在 setup.py 的 console_scripts 中添加了: 'dummy_manipulator = your_pkg.dummy_manipulator:main'
    dummy_manipulator = Node(
        package=package_name,
        executable="dummy_manipulator", 
        name="dummy_manipulator",
        parameters=common_params,
        output="screen"
    )

    # --- 3. 导航节点 ---
    nav_node = Node(
        package=package_name,
        executable="nav_node",
        name="nav_node",
        parameters=common_params,
        output="screen"
    )

    # --- 4. 探索节点 (延迟启动以确保 Nav 准备就绪) ---
    explore_node = Node(
        package="explore_lite",
        executable="explore",
        name="explore_node",
        parameters=[explore_params_file, {"use_sim_time": use_sim_time}],
        output="screen"
    )
    
    delayed_explore_node = TimerAction(
        period=5.0,
        actions=[explore_node]
    )

    # --- 5. FSM 主控节点 ---
    fsm_node = Node(
        package=package_name,
        executable="robot_fsm",
        name="robot_fsm",
        parameters=common_params,
        output="screen"
    )

    return LaunchDescription([
        camera_node,
        dummy_manipulator, # 启动 dummy 节点，FSM 此时能检测到订阅者
        nav_node,
        delayed_explore_node,
        fsm_node
    ])