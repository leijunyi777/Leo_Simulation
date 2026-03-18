import os

from launch import LaunchDescription
from ament_index_python.packages import get_package_share_directory
from launch_ros.actions import Node


def generate_launch_description():
    """
    Launch Nav2 and Explore Lite, reusing the simulation/nav config.

    使用说明：
    - 适用于已经有 TF、/scan、/odom 等基础话题（来自仿真或真实机器人）的场景。
    - 仅负责启动 Nav2 堆栈 和 explore_lite 节点，不启动 Gazebo / SLAM 等。
    - 参数文件复用 `my_robot_sim` 包中的 `explore_params.yaml`。
    """

    # 1. 强制使用仿真时间
    use_sim_time = True
    
    # 定义通用参数，将强制的仿真时间加入其中
    common_params = [{"use_sim_time": use_sim_time}]
    # 替换为你实际存放 nav_node 的包名
    package_name = "robot_control_system" 
    # 复用 my_robot_sim 包中的 Nav2/Explore 参数
    sim_pkg_dir = get_package_share_directory("robot_control_system")
    explore_params_file = os.path.join(sim_pkg_dir, "config", "explore_params.yaml")
    
    # 2. 导航节点 (nav_node)
    nav_node = Node(
        package=package_name,
        executable="nav_node",  
        name="nav_node",
        parameters=common_params,
        output="screen",
        emulate_tty=True,
    )

    # 3. 自动探索节点（explore_lite）
    explore_node = Node(
        package="explore_lite",
        executable="explore",
        name="explore_node",
        # 同时加载 yaml 配置文件和强制开启的仿真时间参数
        parameters=[explore_params_file, {"use_sim_time": use_sim_time}],
        output="screen",
    )


    return LaunchDescription(
        [
            nav_node,
            explore_node
        ]
    )