import os
from launch import LaunchDescription
from launch.actions import TimerAction
from launch_ros.actions import Node
from ament_index_python.packages import get_package_share_directory

def generate_launch_description():
    """
    Launch file for starting FSM, Camera, and Nav immediately, 
    while delaying the Explore Lite node by 5 seconds.
    """
    
    # 你的工作空间包名
    package_name = "robot_control_system"
    
    # [注意] 这里设置为 False 适用于真机，如果是在 Gazebo 仿真环境中运行，请改为 True
    use_sim_time = False 
    common_params = [{"use_sim_time": use_sim_time}]

    # ==========================================
    # 第一批：立即启动的节点 (Immediate Nodes)
    # ==========================================
    
    # 1. 状态机主控节点 (FSM Node)
    fsm_node = Node(
        package=package_name,
        executable="robot_fsm",  
        name="robot_fsm",
        parameters=common_params,
        output="screen",
        emulate_tty=True,
    )

    # 2. 视觉感知节点 (Camera Node)
    camera_node = Node(
        package=package_name,
        executable="camera_node",  
        name="camera_node",
        parameters=common_params,
        output="screen",
        emulate_tty=True,
    )

    # 3. 导航控制节点 (Nav Node)
    nav_node = Node(
        package=package_name,
        executable="nav_node",  
        name="nav_node",
        parameters=common_params,
        output="screen",
        emulate_tty=True,
    )

    # ==========================================
    # 第二批：延迟启动的节点 (Delayed Nodes)
    # ==========================================
    
    # 获取 explore_lite 的配置文件路径
    pkg_dir = get_package_share_directory(package_name)
    explore_params_file = os.path.join(pkg_dir, "config", "explore_params.yaml")

    # 定义探索节点
    explore_node = Node(
        package="explore_lite",
        executable="explore",
        name="explore_node",
        parameters=[explore_params_file, {"use_sim_time": use_sim_time}],
        output="screen",
    )

    # 使用 TimerAction 包装 explore_node，设置 5 秒的延迟 (period=5.0)
    delayed_fsm_node = TimerAction(
        period=5.0,
        actions=[fsm_node]
    )

    # 返回加载描述，将所有节点（包含被 TimerAction 包装的节点）组合起来
    return LaunchDescription(
        [
            fsm_node,
            camera_node,
            nav_node,
            delayed_fsm_node  # 这里的节点会被自动延迟 5 秒后触发
        ]
    )