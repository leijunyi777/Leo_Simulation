import os
from launch import LaunchDescription
from launch.actions import TimerAction, ExecuteProcess
from launch_ros.actions import Node
from ament_index_python.packages import get_package_share_directory

def generate_launch_description():
    """
    Launch file for starting FSM, Camera, and Nav immediately, 
    delaying the Explore Lite node by 5 seconds, 
    and automatically publishing a True signal to start exploration after 10 seconds.
    """
    # 你的工作空间包名
    package_name = "robot_control_system"
    
    # [注意] 这里设置为 False 适用于真机，如果是在 Gazebo 仿真环境中运行，请改为 True
    use_sim_time = True 
    common_params = [{"use_sim_time": use_sim_time}]

    # 1. 导航控制节点 (Nav Node)
    nav_node = Node(
        package=package_name,
        executable="nav_node",  
        name="nav_node",
        parameters=common_params,
        output="screen",
        emulate_tty=True,
    )

    # ==========================================
    # 第二批：延迟启动的节点与动作 (Delayed Actions)
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

    # 【修正】使用 TimerAction 包装 explore_node，延迟 5 秒
    delayed_explore_node = TimerAction(
        period=5.0,
        actions=[explore_node]
    )

    # ==========================================
    # 第三批：延迟发布话题消息触发探索
    # ==========================================
    
    # 定义发布命令行指令：ros2 topic pub --once /nav/cmd_explore std_msgs/msg/Bool "{data: true}"
    trigger_explore_cmd = ExecuteProcess(
        cmd=['ros2', 'topic', 'pub', '--once', '/nav/cmd_explore', 'std_msgs/msg/Bool', '{data: true}'],
        output='screen'
    )

    # 延迟 10 秒发送该指令 (保证在 nav_node 和 explore_node 都已完全启动后执行)
    delayed_trigger_cmd = TimerAction(
        period=10.0,
        actions=[trigger_explore_cmd]
    )

    # 返回加载描述，组合所有节点和定时动作
    return LaunchDescription(
        [
            nav_node,
            delayed_explore_node,  # 5秒后启动 explore_lite
            delayed_trigger_cmd    # 10秒后发布 True 触发探索
        ]
    )