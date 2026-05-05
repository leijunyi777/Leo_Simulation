import os
from launch import LaunchDescription
from launch_ros.actions import Node
from moveit_configs_utils import MoveItConfigsBuilder

def generate_launch_description():
    # 自动读取 mycobot_moveit_config 包里所有的 yaml 配置
    moveit_config = MoveItConfigsBuilder("leo_sim", package_name="mycobot_moveit_config").to_moveit_configs()

    # 启动我们的 C++ 节点，并强制注入所有参数和仿真时间
    arm_moveit_node = Node(
        package="mycobot_control",
        executable="arm_moveit_node",
        output="screen",
        parameters=[
            moveit_config.to_dict(),  # <--- 核心魔法：注入运动学、碰撞等所有参数
            {"use_sim_time": True}    # <--- 强制时间同步
        ],
    )

    return LaunchDescription([arm_moveit_node])