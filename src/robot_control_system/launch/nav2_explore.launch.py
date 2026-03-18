import os

from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription, DeclareLaunchArgument
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration
from ament_index_python.packages import get_package_share_directory
from launch_ros.actions import Node


def generate_launch_description():
    """
    Launch Nav2 and Explore Lite, reusing the simulation/nav config.

    使用说明：
    - 适用于已经有 TF、/scan、/odom 等基础话题（来自仿真或真实机器人）的场景。
    - 仅负责启动 Nav2 堆栈 和 explore_lite 节点，不启动 Gazebo / SLAM 等。
    - 参数文件复用 `my_robot_sim` 包中的 `nav2_params.yaml` 与 `explore_params.yaml`。
    """

    # 是否使用仿真时间，默认 false（真机）。如在 Gazebo 中使用，可设为 true：
    #   ros2 launch robot_control_system nav2_explore.launch.py use_sim_time:=true
    use_sim_time_arg = DeclareLaunchArgument(
        "use_sim_time",
        default_value="true",
        description="Use simulation (Gazebo) clock if true.",
    )
    use_sim_time = LaunchConfiguration("use_sim_time")

    # 复用 my_robot_sim 包中的 Nav2/Explore 参数
    sim_pkg_dir = get_package_share_directory("my_robot_sim")
    nav2_params_file = os.path.join(sim_pkg_dir, "config", "nav2_params.yaml")
    explore_params_file = os.path.join(sim_pkg_dir, "config", "explore_params.yaml")
    custom_bt_path = os.path.join(
        sim_pkg_dir, "config", "navigate_to_pose_w_replanning_and_recovery.xml"
    )

    # 1. Nav2 bringup（导航堆栈）
    nav2_bringup = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(
                get_package_share_directory("nav2_bringup"),
                "launch",
                "navigation_launch.py",
            )
        ),
        launch_arguments={
            "use_sim_time": use_sim_time,
            "params_file": nav2_params_file,
            "default_nav_to_pose_bt_xml": custom_bt_path,
        }.items(),
    )

    # 2. 自动探索（explore_lite）
    explore_node = Node(
        package="explore_lite",
        executable="explore",
        name="explore_node",
        parameters=[explore_params_file, {"use_sim_time": use_sim_time}],
        output="screen",
    )

    return LaunchDescription(
        [
            use_sim_time_arg,
            nav2_bringup,
            explore_node,
        ]
    )

