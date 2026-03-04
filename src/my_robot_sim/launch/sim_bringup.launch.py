import os
from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch_ros.actions import Node
from ament_index_python.packages import get_package_share_directory

def generate_launch_description():
    pkg_dir = get_package_share_directory('my_robot_sim')

    urdf_file = os.path.join(pkg_dir, 'urdf', 'my_robot.urdf')
    world_file = os.path.join(pkg_dir, 'worlds', 'simple.sdf')
    bridge_config = os.path.join(pkg_dir, 'config', 'bridge.yaml')
    # === 新增：获取我们的 SLAM 配置文件路径 ===
    slam_params_file = os.path.join(pkg_dir, 'config', 'mapper_params_online_async.yaml')
    nav2_params_file = os.path.join(pkg_dir, 'config', 'nav2_params.yaml')

    with open(urdf_file, 'r') as infp:
        robot_desc = infp.read()

    # 1. 启动 Gazebo
    gz_sim = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(get_package_share_directory('ros_gz_sim'), 'launch', 'gz_sim.launch.py')
        ),
        launch_arguments={'gz_args': f'-r {world_file}'}.items()
    )

    # 2. 启动 Robot State Publisher
    robot_state_publisher = Node(
        package='robot_state_publisher',
        executable='robot_state_publisher',
        parameters=[{'robot_description': robot_desc, 'use_sim_time': True}]
    )

    # 3. 启动 Joint State Publisher
    joint_state_publisher = Node(
        package='joint_state_publisher',
        executable='joint_state_publisher',
        parameters=[{'use_sim_time': True}]
    )

    # 4. Spawn 机器人
    spawn_robot = Node(
        package='ros_gz_sim',
        executable='create',
        arguments=['-string', robot_desc, '-name', 'my_robot', '-z', '0.2'],
    )

    # 5. ROS-GZ 桥接
    bridge = Node(
        package='ros_gz_bridge',
        executable='parameter_bridge',
        parameters=[{'config_file': bridge_config, 'use_sim_time': True}],
    )

    # 6. TF 静态桥接翻译官
    static_tf_node = Node(
        package='tf2_ros',
        executable='static_transform_publisher',
        arguments=['0', '0', '0', '0', '0', '0', 'laser', 'my_robot/base_link/laser_sensor']
    )

    # ================= 7. 启动 Nav2 (纯导航模式) =================
    # 注意这里使用的是 navigation_launch.py，它不包含 amcl 和 map_server
    nav2_bringup_node = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(get_package_share_directory('nav2_bringup'), 'launch', 'navigation_launch.py')
        ),
        launch_arguments={
            'use_sim_time': 'true',
            'params_file': nav2_params_file
        }.items()
    )

    # 7. 键盘控制
    teleop_node = Node(
        package='teleop_twist_keyboard',
        executable='teleop_twist_keyboard',
        prefix='xterm -e'
    )

    # 8. RViz2
    rviz2_node = Node(
        package='rviz2',
        executable='rviz2',
        parameters=[{'use_sim_time': True}]
    )

    # ================= 新增节点 9：启动 SLAM Toolbox =================
    # 调用 slam_toolbox 的 online_async 模式启动文件
    slam_toolbox_node = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(get_package_share_directory('slam_toolbox'), 'launch', 'online_async_launch.py')
        ),
        launch_arguments={
            'use_sim_time': 'true',
            'slam_params_file': slam_params_file # 传入我们的配置文件
        }.items()
    )
    # ==============================================================

    return LaunchDescription([
        gz_sim,
        robot_state_publisher,
        joint_state_publisher,
        spawn_robot,
        bridge,
        static_tf_node,
        teleop_node,
        rviz2_node,
        slam_toolbox_node, # <--- 别忘了加入返回列表！
        nav2_bringup_node
    ])