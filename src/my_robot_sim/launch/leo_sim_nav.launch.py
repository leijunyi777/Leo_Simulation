import os
from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription, TimerAction, ExecuteProcess
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch_ros.actions import Node
from ament_index_python.packages import get_package_share_directory
from launch.substitutions import Command # <--- 新增：用于解析 xacro 文件

def generate_launch_description():
    # 获取两个不同包的路径
    sim_pkg_dir = get_package_share_directory('my_robot_sim')
    leo_pkg_dir = get_package_share_directory('leo_description') # 获取 Leo 小车的包路径

    # 配置文件与模型路径
    # 修改点 1：指向 leo_description 包中的 xacro 文件，用于生成 robot_description 话题
    urdf_file = os.path.join(leo_pkg_dir, 'urdf', 'leo_sim.urdf.xacro') 
    # 修改点 2：指向我们刚刚创建的新房间
    world_file = os.path.join(sim_pkg_dir, 'worlds', 'simple_room.sdf')  
    # world_file = os.path.join(sim_pkg_dir, 'worlds', 'testing_world.sdf') 
    #os.path.join: 将多个路径组合成一个路径，用于获取文件路径
    bridge_config = os.path.join(sim_pkg_dir, 'config', 'bridge.yaml')
    slam_params_file = os.path.join(sim_pkg_dir, 'config', 'mapper_params_online_async.yaml')
    nav2_params_file = os.path.join(sim_pkg_dir, 'config', 'nav2_params.yaml') # Nav2 参数文件路径
    explore_params_file = os.path.join(sim_pkg_dir, 'config', 'explore_params.yaml') # 自动探索参数文件路径
    rviz_config_file = os.path.join(sim_pkg_dir, 'rviz', 'sim.rviz')
    custom_bt_path = os.path.join(sim_pkg_dir, 'config', 'navigate_to_pose_w_replanning_and_recovery.xml')
    # ================= 修改点 3：使用 Command 动态解析 xacro =================
    # 以前是直接 read() 文本，现在需要调用系统命令 'xacro' 转换它
    robot_desc = Command(['xacro ', urdf_file])

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

    # 3. 启动 Joint State Publisher（唯一节点名，避免与 sim_bringup 同时运行时重名）
    joint_state_publisher = Node(
        package='joint_state_publisher',
        executable='joint_state_publisher',
        name='joint_state_publisher_nav',
        parameters=[{'use_sim_time': True}]
    )

    # 4. Spawn 机器人
    # 修改点 4：将 -string 替换为 -topic，让 Gazebo 监听 RSP 发布解析好的 urdf
    spawn_robot = Node(
        package='ros_gz_sim',
        executable='create',
        arguments=['-world', 'simple_room','-topic', 'robot_description', '-name', 'leo_sim', '-z', '0.02'],
        output='screen'
    )

    delayed_spawn = TimerAction(
        period=8.0,  # 延迟时间（秒），如果电脑加载 Gazebo 较慢，可以改成 8.0 或 10.0
        actions=[spawn_robot]
    )

    # 使用 ros2 命令行工具，只发送一次 (--once) 全 0 的速度指令
    kickstart_odom = ExecuteProcess(
        cmd=[
            'ros2', 'topic', 'pub', 
            '-t','30',
            '-r','10', 
            '/cmd_vel', 
            'geometry_msgs/msg/Twist', 
            '{linear: {x: 0.001, y: 0.0, z: 0.0}, angular: {x: 0.0, y: 0.0, z: 0.0}}'
        ],
        output='screen'
    )

    # 延迟 8 秒执行（等小车出生并在 Gazebo 里加载稳妥后，再发指令）
    delayed_kickstart = TimerAction(
        period=15.0, 
        actions=[kickstart_odom]
    )

    # 5. ROS-GZ 桥接
    bridge = Node(
        package='ros_gz_bridge',
        executable='parameter_bridge',
        parameters=[{'config_file': bridge_config, 'use_sim_time': True}],
    )

    # 6 四方向车轮过滤：remapping 订阅 /scan、发布 /scan_filtered；
    #     /scan_filtered 被 nav2_params.yaml 与 mapper_params_online_async.yaml（SLAM）使用。
    #     注意：与 sim_bringup 二选一运行，避免重复启动同名节点。
    laser_filter_node = Node(
        package='robot_control_system',
        executable='four_wheel_filter',
        name='four_wheel_filter_nav',
        parameters=[
            {'use_sim_time': True},
            {'min_range': 0.2},
            {'max_range': 12.0},
        ],
        remappings=[
            ('scan', '/scan'),              # 原始雷达（bridge 发布）
            ('scan_filtered', '/scan_filtered'),
        ],
    )


    # 7. 启动 Nav2 (纯导航模式)，延迟 10 秒让 SLAM 先发布 map 再起 Nav2
    nav2_bringup_node = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(get_package_share_directory('nav2_bringup'), 'launch', 'navigation_launch.py')
        ),
        launch_arguments={
            'use_sim_time': 'true',
            'params_file': nav2_params_file,
           'default_nav_to_pose_bt_xml': custom_bt_path # 自定义行为树文件路径
        }.items()
    )
    delayed_nav2 = TimerAction(period=12.0, actions=[nav2_bringup_node])

    # 8. 键盘控制
    teleop_node = Node(
        package='teleop_twist_keyboard',
        executable='teleop_twist_keyboard',
        prefix='xterm -e'
    )

    # 9. RViz2
    rviz2_node = Node(
        package='rviz2',
        executable='rviz2',
        name='rviz2',
        # 使用 arguments 加载配置文件
        arguments=['-d', rviz_config_file],
        # 参数设置
        parameters=[{'use_sim_time': True}],
        output='screen'
    )

    # 10. 启动 SLAM Toolbox
    slam_toolbox_node = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(get_package_share_directory('slam_toolbox'), 'launch', 'online_async_launch.py')
        ),
        launch_arguments={
            'use_sim_time': 'true',
            'slam_params_file': slam_params_file
        }.items()
    )

 
    return LaunchDescription([
        gz_sim,
        robot_state_publisher,
        joint_state_publisher,
        bridge,
        laser_filter_node,
        delayed_spawn,
        delayed_kickstart,
        #teleop_node,
        rviz2_node,
        slam_toolbox_node,
        delayed_nav2,
    ])