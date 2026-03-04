import os
from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch_ros.actions import Node
from ament_index_python.packages import get_package_share_directory
from launch.substitutions import Command # <--- 新增：用于解析 xacro 文件

def generate_launch_description():
    # 获取两个不同包的路径
    sim_pkg_dir = get_package_share_directory('my_robot_sim')
    leo_pkg_dir = get_package_share_directory('leo_description') # 获取 Leo 小车的包路径

    # 配置文件与模型路径
    # 修改点 1：指向 leo_description 包中的 xacro 文件
    urdf_file = os.path.join(leo_pkg_dir, 'urdf', 'leo_sim.urdf.xacro') 
    # 修改点 2：指向我们刚刚创建的新房间
    world_file = os.path.join(sim_pkg_dir, 'worlds', 'simple_room.sdf')  
    
    bridge_config = os.path.join(sim_pkg_dir, 'config', 'bridge.yaml')
    slam_params_file = os.path.join(sim_pkg_dir, 'config', 'mapper_params_online_async.yaml')
    nav2_params_file = os.path.join(sim_pkg_dir, 'config', 'nav2_params.yaml')

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

    # 3. 启动 Joint State Publisher
    joint_state_publisher = Node(
        package='joint_state_publisher',
        executable='joint_state_publisher',
        parameters=[{'use_sim_time': True}]
    )

    # 4. Spawn 机器人
    # 修改点 4：将 -string 替换为 -topic，让 Gazebo 监听 RSP 发布解析好的 urdf
    spawn_robot = Node(
        package='ros_gz_sim',
        executable='create',
        arguments=['-topic', 'robot_description', '-name', 'leo_sim', '-z', '0.5'],
        output='screen'
    )

    # 5. ROS-GZ 桥接
    bridge = Node(
        package='ros_gz_bridge',
        executable='parameter_bridge',
        parameters=[{'config_file': bridge_config, 'use_sim_time': True}],
    )

    # 6. TF 静态桥接翻译官 (根据 Ignition Gazebo 的默认习惯调整)
    static_tf_node = Node(
        package='tf2_ros',
        executable='static_transform_publisher',
        # 将 Gazebo Lidar 内部生成的帧链接到 URDF 中的 laser 帧
        arguments=['0', '0', '0', '0', '0', '0', 'laser', 'leo_sim/base_link/laser_sensor']
    )

    # 7. 启动 Nav2 (纯导航模式)
    nav2_bringup_node = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(get_package_share_directory('nav2_bringup'), 'launch', 'navigation_launch.py')
        ),
        launch_arguments={
            'use_sim_time': 'true',
            'params_file': nav2_params_file
        }.items()
    )

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
        parameters=[{'use_sim_time': True}]
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
        spawn_robot,
        bridge,
        static_tf_node,
        teleop_node,
        rviz2_node,
        slam_toolbox_node,
        nav2_bringup_node
    ])