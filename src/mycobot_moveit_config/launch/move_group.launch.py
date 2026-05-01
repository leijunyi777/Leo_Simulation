from moveit_configs_utils import MoveItConfigsBuilder
from moveit_configs_utils.launches import generate_move_group_launch
from launch import LaunchDescription
from launch_ros.actions import SetParameter

def generate_launch_description():
    moveit_config = MoveItConfigsBuilder("leo_sim", package_name="mycobot_moveit_config").to_moveit_configs()
    
    # 获取默认生成的 launch 描述
    ld = generate_move_group_launch(moveit_config)
    
    # 强制给这个 Launch 文件启动的所有节点打上 use_sim_time = True 的思想钢印
    sim_time_param = SetParameter(name='use_sim_time', value=True)
    ld.entities.insert(0, sim_time_param)
    
    return ld