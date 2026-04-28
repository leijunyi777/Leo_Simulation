#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from rclpy.action import ActionClient
from builtin_interfaces.msg import Duration
from trajectory_msgs.msg import JointTrajectoryPoint
from control_msgs.action import FollowJointTrajectory
import math
import sys

class SimpleArmTester(Node):
    def __init__(self):
        super().__init__('simple_arm_tester')
        
        # 1. 创建 Action 客户端，连接到我们配置的轨迹控制器
        # 注意话题名称要匹配：/arm_controller/follow_joint_trajectory
        self._action_client = ActionClient(
            self, 
            FollowJointTrajectory, 
            '/arm_controller/follow_joint_trajectory'
        )
        
        # 2. 定义我们要控制的关节名称 (必须与 yaml 中的顺序一致)
        self.joint_names = [
            'arm_g_base_to_joint1',
            'arm_joint2_to_joint1',
            'arm_joint3_to_joint2',
            'arm_joint4_to_joint3',
            'arm_joint5_to_joint4',
            'arm_joint6_to_joint5',
            'arm_joint6output_to_joint6'
        ]

    def send_goal(self):
        self.get_logger().info('等待动作服务器连接...')
        self._action_client.wait_for_server()
        
        goal_msg = FollowJointTrajectory.Goal()
        goal_msg.trajectory.joint_names = self.joint_names

        # === 动作 1：全部回零 (直立状态) ===
        point1 = JointTrajectoryPoint()
        point1.positions = [0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0]
        point1.time_from_start = Duration(sec=3, nanosec=0) # 3秒到达
        goal_msg.trajectory.points.append(point1)

        # === 动作 2：向前弯曲测试 ===
        # 关节2 和 关节3 弯曲，像在“点头”或者伸手
        point2 = JointTrajectoryPoint()
        point2.positions = [0.0, math.pi/4, -math.pi/4, 0.0, 0.0, 0.0, 0.0]
        point2.time_from_start = Duration(sec=6, nanosec=0) # 累计6秒到达
        goal_msg.trajectory.points.append(point2)
        
        # === 动作 3：基座旋转测试 ===
        # 保持弯曲姿态，底座旋转 90 度
        point3 = JointTrajectoryPoint()
        point3.positions = [math.pi/2, math.pi/4, -math.pi/4, 0.0, 0.0, 0.0, 0.0]
        point3.time_from_start = Duration(sec=9, nanosec=0) # 累计9秒到达
        goal_msg.trajectory.points.append(point3)

        self.get_logger().info('发送轨迹目标：回零 -> 向前弯曲 -> 基座旋转...')
        
        # 发送请求并设置回调
        self._send_goal_future = self._action_client.send_goal_async(goal_msg)
        self._send_goal_future.add_done_callback(self.goal_response_callback)

    def goal_response_callback(self, future):
        goal_handle = future.result()
        if not goal_handle.accepted:
            self.get_logger().error('目标被控制器拒绝！请检查配置或终端报错。')
            sys.exit(1)

        self.get_logger().info('目标已接受，正在执行...')
        self._get_result_future = goal_handle.get_result_async()
        self._get_result_future.add_done_callback(self.get_result_callback)

    def get_result_callback(self, future):
        result = future.result().result
        self.get_logger().info(f'动作执行完毕! 状态码: {result.error_code} (0表示成功)')
        rclpy.shutdown()


def main(args=None):
    rclpy.init(args=args)
    action_client = SimpleArmTester()
    action_client.send_goal()
    rclpy.spin(action_client)

if __name__ == '__main__':
    main()