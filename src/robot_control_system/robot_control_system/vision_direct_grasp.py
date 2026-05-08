#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from rclpy.action import ActionClient

import threading
from threading import Event

# TF2 相关
import tf2_ros
import tf2_geometry_msgs
from geometry_msgs.msg import PointStamped, Pose, Quaternion

# 自定义视觉消息
from my_robot_interfaces.msg import ObjectTarget

# MoveIt Action 和 消息格式
from moveit_msgs.action import MoveGroup
from moveit_msgs.msg import MotionPlanRequest, Constraints, PositionConstraint, OrientationConstraint, JointConstraint, BoundingVolume
from shape_msgs.msg import SolidPrimitive

class DirectGraspNode(Node):
    def __init__(self):
        super().__init__('direct_grasp_node')
        
        self.tf_buffer = tf2_ros.Buffer()
        self.tf_listener = tf2_ros.TransformListener(self.tf_buffer, self)
        
        self.sub = self.create_subscription(ObjectTarget, '/detected_object', self.obj_callback, 10)
        self.move_client = ActionClient(self, MoveGroup, 'move_action')
        
        self.is_busy = False
        
        # --- 基础配置 ---
        self.camera_frame = 'camera_link'
        self.arm_base_frame = 'arm_g_base'
        self.ee_link = 'arm_joint6_flange'
        
        # --- 规划组配置 ---
        self.arm_group = 'arm_group'
        self.gripper_group = 'gripper_group'
        
        # ⚠️ 请根据你的 Xacro 替换实际的关节名称和想要的默认角度！
        self.arm_joints = [
            'arm_joint2_to_joint1', 'arm_joint3_to_joint2', 'arm_joint4_to_joint3',
            'arm_joint5_to_joint4', 'arm_joint6_to_joint5', 'arm_joint6output_to_joint6'
        ]
        self.gripper_joints = ['arm_gripper_controller']
        
        # 预设姿态的关节角度 (弧度)
        self.home_angles = [0.0, 0.0, 0.0, 0.0, 0.0, 0.0]  # Home 姿态
        self.open_angles = [5.0]                          # 张开夹爪
        self.close_angles = [-40.0]                        # 闭合夹爪

        # 用于线程同步的事件锁
        self.action_done_event = Event()
        self.action_success = False

        self.get_logger().info("🤖 直连抓取序列节点启动！正在连接...")
        self.move_client.wait_for_server()
        self.get_logger().info("✅ 成功连接到 /move_action！等待目标...")

    def obj_callback(self, msg):
        if self.is_busy:
            return
            
        try:
            if not self.tf_buffer.can_transform(self.arm_base_frame, self.camera_frame, rclpy.time.Time()):
                return

            pt = PointStamped()
            pt.header.frame_id = self.camera_frame
            pt.header.stamp = self.get_clock().now().to_msg()
            pt.point.x, pt.point.y, pt.point.z = msg.x, msg.y, msg.z
            
            trans = self.tf_buffer.lookup_transform(self.arm_base_frame, self.camera_frame, rclpy.time.Time())
            pt_trans = tf2_geometry_msgs.do_transform_point(pt, trans)
            
            target_x, target_y = pt_trans.point.x, pt_trans.point.y
            target_z = pt_trans.point.z + 0.1 # 补偿高度
            
            if target_z < 0.0:
                return

            self.get_logger().info(f"🎯 锁定目标: X={target_x:.2f}, Y={target_y:.2f}, Z={target_z:.2f}")
            self.is_busy = True
            
            # 🔥 启动独立线程执行序列，防止阻塞 ROS 回调 🔥
            threading.Thread(target=self.execute_grasp_sequence, args=(target_x, target_y, target_z)).start()
            
        except Exception as e:
            self.get_logger().error(f"处理失败: {e}")
            self.is_busy = False

    # ==========================================================
    #                    核心状态机序列
    # ==========================================================
    # ==========================================================
    #                    核心状态机序列 (两段式抓取)
    # ==========================================================
    def execute_grasp_sequence(self, x, y, z):
        self.get_logger().info("========== 🎬 开始两段式抓取序列 ==========")
        
        # 悬停高度：在目标 Z 坐标（已经加过 8cm 补偿）的基础上，再往上加 10cm (0.1m)
        hover_z = z + 0.10  

        # 1. 回 Home 姿态
        self.get_logger().info("[1/7] 正在移动到 Home...")
        req1 = self.build_joint_request(self.arm_group, self.arm_joints, self.home_angles)
        if not self.send_goal_and_wait(req1): return self.abort()

        # 2. 张开夹爪
        self.get_logger().info("[2/7] 正在张开夹爪...")
        req2 = self.build_joint_request(self.gripper_group, self.gripper_joints, self.open_angles)
        if not self.send_goal_and_wait(req2): return self.abort()

        # 3. 移动到目标点上方 (悬停 Approach)
        self.get_logger().info(f"[3/7] 正在移动到目标上方悬停点 (Z={hover_z:.2f})...")
        req3 = self.build_pose_request(x, y, hover_z)
        if not self.send_goal_and_wait(req3): return self.abort()

        # 4. 直线下滑到抓取点
        self.get_logger().info(f"[4/7] 正在垂直下探抓取目标 (Z={z:.2f})...")
        req4 = self.build_pose_request(x, y, z)
        if not self.send_goal_and_wait(req4): return self.abort()

        # 5. 闭合夹爪
        self.get_logger().info("[5/7] 正在闭合夹爪...")
        req5 = self.build_joint_request(self.gripper_group, self.gripper_joints, self.close_angles)
        if not self.send_goal_and_wait(req5): return self.abort()

        # 6. 直线提起物体 (退回悬停点 Retreat)
        self.get_logger().info("[6/7] 抓取成功，正在提起物体...")
        if not self.send_goal_and_wait(req3): return self.abort()

        # 7. 再回 Home 姿态
        self.get_logger().info("[7/7] 提钩完成，带回 Home...")
        if not self.send_goal_and_wait(req1): return self.abort()

        self.get_logger().info("🎉🎉 完美！两段式抓取序列执行成功！")
        self.is_busy = False

    def abort(self):
        self.get_logger().error("❌ 序列中断！重置状态等待下次触发。")
        self.is_busy = False

    # ==========================================================
    #                  MoveIt 请求构造与同步执行器
    # ==========================================================
    def build_joint_request(self, group, joint_names, angles):
        """构造关节角度约束的请求"""
        req = MotionPlanRequest()
        req.group_name = group
        req.allowed_planning_time = 5.0
        
        constraint = Constraints()
        for name, angle in zip(joint_names, angles):
            jc = JointConstraint()
            jc.joint_name = name
            jc.position = angle
            jc.tolerance_above = 0.05
            jc.tolerance_below = 0.05
            jc.weight = 1.0
            constraint.joint_constraints.append(jc)
        
        req.goal_constraints.append(constraint)
        return req

    def build_pose_request(self, x, y, z):
        """构造 XYZ 位置 + 竖直向下姿态约束的请求"""
        req = MotionPlanRequest()
        req.group_name = self.arm_group
        req.num_planning_attempts = 5
        req.allowed_planning_time = 10.0
        req.max_velocity_scaling_factor = 0.4
        
        constraint = Constraints()
        
        # 1. 位置约束
        pos_c = PositionConstraint()
        pos_c.header.frame_id = self.arm_base_frame
        pos_c.link_name = self.ee_link
        pos_c.weight = 1.0
        bv = BoundingVolume()
        sphere = SolidPrimitive(type=SolidPrimitive.SPHERE, dimensions=[0.02]) # 2cm容差
        target_pose = Pose()
        target_pose.position.x = x
        target_pose.position.y = y
        target_pose.position.z = z
        bv.primitives.append(sphere)
        bv.primitive_poses.append(target_pose)
        pos_c.constraint_region = bv
        constraint.position_constraints.append(pos_c)
        
        # 2. 姿态约束 (竖直向下)
        ori_c = OrientationConstraint()
        ori_c.header.frame_id = self.arm_base_frame
        ori_c.link_name = self.ee_link
        
        # ⚠️ 关键点：假设底座 Z 向上，这里的四元数代表让末端连杆发生 180度翻转，指地。
        # 如果你的机械臂手爪方向不对，请尝试把这里改成 x=0.707, y=0.0, z=0.707, w=0.0 等。
        ori_c.orientation.x = 0.0
        ori_c.orientation.y = 1.0
        ori_c.orientation.z = 0.0
        ori_c.orientation.w = 0.0
        
        # 允许手腕有一点点歪斜 (0.2 弧度，约 11度)，极大提高规划成功率
        ori_c.absolute_x_axis_tolerance = 0.2
        ori_c.absolute_y_axis_tolerance = 0.2
        ori_c.absolute_z_axis_tolerance = 3.14  # 允许夹爪绕自身的 Z 轴随意旋转，不在乎抓取方向
        ori_c.weight = 1.0
        constraint.orientation_constraints.append(ori_c)
        
        req.goal_constraints.append(constraint)
        return req

    def send_goal_and_wait(self, req):
        """核心魔法：把异步的 Action 调用变成阻塞等待的同步调用"""
        goal_msg = MoveGroup.Goal()
        goal_msg.request = req
        
        self.action_success = False
        self.action_done_event.clear() # 重置锁
        
        future = self.move_client.send_goal_async(goal_msg)
        future.add_done_callback(self.goal_response_callback)
        
        # 挂起当前子线程，直到 get_result_callback 把它唤醒
        self.action_done_event.wait()
        return self.action_success

    def goal_response_callback(self, future):
        goal_handle = future.result()
        if not goal_handle.accepted:
            self.get_logger().error("规划被拒绝 (不可达或碰撞)")
            self.action_success = False
            self.action_done_event.set() # 唤醒线程
            return

        future_result = goal_handle.get_result_async()
        future_result.add_done_callback(self.get_result_callback)

    def get_result_callback(self, future):
        error_code = future.result().result.error_code.val
        if error_code == 1:
            self.action_success = True
        else:
            self.get_logger().error(f"执行失败，错误码: {error_code}")
            self.action_success = False
            
        self.action_done_event.set() # 唤醒线程

def main(args=None):
    rclpy.init(args=args)
    node = DirectGraspNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()