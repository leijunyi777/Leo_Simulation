#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from rclpy.action import ActionClient

# TF2 相关
import tf2_ros
import tf2_geometry_msgs
from geometry_msgs.msg import PointStamped, Pose

# 自定义视觉消息
from my_robot_interfaces.msg import ObjectTarget

# MoveIt Action 和 消息格式
from moveit_msgs.action import MoveGroup
from moveit_msgs.msg import MotionPlanRequest, Constraints, PositionConstraint, BoundingVolume
from shape_msgs.msg import SolidPrimitive

class DirectGraspNode(Node):
    def __init__(self):
        super().__init__('direct_grasp_node')
        
        # 1. TF2 监听器
        self.tf_buffer = tf2_ros.Buffer()
        self.tf_listener = tf2_ros.TransformListener(self.tf_buffer, self)
        
        # 2. 订阅视觉话题
        self.sub = self.create_subscription(ObjectTarget, '/detected_object', self.obj_callback, 10)
        
        # 3. 创建 MoveIt 的 Action Client
        self.move_client = ActionClient(self, MoveGroup, 'move_action')
        
        self.is_busy = False
        
        # --- 请确认这些名称与你 Xacro 中一致 ---
        self.camera_frame = 'camera_link'
        self.arm_base_frame = 'arm_g_base'
        self.ee_link = 'arm_joint6_flange'  # 机械臂的末端执行器 link
        self.group_name = 'arm_group'       # MoveIt Setup Assistant 里设置的规划组名称
        
        self.get_logger().info("🤖 直连抓取节点启动！正在连接 MoveIt Action Server...")
        self.move_client.wait_for_server()
        self.get_logger().info("✅ 成功连接到 /move_action！等待视觉目标...")

    def obj_callback(self, msg):
        # 正在执行时忽略新目标
        if self.is_busy:
            return
            
        try:
            # 检查 TF 是否就绪
            if not self.tf_buffer.can_transform(self.arm_base_frame, self.camera_frame, rclpy.time.Time()):
                self.get_logger().warn("等待 TF 树转换...")
                return

            # --- 1. TF 坐标转换 ---
            pt = PointStamped()
            pt.header.frame_id = self.camera_frame
            pt.header.stamp = self.get_clock().now().to_msg()
            pt.point.x = msg.x
            pt.point.y = msg.y
            pt.point.z = msg.z
            
            trans = self.tf_buffer.lookup_transform(self.arm_base_frame, self.camera_frame, rclpy.time.Time())
            pt_transformed = tf2_geometry_msgs.do_transform_point(pt, trans)
            
            target_x = pt_transformed.point.x
            target_y = pt_transformed.point.y
            target_z = pt_transformed.point.z + 0.08 # Z轴安全抓取高度补偿
            
            if target_z < 0.0:
                self.get_logger().warn(f"目标点过低 (Z={target_z:.2f})，忽略以防撞地！")
                return

            self.get_logger().info(f"🎯 锁定目标 [{msg.name}]: X={target_x:.2f}, Y={target_y:.2f}, Z={target_z:.2f}")
            self.is_busy = True
            
            # --- 2. 构建并发送 MoveIt Action 请求 ---
            self.send_move_goal(target_x, target_y, target_z)
            
        except Exception as e:
            self.get_logger().error(f"处理失败: {e}")
            self.is_busy = False

    def send_move_goal(self, x, y, z):
        goal_msg = MoveGroup.Goal()
        
        # 构建运动规划请求
        req = MotionPlanRequest()
        req.group_name = self.group_name
        req.num_planning_attempts = 5
        req.allowed_planning_time = 10.0
        req.max_velocity_scaling_factor = 0.5      # 限速 50%
        req.max_acceleration_scaling_factor = 0.5  # 限加速度 50%

        # === 构建位置约束 (相当于 C++ 的 setPositionTarget) ===
        # 也就是告诉 MoveIt: 无论姿态如何，只要末端连杆的中心点进入这个容差球体即可
        constraint = Constraints()
        pos_constraint = PositionConstraint()
        pos_constraint.header.frame_id = self.arm_base_frame
        pos_constraint.link_name = self.ee_link
        pos_constraint.weight = 1.0
        
        # 设置目标坐标和容差球体
        bv = BoundingVolume()
        sphere = SolidPrimitive()
        sphere.type = SolidPrimitive.SPHERE
        sphere.dimensions = [0.01]  # 允许 1 厘米的误差
        bv.primitives.append(sphere)
        
        target_pose = Pose()
        target_pose.position.x = x
        target_pose.position.y = y
        target_pose.position.z = z
        # (忽略姿态 target_pose.orientation，让 MoveIt 自由发挥)
        bv.primitive_poses.append(target_pose)
        
        pos_constraint.constraint_region = bv
        constraint.position_constraints.append(pos_constraint)
        req.goal_constraints.append(constraint)
        
        goal_msg.request = req

        # 异步发送请求
        self.get_logger().info("🚀 正在向 MoveIt 发送规划请求...")
        send_goal_future = self.move_client.send_goal_async(goal_msg)
        send_goal_future.add_done_callback(self.goal_response_callback)

    def goal_response_callback(self, future):
        goal_handle = future.result()
        if not goal_handle.accepted:
            self.get_logger().error("❌ MoveIt 拒绝了该规划请求 (可能不可达或自碰撞)")
            self.is_busy = False
            return

        self.get_logger().info("✅ MoveIt 接受请求，正在执行轨迹...")
        get_result_future = goal_handle.get_result_async()
        get_result_future.add_done_callback(self.get_result_callback)

    def get_result_callback(self, future):
        result = future.result().result
        error_code = result.error_code.val
        
        # MoveIt 错误码 1 代表 SUCCESS
        if error_code == 1:
            self.get_logger().info("🎉 抓取动作执行成功！")
        else:
            self.get_logger().error(f"⚠️ 动作失败，MoveIt 错误码: {error_code}")
            
        self.get_logger().info("等待下一个目标...\n")
        self.is_busy = False

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