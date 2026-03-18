"""
================================================================================
组件：导航控制节点（实机版）
================================================================================
设计思路：
    作为智能执行器，自行管理探索循环与目标导航，严格使用主 FSM 定义的话题。
    集成 explore_lite：收到物体/箱子 GOTO 时暂停，机械臂完成一轮（2 次
    manip_feedback）后恢复；explore_lite 报告探索完成后切到随机探索。

接口约定（严格遵循主 FSM）：
    - 订阅: /nav/cmd_explore (Bool) -> True=探索循环, False=停止/刹车
    - 订阅: /nav/goal_point (PointStamped) -> FSM 下发的目标点
    - 发布: /nav/goal_reached (Bool) -> GOTO 任务完成时为 True
================================================================================
"""

import math
import random
import rclpy
from rclpy.node import Node
from rclpy.parameter import Parameter
from rclpy.action import ActionClient
from std_msgs.msg import Bool
from geometry_msgs.msg import PointStamped, PoseStamped
from nav2_msgs.action import NavigateToPose
from action_msgs.msg import GoalStatus
from tf2_ros import Buffer, TransformListener, TransformException
from explore_lite_msgs.msg import ExploreStatus

class NavControllerNode(Node):
    def __init__(self):
        super().__init__('nav_controller_node', parameter_overrides=[Parameter('use_sim_time', Parameter.Type.BOOL, True)])# <-记得改动！！！
        
        self.explore_bounds = [-1.5, 1.5]
        # 仅用于物体/箱子 GOTO：车停在目标沿朝向后退 432mm 处，其它导航发到哪开到哪
        self.standoff_dist = 0.432
        
        self.nav_mode = "IDLE"  # "IDLE", "EXPLORE", "GOTO"
        self.goal_handle = None

        self.last_target_x = 999.0
        self.last_target_y = 999.0

        # explore_lite 是否处于前沿探索阶段（直到 EXPLORATION_COMPLETE / RETURNED_TO_ORIGIN）
        self.explore_lite_active = False  #默认不探索，开始探索后为 True
        # 因 GOTO 暂停后，需收到 2 次机械臂反馈（抓取+放置）再恢复，剩余次数
        self.pending_manip_fb_for_resume = -1
        # 当前 /nav/cmd_explore 值，用于恢复 explore_lite 时判断是否仍处于探索循环
        self.cmd_explore_enabled = False
        # 若 explore_lite 未配置 return_to_origin，EXPLORATION_COMPLETE 后延迟开始随机探索
        self._random_after_complete_timer = None

        self.tf_buffer = Buffer()
        self.tf_listener = TransformListener(self.tf_buffer, self)

        self.nav_client = ActionClient(self, NavigateToPose, 'navigate_to_pose')
        self.nav_client.wait_for_server()
        self.get_logger().info('Nav2 已就绪，等待 FSM 指令.')

        self.sub_cmd_explore = self.create_subscription(Bool, '/nav/cmd_explore', self.explore_cb, 10)
        self.sub_goal_point  = self.create_subscription(PointStamped, '/nav/goal_point', self.point_cb, 10)
        self.pub_fb = self.create_publisher(Bool, '/nav/goal_reached', 10)

        # explore_lite：GOTO 时暂停，机械臂一轮结束后恢复
        self.pub_explore_resume = self.create_publisher(Bool, 'explore/resume', 10)
        self.sub_explore_status = self.create_subscription(
            ExploreStatus, 'explore/status', self.explore_status_cb, 10)
        self.sub_manip_fb = self.create_subscription(
            Bool, '/manipulator_feedback', self.manip_feedback_cb, 10)

        # 默认不探索：延迟发布 explore/resume=False，确保 explore_lite 由 cmd_explore 控制才工作
        #（持续压制定时器）：
        self._idle_timer = self.create_timer(0.1, self._keep_explore_paused_cb)

    # 删除旧的 _publish_initial_explore_resume 函数，替换为：
    def _keep_explore_paused_cb(self):
        """只要 FSM 没有下发探索指令，就持续按住 explore_lite 不让它动"""
        if not self.cmd_explore_enabled:
            self.pub_explore_resume.publish(Bool(data=False))

    # ==========================================================================
    # 输入接口
    # ==========================================================================
    def explore_status_cb(self, msg: ExploreStatus):
        """根据 explore_lite 状态维护前沿探索标志；探索结束/返回起点后发出信号，返回起点后开始随机探索。"""
        if msg.status == ExploreStatus.EXPLORATION_COMPLETE:
            if self.explore_lite_active:
                self.get_logger().info('explore_lite 已结束')
            self.explore_lite_active = False
            # 若未配置 return_to_origin 则不会收到 RETURNED_TO_ORIGIN，3 秒后开始随机探索
            if self._random_after_complete_timer is not None:
                self._random_after_complete_timer.cancel()
            def _start_random_after_complete():
                self._random_after_complete_timer.cancel()
                self._random_after_complete_timer = None
                if self.nav_mode == "EXPLORE":
                    self.get_logger().info('开始随机探索')
                    self.dispatch_goal()
            self._random_after_complete_timer = self.create_timer(3.0, _start_random_after_complete)
        elif msg.status == ExploreStatus.RETURNED_TO_ORIGIN:
            if self._random_after_complete_timer is not None:
                self._random_after_complete_timer.cancel()
                self._random_after_complete_timer = None
            self.get_logger().info('返回起点')
            self.explore_lite_active = False
            if self.nav_mode == "EXPLORE":
                self.get_logger().info('开始随机探索')
                self.dispatch_goal()
        else:
            self.explore_lite_active = True

    def manip_feedback_cb(self, msg: Bool):
        """收到 2 次机械臂反馈（抓取+放置）后，若曾因 GOTO 暂停且 cmd_explore 仍为 True 则恢复 explore_lite。"""
        if not msg.data or self.pending_manip_fb_for_resume <= 0:
            return
        self.pending_manip_fb_for_resume -= 1
        if self.pending_manip_fb_for_resume == 0:
            self.pending_manip_fb_for_resume = -1
            if self.cmd_explore_enabled:
                self.pub_explore_resume.publish(Bool(data=True))
                self.get_logger().info('机械臂一轮完成 -> 恢复 explore_lite.')

    def explore_cb(self, msg: Bool):
        """由 /nav/cmd_explore 控制是否开启探索：True=探索循环（驱动 explore/resume），False=停止/刹车。"""
        self.cmd_explore_enabled = msg.data
        self.get_logger().info('cmd_explore: %s' % ('true' if msg.data else 'false'))

        if msg.data:
            if self.nav_mode != "EXPLORE":
                self.nav_mode = "EXPLORE"
            # 仅在未因 GOTO 等待机械臂反馈时，才让 explore_lite 工作
            if self.pending_manip_fb_for_resume <= 0:
                self.pub_explore_resume.publish(Bool(data=True))
            # 若 explore_lite 已结束，则由本节点发随机点
            if not self.explore_lite_active:
                self.dispatch_goal()
        else:
            self.pub_explore_resume.publish(Bool(data=False))
            if self.nav_mode == "EXPLORE":
                self.nav_mode = "IDLE"
                if self.goal_handle:
                    self.goal_handle.cancel_goal_async()

    def point_cb(self, msg: PointStamped):
        """计算 standoff 位姿并下发目标导航。"""
        tx, ty = msg.point.x, msg.point.y
        
        # Anti-spam filter (20cm tolerance)
        if math.hypot(tx - self.last_target_x, ty - self.last_target_y) < 0.2:
            return

        self.nav_mode = "GOTO"
        self.last_target_x, self.last_target_y = tx, ty

        # FSM 下发物体/箱子目标时暂停 explore_lite，等 2 次机械臂反馈后再恢复
        if self.explore_lite_active:
            self.pub_explore_resume.publish(Bool(data=False))
            self.pending_manip_fb_for_resume = 2
            self.get_logger().info('收到 GOTO 目标 -> 暂停 explore_lite，待机械臂本轮完成后再恢复.')

        try:
            t = self.tf_buffer.lookup_transform('map', 'base_link', rclpy.time.Time())
            rx, ry = t.transform.translation.x, t.transform.translation.y
            yaw = math.atan2(ty - ry, tx - rx)
            dist = math.hypot(tx - rx, ty - ry)

            # 计算 standoff 坐标（与目标保持 standoff_dist）
            gx = tx - self.standoff_dist * math.cos(yaw) if dist > self.standoff_dist else rx
            gy = ty - self.standoff_dist * math.sin(yaw) if dist > self.standoff_dist else ry

            self.dispatch_goal(gx, gy, yaw)

        except TransformException as ex:
            self.get_logger().warn(f"TF Error: {ex}")

    # ==========================================================================
    # 核心：统一调度器
    # ==========================================================================
    def dispatch_goal(self, x=None, y=None, yaw=None):
        """
        取消当前目标；若未给出坐标则生成随机点，然后向 Nav2 下发新的 PoseStamped。
        """
        # 1. 先取消已有目标
        if self.goal_handle:
            self.goal_handle.cancel_goal_async()
            self.goal_handle = None

        # 2. EXPLORE 模式且未给坐标时生成随机点
        if x is None or y is None or yaw is None:
            x = random.uniform(self.explore_bounds[0], self.explore_bounds[1])
            y = random.uniform(self.explore_bounds[0], self.explore_bounds[1])
            yaw = random.uniform(-math.pi, math.pi)

        # 3. 组装并下发
        pose = PoseStamped()
        pose.header.frame_id = 'map'
        pose.header.stamp.sec = 0  # 实机可避免 TF 缓冲延迟
        
        pose.pose.position.x, pose.pose.position.y = float(x), float(y)
        pose.pose.orientation.z = math.sin(yaw / 2.0)
        pose.pose.orientation.w = math.cos(yaw / 2.0)
        
        self.nav_client.send_goal_async(NavigateToPose.Goal(pose=pose)).add_done_callback(self.goal_response_cb)

    # ==========================================================================
    # 异步反馈链
    # ==========================================================================
    # 收到 Nav2 目标响应回调
    # 若 Nav2 目标被拒绝（通常是因碰撞），且当前模式为 EXPLORE 且 explore_lite 未激活，则重新下发目标
    def goal_response_cb(self, future):
        self.goal_handle = future.result()
        if not self.goal_handle.accepted:
            if self.nav_mode == "EXPLORE" and not self.explore_lite_active:
                self.dispatch_goal()
            return
            
        self.goal_handle.get_result_async().add_done_callback(self.result_cb)

    # 收到 Nav2 结果响应回调
    # 若 Nav2 目标成功，且当前模式为 GOTO，则发布 /nav/goal_reached=True 信号
    # 若 Nav2 目标被拒绝（通常是因碰撞），且当前模式为 EXPLORE 且 explore_lite 未激活，则重新下发目标
    def result_cb(self, future):
        status = future.result().status
        self.goal_handle = None

        if status == GoalStatus.STATUS_SUCCEEDED:
            if self.nav_mode == "GOTO":
                self.pub_fb.publish(Bool(data=True))
                self.nav_mode = "IDLE"
            elif self.nav_mode == "EXPLORE" and not self.explore_lite_active:
                self.dispatch_goal()

        elif status != GoalStatus.STATUS_CANCELED and self.nav_mode == "EXPLORE" and not self.explore_lite_active:
            self.dispatch_goal()

def main():
    rclpy.init()
    node = NavControllerNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()