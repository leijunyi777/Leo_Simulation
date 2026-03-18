import math                                      
import random                                    
import rclpy                                     
from rclpy.node import Node                      
from rclpy.parameter import Parameter            
from rclpy.action import ActionClient            

from std_msgs.msg import Bool                    
from geometry_msgs.msg import PointStamped, PoseStamped, Quaternion  
from nav_msgs.msg import OccupancyGrid           
from nav2_msgs.action import NavigateToPose      
from action_msgs.msg import GoalStatus           
from tf2_ros import Buffer, TransformListener, TransformException 
from explore_lite_msgs.msg import ExploreStatus  

class NavControllerNode(Node):
    def __init__(self):
        super().__init__('nav_controller_node')
        
        # --- 核心状态变量 ---
        self.nav_mode = "IDLE"  
        self.target_phase = "OBJECT" 

        # 【核心修改 1】引入永久压制开关，初始为开启状态
        self.suppression_enabled = True

        self.goal_handle = None
        self.latest_map = None  
        self.current_pose = None 
        
        self.last_target_x = 999.0
        self.last_target_y = 999.0
        self.pending_box_x = 999.0
        self.pending_box_y = 999.0
        
        self.standoff_dist = 0.37  

        self.explore_lite_active = False     
        self.cmd_explore_enabled = False     
        self.pending_manip_fb_for_resume = -1 
        self._random_after_complete_timer = None

        self.tf_buffer = Buffer()
        self.tf_listener = TransformListener(self.tf_buffer, self)

        self.nav_client = ActionClient(self, NavigateToPose, 'navigate_to_pose')
        self.get_logger().info('正在等待 Nav2 动作服务端 (navigate_to_pose) 启动...')
        while not self.nav_client.wait_for_server(timeout_sec=2.0):
            self.get_logger().info('Nav2 服务端尚未就绪，继续等待中...')

        self.get_logger().info('Nav2 服务端已连接！节点准备就绪。') 

        self.sub_cmd_explore = self.create_subscription(Bool, '/nav/cmd_explore', self.explore_cb, 10)
        self.sub_goal_point  = self.create_subscription(PointStamped, '/nav/goal_point', self.point_cb, 10)
        self.pub_fb = self.create_publisher(Bool, '/nav/goal_reached', 10)
        self.sub_manip_fb = self.create_subscription(Bool, '/manipulator_feedback', self.manip_feedback_cb, 10)

        self.pub_explore_resume = self.create_publisher(Bool, 'explore/resume', 10)
        self.sub_explore_status = self.create_subscription(ExploreStatus, 'explore/status', self.explore_status_cb, 10)
        self.sub_map = self.create_subscription(OccupancyGrid, '/map', self.map_cb, 10)

        self.pose_timer = self.create_timer(0.05, self.update_current_pose_cb)
        
        # 压制定时器
        self._idle_timer = self.create_timer(0.1, self._keep_explore_paused_cb)

    # ==========================================================================
    # 基础位姿与计算工具
    # ==========================================================================
    def update_current_pose_cb(self):
        try:
            t = self.tf_buffer.lookup_transform('map', 'base_footprint', rclpy.time.Time())
            self.current_pose = (t.transform.translation.x, t.transform.translation.y)
        except TransformException:
            pass 

    def map_cb(self, msg: OccupancyGrid):
        self.latest_map = msg

    def get_quaternion_from_a_to_b(self, a_pos: tuple, b_pos: tuple) -> Quaternion:
        dx = b_pos[0] - a_pos[0]
        dy = b_pos[1] - a_pos[1]
        yaw = math.atan2(dy, dx)
        q = Quaternion()
        q.x, q.y = 0.0, 0.0
        q.z = math.sin(yaw / 2.0)
        q.w = math.cos(yaw / 2.0)
        return q

    # ==========================================================================
    # m-explore 控制逻辑
    # ==========================================================================
    def _keep_explore_paused_cb(self):
        """
        压制定时器回调函数。
        【核心修改 2】增加了 suppression_enabled 的判断。
        一旦该开关变为 False，即便处于 IDLE 状态，也不会再持续发布 False 指令。
        """
        if self.suppression_enabled and self.nav_mode == "IDLE":
            self.pub_explore_resume.publish(Bool(data=False))

    def explore_cb(self, msg: Bool):
        self.cmd_explore_enabled = msg.data
        if msg.data:
            # 【核心修改 3】只要收到第一次 True，永久关闭压制功能
            if self.suppression_enabled:
                self.get_logger().info('收到首次启动指令，永久解除初始压制状态。')
                self.suppression_enabled = False

            self.nav_mode = "EXPLORE"
            self.target_phase = "OBJECT"
            
            if self.pending_manip_fb_for_resume <= 0:
                self.pub_explore_resume.publish(Bool(data=True))
            if not self.explore_lite_active:
                self.dispatch_random_goal()
        else:
            self.pub_explore_resume.publish(Bool(data=False))
            # 注意：此处可以根据需要选择是否回 IDLE，
            # 哪怕回了 IDLE，由于开关已关，也不会触发持续压制。
            if self.goal_handle:
                self.goal_handle.cancel_goal_async()

    def explore_status_cb(self, msg: ExploreStatus):
        if msg.status in [ExploreStatus.EXPLORATION_COMPLETE, ExploreStatus.RETURNED_TO_ORIGIN]:
            self.explore_lite_active = False
            if self._random_after_complete_timer is not None:
                self._random_after_complete_timer.cancel()
            def _start_random_after_complete():
                self._random_after_complete_timer.cancel()
                self._random_after_complete_timer = None
                if self.nav_mode == "EXPLORE":
                    self.dispatch_random_goal()
            self._random_after_complete_timer = self.create_timer(3.0, _start_random_after_complete)
        else:
            self.explore_lite_active = True

    # ==========================================================================
    # 导航计算逻辑
    # ==========================================================================
    def point_cb(self, msg: PointStamped):
        tx, ty = msg.point.x, msg.point.y
        if math.hypot(tx - self.last_target_x, ty - self.last_target_y) < 0.2:
            return
        self.last_target_x, self.last_target_y = tx, ty

        if self.explore_lite_active:
            self.pub_explore_resume.publish(Bool(data=False))
            self.pending_manip_fb_for_resume = 2

        if self.target_phase == "OBJECT":
            self.nav_mode = "GOTO_OBJECT"
            self.get_logger().info("【前往抓取】：计算距离目标 370mm 处的停靠点")
            self._execute_standoff_nav(tx, ty)
        else:
            if self.nav_mode not in ["GOTO_BOX_ORIGIN", "GOTO_BOX_TARGET"]:
                self.nav_mode = "GOTO_BOX_ORIGIN"
                self.pending_box_x, self.pending_box_y = tx, ty
                self.get_logger().info("【前往放置-阶段1】：先返回(0,0)，朝向起始反方向(Yaw=pi)")
                q_origin = Quaternion()
                q_origin.x, q_origin.y = 0.0, 0.0
                q_origin.z, q_origin.w = math.sin(math.pi / 2.0), math.cos(math.pi / 2.0)
                self.send_nav_goal(0.0, 0.0, q_origin)
            elif self.nav_mode == "GOTO_BOX_ORIGIN":
                self.pending_box_x, self.pending_box_y = tx, ty
            elif self.nav_mode == "GOTO_BOX_TARGET":
                self.pending_box_x, self.pending_box_y = tx, ty
                self._execute_standoff_nav(tx, ty)

    def _execute_standoff_nav(self, tx, ty):
        if self.current_pose is None:
            self.get_logger().warn("尚未获取到自身位姿，无法计算连线避让路径！")
            return
        rx, ry = self.current_pose
        q_facing_target = self.get_quaternion_from_a_to_b((rx, ry), (tx, ty))
        yaw = math.atan2(ty - ry, tx - rx)
        gx = tx - self.standoff_dist * math.cos(yaw)
        gy = ty - self.standoff_dist * math.sin(yaw)
        self.send_nav_goal(gx, gy, q_facing_target)

    def manip_feedback_cb(self, msg: Bool):
        if not msg.data or self.pending_manip_fb_for_resume <= 0:
            return
        self.pending_manip_fb_for_resume -= 1
        if self.pending_manip_fb_for_resume == 0:
            self.pending_manip_fb_for_resume = -1
            if self.cmd_explore_enabled:
                self.pub_explore_resume.publish(Bool(data=True))

    # ==========================================================================
    # Nav2 核心调度
    # ==========================================================================
    def get_random_free_point(self) -> tuple:
        if self.latest_map is None: return None
        map_data = self.latest_map
        for _ in range(1000):
            idx = random.randint(0, map_data.info.width * map_data.info.height - 1)
            if map_data.data[idx] == 0:
                x = map_data.info.origin.position.x + (idx % map_data.info.width + 0.5) * map_data.info.resolution
                y = map_data.info.origin.position.y + (idx // map_data.info.width + 0.5) * map_data.info.resolution
                return (x, y)
        return None

    def dispatch_random_goal(self):
        if self.goal_handle:
            self.goal_handle.cancel_goal_async()
            self.goal_handle = None
        target = self.get_random_free_point()
        if target is None or self.current_pose is None:
            self.create_timer(1.0, self.dispatch_random_goal)
            return
        q_facing_target = self.get_quaternion_from_a_to_b(self.current_pose, target)
        self.send_nav_goal(target[0], target[1], q_facing_target)

    def send_nav_goal(self, x: float, y: float, orientation_q: Quaternion):
        pose = PoseStamped()
        pose.header.frame_id = 'map'
        pose.header.stamp = self.get_clock().now().to_msg()
        pose.pose.position.x = float(x)
        pose.pose.position.y = float(y)
        pose.pose.orientation = orientation_q
        self.nav_client.send_goal_async(NavigateToPose.Goal(pose=pose)).add_done_callback(self.goal_response_cb)

    def goal_response_cb(self, future):
        self.goal_handle = future.result()
        if not self.goal_handle.accepted:
            if self.nav_mode == "EXPLORE" and not self.explore_lite_active:
                self.dispatch_random_goal()
            return
        self.goal_handle.get_result_async().add_done_callback(self.result_cb)

    def result_cb(self, future):
        status = future.result().status
        self.goal_handle = None

        if status == GoalStatus.STATUS_SUCCEEDED:
            if self.nav_mode == "GOTO_OBJECT":
                self.pub_fb.publish(Bool(data=True)) 
                self.nav_mode = "IDLE"  # 正常回 IDLE，压制不会再触发
                self.target_phase = "BOX"            
                self.last_target_x, self.last_target_y = 999.0, 999.0
                
            elif self.nav_mode == "GOTO_BOX_ORIGIN":
                self.get_logger().info("【前往放置-阶段2】：已到达(0,0)，前往避让点")
                self.nav_mode = "GOTO_BOX_TARGET"
                self._execute_standoff_nav(self.pending_box_x, self.pending_box_y)
                
            elif self.nav_mode == "GOTO_BOX_TARGET":
                self.pub_fb.publish(Bool(data=True)) 
                self.nav_mode = "IDLE"  # 正常回 IDLE，压制不会再触发
                self.last_target_x, self.last_target_y = 999.0, 999.0

            elif self.nav_mode == "EXPLORE" and not self.explore_lite_active:
                self.dispatch_random_goal()

        elif status != GoalStatus.STATUS_CANCELED:
            if self.nav_mode == "EXPLORE" and not self.explore_lite_active:
                self.dispatch_random_goal()
            elif self.nav_mode == "GOTO_BOX_ORIGIN":
                q_origin = Quaternion()
                q_origin.x, q_origin.y = 0.0, 0.0
                q_origin.z, q_origin.w = math.sin(math.pi / 2.0), math.cos(math.pi / 2.0)
                self.send_nav_goal(0.0, 0.0, q_origin)
            elif self.nav_mode in ["GOTO_OBJECT", "GOTO_BOX_TARGET"]:
                tx = self.last_target_x if self.nav_mode == "GOTO_OBJECT" else self.pending_box_x
                ty = self.last_target_y if self.nav_mode == "GOTO_OBJECT" else self.pending_box_y
                self._execute_standoff_nav(tx, ty)

def main():
    rclpy.init()
    node = NavControllerNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt: 
        pass

if __name__ == '__main__':
    main()