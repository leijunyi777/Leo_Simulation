import math
import random
import rclpy
from rclpy.node import Node
from rclpy.action import ActionClient

from std_msgs.msg import Bool, String
from geometry_msgs.msg import PointStamped, PoseStamped, Quaternion, Twist
from nav_msgs.msg import OccupancyGrid
from nav2_msgs.action import NavigateToPose
from action_msgs.msg import GoalStatus
from tf2_ros import Buffer, TransformListener, TransformException
from explore_lite_msgs.msg import ExploreStatus

# 导入自定义消息类型 / Import custom message types
from my_robot_interfaces.msg import ObjectTarget

class NavControllerNode(Node):
    def __init__(self):
        """
        节点初始化函数。/ Node initialization.
        负责：状态变量、TF监听、Nav2动作客户端、发布/订阅及PD接近控制器的初始化。
        Handles: State variables, TF listener, Nav2 Action Client, Pub/Sub, and PD controller init.
        """
        super().__init__('nav_controller_node')
        
        # --- 核心状态变量 / Core State Variables ---
        self.nav_mode = "IDLE"  
        self.target_phase = "OBJECT" 
        self.suppression_enabled = True

        self.goal_handle = None
        self.latest_map = None  
        self.latest_costmap = None  
        self.current_pose = None 
        
        self.last_target_x = 999.0
        self.last_target_y = 999.0
        self.pending_box_x = 999.0
        self.pending_box_y = 999.0
        
        self.standoff_dist = 0.5  # 代价地图智能找点的半径 0.5m

        self.explore_lite_active = False     
        self.cmd_explore_enabled = False     
        self._random_after_complete_timer = None
        
        # 从 FSM 接收的当前任务目标 / Target info from FSM
        self.active_task_color = "NONE" 
        self.active_task_name = "NONE"

        # --- 接近阶段 PD 控制参数 / Close Approach PD Control Params ---
        self.kp_linear = 0.5
        self.kd_linear = 0.1
        self.kp_angular = 0.5
        self.kd_angular = 0.1
        
        self.prev_error_x = 0.0
        self.prev_error_z = 0.0
        self.prev_time = self.get_clock().now()
        
        self.last_msg_time = self.get_clock().now()
        self.timeout_sec = 0.5

        # 初始化 TF 监听器 / Init TF Listener
        self.tf_buffer = Buffer()
        self.tf_listener = TransformListener(self.tf_buffer, self)

        # 初始化 Nav2 动作客户端 / Init Nav2 Action Client
        self.nav_client = ActionClient(self, NavigateToPose, 'navigate_to_pose')
        self.get_logger().info('[Init] Waiting for Nav2 Action Server...')
        while not self.nav_client.wait_for_server(timeout_sec=2.0):
            pass
        self.get_logger().info('[Init] Nav2 Server Connected!') 

        # --- 订阅与发布设置 / Pub & Sub Setup ---
        # 导航相关 / Navigation related
        self.sub_cmd_explore = self.create_subscription(Bool, '/nav/cmd_explore', self.explore_cb, 10)
        self.sub_goal_point  = self.create_subscription(PointStamped, '/nav/goal_point', self.point_cb, 10)
        self.pub_fb = self.create_publisher(Bool, '/nav/goal_reached', 10)
        
        # 视觉/接近相关 / Vision & Approach related
        # 【修改点】：订阅 FSM 发出的 /nav/target_info 以获取准确的颜色和类型
        self.sub_target_info = self.create_subscription(String, '/nav/target_info', self.target_info_cb, 10)
        self.sub_vision_target = self.create_subscription(ObjectTarget, '/detected_object', self.vision_target_cb, 10)
        self.pub_cmd_vel = self.create_publisher(Twist, '/cmd_vel', 10)

        # 探索相关 / Exploration related
        self.pub_explore_resume = self.create_publisher(Bool, 'explore/resume', 10)
        self.sub_explore_status = self.create_subscription(ExploreStatus, 'explore/status', self.explore_status_cb, 10)
        
        # 地图相关 / Map related
        self.sub_map = self.create_subscription(OccupancyGrid, '/map', self.map_cb, 10)
        self.sub_costmap = self.create_subscription(OccupancyGrid, '/global_costmap/costmap', self.costmap_cb, 10)

        # 定时器设置 / Timers
        self.pose_timer = self.create_timer(0.05, self.update_current_pose_cb) # 50ms获取一次位姿
        self._idle_timer = self.create_timer(0.1, self._keep_explore_paused_cb) # 探索压制
        self.watchdog_timer = self.create_timer(0.1, self.watchdog_callback) # 接近时的安全看门狗

    # ==========================================================================
    # 基础状态更新 / Basic State Updates
    # ==========================================================================
    def update_current_pose_cb(self):
        try:
            t = self.tf_buffer.lookup_transform('map', 'base_footprint', rclpy.time.Time())
            self.current_pose = (t.transform.translation.x, t.transform.translation.y)
        except TransformException:
            pass 

    def map_cb(self, msg: OccupancyGrid): self.latest_map = msg
    def costmap_cb(self, msg: OccupancyGrid): self.latest_costmap = msg

    def target_info_cb(self, msg: String):
        """
        【新增解析器】解析 FSM 发送的目标信息，格式例如: "color:red,name:object"
        """
        try:
            parts = msg.data.split(',')
            for part in parts:
                if part.startswith('color:'):
                    self.active_task_color = part.split(':')[1]
                elif part.startswith('name:'):
                    self.active_task_name = part.split(':')[1]
            self.get_logger().info(f"[Task Locked] Target updated -> Color: {self.active_task_color}, Class: {self.active_task_name}")
        except Exception as e:
            self.get_logger().warn(f"[Parse Error] Failed to parse /nav/target_info: {e}, received: {msg.data}")

    def get_quaternion_from_a_to_b(self, a_pos: tuple, b_pos: tuple) -> Quaternion:
        dx, dy = b_pos[0] - a_pos[0], b_pos[1] - a_pos[1]
        yaw = math.atan2(dy, dx)
        q = Quaternion()
        q.z, q.w = math.sin(yaw / 2.0), math.cos(yaw / 2.0)
        return q

    def get_cost_at(self, x: float, y: float) -> int:
        """获取代价地图某物理坐标点的值 / Get costmap value at physical coordinates"""
        if not self.latest_costmap:
            return 255
        info = self.latest_costmap.info
        cm_x = int((x - info.origin.position.x) / info.resolution)
        cm_y = int((y - info.origin.position.y) / info.resolution)
        if 0 <= cm_x < info.width and 0 <= cm_y < info.height:
            return self.latest_costmap.data[cm_y * info.width + cm_x]
        return 255

    # ==========================================================================
    # 辅助功能 / Auxiliary Functions
    # ==========================================================================
    def cancel_current_nav_goal(self):
        """
        无论处于什么状态，强制取消当前的 Nav2 任务目标 
        Force cancel the current Nav2 goal regardless of its state
        """
        if self.goal_handle is not None:
            self.get_logger().info("[Nav2] Canceling current goal before sending a new task.")
            try:
                self.goal_handle.cancel_goal_async()
            except Exception:
                pass
            self.goal_handle = None

    # ==========================================================================
    # 探索与 FSM 指令响应 / Exploration & Command Response
    # ==========================================================================
    def _keep_explore_paused_cb(self):
        if self.suppression_enabled and self.nav_mode == "IDLE":
            self.pub_explore_resume.publish(Bool(data=False))

    def explore_cb(self, msg: Bool):
        self.cmd_explore_enabled = msg.data
        if msg.data:
            if self.suppression_enabled:
                self.suppression_enabled = False

            self.nav_mode = "EXPLORE"
            # 重置阶段
            self.target_phase = "OBJECT" 
            self.last_target_x, self.last_target_y = 999.0, 999.0
            
            self.pub_explore_resume.publish(Bool(data=True))
        else:
            if self.nav_mode == "EXPLORE":
                self.nav_mode = "IDLE"
            self.pub_explore_resume.publish(Bool(data=False))
            self.cancel_current_nav_goal()

    def explore_status_cb(self, msg: ExploreStatus):
        if msg.status in [ExploreStatus.EXPLORATION_COMPLETE, ExploreStatus.RETURNED_TO_ORIGIN]:
            self.explore_lite_active = False
            if self._random_after_complete_timer: self._random_after_complete_timer.cancel()
            
            def _start_random_after_complete():
                self._random_after_complete_timer.cancel()
                self._random_after_complete_timer = None
                if self.nav_mode == "EXPLORE":
                    self.dispatch_random_goal()
                    
            self._random_after_complete_timer = self.create_timer(5.0, _start_random_after_complete)
        else:
            self.explore_lite_active = True

    # ==========================================================================
    # 智能避让与 Nav2 控制 (Costmap Smart Standoff)
    # ==========================================================================
    def point_cb(self, msg: PointStamped):
        tx, ty = msg.point.x, msg.point.y
        # 滤除微小抖动
        if math.hypot(tx - self.last_target_x, ty - self.last_target_y) < 0.2:
            return
        self.last_target_x, self.last_target_y = tx, ty

        if self.explore_lite_active:
            self.pub_explore_resume.publish(Bool(data=False))

        # 中断可能正在进行的接近行为
        if self.nav_mode in ["APPROACH_OBJECT", "APPROACH_BOX"]:
            self.pub_cmd_vel.publish(Twist()) 

        if self.target_phase == "OBJECT":
            self.nav_mode = "GOTO_OBJECT"
            self.get_logger().info("[Nav2] Calculating safe staging point for object based on Costmap...")
            self._execute_smart_standoff_nav(tx, ty)
        else:
            if self.nav_mode not in ["GOTO_BOX_ORIGIN", "GOTO_BOX_TARGET"]:
                self.nav_mode = "GOTO_BOX_ORIGIN"
                self.pending_box_x, self.pending_box_y = tx, ty
                self.get_logger().info("[Nav2] Moving to drop-off: Retreating to origin (0.0, 0.0) first...")
                q_origin = Quaternion()
                q_origin.z, q_origin.w = math.sin(math.pi / 2.0), math.cos(math.pi / 2.0)
                self.send_nav_goal(0.0, 0.0, q_origin)
            elif self.nav_mode == "GOTO_BOX_ORIGIN":
                self.pending_box_x, self.pending_box_y = tx, ty
            elif self.nav_mode == "GOTO_BOX_TARGET":
                self.pending_box_x, self.pending_box_y = tx, ty
                self.get_logger().info("[Nav2] Calculating safe staging point for box based on Costmap...")
                self._execute_smart_standoff_nav(tx, ty)

    def _execute_smart_standoff_nav(self, tx, ty):
        """
        以目标点为中心生成12个候选点(距离0.5m)，根据代价地图选取最优停靠点。
        """
        if self.current_pose is None or self.latest_costmap is None:
            self.get_logger().warn("[Warn] Missing pose or costmap, using fallback linear standoff!")
            rx, ry = self.current_pose if self.current_pose else (0.0, 0.0)
            yaw = math.atan2(ty - ry, tx - rx)
            best_x = tx - self.standoff_dist * math.cos(yaw)
            best_y = ty - self.standoff_dist * math.sin(yaw)
        else:
            rx, ry = self.current_pose
            best_x, best_y = tx, ty
            min_cost = 256
            min_dist = 9999.0

            # 评估 12 个方向 / 30 degree intervals
            for i in range(12):
                angle = i * (math.pi / 6.0) 
                px = tx + self.standoff_dist * math.cos(angle)
                py = ty + self.standoff_dist * math.sin(angle)
                
                cost = self.get_cost_at(px, py)
                dist = math.hypot(px - rx, py - ry)
                
                # 选择代价最小的，代价相同时选距离机器人最近的
                if cost < min_cost or (cost == min_cost and dist < min_dist):
                    min_cost = cost
                    min_dist = dist
                    best_x, best_y = px, py
            
            if min_cost >= 253:
                self.get_logger().warn("[Warn] All staging points within 0.5m are in lethal obstacles!")

        # 让机器人抵达后正对目标
        q_facing_target = self.get_quaternion_from_a_to_b((best_x, best_y), (tx, ty))
        self.send_nav_goal(best_x, best_y, q_facing_target)

    def get_random_free_point(self) -> tuple:
        if self.latest_map is None or self.latest_costmap is None or self.current_pose is None: return None
        map_data, costmap_data = self.latest_map, self.latest_costmap
        rx, ry = self.current_pose
        for _ in range(1000):
            idx = random.randint(0, map_data.info.width * map_data.info.height - 1)
            if map_data.data[idx] == 0:
                x = map_data.info.origin.position.x + (idx % map_data.info.width + 0.5) * map_data.info.resolution
                y = map_data.info.origin.position.y + (idx // map_data.info.width + 0.5) * map_data.info.resolution
                if math.hypot(x - rx, y - ry) > 1.0:
                    cm_info = costmap_data.info
                    cm_x, cm_y = int((x - cm_info.origin.position.x) / cm_info.resolution), int((y - cm_info.origin.position.y) / cm_info.resolution)
                    if 0 <= cm_x < cm_info.width and 0 <= cm_y < cm_info.height:
                        if costmap_data.data[cm_y * cm_info.width + cm_x] <= 250:
                            return (x, y)
        return None

    def dispatch_random_goal(self):
        self.cancel_current_nav_goal()
        target = self.get_random_free_point()
        if target is None or self.current_pose is None:
            self.create_timer(1.0, self.dispatch_random_goal)
            return
        
        self.get_logger().info(f"[Explore] Dispatching random goal to: x={target[0]:.2f}, y={target[1]:.2f}")
        q_facing_target = self.get_quaternion_from_a_to_b(self.current_pose, target)
        self.send_nav_goal(target[0], target[1], q_facing_target)

    def send_nav_goal(self, x: float, y: float, orientation_q: Quaternion):
        self.cancel_current_nav_goal() # 在发送新目标前，保证彻底取消旧目标
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
        """Nav2 到达预定地点后，进入视觉接近模式"""
        status = future.result().status
        self.goal_handle = None

        if status == GoalStatus.STATUS_SUCCEEDED:
            if self.nav_mode == "GOTO_OBJECT":
                self.get_logger().info("[Nav2 Complete] Reached staging point, starting visual approach -> APPROACH_OBJECT")
                self.nav_mode = "APPROACH_OBJECT"
                self.start_approach_sequence()
                
            elif self.nav_mode == "GOTO_BOX_ORIGIN":
                self.get_logger().info("[Nav2 Complete] Reached origin (0.0, 0.0), continuing to drop-off staging point")
                self.nav_mode = "GOTO_BOX_TARGET"
                self._execute_smart_standoff_nav(self.pending_box_x, self.pending_box_y)
                
            elif self.nav_mode == "GOTO_BOX_TARGET":
                self.get_logger().info("[Nav2 Complete] Reached drop-off staging point, starting visual approach -> APPROACH_BOX")
                self.nav_mode = "APPROACH_BOX"
                self.start_approach_sequence()

            elif self.nav_mode == "EXPLORE" and not self.explore_lite_active:
                self.dispatch_random_goal()

        elif status != GoalStatus.STATUS_CANCELED:
            # 失败重试
            if self.nav_mode == "EXPLORE" and not self.explore_lite_active:
                self.dispatch_random_goal()
            elif self.nav_mode == "GOTO_BOX_ORIGIN":
                q_origin = Quaternion()
                q_origin.z, q_origin.w = math.sin(math.pi / 2.0), math.cos(math.pi / 2.0)
                self.send_nav_goal(0.0, 0.0, q_origin)
            elif self.nav_mode in ["GOTO_OBJECT", "GOTO_BOX_TARGET"]:
                tx = self.last_target_x if self.nav_mode == "GOTO_OBJECT" else self.pending_box_x
                ty = self.last_target_y if self.nav_mode == "GOTO_OBJECT" else self.pending_box_y
                self._execute_smart_standoff_nav(tx, ty)

    # ==========================================================================
    # 视觉精准接近阶段 / Visual Close Approach Phase (PD Control)
    # ==========================================================================
    def start_approach_sequence(self):
        """初始化 PD 状态"""
        self.prev_error_x = 0.0
        self.prev_error_z = 0.0
        self.prev_time = self.get_clock().now()
        self.last_msg_time = self.get_clock().now()
        self.pub_cmd_vel.publish(Twist()) # 停机消除残余速度

    def vision_target_cb(self, msg: ObjectTarget):
        """
        视觉控制介入：根据 FSM 传来的 task info 过滤物体
        """
        if self.nav_mode not in ["APPROACH_OBJECT", "APPROACH_BOX"]:
            return
            
        # 根据目标要求过滤，防止被别的颜色块吸走
        target_type = "object" if self.nav_mode == "APPROACH_OBJECT" else "box"
        if msg.color != self.active_task_color or msg.name != target_type:
            return

        self.last_msg_time = self.get_clock().now()
        
        cmd_vel_msg = self.calculate_pd_command(msg)
        self.pub_cmd_vel.publish(cmd_vel_msg)

    def calculate_pd_command(self, msg: ObjectTarget) -> Twist:
        cmd = Twist()
        current_time = self.get_clock().now()
        
        dt = (current_time - self.prev_time).nanoseconds / 1e9
        if dt <= 0: dt = 0.001 

        # 计算误差 (期望Z距摄像头=0.325m, 期望横向居中X=0)
        error_x = msg.x - 0.0
        error_z = msg.z - 0.325

        distance_error = math.hypot(error_x, error_z)
        
        if distance_error < 0.01:
            self.get_logger().info(f'[Approach Complete] Position error {distance_error:.3f}m, notifying main controller.')
            self.pub_fb.publish(Bool(data=True)) # 彻底完成，交给 FSM
            
            # 状态交接
            if self.nav_mode == "APPROACH_OBJECT":
                self.target_phase = "BOX" 
            self.nav_mode = "IDLE"
            self.last_target_x, self.last_target_y = 999.0, 999.0
            
            return Twist()

        # PD 计算
        deriv_z = (error_z - self.prev_error_z) / dt
        linear_output = (self.kp_linear * error_z) + (self.kd_linear * deriv_z)

        deriv_x = (error_x - self.prev_error_x) / dt
        angular_output = (self.kp_angular * error_x) + (self.kd_angular * deriv_x)

        # 限幅输出
        cmd.linear.x = max(-0.1, min(0.1, linear_output))
        cmd.angular.z = max(-0.1, min(0.1, -angular_output))

        self.prev_error_z = error_z
        self.prev_error_x = error_x
        self.prev_time = current_time

        return cmd

    def watchdog_callback(self):
        """看门狗：目标丢失超过0.5秒发急停"""
        if self.nav_mode not in ["APPROACH_OBJECT", "APPROACH_BOX"]:
            return

        current_time = self.get_clock().now()
        time_since_last_msg = (current_time - self.last_msg_time).nanoseconds / 1e9
        
        if time_since_last_msg > self.timeout_sec:
            self.pub_cmd_vel.publish(Twist())
            self.get_logger().warn('[Watchdog] Visual target lost! Sending emergency stop command.', throttle_duration_sec=2.0)
            self.prev_error_x = 0.0
            self.prev_error_z = 0.0
            self.prev_time = current_time

def main():
    rclpy.init()
    node = NavControllerNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt: 
        pass

if __name__ == '__main__':
    main()