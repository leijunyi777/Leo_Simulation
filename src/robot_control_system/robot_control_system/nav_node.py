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
        """
        节点初始化函数。
        主要负责：声明状态变量、初始化 TF 监听器、连接 Nav2 动作服务端、创建各个话题的发布者和订阅者。
        """
        super().__init__('nav_controller_node')
        
        # --- 核心状态变量 ---
        self.nav_mode = "IDLE"  
        self.target_phase = "OBJECT" 

        # 引入永久压制开关，初始为开启状态
        self.suppression_enabled = True

        self.goal_handle = None
        self.latest_map = None  
        self.latest_costmap = None  # 用于存储最新的全局代价地图数据
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

        # 初始化 TF 监听器，用于获取机器人自身在地图中的位姿
        self.tf_buffer = Buffer()
        self.tf_listener = TransformListener(self.tf_buffer, self)

        # 初始化 Nav2 动作客户端，用于发送导航目标点
        self.nav_client = ActionClient(self, NavigateToPose, 'navigate_to_pose')
        self.get_logger().info('正在等待 Nav2 动作服务端 (navigate_to_pose) 启动...')
        while not self.nav_client.wait_for_server(timeout_sec=2.0):
            self.get_logger().info('Nav2 服务端尚未就绪，继续等待中...')

        self.get_logger().info('Nav2 服务端已连接！节点准备就绪。') 

        # 订阅与发布者的设置
        self.sub_cmd_explore = self.create_subscription(Bool, '/nav/cmd_explore', self.explore_cb, 10)
        self.sub_goal_point  = self.create_subscription(PointStamped, '/nav/goal_point', self.point_cb, 10)
        self.pub_fb = self.create_publisher(Bool, '/nav/goal_reached', 10)
        self.sub_manip_fb = self.create_subscription(Bool, '/manipulator_feedback', self.manip_feedback_cb, 10)

        self.pub_explore_resume = self.create_publisher(Bool, 'explore/resume', 10)
        self.sub_explore_status = self.create_subscription(ExploreStatus, 'explore/status', self.explore_status_cb, 10)
        
        # 订阅静态地图和代价地图
        self.sub_map = self.create_subscription(OccupancyGrid, '/map', self.map_cb, 10)
        # 【新增】订阅全局代价地图以供后续取点时检查代价值
        self.sub_costmap = self.create_subscription(OccupancyGrid, '/global_costmap/costmap', self.costmap_cb, 10)

        # 定时器设置
        self.pose_timer = self.create_timer(0.05, self.update_current_pose_cb) # 50ms 获取一次位姿
        self._idle_timer = self.create_timer(0.1, self._keep_explore_paused_cb) # 100ms 执行一次探索压制

    # ==========================================================================
    # 基础位姿与计算工具
    # ==========================================================================
    def update_current_pose_cb(self):
        """
        定时器回调函数：更新机器人当前在 map 坐标系下的位姿。
        通过 TF 树查询 map 到 base_footprint 的变换关系。
        """
        try:
            t = self.tf_buffer.lookup_transform('map', 'base_footprint', rclpy.time.Time())
            self.current_pose = (t.transform.translation.x, t.transform.translation.y)
        except TransformException:
            pass 

    def map_cb(self, msg: OccupancyGrid):
        """
        话题回调函数：接收并保存最新的静态地图数据。
        """
        self.latest_map = msg

    def costmap_cb(self, msg: OccupancyGrid):
        """
        话题回调函数：接收并保存最新的代价地图(costmap)数据。
        供 get_random_free_point 中检查代价值使用。
        """
        self.latest_costmap = msg

    def get_quaternion_from_a_to_b(self, a_pos: tuple, b_pos: tuple) -> Quaternion:
        """
        辅助计算函数：计算从点 A 指向点 B 的四元数朝向。
        返回的四元数代表的偏航角 (Yaw) 是机器人在 A 点面朝 B 点的角度。
        """
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
        压制定时器回调函数：
        在初始状态且处于 IDLE 时，持续向 m-explore 发布 False（暂停）指令。
        一旦 suppression_enabled 开关变为 False，则不再触发。
        """
        if self.suppression_enabled and self.nav_mode == "IDLE":
            self.pub_explore_resume.publish(Bool(data=False))

    def explore_cb(self, msg: Bool):
        """
        话题回调函数：处理探索启停指令。
        如果是首次接收到 True，则永久解除对 m-explore 的初始压制。
        若不处于 explore_lite 激活状态，则派发一个随机导航点作为过渡。
        """
        self.cmd_explore_enabled = msg.data
        if msg.data:
            # 只要收到第一次 True，永久关闭压制功能
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
            if self.goal_handle:
                self.goal_handle.cancel_goal_async()

    def explore_status_cb(self, msg: ExploreStatus):
        """
        话题回调函数：监听 m-explore 包的运行状态。
        当探索完成或回到起点时，取消活跃状态，并在3秒后派发一个随机点重新激活机器人。
        """
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
        """
        话题回调函数：处理收到的具体目标点 (如抓取目标或放置目标)。
        根据当前的阶段 (OBJECT 或 BOX) 规划前往目标附近避让点的路径。
        """
        tx, ty = msg.point.x, msg.point.y
        # 滤除抖动：如果新目标点与上一次的目标点距离过近，则忽略
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
        """
        计算逻辑函数：给定一个物体中心坐标，计算并发送一个距离其 standoff_dist 距离的停靠点，
        同时使机器人的车头面向目标物。
        """
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
        """
        话题回调函数：接收机械臂执行状态反馈。
        用来判定是否可以恢复底盘的自主探索状态。
        """
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
        """
        【修改的重点函数】获取地图上的随机安全点。
        条件：
        1. static map 对应值为 0 (空闲)。
        2. 距离当前位姿的距离大于 1m。
        3. 代价地图(costmap) 对应位置的值 <= 250。
        """
        # 确保必须的数据都已加载
        if self.latest_map is None or self.latest_costmap is None or self.current_pose is None: 
            return None
            
        map_data = self.latest_map
        costmap_data = self.latest_costmap
        rx, ry = self.current_pose
        
        for _ in range(1000):
            # 在静态地图中随机抽取一个索引
            idx = random.randint(0, map_data.info.width * map_data.info.height - 1)
            
            # 条件 1: 静态地图为空闲
            if map_data.data[idx] == 0:
                # 将地图索引转换为物理世界坐标 (x, y)
                x = map_data.info.origin.position.x + (idx % map_data.info.width + 0.5) * map_data.info.resolution
                y = map_data.info.origin.position.y + (idx // map_data.info.width + 0.5) * map_data.info.resolution
                
                # 条件 2: 目标点距离当前位置大于 1m
                if math.hypot(x - rx, y - ry) > 1.0:
                    
                    # 条件 3: 检查 costmap 值
                    # 根据物理坐标反算在代价地图中的网格坐标 (因为 costmap 和 map 的分辨率/原点可能不同)
                    cm_info = costmap_data.info
                    cm_x = int((x - cm_info.origin.position.x) / cm_info.resolution)
                    cm_y = int((y - cm_info.origin.position.y) / cm_info.resolution)
                    
                    # 确保该点处于代价地图范围之内
                    if 0 <= cm_x < cm_info.width and 0 <= cm_y < cm_info.height:
                        cm_idx = cm_y * cm_info.width + cm_x
                        # costmap 值 <= 250 才采纳（避免生成在膨胀致命层内）
                        if costmap_data.data[cm_idx] <= 250:
                            return (x, y)
        return None

    def dispatch_random_goal(self):
        """
        调度函数：取消当前目标(如有)，并生成一个新的随机点，发送给 Nav2 服务端。
        """
        if self.goal_handle:
            self.goal_handle.cancel_goal_async()
            self.goal_handle = None
            
        target = self.get_random_free_point()
        # 如果找不到合适的点或者未就绪，1秒后重试
        if target is None or self.current_pose is None:
            self.create_timer(1.0, self.dispatch_random_goal)
            return
            
        # 让机器人面向生成的随机点
        q_facing_target = self.get_quaternion_from_a_to_b(self.current_pose, target)
        self.send_nav_goal(target[0], target[1], q_facing_target)

    def send_nav_goal(self, x: float, y: float, orientation_q: Quaternion):
        """
        动作请求函数：将坐标点 (x,y) 结合朝向四元数打包成 NavigateToPose 目标，发送给 Nav2 动作服务端。
        """
        pose = PoseStamped()
        pose.header.frame_id = 'map'
        pose.header.stamp = self.get_clock().now().to_msg()
        pose.pose.position.x = float(x)
        pose.pose.position.y = float(y)
        pose.pose.orientation = orientation_q
        self.nav_client.send_goal_async(NavigateToPose.Goal(pose=pose)).add_done_callback(self.goal_response_cb)

    def goal_response_cb(self, future):
        """
        动作回调函数：接收 Nav2 服务端是否接受目标点的响应。
        若被拒绝且处于探索状态，则重新派发新的随机点。
        """
        self.goal_handle = future.result()
        if not self.goal_handle.accepted:
            if self.nav_mode == "EXPLORE" and not self.explore_lite_active:
                self.dispatch_random_goal()
            return
        self.goal_handle.get_result_async().add_done_callback(self.result_cb)

    def result_cb(self, future):
        """
        动作回调函数：处理 Nav2 导航执行的结果 (成功、失败、取消等)。
        根据不同的导航模式 (抓取停靠、回到原点、放置停靠) 推进状态机流转。
        """
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
            # 导航失败(且未被取消)的容错处理，重发相应的指令
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