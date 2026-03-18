"""
================================================================================
组件：导航控制节点（实机融合版 - 抓取/放置导航强化）
================================================================================
核心修改与满足的需求：
1. 抓取与放置目标强化：导航目标不再是输入点本身，而是严格位于“当前位置->目标点”连线上，距离目标点 370mm 的坐标处。
2. 朝向控制：机器人到达 370mm 停靠点时，朝向严格对准目标点。
3. 放置阶段(DROP)分段导航策略：
   - 收到放置(Box)坐标后，拦截直接前往的指令。
   - 阶段一：先强制开回 (0,0) 点，朝向设为 math.pi (起始方向的反方向)。
   - 阶段二：到达 (0,0) 后，再利用当前点(即起点)与 Box 的连线计算 370mm 的目标，执行前往，保证极佳的进场角度。
================================================================================
"""

import math                                      # 导入数学库，用于计算角度、距离、三角函数等
import random                                    # 导入随机数库，用于在地图上随机选取空闲目标点
import rclpy                                     # 导入ROS 2 Python客户端核心库
from rclpy.node import Node                      # 导入ROS 2节点基类
from rclpy.parameter import Parameter            # 导入ROS 2参数类，用于声明和获取节点参数
from rclpy.action import ActionClient            # 导入ROS 2动作客户端类，用于调用Nav2的导航Action

from std_msgs.msg import Bool                    # 导入布尔类型消息，用于控制指令和状态反馈
from geometry_msgs.msg import PointStamped, PoseStamped, Quaternion  # 导入几何消息（带时间戳的点、位姿、四元数）
from nav_msgs.msg import OccupancyGrid           # 导入栅格地图消息类型
from nav2_msgs.action import NavigateToPose      # 导入Nav2的核心导航Action接口
from action_msgs.msg import GoalStatus           # 导入动作目标状态枚举，用于判断导航成功与否
from tf2_ros import Buffer, TransformListener, TransformException # 导入TF2相关类，用于坐标系变换监听
from explore_lite_msgs.msg import ExploreStatus  # 导入m-explore-ros的状态消息，用于监听前沿探索状态

class NavControllerNode(Node):
    """
    导航控制节点主类。
    负责监听状态机指令、接管与调度m-explore前沿探索、计算抓取/放置的避让位姿，
    并作为客户端向Nav2发送最终的导航目标。
    """
    def __init__(self):
        # 初始化ROS 2节点，节点名为'nav_controller_node'
        super().__init__('nav_controller_node')
        
        # 声明一个名为'use_sim_time'的参数，默认值为False（实机运行模式）
        self.declare_parameter('use_sim_time', False)
        
        # --- 核心状态变量 ---
        # 导航底层模式，可选值: IDLE(待机), EXPLORE(探索), GOTO_OBJECT(去抓物体), GOTO_BOX_ORIGIN(去放箱子前的回零), GOTO_BOX_TARGET(去放箱子)
        self.nav_mode = "IDLE"  
        # 周期追踪器：用于区分当前FSM发来的坐标是"OBJECT" (准备去抓) 还是 "BOX" (准备去放)
        self.target_phase = "OBJECT" 

        # 存放当前正在执行的Nav2 Action目标句柄，便于随时取消当前导航动作
        self.goal_handle = None
        # 存放最新接收到的全局栅格地图数据
        self.latest_map = None  
        # 存放机器人当前在map坐标系下的实时位置 (x, y)
        self.current_pose = None 
        
        # 记录上一次接收到的目标点X坐标，用于防抖（避免重复发送相同点）
        self.last_target_x = 999.0
        # 记录上一次接收到的目标点Y坐标
        self.last_target_y = 999.0
        # 暂存箱子的X坐标，用于在返回(0,0)点期间记忆箱子最终目标位置
        self.pending_box_x = 999.0
        # 暂存箱子的Y坐标
        self.pending_box_y = 999.0
        
        # 定义机器人与目标（物体或箱子）连线时，需要保持的后退避让距离：370mm (0.37米)
        self.standoff_dist = 0.37  

        # 标志位：记录m-explore节点当前是否处于活跃探索状态
        self.explore_lite_active = False     
        # 标志位：记录FSM下发的探索指令(cmd_explore)当前是否为True
        self.cmd_explore_enabled = False     
        # 计数器：记录因执行GOTO任务而打断探索后，需要收到几次机械臂反馈才能恢复探索（-1代表无需等待）
        self.pending_manip_fb_for_resume = -1 
        # 定时器句柄：用于在前沿探索完成后，延迟几秒再启动随机目标探索
        self._random_after_complete_timer = None

        # --- TF2 监听器 ---
        # 创建一个TF缓存器，用于存储一段时间内的坐标系变换关系
        self.tf_buffer = Buffer()
        # 创建TF监听器，自动在后台接收TF话题并将其填入缓存器
        self.tf_listener = TransformListener(self.tf_buffer, self)

        # --- Action 客户端 (Nav2) ---
        # 创建指向Nav2的'navigate_to_pose'动作服务器的客户端
        self.nav_client = ActionClient(self, NavigateToPose, 'navigate_to_pose')
        # 阻塞等待，直到Nav2动作服务器在线
        self.nav_client.wait_for_server()
        # 打印日志提示Nav2已准备就绪
        self.get_logger().info('Nav2 已就绪，等待 FSM 指令.')

        # --- 订阅与发布者 ---
        # 订阅FSM下发的探索启停指令，话题'/nav/cmd_explore'，触发explore_cb回调
        self.sub_cmd_explore = self.create_subscription(Bool, '/nav/cmd_explore', self.explore_cb, 10)
        # 订阅FSM下发的具体目标点坐标，话题'/nav/goal_point'，触发point_cb回调
        self.sub_goal_point  = self.create_subscription(PointStamped, '/nav/goal_point', self.point_cb, 10)
        # 创建发布者，向FSM汇报导航到位状态，话题'/nav/goal_reached'
        self.pub_fb = self.create_publisher(Bool, '/nav/goal_reached', 10)
        # 订阅机械臂的动作反馈，话题'/manipulator_feedback'，触发manip_feedback_cb回调
        self.sub_manip_fb = self.create_subscription(Bool, '/manipulator_feedback', self.manip_feedback_cb, 10)

        # 创建发布者，用于控制m-explore的恢复与暂停，话题'explore/resume'
        self.pub_explore_resume = self.create_publisher(Bool, 'explore/resume', 10)
        # 订阅m-explore自身发布的状态，话题'explore/status'，触发explore_status_cb回调
        self.sub_explore_status = self.create_subscription(ExploreStatus, 'explore/status', self.explore_status_cb, 10)
        # 订阅全局地图数据，话题'/map'，触发map_cb回调更新本地地图
        self.sub_map = self.create_subscription(OccupancyGrid, '/map', self.map_cb, 10)

        # --- 定时器 ---
        # 创建周期为0.05秒(20Hz)的定时器，用于高频刷新自身最新位姿
        self.pose_timer = self.create_timer(0.05, self.update_current_pose_cb)
        # 创建周期为0.1秒的定时器，用于在无探索指令时，持续向m-explore发送暂停指令(死区压制)
        self._idle_timer = self.create_timer(0.1, self._keep_explore_paused_cb)

    # ==========================================================================
    # 基础位姿与计算工具
    # ==========================================================================
    def update_current_pose_cb(self):
        """
        定时器回调函数 (20Hz)。
        通过TF2实时查询map坐标系到base_footprint坐标系的变换，并更新当前位置。
        规避了按需查询可能导致的线程阻塞。
        """
        try:
            # 尝试查询从'map'到'base_footprint'的最新坐标变换
            t = self.tf_buffer.lookup_transform('map', 'base_footprint', rclpy.time.Time())
            # 提取平移向量中的x和y，存入current_pose变量
            self.current_pose = (t.transform.translation.x, t.transform.translation.y)
        except TransformException:
            # 如果TF树中断或未就绪，引发异常时选择忽略（等待下一周期更新）
            pass 

    def map_cb(self, msg: OccupancyGrid):
        """
        地图订阅回调函数。
        将接收到的最新栅格地图保存到成员变量中，供随机点寻路算法使用。
        """
        # 直接将传入的栅格地图消息存入latest_map中
        self.latest_map = msg

    def get_quaternion_from_a_to_b(self, a_pos: tuple, b_pos: tuple) -> Quaternion:
        """
        几何工具函数。
        根据平面上的两点坐标 A 和 B，计算出从 A 指向 B 的偏航角(Yaw)，
        并将其转换为ROS标准四元数(Quaternion)返回。
        """
        # 计算B点相对于A点在X轴的差值
        dx = b_pos[0] - a_pos[0]
        # 计算B点相对于A点在Y轴的差值
        dy = b_pos[1] - a_pos[1]
        # 使用反正切函数atan2算出由A指向B的弧度夹角(Yaw)
        yaw = math.atan2(dy, dx)
        # 创建一个新的四元数对象
        q = Quaternion()
        # 平面旋转，四元数的x和y分量恒为0.0
        q.x, q.y = 0.0, 0.0
        # 根据公式计算四元数的z分量：sin(yaw / 2)
        q.z = math.sin(yaw / 2.0)
        # 根据公式计算四元数的w分量：cos(yaw / 2)
        q.w = math.cos(yaw / 2.0)
        # 返回计算好的四元数对象
        return q

    # ==========================================================================
    # m-explore 控制与状态机同步逻辑
    # ==========================================================================
    def _keep_explore_paused_cb(self):
        """
        压制定时器回调函数。
        如果FSM没有允许探索，则不断发布False压制m-explore，防止其自动苏醒。
        """
        # 如果FSM探索使能标志为False
        if not self.cmd_explore_enabled:
            # 向m-explore的话题发布False，强制其处于暂停状态
            self.pub_explore_resume.publish(Bool(data=False))

    def explore_cb(self, msg: Bool):
        """
        FSM探索指令总开关回调。
        控制导航节点进入探索模式，重置操作周期(OBJECT/BOX)，并唤醒m-explore或启动随机点导航。
        """
        # 将FSM发来的探索开关状态保存到局部变量中
        self.cmd_explore_enabled = msg.data
        # 如果FSM发来的是True（开始探索）
        if msg.data:
            # 开始新的一轮探索，重置周期追踪器为"OBJECT"，表示下一个目标是抓取物体
            self.target_phase = "OBJECT"
            
            # 如果当前底层模式不是EXPLORE，则切换到EXPLORE
            if self.nav_mode != "EXPLORE":
                self.nav_mode = "EXPLORE"
            # 只有当不在等待机械臂完成反馈的阶段时，才唤醒m-explore
            if self.pending_manip_fb_for_resume <= 0:
                # 给m-explore发送True，允许其开始前沿探索
                self.pub_explore_resume.publish(Bool(data=True))
            # 如果m-explore已经完成前沿探索退出，则本节点接管，直接发随机点
            if not self.explore_lite_active:
                self.dispatch_random_goal()
        # 如果FSM发来的是False（停止探索）
        else:
            # 给m-explore发送False，让其立刻闭嘴
            self.pub_explore_resume.publish(Bool(data=False))
            # 如果目前还处于EXPLORE模式，则需要执行刹车
            if self.nav_mode == "EXPLORE":
                # 将模式切回待机IDLE
                self.nav_mode = "IDLE"
                # 如果当前有正在执行的Nav2目标
                if self.goal_handle:
                    # 异步取消该目标，让底盘立刻停车
                    self.goal_handle.cancel_goal_async()

    def explore_status_cb(self, msg: ExploreStatus):
        """
        监听m-explore探索状态回调。
        如果m-explore报告探索完成，则自动接管控制权，切换到随机空闲点导航模式。
        """
        # 如果m-explore发来的状态是“探索完成”或“返回原点”
        if msg.status in [ExploreStatus.EXPLORATION_COMPLETE, ExploreStatus.RETURNED_TO_ORIGIN]:
            # 标志位设为False，表示m-explore已经退役
            self.explore_lite_active = False
            # 如果之前有遗留的定时器，则取消它
            if self._random_after_complete_timer is not None:
                self._random_after_complete_timer.cancel()
            # 定义一个内部函数，用于在几秒后执行接管动作
            def _start_random_after_complete():
                # 清理定时器自身
                self._random_after_complete_timer.cancel()
                self._random_after_complete_timer = None
                # 如果依然处于EXPLORE模式下
                if self.nav_mode == "EXPLORE":
                    # 发送随机点，接管探索控制权
                    self.dispatch_random_goal()
            # 启动一个3秒的单次定时器，3秒后执行接管（留出缓冲时间）
            self._random_after_complete_timer = self.create_timer(3.0, _start_random_after_complete)
        # 如果发来的不是完成状态，则说明m-explore正在活跃工作中
        else:
            self.explore_lite_active = True

    # ==========================================================================
    # 重点更改：分发与避让计算逻辑
    # ==========================================================================
    def point_cb(self, msg: PointStamped):
        """
        FSM下发具体坐标点的回调函数。
        根据当前的追踪阶段(OBJECT/BOX)，判断是直接去抓物体(计算避让点)，
        还是去放箱子(执行先回零点，再逼近目标的二段路线)。
        """
        # 提取目标点坐标X、Y
        tx, ty = msg.point.x, msg.point.y
        
        # 防抖检查：如果新发来的目标和上次一样（误差在0.2米内），则直接丢弃不处理
        if math.hypot(tx - self.last_target_x, ty - self.last_target_y) < 0.2:
            return

        # 更新最新的目标坐标缓存
        self.last_target_x, self.last_target_y = tx, ty

        # 收到明确GOTO目标时，打断探索行为，向m-explore发送False
        if self.explore_lite_active:
            self.pub_explore_resume.publish(Bool(data=False))
            # 记录此时需要等待2次机械臂反馈才能恢复探索（防止底盘乱跑）
            self.pending_manip_fb_for_resume = 2

        # 判断当前周期追踪器：是否处于“前往抓取物体”阶段
        if self.target_phase == "OBJECT":
            # 切换导航模式为去抓物体
            self.nav_mode = "GOTO_OBJECT"
            # 打印日志提示计算避让点
            self.get_logger().info("【前往抓取】：计算距离目标 370mm 处的停靠点")
            # 调用避让导航函数，计算与目标点连线上退后370mm的坐标并发给Nav2
            self._execute_standoff_nav(tx, ty)
        
        # 否则，当前处于“前往放置箱子”阶段
        else:
            # 检查是否是首次收到该阶段的指令（即模式尚未切入放置专用模式）
            if self.nav_mode not in ["GOTO_BOX_ORIGIN", "GOTO_BOX_TARGET"]:
                # 切换导航模式为：前往放置箱子的阶段一（回起点）
                self.nav_mode = "GOTO_BOX_ORIGIN"
                # 将传来的箱子坐标暂存入pending变量中，留待抵达起点后使用
                self.pending_box_x, self.pending_box_y = tx, ty
                
                # 打印日志提示开启回零操作
                self.get_logger().info("【前往放置-阶段1】：先返回(0,0)，朝向起始反方向(Yaw=pi)")
                # 实例化一个朝向起点的四元数
                q_origin = Quaternion()
                q_origin.x, q_origin.y = 0.0, 0.0
                # 设置Yaw=pi的四元数z分量
                q_origin.z = math.sin(math.pi / 2.0)
                # 设置Yaw=pi的四元数w分量
                q_origin.w = math.cos(math.pi / 2.0)
                # 直接向Nav2发送目标(0.0, 0.0)，带着上述反向四元数
                self.send_nav_goal(0.0, 0.0, q_origin)
                
            # 如果已经在前往起点(0,0)的途中
            elif self.nav_mode == "GOTO_BOX_ORIGIN":
                # 途中若视觉持续刷新箱子坐标，这里静默更新暂存变量，不打断正前往起点的车
                self.pending_box_x, self.pending_box_y = tx, ty
                
            # 如果已经到达起点，正处于前往箱子最终避让点的途中
            elif self.nav_mode == "GOTO_BOX_TARGET":
                # 更新暂存变量
                self.pending_box_x, self.pending_box_y = tx, ty
                # 再次调用避让函数，修正最新的箱子坐标进行逼近
                self._execute_standoff_nav(tx, ty)

    def _execute_standoff_nav(self, tx, ty):
        """
        连线避让计算核心函数。
        根据机器人当前点和目标点(tx, ty)，算出距离目标370mm的点，
        并让机器人朝向严格对准目标，最后下发给Nav2。
        """
        # 如果还未获取到当前位姿，则报警并直接退出
        if self.current_pose is None:
            self.get_logger().warn("尚未获取到自身位姿，无法计算连线避让路径！")
            return

        # 提取当前自身坐标X、Y
        rx, ry = self.current_pose
        
        # 1. 获得由当前点指向目标点的四元数朝向，用于最终停车姿态
        q_facing_target = self.get_quaternion_from_a_to_b((rx, ry), (tx, ty))
        # 计算当前点指向目标点的偏航角Yaw，辅助后续三角运算
        yaw = math.atan2(ty - ry, tx - rx)
        
        # 2. 从目标点tx, ty沿着刚算出的夹角反方向，倒退standoff_dist(370mm)作为实际导航点gx, gy
        gx = tx - self.standoff_dist * math.cos(yaw)
        gy = ty - self.standoff_dist * math.sin(yaw)

        # 组装计算好的点位和朝向，下发给Nav2执行
        self.send_nav_goal(gx, gy, q_facing_target)

    def manip_feedback_cb(self, msg: Bool):
        """
        机械臂反馈回调。
        用于在GOTO任务打断探索后，依靠机械臂传来的反馈信号来逐级解封并恢复探索。
        """
        # 如果不是有效的成功反馈，或者并不处于等待反馈状态中，则直接忽略
        if not msg.data or self.pending_manip_fb_for_resume <= 0:
            return
        # 收到一次反馈，计数器减一
        self.pending_manip_fb_for_resume -= 1
        # 如果计数器归零，说明机械臂完整的一组动作结束了
        if self.pending_manip_fb_for_resume == 0:
            # 重置计数器标记为不用等待状态
            self.pending_manip_fb_for_resume = -1
            # 如果FSM总开关仍然允许探索，则此时才真正恢复m-explore的探索行为
            if self.cmd_explore_enabled:
                self.pub_explore_resume.publish(Bool(data=True))

    # ==========================================================================
    # Nav2 目标派发与反馈核心
    # ==========================================================================
    def get_random_free_point(self) -> tuple:
        """
        在全局地图中随机采样找空闲点函数。
        尝试最多1000次，寻找栅格值为0的空地，并将其换算为真实物理世界坐标。
        """
        # 如果还没有接收到地图数据，则返回None
        if self.latest_map is None: return None
        # 缓存地图对象
        map_data = self.latest_map
        # 最多尝试随机采样1000次
        for _ in range(1000):
            # 在地图数据数组长度范围内产生一个随机索引
            idx = random.randint(0, map_data.info.width * map_data.info.height - 1)
            # 值为0代表该栅格绝对空闲无障碍物
            if map_data.data[idx] == 0:
                # 换算X坐标：起始X + (列数 + 0.5中心偏移) * 分辨率
                x = map_data.info.origin.position.x + (idx % map_data.info.width + 0.5) * map_data.info.resolution
                # 换算Y坐标：起始Y + (行数 + 0.5中心偏移) * 分辨率
                y = map_data.info.origin.position.y + (idx // map_data.info.width + 0.5) * map_data.info.resolution
                # 找到合格点，打包元组返回
                return (x, y)
        # 若循环耗尽还没找到，返回None
        return None

    def dispatch_random_goal(self):
        """
        随机目标调度主函数。
        取消已有目标，获取新空闲点，算出对齐朝向，最后组装发送给Nav2。
        """
        # 如果当前正有一个Nav2目标正在执行
        if self.goal_handle:
            # 异步取消该目标
            self.goal_handle.cancel_goal_async()
            # 句柄置空
            self.goal_handle = None

        # 调用函数，尝试获取一个空闲随机点
        target = self.get_random_free_point()
        # 如果没找到随机点，或者当前还不知道自己的位姿
        if target is None or self.current_pose is None:
            # 开启一个1秒单次定时器，1秒后重新再试本函数
            self.create_timer(1.0, self.dispatch_random_goal)
            return
            
        # 根据当前位置指向找到的随机点，生成一个对齐的四元数朝向
        q_facing_target = self.get_quaternion_from_a_to_b(self.current_pose, target)
        # 将随机点和算好的朝向下发给Nav2进行导航
        self.send_nav_goal(target[0], target[1], q_facing_target)

    def send_nav_goal(self, x: float, y: float, orientation_q: Quaternion):
        """
        组装目标并向Nav2服务器发送Action请求的基础函数。
        """
        # 实例化位姿对象
        pose = PoseStamped()
        # 设定坐标系为全局map
        pose.header.frame_id = 'map'
        # 打上最新的时间戳
        pose.header.stamp = self.get_clock().now().to_msg()
        # 填入目标平移X坐标
        pose.pose.position.x = float(x)
        # 填入目标平移Y坐标
        pose.pose.position.y = float(y)
        # 填入指定的四元数朝向
        pose.pose.orientation = orientation_q
        # 异步调用Nav2接口发送目标，并绑定goal_response_cb作为接收响应回调
        self.nav_client.send_goal_async(NavigateToPose.Goal(pose=pose)).add_done_callback(self.goal_response_cb)

    def goal_response_cb(self, future):
        """
        Action服务端是否接受目标请求的回调。
        """
        # 从Future对象中获取目标句柄
        self.goal_handle = future.result()
        # 如果Nav2拒绝了我们的导航请求（例如目标在障碍物里不可达）
        if not self.goal_handle.accepted:
            # 如果当时处于EXPLORE模式且是在执行随机导航
            if self.nav_mode == "EXPLORE" and not self.explore_lite_active:
                # 直接重新调度获取下一个新的随机点
                self.dispatch_random_goal()
            return
        # 若目标被接受，则继续请求获取动作的最终执行结果，绑定result_cb回调
        self.goal_handle.get_result_async().add_done_callback(self.result_cb)

    def result_cb(self, future):
        """
        导航结果最终状态回调函数。
        控制状态流转：处理成功抵达、目标切换、避让重试以及下达下一阶段目标。
        """
        # 提取Nav2反馈的状态码
        status = future.result().status
        # 执行完毕，将当前句柄置空
        self.goal_handle = None

        # 如果状态码是执行成功(STATUS_SUCCEEDED)
        if status == GoalStatus.STATUS_SUCCEEDED:
            # 1. 成功抵达待抓取的物体避让点
            if self.nav_mode == "GOTO_OBJECT":
                # 发布True，告知FSM底层到达位姿
                self.pub_fb.publish(Bool(data=True)) 
                # 底层导航切回待机状态
                self.nav_mode = "IDLE"
                # 【关键】翻转追踪状态器为"BOX"，下次FSM发来的点将走放置逻辑
                self.target_phase = "BOX"            
                # 重置缓存的目标坐标点，解开防抖锁
                self.last_target_x, self.last_target_y = 999.0, 999.0
                
            # 2. 成功抵达(0,0)起点的放置阶段一
            elif self.nav_mode == "GOTO_BOX_ORIGIN":
                # 打印日志
                self.get_logger().info("【前往放置-阶段2】：已到达(0,0)，现在前往箱子前 370mm 的避让点")
                # 切换为前往箱子避让点模式
                self.nav_mode = "GOTO_BOX_TARGET"
                # 调用连线避让函数，传入之前暂存的箱子坐标点前往执行
                self._execute_standoff_nav(self.pending_box_x, self.pending_box_y)
                
            # 3. 成功抵达箱子前方避让点的放置阶段二
            elif self.nav_mode == "GOTO_BOX_TARGET":
                # 发布True，告知FSM可以放下箱子了
                self.pub_fb.publish(Bool(data=True)) 
                # 底层导航切回待机状态
                self.nav_mode = "IDLE"
                # 重置缓存目标，完成本轮周期。再次开始寻找时，explore_cb会自动重置追踪器为"OBJECT"
                self.last_target_x, self.last_target_y = 999.0, 999.0

            # 4. 如果是随机点探索模式成功抵达
            elif self.nav_mode == "EXPLORE" and not self.explore_lite_active:
                # 这个随机点走完了，自动调度抛出下一个随机点继续走
                self.dispatch_random_goal()

        # 如果状态码不是主动取消(被意外打断)，而是属于执行失败(规划不通/避障卡死等)
        elif status != GoalStatus.STATUS_CANCELED:
            # 在探索模式失败，说明遇到障碍走不通了
            if self.nav_mode == "EXPLORE" and not self.explore_lite_active:
                # 重新抛出下一个随机点
                self.dispatch_random_goal()
            # 在回原点模式失败（走不到0,0）
            elif self.nav_mode == "GOTO_BOX_ORIGIN":
                # 重新构建原点目标四元数重试
                q_origin = Quaternion()
                q_origin.x, q_origin.y = 0.0, 0.0
                q_origin.z, q_origin.w = math.sin(math.pi / 2.0), math.cos(math.pi / 2.0)
                # 再次向原点发起冲击
                self.send_nav_goal(0.0, 0.0, q_origin)
            # 在前往物体或最终放置点途中失败
            elif self.nav_mode in ["GOTO_OBJECT", "GOTO_BOX_TARGET"]:
                # 提取出因失败打断而属于该模式应当对应的目标X、Y值
                tx = self.last_target_x if self.nav_mode == "GOTO_OBJECT" else self.pending_box_x
                ty = self.last_target_y if self.nav_mode == "GOTO_OBJECT" else self.pending_box_y
                # 基于目前可能微调后的位置，重新计算避让连线并发起重试导航
                self._execute_standoff_nav(tx, ty)

def main():
    """
    ROS 2 程序入口。
    负责初始化ROS库、实例化节点并使节点进入事件循环。
    """
    # 初始化ROS 2环境
    rclpy.init()
    # 实例化导航控制节点
    node = NavControllerNode()
    try:
        # 开始自旋监听，不断执行回调直至被中断
        rclpy.spin(node)
    except KeyboardInterrupt: 
        # 捕捉Ctrl+C终端退出信号，静默通过
        pass
    finally:
        # 安全销毁节点
        node.destroy_node()
        # 清理ROS 2相关资源
        rclpy.shutdown()

# 判断本脚本是否被当做主程序执行
if __name__ == '__main__':
    # 启动main入口
    main()