import math
import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist

# 更新为目标消息的实际包路径
from my_robot_interfaces.msg import ObjectTarget 

class TargetFollowerNode(Node):
    def __init__(self):
        super().__init__('target_follower_node')
        
        # --- 1. 通信接口 ---
        self.sub_detected = self.create_subscription(
            ObjectTarget, 
            '/detected_object', 
            self.detected_callback, 
            10
        )
        self.pub_cmd = self.create_publisher(Twist, '/cmd_vel', 10)
        
        # --- 2. PD 控制器参数与状态 ---
        self.kp_linear = 0.5
        self.kd_linear = 0.1
        self.kp_angular = 0.5
        self.kd_angular = 0.1
        
        # 状态变量重命名以匹配新的坐标系逻辑
        self.prev_error_forward = 0.0  # 原来的 prev_error_z
        self.prev_error_lateral = 0.0  # 原来的 prev_error_x
        self.prev_time = self.get_clock().now()
        
        # --- 3. 超时安全机制 (Watchdog) ---
        self.last_msg_time = self.get_clock().now()
        self.timeout_sec = 0.5
        
        # 创建一个 10Hz (0.1秒) 的定时器，循环检查是否超时
        self.watchdog_timer = self.create_timer(0.1, self.watchdog_callback)

    def calculate_pd_command(self, msg: ObjectTarget) -> Twist:
        """
        核心控制函数：输入目标消息，输出 Twist 速度指令
        """
        cmd = Twist()
        current_time = self.get_clock().now()
        
        # 计算距离上一次控制的时间差 dt (秒)
        dt = (current_time - self.prev_time).nanoseconds / 1e9
        if dt <= 0:
            dt = 0.001  # 防止除以零

        # 1. 计算误差 (新坐标系：X向前，Y向左)
        # 期望距离前方 0.325m (X轴)，期望横向居中 0m (Y轴)
        error_forward = msg.x - 0.175
        error_lateral = msg.y - 0.0

        # 2. 判断停止条件 (位置误差 < 0.01m)
        distance_error = math.hypot(error_forward, error_lateral)
        if distance_error < 0.01:
            self.get_logger().info('位置误差小于 0.01m，到达目标，停止运动。', throttle_duration_sec=1.0)
            self.prev_time = current_time
            return cmd  # 返回全 0 指令

        # 3. PD 计算
        # -- 前后速度 (X轴误差控制 linear.x) --
        deriv_forward = (error_forward - self.prev_error_forward) / dt
        linear_output = (self.kp_linear * error_forward) + (self.kd_linear * deriv_forward)

        # -- 旋转速度 (Y轴误差控制 angular.z) --
        deriv_lateral = (error_lateral - self.prev_error_lateral) / dt
        angular_output = (self.kp_angular * error_lateral) + (self.kd_angular * deriv_lateral)

        # 4. 限幅与方向映射
        # X轴正向代表前方，直接输出即可
        cmd.linear.x = max(-0.1, min(0.1, linear_output))
        
        # Y轴正向代表左方，ROS标准中 angular.z 正向也是左转(逆时针)，因此不需要再加负号反转了
        cmd.angular.z = max(-0.1, min(0.1, angular_output))

        # 5. 更新状态
        self.prev_error_forward = error_forward
        self.prev_error_lateral = error_lateral
        self.prev_time = current_time

        return cmd

    def detected_callback(self, msg: ObjectTarget):
        """
        收到目标消息的回调函数 (期望频率 10Hz)
        """
        self.last_msg_time = self.get_clock().now()
        
        cmd_vel_msg = self.calculate_pd_command(msg)
        self.pub_cmd.publish(cmd_vel_msg)

    def watchdog_callback(self):
        """
        定时器回调：监控是否丢失目标
        """
        current_time = self.get_clock().now()
        time_since_last_msg = (current_time - self.last_msg_time).nanoseconds / 1e9
        
        if time_since_last_msg > self.timeout_sec:
            stop_cmd = Twist()
            self.pub_cmd.publish(stop_cmd)
            
            self.get_logger().warn('超过 0.5s 未收到目标更新，发送 0 速度急停！', throttle_duration_sec=1.0)
            
            self.prev_error_forward = 0.0
            self.prev_error_lateral = 0.0
            self.prev_time = current_time

def main(args=None):
    rclpy.init(args=args)
    node = TargetFollowerNode()
    
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()