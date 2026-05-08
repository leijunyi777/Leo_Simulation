import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist
from action_msgs.srv import CancelGoal
import math

class SpeedMonitorNode(Node):
    def __init__(self):
        super().__init__('speed_monitor_node')
        
        # 订阅 cmd_vel 话题获取速度
        self.subscription = self.create_subscription(
            Twist,
            'cmd_vel',
            self.listener_callback,
            10
        )
        
        # 创建发布者用于发送停车(0速度)命令
        self.publisher = self.create_publisher(Twist, 'cmd_vel', 10)
        
        # 创建客户端用于取消 Nav2 (NavigateToPose) 的动作目标
        self.cancel_nav_client = self.create_client(
            CancelGoal, 
            '/navigate_to_pose/_action/cancel_goal'
        )
        
        # 设置速度阈值
        self.max_linear_speed = 0.4  # m/s
        self.max_angular_speed = math.radians(60.0)  # 60 degrees/s 转换为弧度
        
        self.get_logger().info("Speed monitor node started. Listening to cmd_vel...")

    def listener_callback(self, msg):
        # 计算合向量线速度 (适配全向轮和差速轮机器人)
        lin_speed = math.hypot(msg.linear.x, msg.linear.y)
        # 获取角速度绝对值
        ang_speed = abs(msg.angular.z)

        # 判断是否超速
        if lin_speed > self.max_linear_speed or ang_speed > self.max_angular_speed:
            # 打印英文警告
            self.get_logger().warn(
                f"Warning: Speed/Angular speed exceeded limit! "
                f"(Linear: {lin_speed:.2f} m/s, Angular: {math.degrees(ang_speed):.2f} deg/s)"
            )
            self.get_logger().warn("Cancelling navigation and stopping...")

            # 1. 立刻向 cmd_vel 发布零速度命令
            stop_msg = Twist() # 默认初始化就是全 0
            self.publisher.publish(stop_msg)

            # 2. 向 Nav2 发送取消导航命令
            self.cancel_nav2()

    def cancel_nav2(self):
        # 检查取消服务是否可用，超时设为0.1秒以免阻塞回调函数
        if not self.cancel_nav_client.wait_for_service(timeout_sec=0.1):
            self.get_logger().warn('Nav2 cancel service not available right now.')
            return

        # 创建 CancelGoal 请求
        # 根据 ROS 2 Action 规范，如果 goal_info 为全 0，默认会取消该 Action 下的**所有**目标
        req = CancelGoal.Request()
        
        # 异步调用服务，防止阻塞
        self.cancel_nav_client.call_async(req)
        self.get_logger().info('Sent cancel request to Nav2 Action Server.')

def main(args=None):
    rclpy.init(args=args)
    node = SpeedMonitorNode()
    
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()