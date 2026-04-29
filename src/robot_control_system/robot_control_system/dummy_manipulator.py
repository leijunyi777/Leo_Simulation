import rclpy
from rclpy.node import Node
# 导入自定义消息类型
from my_robot_interfaces.msg import CamArmPose

class DummyManipulator(Node):
    def __init__(self):
        super().__init__('dummy_manipulator')
        # 订阅话题以满足 FSM 的 count_subscribers > 0 条件
        self.subscription = self.create_subscription(
            CamArmPose,
            '/arm/grasp_pose',
            self.listener_callback,
            10)
        self.get_logger().info('虚拟机械臂节点已启动，正在订阅 /arm/grasp_pose 以辅助 FSM 初始化...')

    def listener_callback(self, msg):
        # 仅打印收到的坐标，不执行动作
        self.get_logger().info(f'收到目标位姿: x={msg.x:.3f}, y={msg.y:.3f}, z={msg.z:.3f}')

def main(args=None):
    rclpy.init(args=args)
    node = DummyManipulator()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()