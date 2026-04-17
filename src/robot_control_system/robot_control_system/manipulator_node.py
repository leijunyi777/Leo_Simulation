import rclpy
from rclpy.node import Node
from my_robot_interfaces.msg import GripperState, CamArmPose

class ManipulatorControlNode(Node):
    def __init__(self):
        super().__init__('manipulator_control_node')
        
        # 订阅抓取动作指令
        self.gripper_state_subscriber = self.create_subscription(
            msg_type=GripperState,
            topic='/arm/grasp_status',
            callback=self.gripper_state_subscriber_callback,
            qos_profile=1)

        # 订阅抓取位姿指令
        self.gripper_pose_subscriber = self.create_subscription(
            msg_type=CamArmPose,
            topic='/arm/grasp_pose',
            callback=self.gripper_pose_subscriber_callback,
            qos_profile=1)
        
        # 订阅初始位姿复位指令 (修复了原代码中变量名覆盖的问题)
        self.gripper_initial_pose_subscriber = self.create_subscription(
            msg_type=GripperState,
            topic='/arm/initial_position',
            callback=self.gripper_initial_pose_subscriber_callback,
            qos_profile=1)
            
        self.get_logger().info("Dummy Manipulator Node Started for Simulation. Hardware calls are bypassed.")

    def gripper_initial_pose_subscriber_callback(self, msg: GripperState):
        self.get_logger().info(f"[SIM] Received Initial Pose Cmd: grip={msg.grip}. (Arm moving to init_coords)")
        # 仿真中不执行物理移动
        # if msg.grip: arm.send_coords(init_coords,20,1)

    def gripper_state_subscriber_callback(self, msg: GripperState):
        action = "OPEN" if msg.grip else "CLOSE"
        self.get_logger().info(f"[SIM] Received Gripper Cmd: {action} (grip={msg.grip})")
        # 仿真中不执行物理抓手动作
        # if msg.grip: arm.set_gripper_state(0,100)
        # if not msg.grip: arm.set_gripper_state(1,100)

    def gripper_pose_subscriber_callback(self, msg: CamArmPose):
        self.get_logger().info(
            f"[SIM] Received Grasp Pose from Camera: x={msg.x:.3f}, y={msg.y:.3f}, z={msg.z:.3f}"
        )
        
        # 保留原有的坐标系转换逻辑计算，用于验证数学公式是否正确
        x_arm = 0.1736*msg.y + 0.9848*msg.z + 0.0946
        y_arm = -msg.x + 0.03
        z_arm = 0.9848*msg.y - 0.1736*msg.z - 0.0678

        x_arm = x_arm * 1000
        y_arm = (y_arm * 1000) - 12
        z_arm = 118.0
        roll = -170.4
        pitch = -4.65
        yaw = -45.22

        self.get_logger().info(
            f"[SIM] Calculated IK Target: X={x_arm:.1f}, Y={y_arm:.1f}, Z={z_arm:.1f}. (Arm movement bypassed)"
        )
        # 仿真中不向机械臂发送坐标
        # arm.send_coords([x_arm,y_arm,z_arm,roll,pitch,yaw],20,1)

def main(args=None):
    try:
        rclpy.init(args=args)
        manipulator_control_node = ManipulatorControlNode()
        rclpy.spin(manipulator_control_node)
    except KeyboardInterrupt:
        pass
    except Exception as e:
        print(f"Exception in Manipulator Node: {e}")
    finally:
        if rclpy.ok():
            rclpy.shutdown()

if __name__ == '__main__':
    main()