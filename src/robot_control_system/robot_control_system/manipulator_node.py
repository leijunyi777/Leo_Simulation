import rclpy
from rclpy.node import Node
from my_robot_interfaces.msg import GripperState, CamArmPose
from pymycobot import MyCobot280,PI_PORT,PI_BAUD
import time

arm=MyCobot280(PI_PORT,PI_BAUD)

init_coords = [53.3, -63.6, 418.8, -92.37, -0.35, -90.24]
arm.send_coords(init_coords,20,1)

# print (init_coords)

class ManipulatorControlNode(Node):
    def __init__(self):
        super().__init__('manipulator_control_node')
        self.gripper_state_subscriber = self.create_subscription(
            msg_type=GripperState,
            topic='/arm/grasp_status',
            callback=self.gripper_state_subscriber_callback,
            qos_profile=1)

        self.gripper_pose_subscriber = self.create_subscription(
            msg_type=CamArmPose,
            topic='/arm/grasp_pose',
            callback=self.gripper_pose_subscriber_callback,
            qos_profile=1)
        
        self.gripper_state_subscriber = self.create_subscription(
            msg_type=GripperState,
            topic='/arm/initial_position',
            callback=self.gripper_initial_pose_subscriber_callback,
            qos_profile=1)
        
    def gripper_initial_pose_subscriber_callback(self, msg: GripperState):
        self.get_logger().info(f"Received GripperState: grip={msg.grip}")
        if msg.grip:
            arm.send_coords(init_coords,20,1)

        
    def gripper_state_subscriber_callback(self, msg: GripperState):
        self.get_logger().info(f"Received GripperState: grip={msg.grip}")
        if msg.grip:
            arm.set_gripper_state(0,100)
        if not msg.grip:
            arm.set_gripper_state(1,100)

    def gripper_pose_subscriber_callback(self, msg: CamArmPose):
        self.get_logger().info(
        f"Received GripperPose: x={msg.x}, y={msg.y}, z={msg.z}, "
        # f"roll={msg.roll}, pitch={msg.pitch}, yaw={msg.yaw}"
        )
        x_arm = 0.1736*msg.y+ 0.9848*msg.z + 0.0946
        y_arm = -msg.x + 0.03
        z_arm = 0.9848*msg.y - 0.1736*msg.z - 0.0678

        x_arm = x_arm * 1000
        y_arm = (y_arm * 1000) - 12
        #z_arm = z_arm * 1000
        z_arm = 118.0
        roll = -170.4
        pitch = -4.65
        yaw = -45.22

        #need to hard code the angles
        arm.send_coords([x_arm,y_arm,z_arm,roll,pitch,yaw],20,1)

def main(args=None):
    """
    The main function.
    :param args: Not used directly by the user, but used by ROS2 to configure
    certain aspects of the Node.
    """
    try:
        rclpy.init(args=args)

        manipulator_control_node = ManipulatorControlNode()

        rclpy.spin(manipulator_control_node)
    except KeyboardInterrupt:
        pass
    except Exception as e:
        print(e)


if __name__ == '__main__':
    main()