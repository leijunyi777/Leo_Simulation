import rclpy
from rclpy.node import Node
from geometry_msgs.msg import TransformStamped
from tf2_ros import TransformBroadcaster, StaticTransformBroadcaster
import math
import time


class TFSimulatorNode(Node):
    def __init__(self):
        super().__init__('tf_simulator_node')

        # === 1. Broadcasters ===
        # Static Broadcaster: For fixed joints (Sensor mounting points)
        self.static_broadcaster = StaticTransformBroadcaster(self)
        # Dynamic Broadcaster: For moving frames (Map -> Odom -> Base)
        self.dynamic_broadcaster = TransformBroadcaster(self)
        # [MODIFIED] Track start time to simulate robot movement over time
        # self.start_time = time.time()

        # === 2. Publish Static Transforms (Once at startup) ===
        self.publish_static_transforms()

        # === 3. Timer for Dynamic Transforms (10Hz) ===
        self.timer = self.create_timer(0.1, self.publish_dynamic_transforms)

        self.get_logger().info('TF Simulator Started. Publishing complete tree.')

    def get_quaternion_from_euler(self, roll, pitch, yaw):
        """
        Helper to convert Euler angles to Quaternion.
        """
        qx = math.sin(roll/2) * math.cos(pitch/2) * math.cos(yaw/2) - math.cos(roll/2) * math.sin(pitch/2) * math.sin(yaw/2)
        qy = math.cos(roll/2) * math.sin(pitch/2) * math.cos(yaw/2) + math.sin(roll/2) * math.cos(pitch/2) * math.sin(yaw/2)
        qz = math.cos(roll/2) * math.cos(pitch/2) * math.sin(yaw/2) - math.sin(roll/2) * math.sin(pitch/2) * math.cos(yaw/2)
        qw = math.cos(roll/2) * math.cos(pitch/2) * math.cos(yaw/2) + math.sin(roll/2) * math.sin(pitch/2) * math.sin(yaw/2)
        return [qx, qy, qz, qw]

    def create_transform(self, parent, child, x, y, z, roll, pitch, yaw):
        """
        Helper to create a TransformStamped message.
        """
        t = TransformStamped()
        t.header.stamp = self.get_clock().now().to_msg()
        t.header.frame_id = parent
        t.child_frame_id = child
        
        t.transform.translation.x = float(x)
        t.transform.translation.y = float(y)
        t.transform.translation.z = float(z)
        
        q = self.get_quaternion_from_euler(roll, pitch, yaw)
        t.transform.rotation.x = q[0]
        t.transform.rotation.y = q[1]
        t.transform.rotation.z = q[2]
        t.transform.rotation.w = q[3]
        
        return t

    def publish_static_transforms(self):
        """
        Publishes transforms that don't change over time.
        """
        # 1. Base_link -> Camera_link
        # Simulating camera mounted 0.3m forward and 0.5m up
        t_cam = self.create_transform('base_link', 'camera_link', 0.3, 0.0, 0.5, 0.0, 0.0, 0.0)
        
        # 2. Base_link -> Manipulator_base
        # Simulating arm mounted at the center, 0.2m up
        t_arm = self.create_transform('base_link', 'manipulator_base', 0.0, 0.0, 0.2, 0.0, 0.0, 0.0)

        # Send both
        self.static_broadcaster.sendTransform([t_cam, t_arm])
        self.get_logger().info('Static transforms (Camera & Arm) published.')

    def publish_dynamic_transforms(self):
        """
        Publishes transforms that usually update continuously.
        """
        # 1. Map -> Odom
        # Usually static in simple sims, but handled by AMCL/SLAM in real life.
        # We keep it at (0,0,0) for this test.
        t_map_odom = self.create_transform('map', 'odom', 0.0, 0.0, 0.0, 0.0, 0.0, 0.0)

        # 2. Odom -> Base_link
        # Simulating the robot is stationary at (0,0,0). 
        # You can add sine waves to x/y here to simulate movement if needed.
        t_odom_base = self.create_transform('odom', 'base_link', 0.0, 0.0, 0.0, 0.0, 0.0, 0.0)

        # [MODIFIED] 2. Odom -> Base_link (Simulate robot moving forward at 0.1 m/s)
        # This allows you to test if your Vision node properly converts relative coordinates 
        # to global coordinates while the robot is moving during the SEARCH state.
        # elapsed_time = time.time() - self.start_time
        # simulated_x_position = 0.1 * elapsed_time
        # t_odom_base = self.create_transform('odom', 'base_link', simulated_x_position, 0.0, 0.0, 0.0, 0.0, 0.0)

        # Send
        self.dynamic_broadcaster.sendTransform([t_map_odom, t_odom_base])

def main(args=None):
    rclpy.init(args=args)
    node = TFSimulatorNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()