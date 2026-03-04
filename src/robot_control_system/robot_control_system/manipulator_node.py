"""
=======================================================================================
Manipulator Control Node - Action Execution & Vision Verification
=======================================================================================
Design Philosophy:
    This node executes the physical "Pick" and "Place" sequences. Since the hardware
    gripper lacks tactile sensors, it utilizes "Disappearance Verification" from the
    Vision Node to confirm a successful grasp.

Interface Contract:
    - Subscribes to /status/arm_active: Triggers Pick (if empty) or Place (if holding).
    - Subscribes to /vision_grasp_status: Confirms success if object is no longer seen.
    - Subscribes to /target_object_pose & /target_box_pose: Dynamic task targets.
    - Publishes /manipulator_feedback: Notifies FSM when the sequence is complete.
=======================================================================================
"""

import rclpy
from rclpy.node import Node
from std_msgs.msg import Bool
from geometry_msgs.msg import PoseStamped
from tf2_ros import Buffer, TransformListener
import tf2_geometry_msgs
import time

# State Constants
STATE_IDLE = 0
STATE_GRASPING = 1
STATE_DROPPING = 2


class ManipulatorControlNode(Node):
    def __init__(self):
        super().__init__("manipulator_controller")

        # === 1. Internal State ===
        self.current_state = STATE_IDLE
        self.grasp_mark = False
        self.last_grasp_success = False
        # [MODIFIED] Added non-blocking state machine variables
        self.action_sub_state = 0
        self.action_start_time = 0.0
        # [OPTIONAL] Initialize myCobot (Adjust port for Raspberry Pi if needed)
        # from pymycobot.mycobot import MyCobot
        # self.mc = MyCobot('/dev/ttyAMA0', 1000000)
        # Memory for targets (Synced with Vision/Nav nodes)
        self.object_pose = None
        self.box_pose = None
        # Fixed Poses (Joint space or Task space presets)
        self.pose_initial = {"x": 0.2, "y": 0.0, "z": 0.4, "qw": 1.0}
        self.pose_ready = {"x": 0.2, "y": 0.0, "z": 0.2, "qw": 1.0}

        # === 2. ROS Communications ===
        # Publishers
        self.pub_feedback = self.create_publisher(Bool, "/manipulator_feedback", 10)
        # Subscribers
        self.sub_active = self.create_subscription(
            Bool, "/status/arm_active", self.active_callback, 10
        )
        # [MODIFIED] Using vision status to confirm grasp success
        self.sub_grasp_verify = self.create_subscription(
            Bool, "/vision_grasp_status", self.grasp_verify_callback, 10
        )
        # [MODIFIED] Subscribing to direct pose topics for coordinate-aware picking
        self.sub_obj_pose = self.create_subscription(
            PoseStamped, "/target_object_pose", self.obj_pose_cb, 10
        )
        self.sub_box_pose = self.create_subscription(
            PoseStamped, "/target_box_pose", self.box_pose_cb, 10
        )
        # [MODIFIED] Cleanup memory on reset signal
        self.sub_reset = self.create_subscription(
            Bool, "/status/reset_vision", self.reset_callback, 10
        )
        # TF Support
        self.tf_buffer = Buffer()
        self.tf_listener = TransformListener(self.tf_buffer, self)
        self.get_logger().info("Manipulator Node with Vision-Verification Ready.")
        self.timer = self.create_timer(0.1, self.main_loop)

    # === Callbacks ===
    def active_callback(self, msg):
        if msg.data and self.current_state == STATE_IDLE:
            if not self.grasp_mark:
                self.get_logger().info("[FSM Command] Triggering GRASP sequence.")
                self.current_state = STATE_GRASPING
            else:
                self.get_logger().info("[FSM Command] Triggering DROP sequence.")
                self.current_state = STATE_DROPPING

    def grasp_verify_callback(self, msg):
        self.last_grasp_success = msg.data  # True if object disappeared

    def obj_pose_cb(self, msg):
        self.object_pose = msg

    def box_pose_cb(self, msg):
        self.box_pose = msg

    def reset_callback(self, msg):
        """[MODIFIED] Clear local memory when mission cycle completes."""
        if msg.data:
            self.object_pose = None
            self.box_pose = None
            self.grasp_mark = False

    # === Hardware Abstraction (Non-Blocking) ===
    def move_arm(self, target_pose):
        self.get_logger().info("Sending coordinate command to myCobot...")
        # [MODIFIED] Removed time.sleep()! Serial commands must return instantly.
        # Example for myCobot 280:
        # self.mc.send_coords([target_pose.x, target_pose.y, target_pose.z, rx, ry, rz], 50, 1)

    def control_gripper(self, open_cmd: bool):
        action = "OPENING" if open_cmd else "CLOSING"
        self.get_logger().info(f"Gripper: {action}")
        # [MODIFIED] Removed time.sleep()!
        # Example for myCobot 280 adaptive gripper:
        # if open_cmd:
        #     self.mc.set_gripper_state(0, 50)
        # else:
        #     self.mc.set_gripper_state(1, 50)

    # === Transformation Logic ===
    def get_pose_in_arm_frame(self, target_pose_stamped):
        """Transforms map-frame pose to manipulator_base frame."""
        try:
            # Look up transform from map to arm base
            transform = self.tf_buffer.lookup_transform(
                "manipulator_base",
                target_pose_stamped.header.frame_id,
                rclpy.time.Time(),
            )

            transformed_pose = tf2_geometry_msgs.do_transform_pose(
                target_pose_stamped.pose, transform
            )
            return transformed_pose
        except Exception as e:
            self.get_logger().error(f"TF Error: {e}")
            return None

    # === Main Loop & Sequences ===
    def main_loop(self):
        if self.current_state == STATE_GRASPING:
            self.execute_grasp()
        elif self.current_state == STATE_DROPPING:
            self.execute_drop()

    def execute_grasp(self):
        if self.object_pose is None:
            if self.action_sub_state == 0:
                self.get_logger().warn("Waiting for Object coordinates...")
            return

        # Get current time in seconds
        now = self.get_clock().now().nanoseconds / 1e9

        # --- Sub-state 0: Initialize and move to target ---
        if self.action_sub_state == 0:
            self.get_logger().info("--- Executing Pick ---")
            arm_target = self.get_pose_in_arm_frame(self.object_pose)

            if arm_target:
                self.control_gripper(True)  # Send open command (instant return)
                self.move_arm(arm_target)  # Send move command (instant return)
                self.action_start_time = now
                self.action_sub_state = 1  # Switch to wait state
            else:
                self.get_logger().error("TF Transform Failed. Aborting Grasp.")
                self.current_state = STATE_IDLE

        # --- Sub-state 1: Wait for arm movement (1.5s), then close gripper ---
        elif self.action_sub_state == 1:
            if now - self.action_start_time >= 1.5:
                self.control_gripper(False)  # Send close command
                self.action_start_time = now
                self.action_sub_state = 2

        # --- Sub-state 2: Wait for gripper to close (1.0s), then lift up ---
        elif self.action_sub_state == 2:
            if now - self.action_start_time >= 1.0:
                self.move_arm(self.pose_ready)  # Send lift command
                self.action_start_time = now
                self.action_sub_state = 3

        # --- Sub-state 3: Wait for lift (1.5s), then verify via Vision ---
        elif self.action_sub_state == 3:
            if now - self.action_start_time >= 1.5:
                # Verification phase: check the latest status from Vision Node
                if self.last_grasp_success:
                    self.get_logger().info("Vision confirmed: Object secured!")
                    self.grasp_mark = True

                    # Notify FSM
                    fb_msg = Bool()
                    fb_msg.data = True
                    self.pub_feedback.publish(fb_msg)
                else:
                    self.get_logger().error(
                        "Grasp Failed: Object still visible on floor!"
                    )

                # Reset sub-state for the next cycle
                self.action_sub_state = 0
                self.current_state = STATE_IDLE

    def execute_drop(self):
        if self.box_pose is None:
            self.get_logger().warn("Waiting for Box coordinates...")
            return

        self.get_logger().info("--- Executing Place ---")
        # 1. Transform coordinates
        arm_target = self.get_pose_in_arm_frame(self.box_pose)

        if arm_target:
            # 2. Place Motion
            self.move_arm(arm_target)
            self.control_gripper(True)  # Open to drop

            # 3. Return to safe pose
            self.move_arm(self.pose_initial)
            self.grasp_mark = False

            # 4. Notify FSM
            fb_msg = Bool()
            fb_msg.data = True
            self.pub_feedback.publish(fb_msg)

            self.current_state = STATE_IDLE


def main(args=None):
    rclpy.init(args=args)
    node = ManipulatorControlNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()
