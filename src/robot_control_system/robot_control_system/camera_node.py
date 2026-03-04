"""
=======================================================================================
Vision Node - Perception, Memory, and Verification Manager (Stable Cycle Version)
=======================================================================================
Design Philosophy:
    This node serves as the "eyes" and "short-term memory" of the robotic system.
    It processes RGB-D data, calculates global map coordinates via TF2, and matches
    colored objects with their corresponding boxes.
    
    Key Features:
    - [Coordinate Locking]: Once a pair is matched, coordinates are frozen to provide 
      a stable target for Nav2 and the Manipulator.
    - [Disappearance Verification]: Acts as a grasp feedback sensor by monitoring 
      if the targeted object leaves the camera's FOV.
    - [Task Reset]: Listens for a reset signal to clear memory and start a new search cycle.

Data Flow & Output Matrix:
    | Topic Name            | Type        | Purpose
    |-----------------------|-------------|--------------------------------------------
    | /detected_pair        | Bool        | Triggers FSM (True when Object+Box matched)
    | /target_object_pose   | PoseStamped | Stable global map coordinate for the object
    | /target_box_pose      | PoseStamped | Stable global map coordinate for the box
    | /vision_grasp_status  | Bool        | True if locked object is NO LONGER visible
    | /status/reset_vision  | Bool        | Input: Triggers clearing of locked state [NEW]
=======================================================================================
"""

import rclpy
from rclpy.node import Node

# [MODIFIED] Import rclpy.time explicitly
import rclpy.time

from std_msgs.msg import Bool
from geometry_msgs.msg import PoseStamped

# [MODIFIED] Removed custom ObjectTarget msg to simplify system interfaces.
# from my_robot_interfaces.msg import ObjectTarget

import pyrealsense2 as rs
import numpy as np
import cv2
from ultralytics import YOLO
import os
from ament_index_python.packages import get_package_share_directory

# [MODIFIED] Added TF2 libraries for relative (camera) to global (map) transformations
import tf2_ros
import tf2_geometry_msgs


class VisionNode(Node):
    def __init__(self):
        super().__init__("vision_node")

        # ----------------------------
        # Publishers
        # ----------------------------
        # [MODIFIED] Replaced single ObjectTarget publisher with specific task-oriented publishers
        self.pub_pair_found = self.create_publisher(Bool, "/detected_pair", 10)
        self.pub_obj_pose = self.create_publisher(PoseStamped, "/target_object_pose", 10)
        self.pub_box_pose = self.create_publisher(PoseStamped, "/target_box_pose", 10)
        self.pub_grasp_status = self.create_publisher(Bool, "/vision_grasp_status", 10)

        # ----------------------------
        # [MODIFIED] Subscribers
        # ----------------------------
        # Added to listen for the reset signal from the FSM node
        self.sub_reset = self.create_subscription(Bool, "/status/reset_vision", self.reset_vision_callback, 10)

        # ----------------------------
        # [MODIFIED] TF2 Setup
        # ----------------------------
        self.tf_buffer = tf2_ros.Buffer()
        self.tf_listener = tf2_ros.TransformListener(self.tf_buffer, self)

        # ----------------------------
        # [MODIFIED] Memory Dictionary & State
        # Stores map coordinates of detected items to find matching pairs
        # ----------------------------
        self.memory = {
            "red": {"object": None, "box": None},
            "yellow": {"object": None, "box": None},
            "purple": {"object": None, "box": None},
        }
        self.locked_color = ("")  # Once a pair is found, this holds the color to focus on until task completion)
        self.invisible_counter = 0
        self.invisible_threshold = 5  # Number of consecutive frames the object must be invisible to confirm grasp

        # ----------------------------
        # Load YOLO model
        # ----------------------------
        pkg_path = get_package_share_directory("robot_control_system")
        model_path = os.path.join(pkg_path, "best.pt")

        self.get_logger().info(f"Loading model from: {model_path}")
        self.model = YOLO(model_path)
        self.class_names = self.model.names
        self.get_logger().info("YOLO model loaded")

        # ----------------------------
        # RealSense setup
        # ----------------------------
        self.pipeline = rs.pipeline()
        config = rs.config()
        config.enable_stream(rs.stream.color, 640, 480, rs.format.bgr8, 30)
        config.enable_stream(rs.stream.depth, 640, 480, rs.format.z16, 30)
        self.pipeline.start(config)

        self.align = rs.align(rs.stream.color)

        profile = self.pipeline.get_active_profile()
        color_stream = profile.get_stream(rs.stream.color)
        intr = color_stream.as_video_stream_profile().get_intrinsics()

        self.fx = intr.fx
        self.fy = intr.fy
        self.cx = intr.ppx
        self.cy = intr.ppy

        self.get_logger().info("RealSense initialized. Searching for items...")

        # Timer (10 Hz)
        self.timer = self.create_timer(0.1, self.main_loop)

    # =========================================================================
    # [MODIFIED] Helper Functions
    # =========================================================================
    # New Callback for Resetting State
    def reset_vision_callback(self, msg):
        """
        Clears the current locked color and memory slots to allow 
        the robot to search for the next task item.
        """
        if msg.data:
            self.get_logger().info(f"Resetting Vision Node. Clearing lock for: {self.locked_color}")
            # Clear memory for the color that was just completed to prevent stale triggers
            if self.locked_color in self.memory:
                self.memory[self.locked_color]["object"] = None
                self.memory[self.locked_color]["box"] = None
            # Unlock the color to resume searching
            self.locked_color = ""


    def parse_object_info(self, object_name: str):
        """
        Extracts color and item type from the YOLO class name.
        Returns: (color, item_type) or ("unknown", None)
        """
        # Original implementation logic
        if "red" in object_name:
            color = "red"
        elif "yellow" in object_name:
            color = "yellow"
        elif "purple" in object_name:
            color = "purple"
        else:
            color = "unknown"
        item_type = None
        # Detect if the bounding box represents the graspable object or the target box
        if "box" in object_name:
            item_type = "box"
        elif "object" in object_name or "block" in object_name:
            item_type = "object"
        return color, item_type

    def get_camera_pose(self, box, depth_frame):
        """
        Converts YOLO 2D pixel coordinates to 3D PoseStamped in camera_link frame.
        Uses pure camera intrinsics and depth distance.
        """
        # Original implementation logic
        x1, y1, x2, y2 = map(int, box.xyxy[0])
        u = int((x1 + x2) / 2)  # Center pixel U
        v = int((y1 + y2) / 2)  # Center pixel V
        # Retrieve real-world distance from depth map at aligned pixel
        depth = depth_frame.get_distance(u, v)
        if depth <= 0.0:
            return None
        # Standard Pinhole Camera Model: pixel -> camera space
        X = (u - self.cx) / self.fx * depth
        Y = (v - self.cy) / self.fy * depth
        Z = depth

        pose_cam = PoseStamped()
        pose_cam.header.stamp = self.get_clock().now().to_msg()
        pose_cam.header.frame_id = "camera_link"

        # Explicit float casting for ROS2 message compatibility
        pose_cam.pose.position.x = float(X)
        pose_cam.pose.position.y = float(Y)
        pose_cam.pose.position.z = float(Z)
        pose_cam.pose.orientation.w = 1.0  # No rotation assigned in camera frame

        return pose_cam

    # [MODIFIED] Optimization: Passing pre-calculated transform into the loop
    def transform_to_map(self, pose_cam, transform):
        """
        Transforms a camera_link Pose (relative) to a global map Pose (absolute).
        Returns a PoseStamped with 'map' frame header.
        """
        try:
            # Performs matrix multiplication using the pre-fetched transform
            transformed_pose = tf2_geometry_msgs.do_transform_pose(pose_cam.pose, transform)
            # Re-wrap the raw Pose into a PoseStamped for publication
            pose_map = PoseStamped()
            pose_map.header.stamp = self.get_clock().now().to_msg()
            pose_map.header.frame_id = "map"  # Essential for Navigation/Arm nodes
            pose_map.pose = transformed_pose
            return pose_map
        except Exception:
            return None
        

    def check_and_lock_pair(self):
        """
        Checks the Hash Map (Memory) to find if both object and box of the same
        color have been recorded. If so, locks onto that color.
        """
        if self.locked_color == "":
            for c in ["red", "yellow", "purple"]:
                # Check if hash bucket slots are both populated
                if (
                    self.memory[c]["object"] is not None
                    and self.memory[c]["box"] is not None
                ):
                    self.locked_color = c
                    self.get_logger().info(
                        f"Match found for {c.upper()} pair! Locking targets."
                    )
                    break

    def destroy_node(self):
        """
        Critical hardware cleanup. Stops the USB data stream to prevent
        hardware locks on next launch.
        """
        self.get_logger().info("Stopping RealSense pipeline...")
        self.pipeline.stop()
        super().destroy_node()

    # Main Execution Loop (10Hz)
    def main_loop(self):
        # Retrieve and align color/depth frames
        frames = self.pipeline.wait_for_frames()
        frames = self.align.process(frames)

        color_frame = frames.get_color_frame()
        depth_frame = frames.get_depth_frame()

        if not color_frame or not depth_frame:
            return

        # Convert to NumPy for YOLO inference
        color_image = np.asanyarray(color_frame.get_data())
        results = self.model(color_image, conf=0.5, verbose=False)

        # Boolean to track if the specific locked object is visible right now
        is_target_object_in_view = False

        # [MODIFIED] Batch TF Lookup: Get the transform matrix once per frame
        # to save CPU resources when multiple objects are detected.
        try:
            current_transform = self.tf_buffer.lookup_transform(
                "map",
                "camera_link",
                rclpy.time.Time(),  # Latest transform
                rclpy.duration.Duration(seconds=0.05),  # Timeout
            )
        except Exception:
            return  # Skip frame if TF tree is not ready

        # Process detections
        if len(results[0].boxes) > 0:
            for box in results[0].boxes:
                cls_id = int(box.cls[0])
                object_name = self.class_names[cls_id].lower()

                color, item_type = self.parse_object_info(object_name)

                # Filter noise
                if color == "unknown" or not item_type:
                    continue

                # Update visibility flag for the current target task
                if self.locked_color == color and item_type == "object":
                    is_target_object_in_view = True
                # [FIXED] COORDINATE LOCK LOGIC
                # Only update memory if the color isn't locked OR it's not the locked color.
                # This ensures the Nav2 goal remains static and stable during execution.
                if self.locked_color == "" or self.locked_color != color:
                    # Step 1: Get 3D Pose in Camera Frame
                    pose_cam = self.get_camera_pose(box, depth_frame)
                    if not pose_cam:
                        continue
                    # Step 2: Convert to Global Map Frame
                    pose_map = self.transform_to_map(pose_cam, current_transform)
                    if pose_map:
                        # Step 3: Store in Hash Map (Memory)
                        self.memory[color][item_type] = pose_map


        # Evaluate if search phase is complete
        self.check_and_lock_pair()

        # Publish logic
        if self.locked_color != "":
            msg_trigger = Bool()
            msg_trigger.data = True
            self.pub_pair_found.publish(msg_trigger)
            # Ensure data is not None before publishing
            obj_pose = self.memory[self.locked_color]["object"]
            box_pose = self.memory[self.locked_color]["box"]
            if obj_pose and box_pose:
                self.pub_obj_pose.publish(obj_pose)
                self.pub_box_pose.publish(box_pose)
            # [FIXED] Grasp Status Logic
            # Only report success (invisibility) IF we have a locked target.
            if not is_target_object_in_view:
                self.invisible_counter += 1
            else:
                self.invisible_counter = 0 # Reset counter if object is seen again
            msg_verify = Bool()
            # Only publish True if object has been invisible for more than threshold frames
            msg_verify.data = (self.invisible_counter >= self.invisible_threshold)
            self.pub_grasp_status.publish(msg_verify)
        else:
            # [FIXED] If no color locked, grasp status must be False (not success)
            self.invisible_counter = 0  # Reset counter when no target is locked
            msg_verify = Bool()
            msg_verify.data = False
            self.pub_grasp_status.publish(msg_verify)


def main():
    rclpy.init()
    node = VisionNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()
