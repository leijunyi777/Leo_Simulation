import rclpy
from rclpy.node import Node

from std_msgs.msg import Bool
from geometry_msgs.msg import PoseStamped
from my_robot_interfaces.msg import ObjectTarget


import pyrealsense2 as rs
import numpy as np
import cv2
from ultralytics import YOLO
import os
from ament_index_python.packages import get_package_share_directory

##ADD Only then: add TF (camera_link → base_link) 

class VisionNode(Node):
    def __init__(self):
        super().__init__('vision_node')

        # ----------------------------
        # Publishers
        # ----------------------------
        self.pub_detected = self.create_publisher(ObjectTarget, '/detected_object', 10)
        self.pub_state = self.create_publisher(Bool, '/detection_state', 10)


        # ----------------------------
        # Load YOLO model
        # ----------------------------
        pkg_path = get_package_share_directory('robot_control_system')
        model_path = os.path.join(pkg_path, 'best.pt')

        self.get_logger().info(f"Loading model from: {model_path}")
        self.model = YOLO(model_path)
        self.class_names = self.model.names

        # self.model = YOLO("best.pt")
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

        # Camera intrinsics
        profile = self.pipeline.get_active_profile()
        color_stream = profile.get_stream(rs.stream.color)
        intr = color_stream.as_video_stream_profile().get_intrinsics()

        self.fx = intr.fx
        self.fy = intr.fy
        self.cx = intr.ppx
        self.cy = intr.ppy

        self.get_logger().info("RealSense initialized")

        # ----------------------------
        # Timer (10 Hz)
        # ----------------------------
        self.timer = self.create_timer(0.1, self.main_loop)

    def main_loop(self):
        # ----------------------------
        # Get frames
        # ----------------------------
        frames = self.pipeline.wait_for_frames()
        frames = self.align.process(frames)

        color_frame = frames.get_color_frame()
        depth_frame = frames.get_depth_frame()

        if not color_frame or not depth_frame:
            return

        color_image = np.asanyarray(color_frame.get_data())

        # ----------------------------
        # Run YOLO
        # ----------------------------
        results = self.model(color_image, conf=0.5, verbose=False)
        state_msg = Bool()

        if len(results[0].boxes) == 0:
            state_msg.data = False
            self.pub_state.publish(state_msg)
            return

        state_msg.data = True
        self.pub_state.publish(state_msg)



        # ----------------------------
        # Detected Objects List
        # ----------------------------        

        boxes = results[0].boxes

        # Sort by confidence (highest first)
        confidences = boxes.conf.cpu().numpy()
        sorted_indices = np.argsort(-confidences)

        top_k = min(3, len(sorted_indices))


        for i in range(top_k):

            box = boxes[sorted_indices[i]]

            cls_id = int(box.cls[0])
            object_name_full = self.class_names[cls_id]

            # Determine type
            if "box" in object_name_full:
                name = "box"
            else:
                name = "object"

            # Extract color
            if "red" in object_name_full:
                color = "red"
            elif "yellow" in object_name_full:
                color = "yellow"
            elif "purple" in object_name_full:
                color = "purple"
            else:
                color = "unknown"

            x1, y1, x2, y2 = map(int, box.xyxy[0])
            u = int((x1 + x2) / 2)
            v = int((y1 + y2) / 2)

            depth = depth_frame.get_distance(u, v)

            if depth <= 0.0:
                continue

            X = (u - self.cx) / self.fx * depth
            Y = (v - self.cy) / self.fy * depth
            Z = depth

            msg = ObjectTarget()
            msg.name = name
            msg.color = color
            msg.x = float(X)
            msg.y = float(Y)
            msg.z = float(Z)

            self.pub_detected.publish(msg)

     

    def destroy_node(self):
        self.pipeline.stop()
        super().destroy_node()


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


if __name__ == '__main__':
    main()
