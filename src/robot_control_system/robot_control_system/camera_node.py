import rclpy
from rclpy.node import Node
from std_msgs.msg import Bool
from geometry_msgs.msg import PoseStamped
from sensor_msgs.msg import Image
from my_robot_interfaces.msg import ObjectTarget

import numpy as np
import cv2
import message_filters
from ultralytics import YOLO
import os
from ament_index_python.packages import get_package_share_directory
import math

class VisionNodeSim(Node):
    def __init__(self):
        super().__init__('vision_node_sim')

        # Publishers
        self.pub_detected = self.create_publisher(ObjectTarget, '/detected_object', 10)
        self.pub_state = self.create_publisher(Bool, '/detection_state', 10)

        # Load YOLO model
        pkg_path = get_package_share_directory('robot_control_system')
        model_path = os.path.join(pkg_path, 'best.pt')
        self.get_logger().info(f"Loading model from: {model_path}")
        self.model = YOLO(model_path)
        self.class_names = self.model.names
        self.get_logger().info("YOLO model loaded")

        # 【已移除】self.bridge = CvBridge()

        # Camera intrinsics calculated from your Xacro file 
        self.fx = 336.8
        self.fy = 336.8
        self.cx = 320.0
        self.cy = 240.0

        # ROS 2 Subscribers (Syncing Color and Depth)
        self.color_sub = message_filters.Subscriber(self, Image, '/d435/color/image_raw')
        self.depth_sub = message_filters.Subscriber(self, Image, '/d435/depth/image_raw')
        
        self.ts = message_filters.ApproximateTimeSynchronizer(
            [self.color_sub, self.depth_sub], queue_size=10, slop=0.1
        )
        self.ts.registerCallback(self.image_callback)
        self.get_logger().info("Simulation Vision Node Initialized. Waiting for Gazebo images...")

    def image_callback(self, color_msg, depth_msg):
        # ==========================================================
        # 【核心修改】：使用纯 Numpy 替代 cv_bridge 进行图像解析
        # ==========================================================
        try:
            # 1. 解析彩色图像
            # 从字节流中读取 8 位无符号整数，并重塑为 (高度, 宽度, 通道数) 的矩阵
            color_image = np.frombuffer(color_msg.data, dtype=np.uint8).reshape(color_msg.height, color_msg.width, -1)
            
            # Gazebo 通常输出 rgb8 格式，而 OpenCV/YOLO 默认使用 BGR 格式，所以需要转换一下通道顺序
            if color_msg.encoding == 'rgb8':
                color_image = cv2.cvtColor(color_image, cv2.COLOR_RGB2BGR)

            # 2. 解析深度图像
            # 深度图通常是 32FC1 格式（32位浮点数 单通道），直接读取为 float32 并重塑
            depth_image = np.frombuffer(depth_msg.data, dtype=np.float32).reshape(depth_msg.height, depth_msg.width)

        except Exception as e:
            self.get_logger().error(f"Numpy Image conversion error: {e}")
            return
        # ==========================================================

        # Run YOLO
        results = self.model(color_image, conf=0.5, verbose=False)
        state_msg = Bool()

        if len(results[0].boxes) == 0:
            state_msg.data = False
            self.pub_state.publish(state_msg)
            return

        state_msg.data = True
        self.pub_state.publish(state_msg)

        boxes = results[0].boxes
        confidences = boxes.conf.cpu().numpy()
        sorted_indices = np.argsort(-confidences)
        top_k = min(3, len(sorted_indices))

        for i in range(top_k):
            box = boxes[sorted_indices[i]]
            cls_id = int(box.cls[0])
            object_name_full = self.class_names[cls_id]

            name = "box" if "box" in object_name_full else "object"

            if "red" in object_name_full: color = "red"
            elif "yellow" in object_name_full: color = "yellow"
            elif "purple" in object_name_full: color = "purple"
            else: color = "unknown"

            x1, y1, x2, y2 = map(int, box.xyxy[0])
            u = int((x1 + x2) / 2)
            v = int((y1 + y2) / 2)

            # Get depth in meters (handle potential NaNs from simulation)
            depth = depth_image[v, u]
            
            # ==========================================================
            # "近视眼" 距离过滤
            # 强行丢弃大于 1.5 米的检测结果。
            # 这能完美解决仿真中因为没有纹理导致的远距离 Box/Object 误识别问题。
            # 小车必须开到 1.5 米以内，视觉节点才会真正把坐标发给 FSM 大脑。
            # ==========================================================
            max_valid_distance = 1.5 
            
            if math.isnan(depth) or depth <= 0.0 or depth > max_valid_distance:
                continue

            X = (u - self.cx) / self.fx * depth
            Y = (v - self.cy) / self.fy * depth
            Z = float(depth)

            msg = ObjectTarget()
            msg.name = name
            msg.color = color
            msg.x = float(X)
            msg.y = float(Y)
            msg.z = float(Z)

            self.pub_detected.publish(msg)

def main():
    rclpy.init()
    node = VisionNodeSim()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()