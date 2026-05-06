import rclpy
from rclpy.node import Node
from std_msgs.msg import Bool
from sensor_msgs.msg import Image
from my_robot_interfaces.msg import ObjectTarget

import numpy as np
import cv2
import message_filters
from ultralytics import YOLO
import os
from ament_index_python.packages import get_package_share_directory

class VisionNodeSim(Node):
    def __init__(self):
        super().__init__('vision_node_sim')

        # Publishers
        self.pub_detected = self.create_publisher(ObjectTarget, '/detected_object', 10)
        self.pub_state = self.create_publisher(Bool, '/detection_state', 10)

        # Load YOLO model
        pkg_path = get_package_share_directory('robot_control_system')
        
        # ----------------------------
        # 使用最新的 model3.pt
        # ----------------------------
        model_path = os.path.join(pkg_path, 'model3.pt')
        self.get_logger().info(f"Loading model from: {model_path}")
        self.model = YOLO(model_path)
        self.class_names = self.model.names
        self.get_logger().info("YOLO model loaded")

        # Camera intrinsics calculated from your Xacro file 
        self.fx = 336.8
        self.fy = 336.8
        self.cx = 320.0
        self.cy = 240.0

        # ROS 2 Subscribers (Syncing Color and Depth)
        # 根据提供的信息修改了对应的话题名称
        self.color_sub = message_filters.Subscriber(self, Image, '/camera/color/image_raw')
        self.depth_sub = message_filters.Subscriber(self, Image, '/camera/depth/image_raw')
        
        self.ts = message_filters.ApproximateTimeSynchronizer(
            [self.color_sub, self.depth_sub], queue_size=10, slop=0.1
        )
        self.ts.registerCallback(self.image_callback)
        self.get_logger().info("Simulation Vision Node Initialized. Waiting for Gazebo images...")

    def image_callback(self, color_msg, depth_msg):
        # ==========================================================
        # 使用纯 Numpy 替代 cv_bridge 进行图像解析
        # ==========================================================
        try:
            # 1. 解析彩色图像
            color_image = np.frombuffer(color_msg.data, dtype=np.uint8).reshape(color_msg.height, color_msg.width, -1)
            
            if color_msg.encoding == 'rgb8':
                color_image = cv2.cvtColor(color_image, cv2.COLOR_RGB2BGR)

            # 2. 解析深度图像
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
            
            # --- 1. 同步真实相机的识别逻辑分类 ---
            cls_id = int(box.cls[0])
            object_name_full = self.class_names[cls_id]
            lowered = object_name_full.lower()

            # Type
            if "box" in lowered:
                name = "box"
            else:
                name = "object"

            # Color
            if "red" in lowered:
                color = "red"
            elif "yellow" in lowered:
                color = "yellow"
            elif "purple" in lowered:
                color = "purple"
            else:
                color = "unknown"
            
            # --- 2. 坐标提取与深度计算 ---
            x1, y1, x2, y2 = map(int, box.xyxy[0])

            # 同步真实相机中心点逻辑
            u = int((x1 + x2) / 2)
            v = int((y1 + y2) / 2)

            if not (0 <= u < depth_image.shape[1] and 0 <= v < depth_image.shape[0]):
                continue

            # ROI 过滤：同步为 radius = 3（7x7 patch）
            patch_radius = 3 
            v_min, v_max = max(0, v - patch_radius), min(depth_image.shape[0], v + patch_radius + 1)
            u_min, u_max = max(0, u - patch_radius), min(depth_image.shape[1], u + patch_radius + 1)
            depth_roi = depth_image[v_min:v_max, u_min:u_max]
            
            valid_mask = (depth_roi > 0) & (~np.isnan(depth_roi)) & (~np.isinf(depth_roi))
            if not np.any(valid_mask):
                continue
            
            depth = float(np.median(depth_roi[valid_mask]))

            # --- 深度过滤校验 ---
            if depth <= 0.0:
                continue

            # 同步实机：全局距离大于0.8丢弃
            if depth > 0.8:
                continue

            # 新增逻辑：当box识别距离大于0.6时为非法数据，丢弃不输出
            if name == "box" and depth > 0.6:
                continue

            # --- 3. 坐标投影与转换 ---
            # 首先计算相机光学坐标系 (Optical Frame): Z向前, X向右, Y向下
            X_cam = (u - self.cx) / self.fx * depth
            Y_cam = (v - self.cy) / self.fy * depth
            Z_cam = depth

            # 同步实机逻辑: 对于 box 调整中心偏移 (13cm)
            if name == "box":
                Z_cam -= 0.13  

            # 坐标转换：从光学坐标系转换至标准ROS机体坐标系 (Base Frame): X向前, Y向左, Z向上
            # 原Z轴(前) -> 现X轴
            # 原X轴(右) -> 现Y轴(左，所以取反)
            # 原Y轴(下) -> 现Z轴(上，所以取反)
            X_ros = Z_cam
            Y_ros = -X_cam
            Z_ros = -Y_cam

            msg = ObjectTarget()
            msg.name = name
            msg.color = color
            msg.x = float(X_ros)
            msg.y = float(Y_ros)
            msg.z = float(Z_ros)
            
            # 循环内直接发布，支持多个物体
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