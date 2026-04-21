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
            
            # --- 1. 获取基础属性 ---
            cls_id = int(box.cls[0])
            object_name_full = self.class_names[cls_id]
            
            # 提取颜色
            color = "unknown"
            for c in ["red", "yellow", "purple"]:
                if c in object_name_full:
                    color = c
                    break
            
            # --- 2. 采样与深度计算 ---
            x1, y1, x2, y2 = map(int, box.xyxy[0])
            pixel_width = x2 - x1
            pixel_height = y2 - y1

            u = int((x1 + x2) / 2)
            v = int(y2 - pixel_height * 0.2)

            if not (0 <= u < depth_image.shape[1] and 0 <= v < depth_image.shape[0]):
                continue

            # ROI 中位数滤波 (5x5 区域)
            roi = 2 
            v_min, v_max = max(0, v - roi), min(depth_image.shape[0], v + roi)
            u_min, u_max = max(0, u - roi), min(depth_image.shape[1], u + roi)
            depth_roi = depth_image[v_min:v_max, u_min:u_max]
            
            valid_mask = (depth_roi > 0) & (~np.isnan(depth_roi)) & (~np.isinf(depth_roi))
            if not np.any(valid_mask):
                continue
            
            depth_surface = float(np.median(depth_roi[valid_mask]))

            # --- [新增] 距离过滤范围优化 ---
            # 0.2m以内是深度相机盲区，数据不可信；2.5m以上太远不处理
            if depth_surface < 0.2 or depth_surface > 2.5:
                continue

            # --- 3. 物理分类与偏移补偿 (增强鲁棒性) ---
            # 通过内参将像素尺寸还原为物理尺寸 (米)
            real_width = (pixel_width / self.fx) * depth_surface
            real_height = (pixel_height / self.fy) * depth_surface
            
            # [策略升级] 双判据校验：Box 不仅宽，而且高度也应该显著大于 Object
            # 只有宽度 > 0.12m 且 高度 > 0.10m 时，才判定为 Box
            if real_width > 0.12 and real_height > 0.10:
                name = "box"
                depth_offset = 0.105 # 21cm 盒子的半径
            else:
                name = "object"
                depth_offset = 0.025 # 5cm 物块的半径

            # --- 4. 坐标投影与发布 ---
            # 计算 3D 坐标 (相对于相机 d435_link)
            X = (u - self.cx) / self.fx * depth_surface
            Y = (v - self.cy) / self.fy * depth_surface
            Z = depth_surface + depth_offset

            msg = ObjectTarget()
            msg.name = name
            msg.color = color
            msg.x, msg.y, msg.z = float(X), float(Y), float(Z)
            
            # 发布识别结果
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