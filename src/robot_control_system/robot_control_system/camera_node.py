import rclpy
from rclpy.node import Node

from std_msgs.msg import Bool
from my_robot_interfaces.msg import ObjectTarget

import pyrealsense2 as rs
import numpy as np
import cv2
from ultralytics import YOLO
import os
from collections import deque
from ament_index_python.packages import get_package_share_directory


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

        self.get_logger().info("RealSense initialized")

        # ----------------------------
        # Debug flags
        # ----------------------------
        self.debug_view = True

        # ----------------------------
        # Box smoothing / filtering
        # ----------------------------
        self.box_history_2d = deque(maxlen=6)
        self.box_history_3d = deque(maxlen=6)
        self.last_box_center = None
        self.max_box_jump_px = 35

        # ----------------------------
        # Timer (10 Hz)
        # ----------------------------
        self.timer = self.create_timer(0.1, self.main_loop)

    def main_loop(self):
        frames = self.pipeline.wait_for_frames()
        frames = self.align.process(frames)

        color_frame = frames.get_color_frame()
        depth_frame = frames.get_depth_frame()

        if not color_frame or not depth_frame:
            return

        color_image = np.asanyarray(color_frame.get_data())
        debug_img = color_image.copy()

        roi_debug_display = np.zeros((320, 320, 3), dtype=np.uint8)
        mask_debug_display = np.zeros((320, 320, 3), dtype=np.uint8)
        depth_debug_display = np.zeros((320, 320, 3), dtype=np.uint8)

        results = self.model(color_image, conf=0.5, verbose=False)
        state_msg = Bool()

        if len(results[0].boxes) == 0:
            state_msg.data = False
            self.pub_state.publish(state_msg)

            if self.debug_view:
                cv2.imshow("vision_debug", debug_img)
                cv2.imshow("box_roi_debug", roi_debug_display)
                cv2.imshow("box_mask_debug", mask_debug_display)
                cv2.imshow("box_depth_debug", depth_debug_display)
                cv2.waitKey(1)
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

            x1, y1, x2, y2 = map(int, box.xyxy[0])

            h, w = color_image.shape[:2]
            x1 = max(0, min(x1, w - 1))
            x2 = max(0, min(x2, w - 1))
            y1 = max(0, min(y1, h - 1))
            y2 = max(0, min(y2, h - 1))

            if x2 <= x1 or y2 <= y1:
                continue

            # ------------------------------------------
            # OBJECT CASE -> unchanged centroid logic
            # ------------------------------------------
            #if name == "object":
            u = int((x1 + x2) / 2)
            v = int((y1 + y2) / 2)

            depth = self.get_patch_depth(depth_frame, u, v, patch_radius=3)
            if depth <= 0.0:
                continue

            X, Y, Z = self.pixel_to_3d(u, v, depth)

            msg = ObjectTarget()
            msg.name = name
            msg.color = color
            msg.x = float(X)
            msg.y = float(Y)
            msg.z = float(Z)
            self.pub_detected.publish(msg)

            cv2.rectangle(debug_img, (x1, y1), (x2, y2), (0, 255, 0), 2)
            cv2.circle(debug_img, (u, v), 5, (0, 255, 0), -1)
            cv2.putText(
                debug_img,
                f"{color} {name}",
                (x1, max(20, y1 - 10)),
                cv2.FONT_HERSHEY_SIMPLEX,
                0.6,
                (0, 255, 0),
                2
            )

            # ------------------------------------------
            # BOX CASE -> opening mask based center
            # ------------------------------------------
            # else:
            #     success, u_raw, v_raw, depth_raw, roi_vis, mask_vis, depth_vis = self.estimate_box_opening_center(
            #         color_image=color_image,
            #         depth_frame=depth_frame,
            #         x1=x1, y1=y1, x2=x2, y2=y2,
            #         color_name=color
            #     )

            #     roi_debug_display = roi_vis
            #     mask_debug_display = mask_vis
            #     depth_debug_display = depth_vis

            #     if not success:
            #         continue

            #     u_filt, v_filt = self.filter_box_center_2d(u_raw, v_raw)

            #     depth = self.get_patch_depth(depth_frame, u_filt, v_filt, patch_radius=3)
            #     if depth <= 0.0:
            #         depth = self.search_valid_depth_nearby(depth_frame, u_filt, v_filt, max_radius=12)
            #     if depth <= 0.0:
            #         continue

            #     X, Y, Z = self.pixel_to_3d(u_filt, v_filt, depth)
            #     X, Y, Z = self.filter_box_position_3d(X, Y, Z)

            #     msg = ObjectTarget()
            #     msg.name = name
            #     msg.color = color
            #     msg.x = float(X)
            #     msg.y = float(Y)
            #     msg.z = float(Z)
            #     self.pub_detected.publish(msg)

            #     cv2.rectangle(debug_img, (x1, y1), (x2, y2), (255, 0, 0), 2)

            #     # raw point in yellow
            #     cv2.circle(debug_img, (u_raw, v_raw), 5, (0, 255, 255), -1)
            #     cv2.putText(
            #         debug_img,
            #         "raw",
            #         (u_raw + 5, v_raw - 5),
            #         cv2.FONT_HERSHEY_SIMPLEX,
            #         0.45,
            #         (0, 255, 255),
            #         1
            #     )

            #     # filtered point in red
            #     cv2.circle(debug_img, (u_filt, v_filt), 6, (0, 0, 255), -1)
            #     cv2.putText(
            #         debug_img,
            #         f"{color} {name}",
            #         (x1, max(20, y1 - 10)),
            #         cv2.FONT_HERSHEY_SIMPLEX,
            #         0.6,
            #         (255, 0, 0),
            #         2
            #     )

        if self.debug_view:
            cv2.imshow("vision_debug", debug_img)
            cv2.imshow("box_roi_debug", roi_debug_display)
            cv2.imshow("box_mask_debug", mask_debug_display)
            cv2.imshow("box_depth_debug", depth_debug_display)
            cv2.waitKey(1)

    def estimate_box_opening_center(self, color_image, depth_frame, x1, y1, x2, y2, color_name="unknown"):
        roi_color = color_image[y1:y2, x1:x2].copy()

        blank = np.zeros((320, 320, 3), dtype=np.uint8)

        if roi_color.size == 0:
            return False, None, None, None, blank, blank, blank

        roi_h, roi_w = roi_color.shape[:2]

        if roi_h < 20 or roi_w < 20:
            u = int((x1 + x2) / 2)
            v = int((y1 + y2) / 2)
            depth = self.get_patch_depth(depth_frame, u, v, patch_radius=3)
            roi_debug = self.resize_debug_image(roi_color)
            return (depth > 0.0), u, v, depth, roi_debug, blank, blank

        roi_debug = roi_color.copy()

        # -----------------------------------
        # 1. Convert to HSV and LAB
        # -----------------------------------
        hsv = cv2.cvtColor(roi_color, cv2.COLOR_BGR2HSV)
        lab = cv2.cvtColor(roi_color, cv2.COLOR_BGR2LAB)

        h_channel = hsv[:, :, 0]
        s_channel = hsv[:, :, 1]
        v_channel = hsv[:, :, 2]
        a_channel = lab[:, :, 1]

        # -----------------------------------
        # 2. Blur
        # -----------------------------------
        v_blur = cv2.GaussianBlur(v_channel, (7, 7), 0)
        a_blur = cv2.GaussianBlur(a_channel, (7, 7), 0)

        # -----------------------------------
        # 3. Opening mask
        # -----------------------------------
        # Dark opening from V
        _, mask_v = cv2.threshold(v_blur, 0, 255, cv2.THRESH_BINARY_INV + cv2.THRESH_OTSU)

        # Optional red object support
        if color_name == "red":
            red1 = cv2.inRange(hsv, np.array([0, 40, 20]), np.array([15, 255, 255]))
            red2 = cv2.inRange(hsv, np.array([165, 40, 20]), np.array([179, 255, 255]))
            red_mask = cv2.bitwise_or(red1, red2)
            red_mask = cv2.GaussianBlur(red_mask, (5, 5), 0)
            _, red_mask = cv2.threshold(red_mask, 80, 255, cv2.THRESH_BINARY)
        else:
            red_mask = np.zeros_like(mask_v)

        # Red-ish structural channel
        _, mask_a = cv2.threshold(a_blur, 0, 255, cv2.THRESH_BINARY + cv2.THRESH_OTSU)

        # Main opening mask starts from dark interior
        opening_mask = mask_v.copy()

        # Restrict dark region to lie inside object color region if possible
        if np.count_nonzero(red_mask) > 100:
            kernel_big = np.ones((9, 9), np.uint8)
            red_mask_dilated = cv2.dilate(red_mask, kernel_big, iterations=1)
            opening_mask = cv2.bitwise_and(opening_mask, red_mask_dilated)

        # Morphology cleanup
        kernel3 = np.ones((3, 3), np.uint8)
        kernel5 = np.ones((5, 5), np.uint8)
        opening_mask = cv2.morphologyEx(opening_mask, cv2.MORPH_OPEN, kernel3, iterations=1)
        opening_mask = cv2.morphologyEx(opening_mask, cv2.MORPH_CLOSE, kernel5, iterations=2)

        # -----------------------------------
        # 4. Depth ROI only for debug / backup
        # -----------------------------------
        depth_roi = self.extract_depth_roi(depth_frame, x1, y1, x2, y2)
        depth_edges = self.compute_depth_edges(depth_roi)

        # -----------------------------------
        # 5. Find best opening contour from mask
        # -----------------------------------
        contours, _ = cv2.findContours(opening_mask, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)

        best_cnt = None
        best_score = -1e9
        best_rect = None

        roi_area = roi_w * roi_h
        roi_cx = roi_w / 2.0
        roi_cy = roi_h / 2.0
        border_margin = 6

        for cnt in contours:
            area = cv2.contourArea(cnt)
            if area < 120:
                continue

            x, y, w, h = cv2.boundingRect(cnt)

            if w < 12 or h < 12:
                continue

            area_ratio = (w * h) / float(max(1, roi_area))
            if area_ratio < 0.04 or area_ratio > 0.8:
                continue

            aspect_ratio = w / float(max(1, h))
            if not (0.5 <= aspect_ratio <= 1.6):
                continue

            touches_border = (
                x <= border_margin or
                y <= border_margin or
                (x + w) >= (roi_w - border_margin) or
                (y + h) >= (roi_h - border_margin)
            )
            if touches_border:
                continue

            cx = x + w / 2.0
            cy = y + h / 2.0

            dist_to_center = np.sqrt((cx - roi_cx) ** 2 + (cy - roi_cy) ** 2)
            norm_dist = dist_to_center / max(1.0, np.sqrt(roi_cx ** 2 + roi_cy ** 2))

            rectangularity = area / float(max(1, w * h))

            score = (
                1.8 * (1.0 - norm_dist) +
                1.2 * rectangularity +
                0.8 * area_ratio
            )

            if score > best_score:
                best_score = score
                best_cnt = cnt
                best_rect = (x, y, w, h)

        # -----------------------------------
        # 6. Estimate center from contour
        # Priority:
        #   a) 4-corner polygon average
        #   b) minAreaRect center
        #   c) contour moments centroid
        #   d) bbox fallback
        # -----------------------------------
        method_name = "bbox fallback"

        if best_cnt is not None:
            perimeter = cv2.arcLength(best_cnt, True)
            approx = cv2.approxPolyDP(best_cnt, 0.03 * perimeter, True)

            # draw contour
            cv2.drawContours(roi_debug, [best_cnt], -1, (0, 255, 0), 2)

            local_u = None
            local_v = None

            if len(approx) == 4:
                pts = approx.reshape(4, 2)

                # draw corners
                for p in pts:
                    cv2.circle(roi_debug, tuple(p), 5, (255, 0, 255), -1)

                local_u = int(np.mean(pts[:, 0]))
                local_v = int(np.mean(pts[:, 1]))
                method_name = "quad corners"

            else:
                rect = cv2.minAreaRect(best_cnt)
                (cx_rect, cy_rect), (rw, rh), angle = rect

                if rw > 5 and rh > 5:
                    local_u = int(cx_rect)
                    local_v = int(cy_rect)

                    box_pts = cv2.boxPoints(rect)
                    box_pts = np.int32(box_pts)
                    cv2.polylines(roi_debug, [box_pts], True, (255, 255, 0), 2)

                    method_name = "minAreaRect"

            if local_u is None or local_v is None:
                M = cv2.moments(best_cnt)
                if M["m00"] > 1e-6:
                    local_u = int(M["m10"] / M["m00"])
                    local_v = int(M["m01"] / M["m00"])
                    method_name = "moments"

            if local_u is None or local_v is None:
                if best_rect is not None:
                    bx, by, bw, bh = best_rect
                    local_u = bx + bw // 2
                    local_v = by + bh // 2
                    method_name = "rect fallback"
                else:
                    local_u = roi_w // 2
                    local_v = roi_h // 2
                    method_name = "bbox fallback"
        else:
            local_u = roi_w // 2
            local_v = roi_h // 2
            method_name = "bbox fallback"

        # -----------------------------------
        # 7. Convert ROI local -> image coords
        # -----------------------------------
        u = x1 + int(local_u)
        v = y1 + int(local_v)

        H_img, W_img = color_image.shape[:2]
        u = max(0, min(u, W_img - 1))
        v = max(0, min(v, H_img - 1))

        depth = self.get_patch_depth(depth_frame, u, v, patch_radius=3)
        if depth <= 0.0:
            depth = self.search_valid_depth_nearby(depth_frame, u, v, max_radius=12)

        success = depth > 0.0

        # -----------------------------------
        # Debug drawing
        # -----------------------------------
        if best_rect is not None:
            bx, by, bw, bh = best_rect
            cv2.rectangle(roi_debug, (bx, by), (bx + bw, by + bh), (0, 255, 0), 2)

        cv2.circle(roi_debug, (local_u, local_v), 6, (0, 0, 255), -1)
        cv2.putText(
            roi_debug,
            method_name,
            (10, 20),
            cv2.FONT_HERSHEY_SIMPLEX,
            0.5,
            (0, 255, 255),
            1
        )

        # mask debug
        mask_debug = np.zeros((roi_h, roi_w, 3), dtype=np.uint8)
        mask_debug[:, :, 0] = mask_a
        mask_debug[:, :, 1] = red_mask
        mask_debug[:, :, 2] = opening_mask
        cv2.circle(mask_debug, (local_u, local_v), 6, (255, 255, 255), -1)
        cv2.putText(
            mask_debug,
            "B:a-mask G:red-mask R:opening-mask",
            (5, max(18, roi_h - 8)),
            cv2.FONT_HERSHEY_SIMPLEX,
            0.4,
            (255, 255, 255),
            1
        )

        # depth debug
        depth_vis = self.visualize_depth_roi(depth_roi)
        depth_edges_vis = cv2.cvtColor(depth_edges, cv2.COLOR_GRAY2BGR)
        depth_debug = cv2.addWeighted(depth_vis, 0.75, depth_edges_vis, 0.8, 0.0)

        if best_rect is not None:
            bx, by, bw, bh = best_rect
            cv2.rectangle(depth_debug, (bx, by), (bx + bw, by + bh), (0, 255, 0), 2)
        cv2.circle(depth_debug, (local_u, local_v), 6, (0, 0, 255), -1)
        cv2.putText(
            depth_debug,
            method_name,
            (10, 20),
            cv2.FONT_HERSHEY_SIMPLEX,
            0.5,
            (255, 255, 255),
            1
        )

        roi_debug = self.resize_debug_image(roi_debug)
        mask_debug = self.resize_debug_image(mask_debug)
        depth_debug = self.resize_debug_image(depth_debug)

        return success, u, v, depth, roi_debug, mask_debug, depth_debug

    def extract_depth_roi(self, depth_frame, x1, y1, x2, y2):
        roi_h = y2 - y1
        roi_w = x2 - x1
        depth_roi = np.zeros((roi_h, roi_w), dtype=np.float32)

        for yy in range(roi_h):
            for xx in range(roi_w):
                d = depth_frame.get_distance(x1 + xx, y1 + yy)
                depth_roi[yy, xx] = d

        return depth_roi

    def compute_depth_edges(self, depth_roi):
        if depth_roi.size == 0:
            return np.zeros((1, 1), dtype=np.uint8)

        depth_valid = depth_roi.copy()
        positive = depth_valid[depth_valid > 0]

        if positive.size == 0:
            return np.zeros(depth_valid.shape, dtype=np.uint8)

        fill_value = np.median(positive)
        depth_valid[depth_valid <= 0] = fill_value

        depth_blur = cv2.GaussianBlur(depth_valid, (5, 5), 0)

        gx = cv2.Sobel(depth_blur, cv2.CV_32F, 1, 0, ksize=3)
        gy = cv2.Sobel(depth_blur, cv2.CV_32F, 0, 1, ksize=3)
        grad_mag = cv2.magnitude(gx, gy)

        grad_norm = cv2.normalize(grad_mag, None, 0, 255, cv2.NORM_MINMAX).astype(np.uint8)
        _, depth_edges = cv2.threshold(grad_norm, 35, 255, cv2.THRESH_BINARY)

        return depth_edges

    def visualize_depth_roi(self, depth_roi):
        if depth_roi.size == 0:
            return np.zeros((1, 1, 3), dtype=np.uint8)

        depth_vis = depth_roi.copy()
        positive = depth_vis[depth_vis > 0]

        if positive.size == 0:
            return np.zeros((depth_vis.shape[0], depth_vis.shape[1], 3), dtype=np.uint8)

        dmin = np.percentile(positive, 5)
        dmax = np.percentile(positive, 95)

        if dmax <= dmin:
            dmax = dmin + 1e-3

        depth_vis = np.clip(depth_vis, dmin, dmax)
        depth_vis[depth_roi <= 0] = dmax

        depth_norm = ((depth_vis - dmin) / (dmax - dmin) * 255.0).astype(np.uint8)
        depth_color = cv2.applyColorMap(depth_norm, cv2.COLORMAP_JET)
        return depth_color

    def filter_box_center_2d(self, u, v):
        """
        Jump rejection + moving average in pixel space.
        """
        if self.last_box_center is not None:
            prev_u, prev_v = self.last_box_center
            dist = np.hypot(u - prev_u, v - prev_v)
            if dist > self.max_box_jump_px:
                u, v = prev_u, prev_v

        self.last_box_center = (u, v)
        self.box_history_2d.append((u, v))

        u_smooth = int(np.mean([p[0] for p in self.box_history_2d]))
        v_smooth = int(np.mean([p[1] for p in self.box_history_2d]))

        return u_smooth, v_smooth

    def filter_box_position_3d(self, X, Y, Z):
        """
        Moving average in 3D space.
        """
        self.box_history_3d.append((X, Y, Z))
        Xs = float(np.mean([p[0] for p in self.box_history_3d]))
        Ys = float(np.mean([p[1] for p in self.box_history_3d]))
        Zs = float(np.mean([p[2] for p in self.box_history_3d]))
        return Xs, Ys, Zs

    def get_patch_depth(self, depth_frame, u, v, patch_radius=3):
        vals = []
        width = depth_frame.get_width()
        height = depth_frame.get_height()

        for dv in range(-patch_radius, patch_radius + 1):
            for du in range(-patch_radius, patch_radius + 1):
                uu = u + du
                vv = v + dv

                if 0 <= uu < width and 0 <= vv < height:
                    d = depth_frame.get_distance(uu, vv)
                    if d > 0.0:
                        vals.append(d)

        if not vals:
            return 0.0

        return float(np.median(vals))

    def search_valid_depth_nearby(self, depth_frame, u, v, max_radius=12):
        width = depth_frame.get_width()
        height = depth_frame.get_height()

        vals = []
        for r in range(1, max_radius + 1):
            for dv in range(-r, r + 1):
                for du in range(-r, r + 1):
                    if abs(du) != r and abs(dv) != r:
                        continue

                    uu = u + du
                    vv = v + dv

                    if 0 <= uu < width and 0 <= vv < height:
                        d = depth_frame.get_distance(uu, vv)
                        if d > 0.0:
                            vals.append(d)

            if len(vals) >= 5:
                return float(np.median(vals))

        return 0.0

    def pixel_to_3d(self, u, v, depth):
        X = (u - self.cx) / self.fx * depth
        Y = (v - self.cy) / self.fy * depth
        Z = depth
        return X, Y, Z

    def resize_debug_image(self, img, target_size=320):
        if img is None or img.size == 0:
            return np.zeros((target_size, target_size, 3), dtype=np.uint8)

        h, w = img.shape[:2]
        scale = min(target_size / max(1, w), target_size / max(1, h))
        new_w = max(1, int(w * scale))
        new_h = max(1, int(h * scale))

        resized = cv2.resize(img, (new_w, new_h))
        canvas = np.zeros((target_size, target_size, 3), dtype=np.uint8)

        x_off = (target_size - new_w) // 2
        y_off = (target_size - new_h) // 2
        canvas[y_off:y_off + new_h, x_off:x_off + new_w] = resized
        return canvas

    def destroy_node(self):
        self.pipeline.stop()
        cv2.destroyAllWindows()
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