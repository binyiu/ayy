#!/usr/bin/env python3
import sys
import os
import rospy
import cv2
import torch
import numpy as np
from std_msgs.msg import String  # ROS标准消息


sys.path.insert(0, "/home/lyb/fire_ws/src/fire_infer/yolov5")
from models.common import DetectMultiBackend
from utils.general import non_max_suppression

sys.path.insert(0, "/home/cwkj/fire/src/fire_infer/scripts")
from target_opencv import Test
from number_memory import NumberMemory
from coordinate_transform import CoordinateTransform


class FireDetectorNode:
    def __init__(self):
        rospy.init_node("fire_detection_node", anonymous=True)
        self.model_target_path = rospy.get_param(
            "~model_path", "/home/cwkj/fire/src/fire_infer/models/best_target.pt"
        )
        self.model_digit_path = rospy.get_param(
            "~digit_model_path", "/home/cwkj/fire/src/fire_infer/models/number_identify.pt"
        )
        self.video_path = rospy.get_param(
            "~video_path", "/home/cwkj/fire/src/fire_infer/videos/bazi.mp4"
        )
        self.output_dir = rospy.get_param(
            "~output_dir", "/home/cwkj/fire/src/fire_infer/output"
        )
        os.makedirs(self.output_dir, exist_ok=True)

        #摄像头内参 
        self.fx = rospy.get_param("~fx", 3385.71)
        self.fy = rospy.get_param("~fy", 3385.71)
        self.cx = rospy.get_param("~cx", 2304.0)
        self.cy = rospy.get_param("~cy", 1296.0)

        #数字识别记忆参数 
        self.digit_conf_th = rospy.get_param("~digit_conf_th", 0.6)
        self.memory_min_count = rospy.get_param("~min_count", 5)
        self.cluster_eps = rospy.get_param("~cluster_eps", 5.0)
        self.blank_class_id = rospy.get_param("~blank_class_id", 100)

        rospy.loginfo("Loading YOLOv5 models...")

        # YOLO
        self.model_target = DetectMultiBackend(self.model_target_path, device="cpu")
        self.model_target.eval()

        self.model_digit = DetectMultiBackend(self.model_digit_path, device="cpu")
        self.model_digit.eval()

        self.processor = Test()  # 传统视觉处理器

        self.transformer = CoordinateTransform(self.fx, self.fy, self.cx, self.cy)

        self.number_memory = NumberMemory(
            transformer=self.transformer,
            confidence_threshold=self.digit_conf_th,
            min_count=self.memory_min_count,
            cluster_eps=self.cluster_eps,
            blank_class_id=self.blank_class_id
        )
        self.all_frame_digits = []

        self.digit_pub = rospy.Publisher("detected_digits", String, queue_size=10)

        # 视频处理
        self.cap = cv2.VideoCapture(self.video_path)
        if not self.cap.isOpened():
            rospy.logerr(f"Cannot open video: {self.video_path}")
            raise RuntimeError("Video open failed")

        self.frame_width = int(self.cap.get(cv2.CAP_PROP_FRAME_WIDTH))
        self.frame_height = int(self.cap.get(cv2.CAP_PROP_FRAME_HEIGHT))
        self.fps = self.cap.get(cv2.CAP_PROP_FPS) or 30

        output_path = os.path.join(self.output_dir, "output_video.mp4")
        fourcc = cv2.VideoWriter_fourcc(*'mp4v')
        self.out = cv2.VideoWriter(output_path, fourcc, self.fps, (self.frame_width, self.frame_height))

        rospy.loginfo("FireDetectorNode initialized. Ready to process video.")

        self.process_video()

    def process_video(self):
        frame_idx = 0
        while not rospy.is_shutdown() and self.cap.isOpened():
            ret, frame = self.cap.read()
            if not ret:
                break

            # YOLOv5 第一阶段输入准备
            img_resized = cv2.resize(frame, (640, 640))
            img = cv2.cvtColor(img_resized, cv2.COLOR_BGR2RGB)
            img = img.transpose((2, 0, 1)) / 255.0
            img = torch.from_numpy(img).float().unsqueeze(0)

            with torch.no_grad():
                pred = self.model_target(img)

            detections = non_max_suppression(pred, conf_thres=0.4, iou_thres=0.5)

            for box in detections[0] if detections[0] is not None else []:
                x1, y1, x2, y2, conf, cls = box.tolist()
                scale_x = self.frame_width / 640
                scale_y = self.frame_height / 640
                x1 = int(x1 * scale_x)
                x2 = int(x2 * scale_x)
                y1 = int(y1 * scale_y)
                y2 = int(y2 * scale_y)

                # 绘制目标框
                cv2.rectangle(frame, (x1, y1), (x2, y2), (0, 0, 255), 2)
                cv2.putText(frame, f'Target: {conf:.2f}', (x1, y1 - 10),
                            cv2.FONT_HERSHEY_SIMPLEX, 0.5, (0, 0, 255), 2)

                crop = frame[y1:y2, x1:x2].copy()
                if crop.size == 0:
                    continue

                try:
                    # 传统视觉处理
                    digit_img = self.processor.process(crop)
                    if digit_img is None:
                        continue

                    # YOLO数字识别输入
                    digit_input = cv2.resize(digit_img, (640, 640))
                    digit_input = cv2.cvtColor(digit_input, cv2.COLOR_BGR2RGB)
                    digit_input = digit_input.transpose((2, 0, 1)) / 255.0
                    digit_input = torch.from_numpy(digit_input).float().unsqueeze(0)

                    with torch.no_grad():
                        digit_pred = self.model_digit(digit_input)
                        results_digit = non_max_suppression(
                            digit_pred, conf_thres=self.digit_conf_th, iou_thres=0.5
                        )

                    digit_boxes = []
                    if results_digit[0] is not None and len(results_digit[0]) > 0:
                        for dbox in results_digit[0]:
                            dx1, dy1, dx2, dy2, dconf, dcls = dbox.tolist()
                            x_center = int((dx1 + dx2) / 2)
                            y_center = int((dy1 + dy2) / 2)
                            digit_value = int(dcls)
                            digit_boxes.append((x_center, y_center, digit_value, float(dconf),
                                                (int(dx1), int(dy1), int(dx2), int(dy2))))

                    if digit_boxes:
                        digits_found = []
                        xs_found = []
                        ys_found = []

                        h_crop, w_crop = crop.shape[:2]
                        scale_x_digit = w_crop / 640
                        scale_y_digit = h_crop / 640

                        # 单循环处理所有绘制和坐标
                        for x_c, y_c, digit_value, dconf, (dx1, dy1, dx2, dy2) in digit_boxes:
                            # 在 digit_img 上绘制
                            label = f"{digit_value} ({dconf:.2f})"
                            cv2.rectangle(digit_img, (dx1, dy1), (dx2, dy2), (0, 255, 0), 2)
                            cv2.putText(digit_img, label, (dx1, dy1 - 5),
                                        cv2.FONT_HERSHEY_SIMPLEX, 0.6, (0, 255, 0), 2)

                            # 转换到 frame 坐标
                            cx_frame = int(x_c * scale_x_digit) + x1
                            cy_frame = int(y_c * scale_y_digit) + y1
                            digits_found.append(str(digit_value))
                            xs_found.append(cx_frame)
                            ys_found.append(cy_frame)

                            # 在 frame 上绘制数字
                            cv2.putText(frame, f"{digit_value}", (cx_frame, cy_frame),
                                        cv2.FONT_HERSHEY_SIMPLEX, 0.8, (255, 255, 0), 2)

                        # 按 x 坐标排序并发布 ROS 消息
                        ordered = [dv for _, dv in sorted(zip(xs_found, digits_found), key=lambda t: t[0])]
                        two_digit = "".join(ordered)[:2]
                        avg_x = int(np.mean(xs_found))
                        avg_y = int(np.mean(ys_found))
                        avg_conf = float(np.mean([d[3] for d in digit_boxes]))
                        pub_msg = f"Digits {two_digit} at pixel ({avg_x},{avg_y}) with confidence {avg_conf:.2f}"
                        self.digit_pub.publish(pub_msg)
                        rospy.loginfo(pub_msg)
                        print(pub_msg)

                except Exception as e:
                    rospy.logwarn(f"Digit processing failed: {e}")

            # 写入视频
            self.out.write(frame)
            frame_idx += 1

            # 显示视频
            cv2.imshow("Fire Detection", frame)
            if cv2.waitKey(1) & 0xFF == 27:
                break

        self.cap.release()
        self.out.release()
        cv2.destroyAllWindows()
        rospy.loginfo("Video processing completed.")
    


if __name__ == "__main__":
    try:
        FireDetectorNode()
    except rospy.ROSInterruptException:
        pass
