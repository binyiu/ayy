#!/usr/bin/env python3
import rospy
import cv2
import torch
import os
from glob import glob
from target_opencv import Test  # 传统视觉模块

class FireDetectorNode:
    def __init__(self):
        rospy.init_node("fire_detection_node", anonymous=True)

        # 参数设置
        self.model_path = rospy.get_param("~model_path", "/home/cwkj/yolov5/best_target.pt")
        self.input_dir = rospy.get_param("~input_dir", "/home/cwkj/yolov5/data/dataset/train/images")
        self.output_dir = rospy.get_param("~output_dir", "/home/cwkj/yolov5/output_images")
        self.crop_dir = rospy.get_param("~crop_dir", "/home/cwkj/yolov5/output_crops")

        os.makedirs(self.output_dir, exist_ok=True)
        os.makedirs(self.crop_dir, exist_ok=True)

        # 加载模型
        rospy.loginfo("Loading YOLOv5 model...")
        self.model = torch.hub.load('.', 'custom', path=self.model_path, source='local', force_reload=True)
        self.model.to(torch.device('cpu'))
        self.model.eval()

        # 初始化传统识别类
        self.traditional_detector = Test()

        # 获取所有图片路径
        self.image_paths = sorted(glob(os.path.join(self.input_dir, "*.*")))
        self.image_paths = [p for p in self.image_paths if p.lower().endswith(('.jpg', '.png', '.jpeg'))]

        if not self.image_paths:
            rospy.logerr("No images found in input directory.")
            raise RuntimeError("No images to process")

        rospy.loginfo(f"{len(self.image_paths)} images found. Starting detection...")
        self.process_images()

    def process_images(self):
        for img_path in self.image_paths:
            filename = os.path.basename(img_path)
            img = cv2.imread(img_path)
            if img is None:
                rospy.logwarn(f"Failed to read image: {img_path}")
                continue

            img_rgb = cv2.cvtColor(img, cv2.COLOR_BGR2RGB)
            results = self.model(img_rgb)

            for i, box in enumerate(results.xyxy[0]):
                x1, y1, x2, y2, conf, cls = box.tolist()
                if conf > 0.4:
                    x1, y1, x2, y2 = map(int, [x1, y1, x2, y2])
                    target_crop = img[y1:y2, x1:x2]

                    # 保存裁剪图
                    crop_filename = f"{os.path.splitext(filename)[0]}_crop_{i}.jpg"
                    crop_path = os.path.join(self.crop_dir, crop_filename)
                    cv2.imwrite(crop_path, target_crop)

                    # 用传统视觉识别数字和中心点
                    try:
                        number_img, center, target_number, reliability = self.traditional_detector.number_recognition([target_crop, 0])
                    except Exception as e:
                        rospy.logwarn(f"Traditional detection failed for {crop_filename}: {e}")
                        continue

                    rospy.loginfo(f"[{filename}] 数字={target_number}, 中心点={center}, 置信度={reliability:.2f}")

                    # 在原图上标注
                    cv2.rectangle(img, (x1, y1), (x2, y2), (0, 0, 255), 2)
                    cv2.putText(img, f'Num:{target_number}', (x1, y1 - 10),
                                cv2.FONT_HERSHEY_SIMPLEX, 0.5, (255, 0, 0), 2)

            output_path = os.path.join(self.output_dir, filename)
            cv2.imwrite(output_path, img)
            rospy.loginfo(f"Processed and saved: {output_path}")

        rospy.loginfo("Image processing completed.")

if __name__ == "__main__":
    try:
        node = FireDetectorNode()
    except rospy.ROSInterruptException:
        pass
