import cv2
import numpy as np


class Test:
    def __init__(self):
        self.lower_white = np.array([0, 0, 200])
        self.upper_white = np.array([179, 60, 255])
        self.lower_red1 = np.array([0, 100, 200])
        self.upper_red1 = np.array([5, 220, 255])
        self.lower_red2 = np.array([160,100, 200])
        self.upper_red2 = np.array([180, 220, 255])

    def extract_color_region(self, img):#红色和白色掩码提取
        hsv = cv2.cvtColor(img, cv2.COLOR_BGR2HSV)
        white_mask = cv2.inRange(hsv, self.lower_white, self.upper_white)
        red_mask1 = cv2.inRange(hsv, self.lower_red1, self.upper_red1)
        red_mask2 = cv2.inRange(hsv, self.lower_red2, self.upper_red2)
        red_mask = cv2.bitwise_or(red_mask1, red_mask2)
        full_mask = cv2.bitwise_or(white_mask, red_mask)
        result = cv2.bitwise_and(img, img, mask=full_mask)
        result = cv2.GaussianBlur(result, (5, 5), 0)

        # 可视化：掩膜和颜色提取结果
        #cv2.imshow("White+Red Mask", full_mask)
        #cv2.imshow("Filtered Color Region", result)

        return result, full_mask

    @staticmethod
    def find_largest_contour(thresh_img):#提取最大轮廓
        contours, _ = cv2.findContours(thresh_img.copy(), cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)

        if contours:
            cnt = max(contours, key=cv2.contourArea)
            area = cv2.contourArea(cnt)
            # print(area)
            # if area > 900 and area < 10000:
            return cnt, contours

        return None, []

    @staticmethod
    def get_min_area_box(cnt):#提取最小外接矩形
        rect = cv2.minAreaRect(cnt)
        box = cv2.boxPoints(rect)
        return np.intp(box), rect

    @staticmethod
    def rotate_to_horizontal(img, rect):
        center, (w, h), angle = rect

        if w < h:
            angle = angle  # 保持不动（通常是竖的）
        else:
            angle = angle + 90  # 横的目标，转正

        # 修正一些极端情况，如 -90 ~ 0，转成正角度
        if angle < -45:
            angle += 90

        rot_mat = cv2.getRotationMatrix2D(center, angle, 1.0)
        img_h, img_w = img.shape[:2]
        cos = np.abs(rot_mat[0, 0])
        sin = np.abs(rot_mat[0, 1])
        new_w = int(img_h * sin + img_w * cos)
        new_h = int(img_h * cos + img_w * sin)

        rot_mat[0, 2] += (new_w / 2) - center[0]
        rot_mat[1, 2] += (new_h / 2) - center[1]

        rotated = cv2.warpAffine(img, rot_mat, (new_w, new_h), flags=cv2.INTER_CUBIC, borderMode=cv2.BORDER_REPLICATE)
        return rotated, rot_mat

    @staticmethod
    def order_points(pts):#为角点排序
        rect = np.zeros((4, 2), dtype="float32")
        s = pts.sum(axis=1)
        diff = np.diff(pts, axis=1)
        rect[0] = pts[np.argmin(s)]
        rect[2] = pts[np.argmax(s)]
        rect[1] = pts[np.argmin(diff)]
        rect[3] = pts[np.argmax(diff)]
        return rect

    @staticmethod
    def transform_points(points, M):
        """仿射变换坐标"""
        ones = np.ones((points.shape[0], 1))
        points_hom = np.hstack([points, ones])  # Nx3
        points_transformed = M @ points_hom.T  # 2xN
        return points_transformed.T  # Nx2

    @staticmethod
    def perspective_transform(image, box, output_size=(100, 100)):#透视变换
        rect = Test.order_points(box)
        dst_pts = np.float32([
            [0, 0],
            [output_size[0] - 1, 0],
            [output_size[0] - 1, output_size[1] - 1],
            [0, output_size[1] - 1]
        ])
        M = cv2.getPerspectiveTransform(rect, dst_pts)
        warped = cv2.warpPerspective(image, M, output_size)
        return warped

    @staticmethod
    def detect_upside_down(img):#判断倒置
        gray = cv2.cvtColor(img, cv2.COLOR_BGR2GRAY)
        _, binary = cv2.threshold(gray, 0, 255, cv2.THRESH_BINARY | cv2.THRESH_OTSU)

        h = binary.shape[0]
        upper_half = binary[:h // 2, :]
        lower_half = binary[h // 2:, :]

        upper_density = np.sum(upper_half == 255)
        lower_density = np.sum(lower_half == 255)

        if upper_density > lower_density:
            return True  # 倒着了
        else:
            return False  # 正着

    def process(self, image):
        # Step 1: 提取颜色区域
        #cv2.imshow("Original", image)
        filtered, _ = self.extract_color_region(image)#红色和白色掩码提取
        gray = cv2.cvtColor(filtered, cv2.COLOR_BGR2GRAY)
        thresh = cv2.threshold(gray, 0, 255, cv2.THRESH_BINARY | cv2.THRESH_OTSU)[1]
        #cv2.imshow("Threshold After Color Filter", thresh)

        # Step 2: 找轮廓 & 最小外接矩形
        cnt, contours = self.find_largest_contour(thresh)
        if cnt is None:
            print("未找到轮廓")
            return None
        box, rect = self.get_min_area_box(cnt)

        image_contour = image.copy()
        cv2.drawContours(image_contour, [box], -1, (0, 255, 0), 2)
        #cv2.imshow("Min Area Rectangle", image_contour)

        # Step 3: 图像旋转，返回rotated和仿射矩阵
        rotated, rot_mat = self.rotate_to_horizontal(image, rect)
        #cv2.imshow("Rotated Image", rotated)

        # Step 4: 计算旋转后ROI（原box点经过rot_mat变换）
        box = np.array(box, dtype=np.float32)
        box_rotated = self.transform_points(box, rot_mat)

        # 计算旋转后ROI的边界框坐标（整数）
        x_min = max(int(np.min(box_rotated[:, 0])), 0)
        x_max = min(int(np.max(box_rotated[:, 0])), rotated.shape[1] - 1)
        y_min = max(int(np.min(box_rotated[:, 1])), 0)
        y_max = min(int(np.max(box_rotated[:, 1])), rotated.shape[0] - 1)

        # 裁剪ROI，减小背景干扰
        roi = rotated[y_min:y_max, x_min:x_max].copy()
        #cv2.imshow("Cropped ROI after Rotation", roi)

        # Step 5: ROI内做颜色掩膜
        hsv_roi = cv2.cvtColor(roi, cv2.COLOR_BGR2HSV)
        white_mask_roi = cv2.inRange(hsv_roi, self.lower_white, self.upper_white)
        red_mask1_roi = cv2.inRange(hsv_roi, self.lower_red1, self.upper_red1)
        red_mask2_roi = cv2.inRange(hsv_roi, self.lower_red2, self.upper_red2)
        red_mask_roi = cv2.bitwise_or(red_mask1_roi, red_mask2_roi)
        mask_roi = cv2.bitwise_or(white_mask_roi, red_mask_roi)
        #cv2.imshow("Mask in ROI", mask_roi)

        # Step 6: ROI掩膜找轮廓
        cnt_roi, _ = self.find_largest_contour(mask_roi)
        if cnt_roi is None:
            print("ROI内未找到轮廓")
            return None

        box_roi, _ = self.get_min_area_box(cnt_roi)

        # box_roi坐标是相对于roi的，需要转换回旋转图像的坐标系用于绘制
        box_roi_global = box_roi + np.array([x_min, y_min])

        rotated_draw = rotated.copy()
        cv2.drawContours(rotated_draw, [np.intp(box_roi_global)], -1, (255, 0, 0), 2)
        #cv2.imshow("Final Rotated Rectangle in ROI", rotated_draw)

        # Step 7: 透视矫正
        final = Test.perspective_transform(rotated, box_roi_global)
        #cv2.imshow("Final Perspective Corrected", final)
        # Step 8: 判断是否倒置
        is_upside_down = Test.detect_upside_down(final)
        if is_upside_down:
            final = cv2.rotate(final, cv2.ROTATE_180)
            print("图像是倒着的，已旋转 180 度")
        else:
            print("图像方向正常")
        #cv2.imshow("Final", final)

        return final


if __name__ == "__main__":
    processor = Test()
    image = cv2.imread("/home/cwkj/yolov5/output_crops/img18_crop_0.jpg")
    result = processor.process(image)

    print("按任意键关闭所有窗口")
    cv2.waitKey(0)
    cv2.destroyAllWindows()