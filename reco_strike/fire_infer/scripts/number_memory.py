#!/usr/bin/env python3
# -*- coding: utf-8 -*-

import rospy
import numpy as np
from sklearn.cluster import DBSCAN

class NumberMemory:

    def __init__(self, transformer,
                 confidence_threshold=0.6,
                 min_count=5,
                 cluster_eps=5.0,
                 blank_class_id=100):
        self.transformer = transformer
        self.conf_th = float(confidence_threshold)
        self.min_count = int(min_count)
        self.cluster_eps = float(cluster_eps)
        self.blank_id = int(blank_class_id)

        # {number: [ (lat,lon), ... ] }
        self.gps_data = {}
        # {number: count}
        self.number_counts = {}

    @staticmethod
    def _to_local_xy(ref_lat, ref_lon, lat, lon):
        x = (lon - ref_lon) * 111320.0 * np.cos(np.radians(ref_lat))
        y = (lat - ref_lat) * 110540.0
        return x, y

    def _cluster_mean(self, coords):

        if len(coords) == 0:
            return None
        if len(coords) < 2:
            return float(coords[0][0]), float(coords[0][1])

        lat0, lon0 = coords[0]
        xs, ys = [], []
        for lat, lon in coords:
            x, y = self._to_local_xy(lat0, lon0, lat, lon)
            xs.append(x); ys.append(y)
        pts = np.stack([xs, ys], axis=1)

        clustering = DBSCAN(eps=self.cluster_eps, min_samples=2).fit(pts)
        labels = clustering.labels_
        uniq = set(labels)
        if len(uniq) <= 1 or (len(uniq) == 1 and -1 in uniq):
            return float(np.mean(coords[:,0])), float(np.mean(coords[:,1]))

        best_label = max([l for l in uniq if l != -1], key=lambda l: np.sum(labels==l))
        cpts = coords[labels == best_label]
        return float(np.mean(cpts[:,0])), float(np.mean(cpts[:,1]))

    def add_digits_with_point(self, digit_boxes, center_point, target_alt_rel=0.0):
        # 过滤空靶、低置信度
        clean = [(x, y, d, c) for (x, y, d, c) in digit_boxes
                 if d != self.blank_id and d != -1 and c >= self.conf_th]
        if len(clean) == 0:
            rospy.loginfo("空靶或低置信度，忽略。")
            return []
        # 按y排序聚近
        clean.sort(key=lambda t: t[1])  # y升序
        groups = []
        line = []
        last_y = None
        Y_THR = 50  # 同行阈值
        for item in clean:
            if last_y is None or abs(item[1] - last_y) <= Y_THR:
                line.append(item)
            else:
                if len(line) >= 2: groups.append(line)
                line = [item]
            last_y = item[1]
        if len(line) >= 2:
            groups.append(line)

        made_numbers = []
        for g in groups:
            if len(g) < 2:
                continue
            # 取该行置信度最高的两枚
            g_sorted_conf = sorted(g, key=lambda t: t[3], reverse=True)[:2]
            # 按 x 从小到大,十位在左
            g_sorted = sorted(g_sorted_conf, key=lambda t: t[0])
            tens, ones = int(g_sorted[0][2]), int(g_sorted[1][2])
            value = tens * 10 + ones

            gps = self.transformer.pixel_to_gps(center_point[0], center_point[1], target_alt_rel)
            if gps is None:
                continue

            self.number_counts[value] = self.number_counts.get(value, 0) + 1
            self.gps_data.setdefault(value, []).append(gps)
            rospy.loginfo(f"检测到两位数 {value}，累计 {self.number_counts[value]} 次,GPS {gps}")
            made_numbers.append(value)

        return made_numbers

    def get_final_coordinate(self, number):
        if number not in self.gps_data or self.number_counts.get(number, 0) < self.min_count:
            return None
        coords = np.array(self.gps_data[number], dtype=float)
        lat, lon = self._cluster_mean(coords)
        return (lat, lon)

    def pick_median_target(self, fallback_coord=None):
        stable = []

        for num, cnt in self.number_counts.items():
            if cnt >= self.min_count:
                gps = self.get_final_coordinate(num)
                if gps is not None:
                    stable.append((num, gps))

        # 如果不足3个靶子
        if len(stable) < 3:
            if fallback_coord is not None:
                rospy.logwarn("未积累到3个数字,返回备用坐标。")
                return ("fallback", fallback_coord)
            else:
                return (None, None)

        stable.sort(key=lambda t: t[0])
        median = stable[1] 
        return median  # (number, (lat,lon))

        
    def reset(self):
        self.gps_data.clear()
        self.number_counts.clear()
