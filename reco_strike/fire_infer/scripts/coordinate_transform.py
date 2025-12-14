#!/usr/bin/env python3
# -*- coding: utf-8 -*-

import math
import rospy
from geometry_msgs.msg import PoseStamped
from uav_gps import uav

EARTH_R = 6378137.0  # WGS84

class CoordinateTransform:

    def __init__(self, fx, fy, cx, cy):
        # 相机内参
        self.fx = float(fx)
        self.fy = float(fy)
        self.cx = float(cx)
        self.cy = float(cy)
        # UAV 状态单例
        self.uav = uav()      
        self.current_yaw = 0.0

        rospy.Subscriber("/mavros/local_position/pose", PoseStamped, self._pose_cb)
   

    def _pose_cb(self, msg: PoseStamped):
        q = msg.pose.orientation
        siny_cosp = 2.0 * (q.w * q.z + q.x * q.y)
        cosy_cosp = 1.0 - 2.0 * (q.y * q.y + q.z * q.z)
        self.current_yaw = math.atan2(siny_cosp, cosy_cosp)

    def ready(self):
        return (self.uav.latitude is not None and
                self.uav.longitude is not None and
                self.uav.altitude is not None)

    def pixel_to_gps(self, u, v, target_alt_rel=0.0,wait=True, timeout=None):
        """
        u, v: 像素坐标（全图坐标）
        target_alt_rel: 目标相对高度
        return: (lat, lon)
        """
        first_warn = True
        while not self.ready():
           if not wait:
               if first_warn:
                  #rospy.logwarn("等待位姿/高度/GPS数据...")
                  return None
           #rospy.logwarn_throttle(2.0, "等待位姿/高度/GPS数据...")
           rospy.sleep(0.1)
          

        x = (float(u) - self.cx) / self.fx
        y = (float(v) - self.cy) / self.fy

        dz = self.uav.altitude - float(target_alt_rel)  # 相机到目标的相对高度差
        dx_cam = x * dz
        dy_cam = y * dz

        # yaw 旋转
        cyaw = math.cos(self.current_yaw)
        syaw = math.sin(self.current_yaw)
        Xr = dx_cam * cyaw - dy_cam * syaw   
        Yr = dx_cam * syaw + dy_cam * cyaw 

        dLat = (Yr / EARTH_R) * (180.0 / math.pi)
        dLon = (Xr / (EARTH_R * math.cos(math.radians(self.uav.latitude)))) * (180.0 / math.pi)
        return (self.uav.latitude + dLat, self.uav.longitude + dLon)

if __name__ == "__main__":
    rospy.init_node("coordinate_transform")
    fx, fy = 3385.71, 3385.71
    cx, cy = 2304.0, 1296.0

    coord_tf = CoordinateTransform(fx, fy, cx, cy)
    u, v = 798, 69
    gps = coord_tf.pixel_to_gps(u, v, target_alt_rel=0.0)
    print(f"Pixel=({u},{v})  -> GPS={gps}")