#!/usr/bin/env python3

import rospy
from math import radians, sin, cos, sqrt, atan2
from single_demo.srv import GetTargetCoord
from uav_mission_utils import uav
import math
from std_msgs.msg import UInt16
from mavros_msgs.msg import OverrideRCIn
from mavros_msgs.msg import MountControl


class UAVDistance:
    def __init__(self):
        self.uav = uav()
        self.target_coord = self.call_service()
        self.mount_pub = rospy.Publisher('/mavros/mount_control/command', MountControl, queue_size=1)


    def call_service(self):
        rospy.wait_for_service('/get_target_coord')
        try:
            get_target = rospy.ServiceProxy('/get_target_coord', GetTargetCoord)
            rospy.loginfo("等待服务响应...")
            resp = get_target()
            if resp.success:
                rospy.loginfo(f"收到目标坐标: lat={resp.lat}, lon={resp.lon}")
                return (resp.lat, resp.lon)
            else:
                rospy.logwarn("目标坐标获取失败")
                return None
        except rospy.ServiceException as e:
            rospy.logerr(f"服务调用失败: {e}")
            return None

    def haversine_distance(self, lat1, lon1, lat2, lon2):
        R = 6371.0
        dlat = radians(lat2 - lat1)
        dlon = radians(lon2 - lon1)
        a = sin(dlat / 2)**2 + cos(radians(lat1)) * cos(radians(lat2)) * sin(dlon / 2)**2
        c = 2 * atan2(sqrt(a), sqrt(1 - a))
        return R * c  # 返回单位：km

    def get_distance_to_target(self):
        #飞机自己的位置
        lat0 = self.uav.latitude
        lon0 = self.uav.longitude
        #返回服务里的坐标
        if self.target_coord is None:
            rospy.logwarn("目标坐标无效")
            return None

        if lat0 is None or lon0 is None:
            rospy.loginfo("尚未收到 UAV 的 GPS 坐标")
            return None
        #取参数，定义目标变量的经纬度为服务里的参数
        target_lat, target_lon = self.target_coord
        distance = self.haversine_distance(lat0, lon0, target_lat, target_lon) * 1000  # 米
        return distance
    
    #判断当前飞机和目标点的距离与想要投掷位置点的距离差
    def compute_release_distance(self,height_m, speed_mps):
         #算在距离目标点多少距离开启舵机
         g = 9.81
         t = math.sqrt(2 * height_m / g)
         x = speed_mps * t
         return  x
    def distance_difference(self):
        speed = self.uav.speed
        high = 30
        #获得当前飞机与目标点的实时距离，套用函数
        current_distance = self.get_distance_to_target()
        #带入实际参数，飞机当前高度，飞行速度，计算理想投放距离
        realdis = self.compute_release_distance(high, speed)
        #返回参数，距离差
        return abs(current_distance - realdis)
    
    
    def release_servo(self):
        rospy.sleep(1)  # 确保 Publisher 建立
        cmd = MountControl()
        cmd.header.stamp = rospy.Time.now()
        cmd.header.frame_id = ""
        cmd.mode = MountControl.MAV_MOUNT_MODE_MAVLINK_TARGETING
        cmd.pitch = 45.0
        cmd.roll = 60.0
        cmd.yaw = 60.0
        cmd.altitude = 0.0
        cmd.latitude = 0.0
        cmd.longitude = 0.0
        self.mount_pub.publish(cmd)

    def drop_mission(self):
        dist_error = self.distance_difference()
        tolerance = 2.0
        rospy.loginfo(f"📏 当前距离误差: {dist_error:.2f} 米（容差范围 ±{tolerance} 米）")

    # 打印调试信息
        speed = self.uav.speed
        current_distance = self.get_distance_to_target()
        ideal_release_distance = self.compute_release_distance(30, speed)

        rospy.loginfo(f"🚁 UAV 当前速度: {speed:.2f} m/s")
        rospy.loginfo(f"🎯 当前 UAV 到目标距离: {current_distance:.2f} 米")
        rospy.loginfo(f"✅ 理想释放距离（根据速度/高度计算）: {ideal_release_distance:.2f} 米")

        if dist_error <= tolerance:
           rospy.loginfo("✅ 进入误差范围，释放舵机")
           self.release_servo()
        else:
           rospy.loginfo("🕒 未进入投放范围，继续等待接近目标...")


