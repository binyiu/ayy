#!/usr/bin/env python3
import rospy
from math import radians, sin, cos, sqrt, atan2
from std_msgs.msg import Float64
from geometry_msgs.msg import *
from mavros_msgs.srv import * 
from mavros_msgs.msg import *
from sensor_msgs.msg import NavSatFix



class uav:
    def __init__(self):
        self.latitude  = None
        self.longitude = None
        self.altitude  = None   # 相对高度
        self.speed     = None

        # 订阅
        rospy.Subscriber("/mavros/global_position/global", NavSatFix, self.gps_callback)
        rospy.Subscriber("/mavros/global_position/rel_alt", Float64, self.rel_alt_callback)
        rospy.Subscriber("/mavros/global_position/raw/gps_vel", TwistStamped, self.gps_vel_callback)

    # ---------- 回调 ----------
    def gps_callback(self, msg):
        self.latitude  = msg.latitude
        self.longitude = msg.longitude

    def rel_alt_callback(self, msg):        # ✅ 新回调
        self.altitude = msg.data            # 相对 home 高度（m）

    def gps_vel_callback(self, msg):
        vx = msg.twist.linear.x
        vy = msg.twist.linear.y
        self.speed = (vx**2 + vy**2)**0.5
    def ready(self):
        """检查 UAV 是否具备完整的状态信息"""
        return (
            self.latitude is not None and
            self.longitude is not None and
            self.altitude is not None and
            self.speed is not None
        )


class WaypointManager:
    def __init__(self):
        # 初始化服务代理和订阅
        self.pull_mission_srv = rospy.ServiceProxy('/mavros/mission/pull', WaypointPull)
        self.home_pos = None
        self.wp_list = []
        self.wp_ready = False
        self.home_ready = False
        
        # 订阅航点和Home点
        self.wp_sub = rospy.Subscriber('/mavros/mission/waypoints', WaypointList, self.waypoint_list_cb)
    def pull_mission(self):
        """拉取最新航点任务"""
        try:
            resp = self.pull_mission_srv(WaypointPullRequest())
            if resp.success:
                rospy.loginfo("成功拉取航点任务，接收%d个航点", resp.wp_received)
            return resp.success
        except rospy.ServiceException as e:
            rospy.logerr("航点拉取服务调用失败: %s", e)
            return False
    def waypoint_list_cb(self, msg):
        """航点列表回调"""
        self.wp_list = msg.waypoints
        self.wp_ready = True
        rospy.logdebug("接收到%d个航点", len(self.wp_list))
    def get_waypoint(self, index=0, timeout=5.0):
        start_time = rospy.Time.now()
        rate = rospy.Rate(10)
        
        # 等待数据就绪
        while not rospy.is_shutdown():
            # 检查超时
            if (rospy.Time.now() - start_time).to_sec() > timeout:
                rospy.logwarn("获取航点超时")
                return None
            
            # 获取任务航点(index>=1)
            elif index > 0:
                if self.wp_ready:
                    if 0 <= index-1 < len(self.wp_list):
                        wp = self.wp_list[index-1]
                        return (wp.x_lat, wp.y_long, wp.z_alt)
                    else:
                        rospy.logwarn("航点索引%d超出范围(共%d个航点)", 
                                     index, len(self.wp_list))
                        return None
            
            rate.sleep()
    def get_all_waypoints(self, timeout=5.0):
        waypoints = []
        # 获取任务航点
        if not self.pull_mission():
            return None
            
        start_time = rospy.Time.now()
        while not rospy.is_shutdown() and not self.wp_ready:
            if (rospy.Time.now() - start_time).to_sec() > timeout:
                rospy.logwarn("获取航点列表超时")
                return None
            rospy.sleep(0.1)
        
        for wp in self.wp_list:
            waypoints.append((wp.x_lat, wp.y_long, wp.z_alt))
            
        return waypoints
    
class UAVDistance:
    @staticmethod
    def haversine_distance(lat1, lon1, lat2, lon2):
        R = 6371.0
        dlat = radians(lat2 - lat1)
        dlon = radians(lon2 - lon1)
        a = sin(dlat / 2)**2 + cos(radians(lat1)) * cos(radians(lat2)) * sin(dlon / 2)**2
        c = 2 * atan2(sqrt(a), sqrt(1 - a))
        return R * c *1000 # 返回单位：m
   
   

