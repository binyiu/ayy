#!/usr/bin/env python3
import rospy
from geometry_msgs.msg import *
from mavros_msgs.srv import * 
from mavros_msgs.msg import *
from geometry_msgs.msg import PoseStamped
from std_msgs.msg import Header
from sensor_msgs.msg import NavSatFix


# MAVLink 系统状态常量
MAV_STATE_UNINIT = 0
MAV_STATE_BOOT = 1
MAV_STATE_CALIBRATING = 2
MAV_STATE_STANDBY = 3
MAV_STATE_ACTIVE = 4
MAV_STATE_CRITICAL = 5
MAV_STATE_EMERGENCY = 6
MAV_STATE_POWEROFF = 7

# MAVLink 模式标志
MAV_MODE_FLAG_CUSTOM_MODE_ENABLED = 1
MAV_MODE_FLAG_SAFETY_ARMED = 128

class Modes:
    def __init__(self):
        pass

    def arm_drone(self):
        rospy.wait_for_service('/mavros/cmd/arming')
        try:
            arm_service = rospy.ServiceProxy('/mavros/cmd/arming', CommandBool)
            return arm_service(True)
        except rospy.ServiceException as e:
            rospy.logerr(f"解锁失败: {e}")
            return False

    def setmode(self, mode="AUTO.MISSION"):
        rospy.wait_for_service('/mavros/set_mode')
        try:
            mode_service = rospy.ServiceProxy('/mavros/set_mode', SetMode)
            return mode_service(
                base_mode=MAV_MODE_FLAG_CUSTOM_MODE_ENABLED | MAV_MODE_FLAG_SAFETY_ARMED,
                custom_mode=mode
            )
        except rospy.ServiceException as e:
            rospy.logerr(f"模式切换失败: {e}")
            return False 
    
    def push_mission(self, wps):
        rospy.wait_for_service('/mavros/mission/push')
        try:
            wp_push_service = rospy.ServiceProxy('/mavros/mission/push', WaypointPush)
            response = wp_push_service(0, wps)
            rospy.loginfo(f"任务推送结果: {response.success}")
            return response.success
        except rospy.ServiceException as e:
            rospy.logerr(f"任务推送失败: {e}")
            return False

    def pull_mission(self):
        rospy.wait_for_service('/mavros/mission/pull')
        try:
            wp_pull_service = rospy.ServiceProxy('/mavros/mission/pull', WaypointPull)
            response = wp_pull_service()
            rospy.loginfo(f"任务拉区结果：收到{response.wp_received}个航点")
            return response.success
        except rospy.ServiceException as e:
            rospy.logerr(f"任务拉取失败: {e}")
            return False 

    def clear_waypoints(self):
        rospy.wait_for_service('/mavros/mission/clear')
        try:
            clear_service = rospy.ServiceProxy('/mavros/mission/clear', WaypointClear)
            response = clear_service()
            rospy.loginfo("全部航点清空成功")
            return response.success
        except rospy.ServiceException as e:
            rospy.logerr('Service call failed: %s' % e)
            return False
        

class uav:
    def __init__(self):
        # 初始化坐标为 None 或某个默认值
        self.latitude = None
        self.longitude = None
        self.altitude = None
        self.speed = None
        # 创建订阅器 + 指定回调函数
        rospy.Subscriber("/mavros/local_position/pose", PoseStamped, self.local_pose_callback)
        rospy.Subscriber("/mavros/global_position/global", NavSatFix, self.gps_callback)
        rospy.Subscriber("/mavros/global_position/raw/gps_vel",TwistStamped,self.gps_vel_callback)
    def gps_callback(self, msg):
        # 从 NavSatFix 消息中提取经纬高
        self.latitude = msg.latitude
        self.longitude = msg.longitude
    def local_pose_callback(self, msg):
        self.altitude = msg.pose.position.z  # 正确的高度，单位米，相对 home 点  
    def gps_vel_callback(self,msg):
          vx = msg.twist.linear.x
          vy = msg.twist.linear.y
          vz = msg.twist.linear.z
          self.speed = (vx**2 + vy**2 + vz**2) ** 0.5


class StateMonitor:
    def __init__(self):
        self.state = State()
        self.connected = False
        rospy.Subscriber("/mavros/state", State, self._state_cb)

        rospy.loginfo("等待飞控连接...")
        while not rospy.is_shutdown() and not self.connected:
            rospy.loginfo_throttle(1, "等待飞控连接")
            rospy.sleep(0.1)
        rospy.loginfo("飞控已连接")

    def _state_cb(self, msg):
        self.state = msg
        self.connected = msg.connected

class WaypointCreator:
    @staticmethod
    def create_waypoint(frame, command, is_current, autocontinue, 
                        param1, param2, param3, param4, 
                        lat, lon, alt): 
        wp = Waypoint()
        wp.frame = frame
        wp.command = command
        wp.is_current = is_current
        wp.autocontinue = autocontinue
        wp.param1 = param1
        wp.param2 = param2
        wp.param3 = param3
        wp.param4 = param4
        wp.x_lat = lat
        wp.y_long = lon
        wp.z_alt = alt
        return wp
    
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
        self.home_sub = rospy.Subscriber('/mavros/home_position/home', HomePosition, self.home_position_cb)

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

    def home_position_cb(self, msg):
        """Home点位置回调"""
        self.home_pos = msg
        self.home_ready = True
        rospy.logdebug("接收到Home点位置")

    def get_waypoint(self, index=0, timeout=5.0):
        """
        获取指定索引的航点坐标(lat, lon, alt)
        :param index: 0=Home点, 1=第一个任务航点
        :param timeout: 超时时间(秒)   
        :return: (lat, lon, alt) 或 None
        """
        start_time = rospy.Time.now()
        rate = rospy.Rate(10)
        
        # 等待数据就绪
        while not rospy.is_shutdown():
            # 检查超时
            if (rospy.Time.now() - start_time).to_sec() > timeout:
                rospy.logwarn("获取航点超时")
                return None
            
            # 获取Home点(index=0)
            if index == 0:
                if self.home_ready and self.home_pos:
                    return (self.home_pos.geo.latitude, 
                            self.home_pos.geo.longitude, 
                            self.home_pos.geo.altitude)
            
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
        """
        获取完整航点列表(包含Home点)
        :return: [(lat, lon, alt), ...] 或 None
        """
        waypoints = []
        
        # 获取Home点
        home = self.get_waypoint(0, timeout)
        if home is None:
            rospy.logwarn("无法获取Home点")
            return None
        waypoints.append(home)
        
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
