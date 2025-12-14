#!/usr/bin/env python3
import rospy
import time
from geometry_msgs.msg import *
from mavros_msgs.srv import *
from mavros_msgs.msg import *
from single_demo.srv import GetTargetCoord
from math import radians, sin, cos, sqrt, atan2, degrees

# 定义系统状态常量 (来自MAVLink)
MAV_STATE_UNINIT = 0
MAV_STATE_BOOT = 1
MAV_STATE_CALIBRATING = 2
MAV_STATE_STANDBY = 3
MAV_STATE_ACTIVE = 4
MAV_STATE_CRITICAL = 5
MAV_STATE_EMERGENCY = 6
MAV_STATE_POWEROFF = 7

# 定义基础模式常量 (来自MAVLink)
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

    def setmode(self, mode="AUTO"):
        rospy.wait_for_service('/mavros/set_mode')
        try:
            mode_service = rospy.ServiceProxy('/mavros/set_mode', SetMode)
            # 使用正确的基础模式标志
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

    def clear_mission(self):
        rospy.wait_for_service('/mavros/mission/clear')
        try:
            clear_client = rospy.ServiceProxy('/mavros/mission/clear', WaypointClear)
            response = clear_client()
            rospy.loginfo("Mission clear: %s")
            return response.success
        except rospy.ServiceException as e:
            rospy.logerr(f"Clear mission failed: {e}")
            return False

    def pull_mission(self):
        rospy.wait_for_service('/mavros/mission/pull')
        try:
            wp_pull_service = rospy.ServiceProxy('/mavros/mission/pull', WaypointPull)
            response = wp_pull_service()
            rospy.loginfo(f"任务拉取结果: 收到{response.wp_received}个航点")
            return response.success
        except rospy.ServiceException as e:
            rospy.logerr(f"任务拉取失败: {e}")
            return False

    @staticmethod
    def destination_point(lat1, lon1, lat2, lon2, d):

        # 地球半径 (米)
        R = 6371000.0

        # 转弧度
        lat1, lon1, lat2, lon2 = map(radians, [lat1, lon1, lat2, lon2])

        # A→B 的总角距离
        dlat = lat2 - lat1
        dlon = lon2 - lon1
        a = sin(dlat / 2) ** 2 + cos(lat1) * cos(lat2) * sin(dlon / 2) ** 2
        total_distance = 2 * atan2(sqrt(a), sqrt(1 - a)) * R  # 单位: 米

        # 比例
        f = d / total_distance

        # 球面插值 (slerp)
        A = sin((1 - f) * total_distance / R) / sin(total_distance / R)
        B = sin(f * total_distance / R) / sin(total_distance / R)

        x = A * cos(lat1) * cos(lon1) + B * cos(lat2) * cos(lon2)
        y = A * cos(lat1) * sin(lon1) + B * cos(lat2) * sin(lon2)
        z = A * sin(lat1) + B * sin(lat2)

        lat3 = atan2(z, sqrt(x * x + y * y))
        lon3 = atan2(y, x)

        return degrees(lat3), degrees(lon3)


class StateMonitor:
    def __init__(self):
        self.state = State()
        self.connected = False

        # 订阅状态话题
        rospy.Subscriber("/mavros/state", State, self._state_cb)

        rospy.loginfo("等待飞控连接...")
        # 等待直到连接成功
        while not rospy.is_shutdown() and not self.connected:
            rospy.loginfo_throttle(1, "等待飞控连接...")
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
        wp.frame = frame  # FRAME_GLOBAL_REL_ALT=3
        wp.command = command  # 命令编号
        wp.is_current = is_current  # 是否为当前航点
        wp.autocontinue = autocontinue
        wp.param1 = param1  # 停留时间(秒)
        wp.param2 = param2  # 接受半径(米)
        wp.param3 = param3  # 通过半径(米)
        wp.param4 = param4  # 偏航角(度)
        wp.x_lat = lat  # 纬度
        wp.y_long = lon  # 经度
        wp.z_alt = alt  # 海拔高度(米)
        return wp


def call_service():
    rospy.wait_for_service('/get_target_coord')
    try:
        get_target = rospy.ServiceProxy('/get_target_coord', GetTargetCoord)
        rospy.loginfo("等待坐标服务响应...")
        resp = get_target()
        rospy.loginfo(f"收到坐标: lat={resp.lat}, lon={resp.lon}, success={resp.success}")
        if resp.success:
            return (resp.lat, resp.lon)
        return None
    except rospy.ServiceException as e:
        rospy.logerr(f"服务调用失败: {e}")
        return None


def start(controller, lat, lon):
    # 初始化状态监视器和控制模块
    state_monitor = StateMonitor()
    lat1, lon1 = controller.destination_point(lat, lon, 30.3218271, 120.3619462, 23)

    # 创建航点任务列表
    # 创建航点任务列表 (来自QGC WPL 110)
    wps = [
        WaypointCreator.create_waypoint(
            frame=3, command=16, is_current=True, autocontinue=True,
            param1=0, param2=0, param3=0, param4=0,
            lat=30.3212540, lon=120.3614770, alt=16.58
        ),
        WaypointCreator.create_waypoint(
            frame=3, command=22, is_current=False, autocontinue=True,
            param1=25, param2=0, param3=0, param4=0,
            lat=30.3218271, lon=120.3619462, alt=15.0
        ),
        WaypointCreator.create_waypoint(
            frame=3, command=16, is_current=False, autocontinue=True,
            param1=0, param2=0, param3=0, param4=0,
            lat=lat1, lon=lon1, alt=25.0
        ),
        WaypointCreator.create_waypoint(
            frame=2, command=183, is_current=False, autocontinue=True,
            param1=5, param2=1900, param3=0, param4=0,
            lat=lat, lon=lon, alt=25.0
        ),
        WaypointCreator.create_waypoint(
            frame=3, command=16, is_current=False, autocontinue=True,
            param1=0, param2=0, param3=0, param4=0,
            lat=30.3232058, lon=120.3624116, alt=25.0
        ),
        WaypointCreator.create_waypoint(
            frame=3, command=16, is_current=False, autocontinue=True,
            param1=0, param2=0, param3=0, param4=0,
            lat=30.3235079, lon=120.3625149, alt=25.0
        ),
        WaypointCreator.create_waypoint(
            frame=3, command=16, is_current=False, autocontinue=True,
            param1=0, param2=0, param3=0, param4=0,
            lat=30.3237904, lon=120.3622198, alt=25.0
        ),
        WaypointCreator.create_waypoint(
            frame=3, command=16, is_current=False, autocontinue=True,
            param1=0, param2=0, param3=0, param4=0,
            lat=30.3238876, lon=120.3618389, alt=25.0
        ),
        WaypointCreator.create_waypoint(
            frame=3, command=16, is_current=False, autocontinue=True,
            param1=0, param2=0, param3=0, param4=0,
            lat=30.3237117, lon=120.3612971, alt=25.0
        ),
        WaypointCreator.create_waypoint(
            frame=3, command=16, is_current=False, autocontinue=True,
            param1=0, param2=0, param3=0, param4=0,
            lat=30.3233412, lon=120.3611845, alt=25.0
        ),
        WaypointCreator.create_waypoint(
            frame=3, command=16, is_current=False, autocontinue=True,
            param1=0, param2=0, param3=0, param4=0,
            lat=30.3224846, lon=120.3613105, alt=20.0
        ),
        WaypointCreator.create_waypoint(
            frame=3, command=21, is_current=False, autocontinue=True,
            param1=0, param2=0, param3=0, param4=0,
            lat=30.3215168, lon=120.3614527, alt=0.0
        )
    ]

    # 推送任务到飞控
    if controller.push_mission(wps):
        rospy.loginfo("航点任务推送成功!")

    else:
        rospy.logerr("航点任务推送失败!")
        return

    # 解锁无人机
    rospy.loginfo("尝试解锁无人机...")
    while not state_monitor.state.armed:
        if controller.arm_drone():
            rospy.loginfo("解锁命令已发送")
        rospy.sleep(1)

    # 等待系统准备就绪
    rospy.loginfo("等待系统准备就绪...")
    while not rospy.is_shutdown():

        if state_monitor.state.armed:
            rospy.loginfo("系统准备就绪，开始执行任务")
            break

        rospy.loginfo(f"等待: 系统状态:{state_monitor.state.system_status} | 解锁:{state_monitor.state.armed}")
        rospy.sleep(1)

    # 切换到AUTO.MISSION模式
    rospy.loginfo("尝试切换到AUTO模式...")
    while not rospy.is_shutdown():
        current_mode = state_monitor.state.mode
        if current_mode == "AUTO":
            rospy.loginfo("已成功进入AUTO模式!")
            break

        if controller.setmode("AUTO"):
            rospy.loginfo("模式切换命令已发送")
        else:
            rospy.logwarn("模式切换失败，重试中...")

        rospy.loginfo(f"当前模式: {current_mode}")
        rospy.sleep(1)

    rospy.loginfo("任务开始执行!")
    rospy.spin()


def main():
    rospy.init_node('auto_mission_node', anonymous=True)
    rospy.loginfo("初始化完成，获取目标坐标...")
    
    coords = call_service()
    if coords is None:          # 节点被中断
        rospy.logwarn("节点被中断，退出任务")
        return

    controller = Modes()
    controller.setmode("LOITER")
    rospy.sleep(1.5)
    rospy.loginfo("清空旧任务...")
    controller.clear_mission()

    rospy.sleep(1)
    rospy.loginfo("开始规划航点并执行任务（APM）")
    start(controller, *coords)
    # coords = call_service()
    # if coords:
    #    rospy.loginfo("开始规划航点并执行任务（APM）")
    #    start(controller,*coords)
    # else:
    #   rospy.logwarn("未获取到目标坐标，终止任务")
    #lat = 30.3226096
    #lon = 120.3622144
    rospy.loginfo("开始规划航点并执行任务（APM）")
    #start(controller, lat, lon)


if __name__ == '__main__':
    try:
        main()
    except rospy.ROSInterruptException:
        pass