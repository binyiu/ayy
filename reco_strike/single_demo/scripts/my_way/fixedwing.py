#!/usr/bin/env python3
import rospy
from geometry_msgs.msg import *
from mavros_msgs.srv import * 
from mavros_msgs.msg import *
from uav_mission_utils import (MAV_STATE_ACTIVE,StateMonitor,Modes,WaypointCreator,WaypointManager)
from single_demo.srv import (GetTargetCoord)
from uav_distance_utils import UAVDistance



def call_service():
    rospy.wait_for_service('/get_target_coord')
    try:
        get_target = rospy.ServiceProxy('/get_target_coord', GetTargetCoord)
        rospy.loginfo("等待服务响应...")
        resp = get_target()
        rospy.loginfo(f"收到坐标: lat={resp.lat}, lon={resp.lon}, success={resp.success}")
        if resp.success:
            return (resp.lat, resp.lon)
        else:
            return None
    except rospy.ServiceException as e:
        rospy.logerr(f"服务调用失败: {e}")
        return None


def start_mission(lat,lon):
    # 初始化状态监控和飞控模式控制
    state_monitor = StateMonitor()
    controller = Modes()
    
    
    wp_manager = WaypointManager()
    # 获取任意航点
    home = wp_manager.get_waypoint(0)
    # 获取第一个任务航点
    wp1 = wp_manager.get_waypoint(1)
    wp2 = wp_manager.get_waypoint(2)
   
    # 构建航点序列
    wps = [
        # 1. 起飞点 (MAV_CMD_NAV_TAKEOFF=22)
        WaypointCreator.create_waypoint(
            frame=3, command=22, is_current=True, autocontinue=True,
            param1=0, param2=5, param3=0, param4=0,
            lat=home[0], lon=home[1], alt=10  # Home点附近
        ),
        # 2. 航点1 (MAV_CMD_NAV_WAYPOINT=16)
        WaypointCreator.create_waypoint(
            frame=3, command=16, is_current=False, autocontinue=True,
            param1=5, param2=5, param3=0, param4=0,
            lat=wp1[0], lon=wp1[1], alt=10
        ),    
        # 投水点
        WaypointCreator.create_waypoint(
            frame=3, command=16, is_current=False, autocontinue=True,
            param1=0, param2=2, param3=0, param4=0,
            lat=lat, lon=lon, alt=20
        ),
        # 降落开始命令 (MAV_CMD_DO_LAND_START=189)
        WaypointCreator.create_waypoint(
            frame=2, command=189, is_current=False, autocontinue=True,
            param1=0, 
            param2=0, 
            param3=0, 
            param4=0,
            lat=wp2[0], 
            lon=wp2[1], 
            alt=10
        ),
        # ➕ 降落前的 approach 航点，高度略高于地面
        WaypointCreator.create_waypoint(
            frame=3, command=16, is_current=False, autocontinue=True,
            param1=0, 
            param2=0, 
            param3=0, 
            param4=0,
            lat=wp1[0],
            lon=wp1[1], 
            alt=5   # 或者 15，确保高于降落点
        ),

          # 降落点（command=21）
        WaypointCreator.create_waypoint(
           frame=3, command=21, is_current=False, autocontinue=True,
           param1=0, 
           param2=3, 
           param3=0, 
           param4=0,
           lat=home[0], 
           lon=home[1], 
           alt=0.0   
        )
    ]
       
    # 推送航点
    if controller.push_mission(wps):
        rospy.loginfo("航点任务推送成功!")
    else:
        rospy.logerr("航点任务推送失败!")
        return

    # 解锁
    rospy.loginfo("尝试解锁无人机...")
    while not rospy.is_shutdown() and not state_monitor.state.armed:
        if controller.arm_drone():
            rospy.loginfo("解锁命令已发送")
        rospy.sleep(1)

    # 等待系统状态为 ACTIVE
    rospy.loginfo("等待系统准备就绪...")
    while not rospy.is_shutdown():
        if state_monitor.state.armed and state_monitor.state.system_status == MAV_STATE_ACTIVE:
            rospy.loginfo("系统准备就绪，开始执行任务")
            break
        rospy.loginfo(f"等待: 系统状态:{state_monitor.state.system_status} | 解锁:{state_monitor.state.armed}")
        rospy.sleep(1)

    # 切换模式为 AUTO.MISSION
    rospy.loginfo("切换至 AUTO.MISSION 模式...")
    while not rospy.is_shutdown():
        if state_monitor.state.mode == "AUTO.MISSION":
            rospy.loginfo("已进入 AUTO.MISSION 模式")
            break
        if controller.setmode("AUTO.MISSION"):
            rospy.loginfo("模式切换命令已发送")
        else:
            rospy.logwarn("模式切换失败，重试中...")
        rospy.sleep(1)
    
    rospy.loginfo("任务开始执行")
   
    distance_checker = UAVDistance()
    rate = rospy.Rate(2)  # 2Hz 频率检查
    #以防飞机在返航过程中，提前靠近目标点
    rospy.loginfo("开始检测 UAV 是否到达航点1...")
    while not rospy.is_shutdown():
        #获取当前坐标
        clat = distance_checker.uav.latitude
        clon = distance_checker.uav.longitude

        if clat is None or clon is None:
            rospy.loginfo("等待 UAV 坐标中...")
            rate.sleep()
            continue

        # 计算当前 UAV 坐标与 wp1 的距离
        dist_to_wp1 = distance_checker.haversine_distance(clat, clon, wp1[0], wp1[1]) * 1000  # 转为米
        rospy.loginfo(f"当前 UAV 与航点1 的距离: {dist_to_wp1:.2f} 米")

        if dist_to_wp1 <= 50:
          rospy.loginfo("UAV 接近航点1，开始进入投放检测区...")
          rate = rospy.Rate(5)  # 提高频率
          while not rospy.is_shutdown():
             result = distance_checker.drop_mission()
             if result:  # 若已投放
                break
             rate.sleep()
          rospy.loginfo("✅ 已完成投放，跳出检测循环")
          break



if __name__ == '__main__':
    try:
        rospy.init_node('uav_mission_entry_node')
        rospy.loginfo("初始化节点完毕，开始调用坐标服务...")

        coords = call_service()  
        if coords:
            rospy.loginfo("开始规划航点并执行任务")
            start_mission(*coords)
        else:
            rospy.logwarn("未获取到坐标，终止任务")

    except rospy.ROSInterruptException:
        pass