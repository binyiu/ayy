#!/usr/bin/env python3
import rospy
import json
from std_msgs.msg import String
from single_demo.srv import GetTargetCoord, GetTargetCoordResponse

gps_lat = None 
gps_lon = None

def final_gps_cb(msg):
    """/final_gps 回调：只提取坐标"""
    global gps_lat, gps_lon
    try:
        obj = json.loads(msg.data)
        if "lat" in obj and "lon" in obj:
            gps_lat = float(obj["lat"])
            gps_lon = float(obj["lon"])
        elif "median" in obj:
            gps_lat = float(obj["median"]["lat"])
            gps_lon = float(obj["median"]["lon"])
        else:
           rospy.logwarn("收到消息没有 lat/lon 字段")
           return
        rospy.loginfo_once("已收到最终坐标：%.7f, %.7f", gps_lat, gps_lon)
    except Exception as e:
        rospy.logerr("解析 /final_gps 失败：%s", e)
def handle_get_target(req):
    """服务回调：有坐标就返回，否则返回 success=False"""
    if gps_lat is None or gps_lon is None:
        return GetTargetCoordResponse(lat=0.0, lon=0.0, success=False)
    return GetTargetCoordResponse(lat=gps_lat, lon=gps_lon, success=True)
def server():
    rospy.init_node('target_coord_server')
    rospy.Service('/get_target_coord', GetTargetCoord, handle_get_target)
    rospy.Subscriber("/final_gps", String,final_gps_cb)
    rospy.loginfo("目标坐标服务已启动")
    rospy.spin()


if __name__ == "__main__":
    server()