#!/usr/bin/env python3
import rospy
import argparse
from sensor_msgs.msg import NavSatFix
from geometry_msgs.msg import TwistStamped
from std_msgs.msg import Float64
import time

parser = argparse.ArgumentParser()
parser.add_argument("--lat",  type=float, default=30.1234567, help="latitude")
parser.add_argument("--lon",  type=float, default=120.1234567, help="longitude")
parser.add_argument("--alt",  type=float, default=50.0, help="relative altitude (m)")
parser.add_argument("--speed",type=float, default=5.0, help="ground speed (m/s)")
args = parser.parse_args()

def main():
    rospy.init_node('fake_uav_data', anonymous=True)
    gps_pub   = rospy.Publisher("/mavros/global_position/global", NavSatFix, queue_size=1)
    alt_pub   = rospy.Publisher("/mavros/global_position/rel_alt", Float64, queue_size=1)
    vel_pub   = rospy.Publisher("/mavros/global_position/raw/gps_vel", TwistStamped, queue_size=1)

    rate = rospy.Rate(10)
    rospy.loginfo("发布假 UAV 数据 lat=%.7f lon=%.7f alt=%.1f speed=%.1f",
                  args.lat, args.lon, args.alt, args.speed)

    while not rospy.is_shutdown():
        now = rospy.Time.now()

        # 1. GPS
        nav = NavSatFix()
        nav.header.stamp = now
        nav.latitude  = args.lat
        nav.longitude = args.lon
        nav.altitude  = args.alt
        gps_pub.publish(nav)

        # 2. 相对高度
        alt_pub.publish(Float64(data=args.alt))

        # 3. 速度
        vel = TwistStamped()
        vel.header.stamp = now
        vel.twist.linear.x = args.speed
        vel.twist.linear.y = 0
        vel.twist.linear.z = 0
        vel_pub.publish(vel)

        rate.sleep()

if __name__ == '__main__':
    try:
        main()
    except rospy.ROSInterruptException:
        pass