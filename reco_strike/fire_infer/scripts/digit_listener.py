#!/usr/bin/env python3
import rospy
import json
import time
import numpy as np
from std_msgs.msg import String
from coordinate_transform import CoordinateTransform
from uav_gps import uav, WaypointManager, UAVDistance
from number_memory import NumberMemory

THRESHOLD_DIST = 50  # 到达航点阈值（米）
TARGET_WP_INDEX = 10   # 第5号航点

FALLBACK_LAT = 47.4004828
FALLBACK_LON = 8.5517463


# Camera 内参
FX, FY = 3385.71, 3385.71
CX, CY = 2304.0, 1296.0

# 全局容器
raw_records = []
pending_pixels = []
evaluated_records = []
#容器大小
MAX_RAW_RECORDS = 200
MAX_EVALUATED = 500
MAX_PENDING = 200


my_uav = uav()
coord_tf = CoordinateTransform(FX, FY, CX, CY)
coord_tf.uav = my_uav

number_memory = NumberMemory(transformer = coord_tf,
                             confidence_threshold=0.6,
                             min_count=5,
                             cluster_eps = 5.0,
                             blank_class_id = 100
                             )
# 发布者
eval_pub = None
final_pub = None

def parse_message(msg_str):
    if not isinstance(msg_str, str):
        return (None, None, None, None)
    import re
    uv_match = re.search(r"pixel\s*\(\s*(\d+)\s*,\s*(\d+)\s*\)", msg_str)
    if uv_match:
        u = int(uv_match.group(1))
        v = int(uv_match.group(2))
    else:
        return (None, None, None, None)
    # 提取数字和置信度
    digit_match = re.search(r"Digits\s+(\d+)", msg_str)
    conf_match = re.search(r"confidence\s+(\d*\.\d+)", msg_str)
    digits = digit_match.group(1) if digit_match else None
    conf = float(conf_match.group(1)) if conf_match else None
    return digits, u, v, conf

def push_raw_record(raw_msg, digits, u, v, conf, uav_snapshot):
    entry = {"ts": time.time(), "raw_msg": raw_msg, "digits": digits,
             "u": u, "v": v, "conf": conf, "uav": uav_snapshot}
    raw_records.append(entry)
    if len(raw_records) > MAX_RAW_RECORDS:
        raw_records.pop(0)

def push_evaluated_record(digits, conf, u, v, lat, lon, uav_snapshot):
    entry = {"ts": time.time(), "digits": digits, "conf": conf,
             "pixel":{"u":int(u),"v":int(v)}, "lat":float(lat), "lon":float(lon),
             "uav": uav_snapshot}
    evaluated_records.append(entry)
    if len(evaluated_records) > MAX_EVALUATED:
        evaluated_records.pop(0)

def push_pending(digits, conf, u, v, raw_msg, uav_snapshot):
    entry = {"ts": time.time(), "raw_msg": raw_msg, "digits": digits,
             "conf": conf, "u": u, "v": v, "uav_snapshot": uav_snapshot}
    pending_pixels.append(entry)
    if len(pending_pixels) > MAX_PENDING:
        pending_pixels.pop(0)

def publish_eval_result(digits, conf, u, v, lat, lon):
    global eval_pub
    if eval_pub is None:
        return
    payload = {"digits":digits,"conf":conf,"pixel":{"u":int(u),"v":int(v)},
               "lat":float(lat),"lon":float(lon),"ts":time.time()}
    msg = json.dumps(payload)
    eval_pub.publish(msg)
    rospy.loginfo(f"[EvalPub] {msg}")

def try_process_pending():
    remaining = []
    for item in pending_pixels:
        gps = coord_tf.pixel_to_gps(item["u"], item["v"], target_alt_rel=0.0, wait=False)
        if gps:
            lat, lon = gps
            push_evaluated_record(item["digits"], item["conf"], item["u"], item["v"],
                                  lat, lon, item["uav_snapshot"])
            publish_eval_result(item["digits"], item["conf"], item["u"], item["v"], lat, lon)
        else:
            remaining.append(item)
    pending_pixels[:] = remaining

def digit_callback(msg):
    raw = msg.data if isinstance(msg.data, str) else str(msg.data)
    rospy.loginfo(f"[Listener recv raw] {raw}")
    digits, u, v, conf = parse_message(raw)
    if digits is None or u is None or v is None:
        rospy.logwarn(f"[Listener] 无法解析消息: {raw}")
        return
    uav_snap = {"lat":my_uav.latitude, "lon":my_uav.longitude,
                "alt":my_uav.altitude, "speed":my_uav.speed}
    push_raw_record(raw, digits, u, v, conf, uav_snap)
    gps = coord_tf.pixel_to_gps(u, v, target_alt_rel=0.0, wait=False)
    if gps:
        lat, lon = gps
        push_evaluated_record(digits, conf, u, v, lat, lon, uav_snap)
        publish_eval_result(digits, conf, u, v, lat, lon)
        rospy.loginfo(f"[Listener] Pixel ({u},{v}) -> GPS ({lat:.7f},{lon:.7f}), digits={digits}, conf={conf}")
    else:
        push_pending(digits, conf, u, v, raw, uav_snap)
        rospy.loginfo(f"[Listener] UAV 数据未就绪，已加入 pending（长度={len(pending_pixels)}）")

def main():
    global eval_pub, final_pub
    rospy.init_node("digit_wp_fusion_node", anonymous=True)

    wp_manager = WaypointManager()

    rospy.Subscriber("/detected_digits", String, digit_callback)
    eval_pub = rospy.Publisher("/detected_digits_eval", String, queue_size=10)
    final_pub = rospy.Publisher("/final_gps", String, queue_size=10)

    rate = rospy.Rate(20)  # 10Hz

    reached = False
    rospy.loginfo("Digit+Waypoint fusion node started...")

    while not rospy.is_shutdown() and not reached:
        # 处理 pending
        try_process_pending()

        # UAV 数据未就绪
        if not my_uav.ready():
            rate.sleep()
            continue

        # 获取目标航点
        wp = wp_manager.get_waypoint(TARGET_WP_INDEX, timeout=1.0)
        if wp is None:
            rate.sleep()
            continue
        wp_lat, wp_lon, wp_alt = wp

        # 距离判断
        dist = UAVDistance.haversine_distance(my_uav.latitude, my_uav.longitude, wp_lat, wp_lon)
        rospy.loginfo_throttle(5, f"Distance to WP{TARGET_WP_INDEX}: {dist:.2f} m")
        
        if dist <= THRESHOLD_DIST and evaluated_records:
            counts = number_memory.number_counts.copy()
            if not counts:
                for rec in evaluated_records:
                    try:
                        n = int(rec["digits"])
                        counts[n] = counts.get(n, 0) + 1
                    except Exception:
                        continue

            sorted_items = sorted(counts.items(), key=lambda x: x[1], reverse=True)
            top3_nums = [int(x[0]) for x in sorted_items[:3]]

            results = []
            for num in top3_nums:
                coord = number_memory.get_final_coordinate(num)
                if coord is None:
                    coords = np.array(number_memory.gps_data.get(num, []), dtype=float)
                    if coords.size == 0:
                        coords_from_eval = np.array([[rec["lat"], rec["lon"]] for rec in evaluated_records if int(rec["digits"])==num], dtype=float)
                        if coords_from_eval.size == 0:
                            continue
                        lat_mean = float(np.mean(coords_from_eval[:,0]))
                        lon_mean = float(np.mean(coords_from_eval[:,1]))
                        results.append((num, lat_mean, lon_mean))
                    else:
                        lat_mean, lon_mean = number_memory._cluster_mean(coords)
                        results.append((num, float(lat_mean), float(lon_mean)))
                else:
                    lat_final, lon_final = coord
                    results.append((num, float(lat_final), float(lon_final)))

            top3_output = [{"digits": int(n), "lat": float(la), "lon": float(lo)} for (n, la, lo) in results]

            # 不足3个
            if len(results) >= 3:
                results_sorted_by_num = sorted(results, key=lambda t: t[0])
                median_num, median_lat, median_lon = results_sorted_by_num[1]
                msg_payload = {
                    "top3": top3_output,
                    "median": {"digits": int(median_num), "lat": float(median_lat), "lon": float(median_lon)},
                    "ts": time.time()
                }
                msg_str = json.dumps(msg_payload)
                rospy.loginfo(f"[FINAL GPS] {msg_str}")
                final_pub.publish(msg_str)
            else:
                rospy.logwarn("不足3个有效数字,使用独立备用坐标")
                fallback_msg = json.dumps({"digits":"fallback","lat":FALLBACK_LAT,"lon":FALLBACK_LON,"ts":time.time()})
                final_pub.publish(fallback_msg)
                rospy.loginfo(f"[FINAL GPS] {fallback_msg}")

            reached = True

        rate.sleep()

       
     

        

if __name__ == "__main__":
    try:
        main()
    except rospy.ROSInterruptException:
        pass
