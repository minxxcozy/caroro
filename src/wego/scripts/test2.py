#!/usr/bin/env python3
# -*- coding: utf-8 -*-

import rospy
from sensor_msgs.msg import LaserScan
import numpy as np
import os

class ObstacleClassify:
    def __init__(self):
        rospy.init_node('obstacle_classify', anonymous=True)
        self.sub = rospy.Subscriber("/scan", LaserScan, self.scan_callback)
        
        self.prev_ranges = None
        self.prev_time = None
        
        self.angle_min = None
        self.angle_increment = None

        # 파라미터
        self.dynamic_threshold = rospy.get_param("~dynamic_threshold", 0.2)  # 거리 변화 임계값 (m)
        self.time_threshold = rospy.get_param("~time_threshold", 0.1)        # 두 스캔 간 최소 시간 간격 (sec)

    def scan_callback(self, msg: LaserScan):
        # 첫 번째 콜백에서는 이전 스캔이 없으므로 저장만 함
        if self.prev_ranges is None:
            self.prev_ranges = np.array(msg.ranges)
            self.prev_time = msg.header.stamp.to_sec()
            self.angle_min = msg.angle_min
            self.angle_increment = msg.angle_increment
            return

        # 시간 체크
        current_time = msg.header.stamp.to_sec()
        dt = current_time - self.prev_time
        if dt < self.time_threshold:
            return  # 너무 짧은 간격이면 무시

        current_ranges = np.array(msg.ranges)
        if len(current_ranges) != len(self.prev_ranges):
            rospy.logwarn("LaserScan 길이 불일치. 건너뜀.")
            self.prev_ranges = current_ranges
            self.prev_time = current_time
            return

        os.system("clear")
        print("동적/정적 장애물 분석 결과:")

        for i in range(len(current_ranges)):
            prev_dist = self.prev_ranges[i]
            curr_dist = current_ranges[i]

            if not np.isfinite(prev_dist) or not np.isfinite(curr_dist):
                continue  

            delta = abs(curr_dist - prev_dist)

            angle = self.angle_min + i * self.angle_increment
            angle_deg = np.rad2deg(angle)



            if -5.0 < angle_deg < 30.0 and 0 < curr_dist < 5:
                if delta > self.dynamic_threshold:
                    print(f"🟥 동적 장애물 | Angle: {angle_deg:.1f}°, Distance: {curr_dist:.3f}, ΔDistance: {delta:.3f} m")
                else:
                    print(f"🟩 정적 장애물 | Angle: {angle_deg:.1f}°, Distance: {curr_dist:.3f}, ΔDistance: {delta:.3f} m")

        # 상태 갱신
        self.prev_ranges = current_ranges
        self.prev_time = current_time

def main():
    try:
        node = ObstacleClassify()
        rospy.spin()
    except rospy.ROSInterruptException:
        pass

if __name__ == "__main__":
    main()
