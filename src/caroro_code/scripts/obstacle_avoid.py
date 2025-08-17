#!/usr/bin/env python3
# -*- coding: utf-8 -*-

import rospy
from sensor_msgs.msg import LaserScan, CompressedImage, Imu
from std_msgs.msg import Float64, Bool
from cv_bridge import CvBridge
import numpy as np
import cv2
import torch
from ultralytics import YOLO
import math

class SensorFusionNode:
    def __init__(self):
        rospy.init_node("yolo_lidar_fusion_node")

        # LiDAR 변수
        # self.prev_ranges = None
        # self.prev_time = None
        self.angle_min = None
        self.angle_increment = None
        # self.dynamic_threshold = rospy.get_param("~dynamic_threshold", 0.3)
        # self.time_threshold = rospy.get_param("~time_threshold", 0.1)
        self.dynamic_lidar_flags = {}
        self.static_lidar_flags = {}

        # YOLO
        self.device = "cuda" if torch.cuda.is_available() else "cpu"
        self.yolo_model = YOLO("yolov8n.pt")
        
        self.yolo_model.overrides['classes'] = [0, 1]   # 사람(0), 자전거(1) 클래스만 탐지
        self.prev_boxes = []

        # 조향각 변수
        self.steer_angle = 0.0
        self.bridge = CvBridge()
        rospy.Subscriber("/scan", LaserScan, self.lidar_callback)
        rospy.Subscriber("/image_jpeg/compressed", CompressedImage, self.image_callback)
        rospy.Subscriber("/curve_flag", Bool, self.corner_callback)
        
        # 조향값 토픽 반영
        rospy.Subscriber('/imu', Imu, self.imu_callback)
        # rospy.Subscriber('/commands/servo/position', Float64, self.steer_callback)

        # 퍼블리셔
        self.obstacle_run_pub = rospy.Publisher("obstacle_run_cmd", Float64, queue_size=10)

        # 추가
        self.last_avoid_time = rospy.Time(0)  # 회피 시작 시각 기록
        
        self.corner_mode = False   # 바깥 콜백에서 True/False만 토글해줘

        
    def corner_callback(self, msg):
        self.corner_mode = msg.data    
        
    def imu_callback(self, msg):
        q = msg.orientation
        siny_cosp = 2 * (q.w * q.z + q.x * q.y)
        cosy_cosp = 1 - 2 * (q.y * q.y + q.z * q.z)
        yaw_rad = math.atan2(siny_cosp, cosy_cosp)
        yaw_deg = (math.degrees(yaw_rad) + 360) % 360
        self.yaw = yaw_deg
        self.steer_angle = self.yaw   # ★ 추가: rel_deg 계산에 쓰이도록

        # rospy.loginfo_throttle(1.0, f"[IMU] 현재 Yaw: {self.yaw:.2f}°")


    def lidar_callback(self, msg: LaserScan):
        if len(msg.ranges) == 0 : return
        # current_time = msg.header.stamp.to_sec()
        current_ranges = np.array(msg.ranges)
        self.angle_min = msg.angle_min
        self.angle_increment = msg.angle_increment

        # if self.prev_ranges is None:
        #     self.prev_ranges = current_ranges
        #     self.prev_time = current_time
        #     self.angle_min = msg.angle_min
        #     self.angle_increment = msg.angle_increment
        #     return

        # if current_time - self.prev_time < self.time_threshold:
        #     return

        self.dynamic_lidar_flags = {}
        self.static_lidar_flags = {}

        for i in range(len(current_ranges)):
            curr = current_ranges[i]
            # prev = self.prev_ranges[i]

            # if not np.isfinite(curr) or not np.isfinite(prev):
            if not np.isfinite(curr) or curr <= 0.0 :
                continue

            # delta = abs(curr - prev)
            angle = self.angle_min + i * self.angle_increment
            deg = (np.rad2deg(angle) + 360) % 360

            if self.corner_mode:
                rel_deg = ((deg - self.steer_angle + 180.0) % 360.0) - 180.0
            else:
                rel_deg = ((deg + 180.0) % 360.0) - 180.0

            rospy.loginfo_throttle(0.5, f"rel_deg: {rel_deg:.1f}")

            # 동적
            if abs(rel_deg) < 45 and 0 < curr < 4.5 :
            # if abs(rel_deg) < 45 and 0 < curr < 4.5 and delta > self.dynamic_threshold:
                self.dynamic_lidar_flags[i] = True

            # 정적
            if abs(rel_deg) < 5 and 0 < curr < 1.7:
                self.static_lidar_flags[i] = True
                # 루프 밖(콜백 끝나기 전에)
        # self.prev_ranges = current_ranges
        # self.prev_time = current_time



    def image_callback(self, msg: CompressedImage):
        try:
            np_img = np.frombuffer(msg.data, np.uint8)
            img = cv2.imdecode(np_img, cv2.IMREAD_COLOR)
        except Exception as e:
            rospy.logerr("이미지 디코딩 실패: %s", e)
            return
        if img is None:
            rospy.logerr("이미지 디코딩 실패: imdecode returned None")
            return

        results = self.yolo_model.predict(img, imgsz=640, device=self.device, verbose=False)[0]
        curr_boxes = []

        for box in results.boxes:
            # x1, y1, x2, y2 = map(int, box.xyxy[0])
            x1, y1, x2, y2 = map(int, box.xyxy[0].tolist())
            conf = float(box.conf[0])
            cls = int(box.cls[0])
            label = self.yolo_model.model.names[cls]
            curr_boxes.append((x1, y1, x2, y2, label))

            # 박스 표시
            cv2.rectangle(img, (x1, y1), (x2, y2), (0, 255, 0), 2)
            cv2.putText(img, f"{label} {conf:.2f}", (x1, y1 - 10),
                        cv2.FONT_HERSHEY_SIMPLEX, 0.5, (0, 255, 0), 2)

        moving_objects = []
        for cb in curr_boxes:
            for pb in self.prev_boxes:
                dx = abs(cb[0] - pb[0])
                dy = abs(cb[1] - pb[1])
                dw = abs((cb[2] - cb[0]) - (pb[2] - pb[0]))
                dh = abs((cb[3] - cb[1]) - (pb[3] - pb[1]))
                if dx + dy + dw + dh > 30:
                    moving_objects.append(cb)
                    break
        
        # 추가
        if rospy.Time.now() - self.last_avoid_time < rospy.Duration(1.0):
            self.prev_boxes = curr_boxes 
            return
        if len(moving_objects) > 0 and len(self.dynamic_lidar_flags) > 2:
            rospy.loginfo("🟥 동적")
            self.obstacle_run_pub.publish(Float64(1.0)) 
            self.last_avoid_time = rospy.Time.now() # 추가 
        elif len(moving_objects) > 0:
            rospy.loginfo("🟧")
            self.obstacle_run_pub.publish(Float64(1.5))  # YOLO
            self.last_avoid_time = rospy.Time.now() # 추가 
        elif len(self.static_lidar_flags) > 2:
            rospy.loginfo("🟨 정적")
            self.obstacle_run_pub.publish(Float64(2.0))  
            self.last_avoid_time = rospy.Time.now() # 추가 
        else:
            # rospy.loginfo("🟩 주행")
            self.obstacle_run_pub.publish(Float64(3.0)) 

        self.prev_boxes = curr_boxes

        cv2.imshow("YOLO Detection", img)
        cv2.waitKey(1)

def main():
    SensorFusionNode()
    rospy.spin()
    cv2.destroyAllWindows()

if __name__ == "__main__":
    main()