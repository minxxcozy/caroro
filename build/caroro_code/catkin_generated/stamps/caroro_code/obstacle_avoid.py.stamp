#!/usr/bin/env python3
# -*- coding: utf-8 -*-

import rospy
from sensor_msgs.msg import LaserScan, CompressedImage, Imu
from cv_bridge import CvBridge
import numpy as np
import cv2
import torch
from ultralytics import YOLO
import math  # [IMU]

class SensorFusionNode:
    def __init__(self):
        rospy.init_node("yolo_lidar_fusion_node")

        # LiDAR 변수
        self.prev_ranges = None
        self.prev_time = None
        self.angle_min = None
        self.angle_increment = None
        self.dynamic_threshold = rospy.get_param("~dynamic_threshold", 0.3)
        self.time_threshold = rospy.get_param("~time_threshold", 0.1)
        self.dynamic_lidar_flags = {}
        self.static_lidar_flags = {}

        # IMU 변수 
        self.yaw = 0.0  # 현재 차량의 heading

        # YOLO
        self.device = "cuda" if torch.cuda.is_available() else "cpu"
        self.yolo_model = YOLO("yolov8n.pt")
        self.prev_boxes = []

        # 기타
        self.bridge = CvBridge()
        rospy.Subscriber("/scan", LaserScan, self.lidar_callback)
        rospy.Subscriber("/image_jpeg/compressed", CompressedImage, self.image_callback)
        
        # IMU 토픽 이름이 다를 경우 수정
        rospy.Subscriber("/imu", Imu, self.imu_callback)  

    # imu 콜백 함수
    def imu_callback(self, msg):
        q = msg.orientation
        siny_cosp = 2 * (q.w * q.z + q.x * q.y)
        cosy_cosp = 1 - 2 * (q.y * q.y + q.z * q.z)
        yaw_rad = math.atan2(siny_cosp, cosy_cosp)
        yaw_deg = (math.degrees(yaw_rad) + 360) % 360  
        self.yaw = yaw_deg

        rospy.loginfo_throttle(1.0, f"[IMU] 현재 Yaw: {self.yaw:.2f}°")  # 1초마다 출력

    def lidar_callback(self, msg: LaserScan):
        current_time = msg.header.stamp.to_sec()
        current_ranges = np.array(msg.ranges)

        if self.prev_ranges is None:
            self.prev_ranges = current_ranges
            self.prev_time = current_time
            self.angle_min = msg.angle_min
            self.angle_increment = msg.angle_increment
            return

        if current_time - self.prev_time < self.time_threshold:
            return

        self.dynamic_lidar_flags = {}
        self.static_lidar_flags = {}

        for i in range(len(current_ranges)):
            curr = current_ranges[i]
            prev = self.prev_ranges[i]

            if not np.isfinite(curr) or not np.isfinite(prev):
                continue

            delta = abs(curr - prev)
            angle = self.angle_min + i * self.angle_increment
            deg = (np.rad2deg(angle) + 360) % 360

            # [IMU] 차량 기준 상대각 계산
            rel_deg = (deg - self.yaw + 360) % 360  

            # [IMU] 정적 장애물 : 차량 전방 -20 ~ 20도 > 340 ~ 20도
            if rel_deg > 340 or rel_deg < 20:
                if 0 < curr < 2.5:
                    self.static_lidar_flags[i] = True

            # [IMU] 동적 장애물 : 차량 전방 -2 ~ 45도 > 358 ~ 45도
            if rel_deg > 358 or rel_deg < 45:
                if 0 < curr < 5.0 and delta > self.dynamic_threshold:
                    self.dynamic_lidar_flags[i] = True

        self.prev_ranges = current_ranges
        self.prev_time = current_time

    def image_callback(self, msg: CompressedImage):
        try:
            np_img = np.frombuffer(msg.data, np.uint8)
            img = cv2.imdecode(np_img, cv2.IMREAD_COLOR)
        except Exception as e:
            rospy.logerr("이미지 디코딩 실패: %s", e)
            return

        # YOLOv8 실행
        results = self.yolo_model.predict(img, imgsz=640, device=self.device, verbose=False)[0]
        curr_boxes = []

        for box in results.boxes:
            x1, y1, x2, y2 = map(int, box.xyxy[0])
            conf = float(box.conf[0])
            cls = int(box.cls[0])
            label = self.yolo_model.model.names[cls]
            curr_boxes.append((x1, y1, x2, y2, label))

            cv2.rectangle(img, (x1, y1), (x2, y2), (0, 255, 0), 2)
            cv2.putText(img, f"{label} {conf:.2f}", (x1, y1 - 10), cv2.FONT_HERSHEY_SIMPLEX, 0.5, (0, 255, 0), 2)

        # 박스 움직임 분석
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

        # 융합 판단
        if len(moving_objects) > 0 and len(self.dynamic_lidar_flags) > 2:
            print("🟥 동적 장애물 (YOLO + LiDAR)")
        elif len(moving_objects) > 0:
            print("🟧 YOLO만 탐지")
        elif len(self.static_lidar_flags) > 2:
            print("🟨 정적 장애물 (LiDAR)")
        else:
            print("🟩 정상 주행")

        self.prev_boxes = curr_boxes

def main():
    try:
        SensorFusionNode()
        rospy.spin()
    except rospy.ROSInterruptException:
        pass

if __name__ == "__main__":
    main()