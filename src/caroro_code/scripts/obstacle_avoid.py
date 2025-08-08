#!/usr/bin/env python3
# -*- coding: utf-8 -*-

import rospy
from sensor_msgs.msg import LaserScan, CompressedImage, Imu
from std_msgs.msg import Float64
from cv_bridge import CvBridge
import numpy as np
import cv2
import torch
from ultralytics import YOLO
import math

class SensorFusionNode:
    def __init__(self):
        rospy.init_node("yolo_lidar_fusion_node")

        # LiDAR 변수 초기화
        self.prev_ranges = None
        self.prev_time = None
        self.angle_min = None
        self.angle_increment = None
        self.dynamic_threshold = rospy.get_param("~dynamic_threshold", 0.3)
        self.time_threshold = rospy.get_param("~time_threshold", 0.1)
        
        # IMU 변수
        self.yaw = 0.0

        # YOLO 모델 로드
        self.device = "cuda" if torch.cuda.is_available() else "cpu"
        self.yolo_model = YOLO("yolov8n.pt") # 적절한 모델 경로로 수정하세요.
        
        # YOLO 감지 결과 및 LiDAR 데이터 저장 변수
        self.yolo_detections = []
        self.current_ranges = None
        
        # ROS 통신
        self.bridge = CvBridge()
        rospy.Subscriber("/scan", LaserScan, self.lidar_callback)
        rospy.Subscriber("/image_jpeg/compressed", CompressedImage, self.image_callback)
        rospy.Subscriber("/imu", Imu, self.imu_callback)

        # 장애물 대응 명령 퍼블리시
        self.obstacle_run_pub = rospy.Publisher("obstacle_run_cmd", Float64, queue_size=10)

    def imu_callback(self, msg: Imu):
        q = msg.orientation
        siny_cosp = 2 * (q.w * q.z + q.x * q.y)
        cosy_cosp = 1 - 2 * (q.y * q.y + q.z * q.z)
        yaw_rad = math.atan2(siny_cosp, cosy_cosp)
        self.yaw = (math.degrees(yaw_rad) + 360) % 360
        rospy.loginfo_throttle(1.0, f"[IMU] 현재 Yaw: {self.yaw:.2f}°")

    def lidar_callback(self, msg: LaserScan):
        current_time = msg.header.stamp.to_sec()
        self.current_ranges = np.array(msg.ranges)

        if self.prev_ranges is None:
            self.prev_ranges = self.current_ranges
            self.prev_time = current_time
            self.angle_min = msg.angle_min
            self.angle_increment = msg.angle_increment
            return

        if current_time - self.prev_time < self.time_threshold:
            return
        
        # LiDAR 데이터만 저장하고, 장애물 판단 로직은 image_callback에서 호출
        self.prev_ranges = self.current_ranges
        self.prev_time = current_time
    
    def fuse_sensors(self):
        # 융합에 필요한 데이터가 모두 있는지 확인
        if not hasattr(self, 'yolo_detections') or not isinstance(self.current_ranges, np.ndarray) or not isinstance(self.prev_ranges, np.ndarray):
            return

        # 1. 동적 장애물 판단 (YOLO + LiDAR)
        is_person_detected = any(d['cls'] == 0 for d in self.yolo_detections) # YOLO의 'person' 클래스 ID는 0
        is_dynamic_liadr = False
        dynamic_points_count = 0

        for i in range(len(self.current_ranges)):
            curr = self.current_ranges[i]
            prev = self.prev_ranges[i]

            if not np.isfinite(curr) or not np.isfinite(prev):
                continue

            delta = abs(curr - prev)
            angle = self.angle_min + i * self.angle_increment
            deg = (np.rad2deg(angle) + 360) % 360
            rel_deg = (deg - self.yaw + 360) % 360

            # 동적 장애물 판단 범위 및 조건 (정면 47도)
            if (rel_deg > 358 or rel_deg < 45) and 0 < curr < 5.0 and delta > self.dynamic_threshold:
                dynamic_points_count += 1

        if is_person_detected and dynamic_points_count > 2:
            rospy.loginfo("🟥 동적 장애물 (YOLO + LiDAR)")
            self.obstacle_run_pub.publish(Float64(1.0))
            return

        # 2. 정적 장애물 판단 (LiDAR만)
        static_points_count = 0
        for i in range(len(self.current_ranges)):
            curr = self.current_ranges[i]
            angle = self.angle_min + i * self.angle_increment
            deg = (np.rad2deg(angle) + 360) % 360
            rel_deg = (deg - self.yaw + 360) % 360
            
            # 정적 장애물 판단 범위 및 조건 (정면 40도)
            if (rel_deg > 340 or rel_deg < 20) and 0 < curr < 2.5:
                static_points_count += 1
        
        if static_points_count > 2:
            rospy.loginfo("🟨 정적 장애물 (LiDAR)")
            self.obstacle_run_pub.publish(Float64(2.0))
            return

        # 3. 정상 주행
        rospy.loginfo("🟩 정상 주행")
        self.obstacle_run_pub.publish(Float64(3.0))

    def image_callback(self, msg: CompressedImage):
        try:
            np_img = np.frombuffer(msg.data, np.uint8)
            img = cv2.imdecode(np_img, cv2.IMREAD_COLOR)
        except Exception as e:
            rospy.logerr("이미지 디코딩 실패: %s", e)
            return
        
        # YOLO 모델 실행 및 결과 저장
        results = self.yolo_model(img, verbose=False)
        self.yolo_detections = []
        for r in results:
            for box in r.boxes:
                conf = float(box.conf[0])
                cls = int(box.cls[0])
                self.yolo_detections.append({'conf': conf, 'cls': cls})
        
        # YOLO 실행 후 센서 융합 함수 호출
        self.fuse_sensors()

def main():
    SensorFusionNode()
    rospy.spin()

if __name__ == "__main__":
    main()