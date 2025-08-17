#!/usr/bin/env python3
# -*- coding: utf-8 -*-

import rospy
from sensor_msgs.msg import LaserScan, CompressedImage
from std_msgs.msg import Float64
from cv_bridge import CvBridge
import numpy as np
import cv2
import torch
from ultralytics import YOLO

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
        self.steer_angle = 0.0  # 조향각 변수

        # YOLO 모델 로드
        self.device = "cuda" if torch.cuda.is_available() else "cpu"
        self.yolo_model = YOLO("yolov8n.pt")

        self.yolo_detections = []
        self.current_ranges = None

        self.bridge = CvBridge()
        rospy.Subscriber("/scan", LaserScan, self.lidar_callback)
        rospy.Subscriber("/image_jpeg/compressed", CompressedImage, self.image_callback)
        rospy.Subscriber('/lane_steer_cmd', Float64, self.steer_callback)   # 조향값 구독

        self.obstacle_run_pub = rospy.Publisher("obstacle_run_cmd", Float64, queue_size=10)

    # 조향값 받아서 저장
    def steer_callback(self, msg: Float64):
        self.steer_angle = msg.data

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

        self.prev_ranges = self.current_ranges
        self.prev_time = current_time

    def fuse_sensors(self):
        if not self.yolo_detections or self.current_ranges is None or self.prev_ranges is None:
            return

        is_person_detected = any(d['cls'] == 0 for d in self.yolo_detections)
        dynamic_points_count = 0
        static_points_count = 0

        for i in range(len(self.current_ranges)):
            curr = self.current_ranges[i]
            prev = self.prev_ranges[i]

            if not np.isfinite(curr) or not np.isfinite(prev):
                continue

            delta = abs(curr - prev)
            angle = self.angle_min + i * self.angle_increment
            deg = (np.rad2deg(angle) + 360) % 360

            # 조향각을 차량 기준 heading으로 사용
            rel_deg = (deg - self.steer_angle + 360) % 360

            if (rel_deg > 358 or rel_deg < 45) and 0 < curr < 5.0 and delta > self.dynamic_threshold:   # 동적 장애물 조건
                dynamic_points_count += 1

            if (rel_deg > 340 or rel_deg < 20) and 0 < curr < 2.5:  # 정적 장애물 조건
                static_points_count += 1

        if is_person_detected and dynamic_points_count > 2:
            rospy.loginfo("🟥 동적 장애물 (YOLO + LiDAR)")
            self.obstacle_run_pub.publish(Float64(1.0))
            return

        if static_points_count > 2:
            rospy.loginfo("🟨 정적 장애물 (LiDAR)")
            self.obstacle_run_pub.publish(Float64(2.0))
            return

        rospy.loginfo("🟩 정상 주행")
        self.obstacle_run_pub.publish(Float64(3.0))

    def image_callback(self, msg: CompressedImage):
        try:
            np_img = np.frombuffer(msg.data, np.uint8)
            img = cv2.imdecode(np_img, cv2.IMREAD_COLOR)
        except Exception as e:
            rospy.logerr(f"이미지 디코딩 실패: {e}")
            return

        results = self.yolo_model(img, verbose=False)
        self.yolo_detections = []
        for r in results:
            for box in r.boxes:
                conf = float(box.conf[0])
                cls = int(box.cls[0])
                self.yolo_detections.append({'conf': conf, 'cls': cls})

        self.fuse_sensors()

def main():
    SensorFusionNode()
    rospy.spin()

if __name__ == "__main__":
    main()