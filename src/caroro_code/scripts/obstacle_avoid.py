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
        self.dynamic_lidar_flags = {}
        self.static_lidar_flags = {}

        # IMU 변수
        self.yaw = 0.0

        # YOLO 모델 로드 (사용은 안 함, 남겨둠)
        self.device = "cuda" if torch.cuda.is_available() else "cpu"
        self.yolo_model = YOLO("yolov8n.pt")

        # ROS 통신
        self.bridge = CvBridge()
        rospy.Subscriber("/scan", LaserScan, self.lidar_callback)
        rospy.Subscriber("/image_jpeg/compressed", CompressedImage, self.image_callback)
        rospy.Subscriber("/imu", Imu, self.imu_callback)

        # 장애물 대응 명령 퍼블리시
        self.obstacle_run_pub = rospy.Publisher("obstacle_run_cmd", Float64, queue_size=10)

    def imu_callback(self, msg):
        q = msg.orientation
        siny_cosp = 2 * (q.w * q.z + q.x * q.y)
        cosy_cosp = 1 - 2 * (q.y * q.y + q.z * q.z)
        yaw_rad = math.atan2(siny_cosp, cosy_cosp)
        self.yaw = (math.degrees(yaw_rad) + 360) % 360
        rospy.loginfo_throttle(1.0, f"[IMU] 현재 Yaw: {self.yaw:.2f}°")

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

        self.dynamic_lidar_flags.clear()
        self.static_lidar_flags.clear()

        for i in range(len(current_ranges)):
            curr = current_ranges[i]
            prev = self.prev_ranges[i]

            if not np.isfinite(curr) or not np.isfinite(prev):
                continue

            delta = abs(curr - prev)
            angle = self.angle_min + i * self.angle_increment
            deg = (np.rad2deg(angle) + 360) % 360

            rel_deg = (deg - self.yaw + 360) % 360

            # 정적 장애물 판단 범위 및 조건
            if rel_deg > 340 or rel_deg < 20:
                if 0 < curr < 2.5:
                    self.static_lidar_flags[i] = True

            # 동적 장애물 판단 범위 및 조건
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

        # 정적/동적 장애물 여부에 따라 퍼블리시
        if len(self.dynamic_lidar_flags) > 2:
            rospy.loginfo("🟥 동적 장애물 (LiDAR만)")
            self.obstacle_run_pub.publish(Float64(1.0))  # 동적 장애물
        elif len(self.static_lidar_flags) > 2:
            rospy.loginfo("🟨 정적 장애물 (LiDAR)")
            self.obstacle_run_pub.publish(Float64(2.0))  # 정적 장애물
        else:
            rospy.loginfo("🟩 정상 주행")
            self.obstacle_run_pub.publish(Float64(3.0))  # 정상 주행

def main():
    SensorFusionNode()
    rospy.spin()

if __name__ == "__main__":
    main()
