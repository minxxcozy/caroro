#!/usr/bin/env python3
# -*- coding:utf-8 -*-

# ROS와 OpenCV 관련 모듈 임포트
import rospy
import cv2
from std_msgs.msg import Float64
from sensor_msgs.msg import CompressedImage
from cv_bridge import CvBridge
import numpy as np
import os


class Lane_sub:
    def __init__(self):
        rospy.init_node("land_sub_node")
        rospy.Subscriber("/image_jpeg/compressed", CompressedImage, self.cam_CB)
        
        # 속도 및 조향각 퍼블리셔 설정
        self.speed_pub = rospy.Publisher("/lane_speed_cmd", Float64, queue_size=1)
        self.steer_pub = rospy.Publisher("/lane_steer_cmd", Float64, queue_size=1)
        
        # 메시지 초기화
        self.image_msg = CompressedImage()
        self.bridge = CvBridge()  # ROS 이미지 ↔ OpenCV 이미지 변환기
        self.speed_msg = Float64()
        self.steer_msg = Float64()
        self.cross_flag = 0  # 허프라인이 감지된 횟수 (교차로 감지 용도 등)
        self.pid = PID(0.02, 0.001, 0.03)

    # ----------- 콜백 함수 ------------
    def cam_CB(self, msg):

        # 속도 및 조향 퍼블리시
        self.steer_msg.data = 0
        self.speed_msg.data = 0
        self.speed_pub.publish(self.speed_msg)
        self.steer_pub.publish(self.steer_msg)

class PID:
    def __init__(self, Kp, Ki, Kd):
        self.Kp = Kp
        self.Ki = Ki
        self.Kd = Kd
        self.prev_error = 0
        self.integral = 0

    def pid_control(self, error):
        self.integral += error
        derivative = error - self.prev_error
        output = self.Kp * error + self.Ki * self.integral + self.Kd * derivative
        self.prev_error = error
        print(f"steer : {output + 0.5}")

        return 0.5 + output  # ROS 조향 기준

# ---------- 메인 함수 ----------
def main():
    try:
        lane_sub = Lane_sub()  # 클래스 인스턴스 생성
        rospy.spin()           # ROS 노드 계속 실행
    except rospy.ROSInterruptException:
        pass

if __name__ == "__main__":
    main()
