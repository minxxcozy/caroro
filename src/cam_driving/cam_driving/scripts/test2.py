#!/usr/bin/env python3
# -*- coding:utf-8 -*-

import rospy
import cv2
from std_msgs.msg import Float64
from sensor_msgs.msg import CompressedImage
from cv_bridge import CvBridge
import numpy as np

class PID:
    """
    PID 제어기 클래스
    """
    def __init__(self, Kp, Ki, Kd):
        self.Kp = Kp
        self.Ki = Ki
        self.Kd = Kd
        self.prev_error = 0
        self.integral = 0
        self.integral_limit = 0.1

    def pid_control(self, error):
        self.integral += error
        self.integral = np.clip(self.integral, -self.integral_limit, self.integral_limit)
        
        derivative = error - self.prev_error
        
        output = self.Kp * error + self.Ki * self.integral + self.Kd * derivative
        
        self.prev_error = error
        
        # 출력을 0.0~1.0 범위로 정규화
        return 0.5 + output / 150

class LaneFollower:
    def __init__(self):
        rospy.init_node("lane_follower_node")
        
        # ROS 토픽 구독 및 발행 설정
        rospy.Subscriber("/image_jpeg/compressed", CompressedImage, self.cam_CB)
        
        self.speed_pub = rospy.Publisher("/commands/motor/speed", Float64, queue_size=1)
        self.steer_pub = rospy.Publisher("/commands/servo/position", Float64, queue_size=1)

        # 클래스 변수 초기화
        self.bridge = CvBridge()
        self.pid = PID(0.6, 0.0, 0.4)
        self.lane_width_guess = 160

        # 차선 인식을 위한 HSV 컬러 범위 (흰색 및 노란색)
        self.lower_white = np.array([0, 0, 192])
        self.upper_white = np.array([179, 64, 255])
        self.lower_yellow = np.array([15, 128, 0])
        self.upper_yellow = np.array([40, 255, 255])
        
        print("Lane Follower System Ready")

    def cam_CB(self, msg):
        img = self.bridge.compressed_imgmsg_to_cv2(msg, "bgr8")
        y, x = img.shape[0:2]

        # ---------------- ROI 설정 및 색상 필터링 ----------------
        roi_height_ratio = 2/5
        lane_roi = img[int(y * (1 - roi_height_ratio)):, :]
        hsv = cv2.cvtColor(lane_roi, cv2.COLOR_BGR2HSV)

        left_hsv = hsv[:, :x//2]
        right_hsv = hsv[:, x//2:]

        left_white_mask = cv2.inRange(left_hsv, self.lower_white, self.upper_white)
        left_yellow_mask = cv2.inRange(left_hsv, self.lower_yellow, self.upper_yellow)
        right_white_mask = cv2.inRange(right_hsv, self.lower_white, self.upper_white)
        right_yellow_mask = cv2.inRange(right_hsv, self.lower_yellow, self.upper_yellow)
        
        left_mask = cv2.bitwise_or(left_white_mask, left_yellow_mask)
        right_mask = cv2.bitwise_or(right_white_mask, right_yellow_mask)
        
        combined_mask = np.hstack([left_mask, right_mask])

        # ---------------- 히스토그램을 통한 차선 위치 분석 ----------------
        histogram = np.sum(combined_mask, axis=0)
        
        left_hist = histogram[0 : x // 2]
        right_hist = histogram[x // 2 :]
        
        left_threshold = max(50, np.max(left_hist) * 0.3)
        right_threshold = max(50, np.max(right_hist) * 0.3)
        
        left_indices = np.where(left_hist > left_threshold)[0]
        right_indices = np.where(right_hist > right_threshold)[0] + x // 2

        center_index = x // 2
        
        if len(left_indices) > 0 and len(right_indices) > 0:
            left_center = (left_indices[0] + left_indices[-1]) // 2
            right_center = (right_indices[0] + right_indices[-1]) // 2
            center_index = (left_center + right_center) // 2
            
        elif len(left_indices) > 0:
            left_center = (left_indices[0] + left_indices[-1]) // 2
            center_index = left_center + self.lane_width_guess
            
        elif len(right_indices) > 0:
            right_center = (right_indices[0] + right_indices[-1]) // 2
            center_index = right_center - self.lane_width_guess
            
        else:
            center_index = x // 2
            
        # ---------------- 조향각 및 속도 계산 및 발행 ----------------
        standard_line = x // 2
        error = center_index - standard_line
        
        steer_output = self.pid.pid_control(error)
        
        # 일정한 속도로 주행
        speed = 1200
        
        # 조향각 및 속도 메시지 발행
        self.steer_pub.publish(Float64(steer_output))
        self.speed_pub.publish(Float64(speed))

        # ---------------- 디버깅용 시각화 ----------------
        display_img = lane_roi.copy()
        cv2.line(display_img, (standard_line, 0), (standard_line, display_img.shape[0]), (255, 0, 0), 2)
        cv2.line(display_img, (int(center_index), 0), (int(center_index), display_img.shape[0]), (0, 0, 255), 2)
        
        cv2.imshow("Original Image", img)
        cv2.imshow("ROI Image", display_img)
        cv2.imshow("Combined Mask", combined_mask)
        cv2.waitKey(1)

def main():
    try:
        LaneFollower()
        rospy.spin()
    except rospy.ROSInterruptException:
        pass

if __name__ == "__main__":
    main()