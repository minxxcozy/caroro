#!/usr/bin/env python3
# -*- coding:utf-8 -*-

# ROS와 OpenCV 관련 모듈 임포트
import rospy
import cv2
from std_msgs.msg import Float64, Bool, Int32
from sensor_msgs.msg import CompressedImage
from cv_bridge import CvBridge
import numpy as np
import os


class Lane_sub:
    def __init__(self):
        rospy.init_node("land_sub_node")
        rospy.Subscriber("/image_jpeg/compressed", CompressedImage, self.cam_CB)
        rospy.Subscriber('/traffic_status', Bool, self.traffic_callback)
        rospy.Subscriber("/intersection_yellow", Int32, self.intersection_callback)
        
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
        self.steer = 0.39

        self.traffic = 0.47
        self.plus_traffic = - 0.0045
        self.x_traffic = 1.0

        self.right = 0.5 
        self.plus_right = 0.03
        self.x_right = 1.2

        self.plus_steer = 0  # difault
        self.x_steer = 1.0
        self.intersection_yellow = 0


    def intersection_callback(self, msg):
        self.intersection_yellow = msg.data
        if self.intersection_yellow == 15 or self.intersection_yellow == 16:
            print("plus_right")
            self.plus_steer = self.plus_right
            self.x_steer = self.x_right
        else:
            print("plus_traffic")
            self.plus_steer = self.plus_traffic
            self.x_steer = self.x_traffic

    def traffic_callback(self, msg):
        self.traffic_status = msg.data
        if self.traffic_status == False:
            print("reset")
            self.steer = self.traffic
            # 신호등 정지일때 계속 초기화. 


    # ----------- 콜백 함수 ------------
    def cam_CB(self, msg):

        img = self.bridge.compressed_imgmsg_to_cv2(msg)
        y, x = img.shape[0:2]  # 이미지 크기 정보 추출(높이h, 너비w)

        img_hsv = cv2.cvtColor(img, cv2.COLOR_BGR2HSV)
        h, s, v = cv2.split(img_hsv) 
        
        yellow_lower = np.array([15, 128, 0])
        yellow_upper = np.array([40, 255, 255])
        yellow_range = cv2.inRange(img_hsv, yellow_lower, yellow_upper)

        filtered_img = cv2.bitwise_and(img, img, mask=yellow_range)

        
        src_points = np.float32([
            [0, 310], [155, 270], [400, 270], [500, 320]
        ])
        # 조감도 이미지에서 그 4점이 가야 할 위치
        dst_points = np.float32([
            [x // 8, 480], [x // 8, 0], [x // 8 * 7, 0], [x // 8 * 7, 480]
        ])

        matrix = cv2.getPerspectiveTransform(src_points, dst_points)
        warped_img = cv2.warpPerspective(filtered_img, matrix, (x, y))  # 조감도 이미지

        # 흑백 및 이진화
        grayed_img = cv2.cvtColor(warped_img, cv2.COLOR_BGR2GRAY)
        _, bin_img = cv2.threshold(grayed_img, 50, 255, cv2.THRESH_BINARY) 

        histogram = np.sum(bin_img, axis=0)
        left_hist = histogram[0 : x // 2]
        right_hist = histogram[x // 2 :]

        left_indices = np.where(left_hist > 30)[0]
        right_indices = np.where(right_hist > 30)[0] + x // 2 
        indices = np.where(histogram > 30)[0]

        try:    
            if len(left_indices) == 0 or len(left_indices) == 1:
                self.steer = self.steer + self.plus_steer
                steer = self.steer
                
            else:
                center_index = (left_indices[0] + left_indices[-1]) // 2 + x * 0.25
                standard_line = x // 2  # 화면 중앙 기준선
                degree_per_pixel = 1 / x
                steer = 0.5 + ((center_index - standard_line) * degree_per_pixel) * self.x_steer
                if steer > 0.67 : # right_turn 할때 조향각 추가로
                    steer = 1.0
                if self.intersection_yellow == 15 or self.intersection_yellow == 16 :
                    self.steer = self.right
                else:
                    self.steer = self.traffic


        except:
            steer = 0.5

        canny_img = cv2.Canny(bin_img, 2, 2)
        # cv2.imshow("canny_img", canny_img)

        lines = cv2.HoughLinesP(bin_img, 1, np.pi/180, 90, 50, 5)
        if lines is not None:
            for line in lines:
                x1, y1, x2, y2 = line[0]
                cv2.line(warped_img, (x1, y1), (x2, y2), (0, 255, 0), 1)
                self.cross_flag += 1
            # print(self.cross_flag)

        speed = 1200
    
        print(f"steer : {steer}")
        print(f"speed : {speed}")

        # 속도 및 조향 퍼블리시
        self.steer_msg.data = steer
        self.speed_msg.data = speed
        self.speed_pub.publish(self.speed_msg)
        self.steer_pub.publish(self.steer_msg)
        self.cross_flag = 0  # 허프라인 감지 횟수 초기화

        # ---------------- 디버깅용 이미지 출력 ----------------
        # cv2.imshow("img", img)
        # cv2.imshow("warped_img", warped_img)
        cv2.waitKey(1)



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
        # print(f"steer : {output + 0.5}")

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