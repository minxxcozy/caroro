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
        os.system("clear")  # 터미널 화면 초기화
        # 압축 이미지를 OpenCV 형식으로 변환
        img = self.bridge.compressed_imgmsg_to_cv2(msg)
        y, x = img.shape[0:2]  # 이미지 크기 정보 추출(높이h, 너비w)

        # HSV 색공간으로 변환
        img_hsv = cv2.cvtColor(img, cv2.COLOR_BGR2HSV)
        h, s, v = cv2.split(img_hsv)  # 필요 시 개별 채널 사용 가능
        
        # ----------- HSV  ------------    
        # H의 임계값 : 0~179 (색상무관)
        # S의 임계값 : 0~64 (채도가 낮음)
        # V의 임계값 : 200~255 (명도가 높음)

        
        # 노란색 범위 마스크 생성
        yellow_lower = np.array([15, 128, 0])
        yellow_upper = np.array([40, 255, 255])
        yellow_range = cv2.inRange(img_hsv, yellow_lower, yellow_upper)

        # # 흰색 범위 마스크 생성
        # white_lower = np.array([0, 0, 192])
        # white_upper = np.array([179, 64, 255])
        # white_range = cv2.inRange(img_hsv, white_lower, white_upper)

        # # 노란색과 흰색 마스크 결합
        # combined_range = cv2.bitwise_or(yellow_range, white_range)

        # 마스크로 필터링된 이미지 생성
        filtered_img = cv2.bitwise_and(img, img, mask=yellow_range)

        # ---------------- 퍼스펙티브 변환 (조감도 시점 만들기)(Bird Width View) ----------------
        # 원본 이미지에서 기준이 되는 4개의 꼭짓점 좌표
        src_points = np.float32([
            [0, 420], [280, 260], [x - 280, 260], [x, 420]
        ])
        # 조감도 이미지에서 그 4점이 가야 할 위치
        dst_points = np.float32([
            [x // 8, 480], [x // 8, 0], [x // 8 * 7, 0], [x // 8 * 7, 480]
        ])

        matrix = cv2.getPerspectiveTransform(src_points, dst_points)
        warped_img = cv2.warpPerspective(filtered_img, matrix, (x, y))  # 조감도 이미지

        # 흑백 및 이진화
        grayed_img = cv2.cvtColor(warped_img, cv2.COLOR_BGR2GRAY)
        _, bin_img = cv2.threshold(grayed_img, 50, 255, cv2.THRESH_BINARY) #픽셀 값이 임계값(여기서는 50)보다 크면 255(흰색), 작으면 0(검은색)으로 변환.

        # 히스토그램을 통한 차선 위치 분석
        histogram = np.sum(bin_img, axis=0) #세로 방향(높이 방향)으로 합산.
        left_hist = histogram[0 : x // 2]
        right_hist = histogram[x // 2 :]

        left_indices = np.where(left_hist > 30)[0] #배열의 인덱스를 반환
        right_indices = np.where(right_hist > 30)[0] + x // 2 # 오른쪽 히스토그램 인덱스는 x // 2를 더해 조정
        indices = np.where(histogram > 30)[0]

        # ---------------- 차선 위치에 따라 조향 판단 ----------------
        try:
            if len(left_indices) == 0 and len(right_indices) == 0:
                center_index = x // 2
                print("no_line")
                
            else:
                center_index = (left_indices[0] + left_indices[-1]) // 2 + x * 0.27
                print(f"left_line : {center_index} : {left_indices[1]}, {left_indices[-1]}")
                print("both_line \n")

        except:
            center_index = x // 2
            print("no_line")

        # ---------------- 허프 선 변환을 통한 선 검출 ----------------
        canny_img = cv2.Canny(bin_img, 2, 2)
        # cv2.imshow("canny_img", canny_img)

        lines = cv2.HoughLinesP(bin_img, 1, np.pi/180, 90, 50, 5) #입력 이미지, 거리 해상도, 각도 해상도, 직선인식 최소 임계값, 최소 직선 길이, 직선 사이 최대 간격
        if lines is not None:
            for line in lines:
                x1, y1, x2, y2 = line[0]
                cv2.line(warped_img, (x1, y1), (x2, y2), (0, 255, 0), 1)
                self.cross_flag += 1
            # print(self.cross_flag)

        # ---------------- 조향 각 계산 ----------------
        standard_line = x // 2  # 화면 중앙 기준선
        degree_per_pixel = 1 / x

        steer = 0.5 + ((center_index - standard_line) * degree_per_pixel)
        speed = 1200
        if(steer < 0.4):
            steer = 0.5 + ((center_index - standard_line) * degree_per_pixel)*2.0
            speed = 1000
        elif(steer == 0):
            speed = 500
            #차선변경때 
            

        
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
