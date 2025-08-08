#!/usr/bin/env python3
# -*- coding:utf-8 -*-

import rospy
import cv2
import numpy as np
from std_msgs.msg import Float64
from sensor_msgs.msg import CompressedImage, Image
from cv_bridge import CvBridge
import os
import matplotlib.pyplot as plt
from math import atan



class SlidingWindow:
    def __init__(self):
        self.bridge = CvBridge()
        rospy.init_node("Sliding_Window_node")
        rospy.Subscriber("/image_jpeg/compressed", CompressedImage, self.cam_CB)
        self.nothing_flag = False
        self.image_pub = rospy.Publisher("/sliding_window/compressed", CompressedImage, queue_size=10)

        self.speed_pub = rospy.Publisher("/lane_speed_cmd", Float64, queue_size=1)
        self.steer_pub = rospy.Publisher("/lane_steer_cmd", Float64, queue_size=1)
    
        # PID 제어용 변수
        self.prev_error = 0.0
        self.integral = 0.0

        # PID 계수
        self.Kp = 0.0025
        self.Ki = 0.00001
        self.Kd = 0.001


    def detect_color(self, img):
        hsv = cv2.cvtColor(img, cv2.COLOR_BGR2HSV)

        yellow_lower = np.array([15, 128, 0])
        yellow_upper = np.array([40, 255, 255])
        yellow_range = cv2.inRange(hsv, yellow_lower, yellow_upper)

        white_lower = np.array([0, 0, 192])
        white_upper = np.array([179, 64, 255])
        white_range = cv2.inRange(hsv, white_lower, white_upper)

        combined_range = cv2.bitwise_or(yellow_range, white_range)
        blend_color = cv2.bitwise_and(img, img, mask=white_range)
        return blend_color
    
    def img_wrap(self, img, blend_color):
        self.img_x, self.img_y = img.shape[1], img.shape[0]
        y, x = img.shape[0:2]

        src_points = np.float32([
            [0, 310], [155, 270], [400, 270], [500, 320]
        ])
        # 조감도 이미지에서 그 4점이 가야 할 위치
        dst_points = np.float32([
            [x // 8, 480], [x // 8, 0], [x // 8 * 7, 0], [x // 8 * 7, 480]
        ])

        matrix = cv2.getPerspectiveTransform(src_points, dst_points)  
        matrix_inv = cv2.getPerspectiveTransform(dst_points, src_points)
        blend_line = cv2.warpPerspective(blend_color, matrix, (self.img_x, self.img_y))

        return blend_line

    def img_binary(self, blend_line):
        bin = cv2.cvtColor(blend_line, cv2.COLOR_BGR2GRAY)
        binary_line = np.zeros_like(bin)
        binary_line[bin != 0] = 1
        return binary_line
    
    def detect_nothing(self):
        self.nothing_left_x_base = round(self.img_x * 0.140625)
        self.nothing_right_x_base = self.img_x - round(self.img_x * 0.140625)

        self.nothing_pixel_left_x = np.zeros(self.nwindows) + round(self.img_x * 0.140625)
        self.nothing_pixel_right_x = np.zeros(self.nwindows) + self.img_x - round(self.img_x * 0.140625)   
        self.nothing_pixel_y = [round(self.window_height / 2) * index for index in range(0, self.nwindows)]


    def window_search(self, binary_line):
        bottom_half_y = binary_line.shape[0] // 2
        histogram = np.sum(binary_line[bottom_half_y:, :], axis=0)
        midpoint = int(histogram.shape[0] / 2)
        left_x_base = np.argmax(histogram[:midpoint])
        right_x_base = np.argmax(histogram[midpoint:]) + midpoint

        if left_x_base == 0:
            left_x_current = self.nothing_left_x_base
        else:
            left_x_current = left_x_base    
        if right_x_base == 0:
            right_x_current = self.nothing_right_x_base
        else:
            right_x_current = right_x_base      

        out_img = np.dstack((binary_line, binary_line, binary_line)) * 255

        nwindows = self.nwindows
        window_height = self.window_height
        margin = 80
        min_pix = min_pix = round((margin * 2) * window_height * 0.0031)

        lane_pixel = binary_line.nonzero()
        lane_pixel_y = np.array(lane_pixel[0])
        lane_pixel_x = np.array(lane_pixel[1]) 

        left_lane_idx = []
        right_lane_idx = []
        for window in range(nwindows):
            win_y_low = binary_line.shape[0] - (window + 1) * window_height
            win_y_high = binary_line.shape[0] - window * window_height
            win_x_left_low = left_x_current - margin
            win_x_left_high = left_x_current + margin
            win_x_right_low = right_x_current - margin
            win_x_right_high = right_x_current + margin

            if left_x_current != 0:
                cv2.rectangle(out_img, (win_x_left_low, win_y_low), (win_x_left_high, win_y_high), (0, 255, 0), 2)
            if right_x_current != 0:
                cv2.rectangle(out_img, (win_x_right_low, win_y_low), (win_x_right_high, win_y_high), (0, 255, 0), 2)

            good_left_idx = ((lane_pixel_y >= win_y_low) & (lane_pixel_y < win_y_high) &
                             (lane_pixel_x >= win_x_left_low) & (lane_pixel_x < win_x_left_high)).nonzero()[0]
            good_right_idx = ((lane_pixel_y >= win_y_low) & (lane_pixel_y < win_y_high) &
                              (lane_pixel_x >= win_x_right_low) & (lane_pixel_x < win_x_right_high)).nonzero()[0]   
            
            left_lane_idx.append(good_left_idx)
            right_lane_idx.append(good_right_idx)

            if len(good_left_idx) > min_pix:
                left_x_current = int(np.mean(lane_pixel_x[good_left_idx]))
            if len(good_right_idx) > min_pix:
                right_x_current = int(np.mean(lane_pixel_x[good_right_idx]))

        left_lane_idx = np.concatenate(left_lane_idx)
        right_lane_idx = np.concatenate(right_lane_idx)

        left_x = lane_pixel_x[left_lane_idx]
        left_y = lane_pixel_y[left_lane_idx]
        right_x = lane_pixel_x[right_lane_idx]
        right_y = lane_pixel_y[right_lane_idx]

        if len(left_x) == 0 and len(right_x) == 0:
            left_x = self.nothing_pixel_left_x
            left_y = self.nothing_pixel_y
            right_x = self.nothing_pixel_right_x
            right_y = self.nothing_pixel_y  
        else:
            if len(left_x) == 0:
                left_x = right_x - round(self.img_x /2)
                left_y = right_y
            elif len(right_x) == 0:
                right_x = left_x + round(self.img_x /2)
                right_y = left_y
        left_fit = np.polyfit(left_y, left_x, 2)
        right_fit = np.polyfit(right_y, right_x, 2) 

        plot_y = np.linspace(0, binary_line.shape[0] - 1, 5)
        left_fit_x = left_fit[0] * plot_y ** 2 + left_fit[1] * plot_y + left_fit[2]
        right_fit_x = right_fit[0] * plot_y ** 2 + right_fit[1] * plot_y + right_fit[2] 
        center_fit_x = (left_fit_x + right_fit_x) / 2

        center = np.asarray(tuple(zip(center_fit_x, plot_y)), dtype=np.int32)
        left = np.asarray(tuple(zip(left_fit_x, plot_y)), dtype=np.int32)
        right = np.asarray(tuple(zip(right_fit_x, plot_y)), dtype=np.int32)

        cv2.polylines(out_img, [left], isClosed=False, color=(0, 0, 255), thickness=5)
        cv2.polylines(out_img, [right], isClosed=False, color=(0, 255, 0), thickness=5)
        sliding_window_img = out_img
        return sliding_window_img, center, left, right, left_x, right_x, left_y, right_y
    
    def cam_CB(self, data):
        img = self.bridge.compressed_imgmsg_to_cv2(data)
        self.nwindows = 10
        self.window_height = round(img.shape[0] / self.nwindows)

        blend_color = self.detect_color(img)
        blend_line = self.img_wrap(img, blend_color)
        binary_line = self.img_binary(blend_line)

        if not self.nothing_flag:
            self.detect_nothing()
            self.nothing_flag = True

        (sliding_window_img, center, left, right, left_x, right_x, left_y, right_y) = self.window_search(binary_line)

        if right[2][0] > 0:
            center_bottom = right[2][0]  # 가장 하단의 중심선 x좌표
            img_center = self.img_x // 2
            pixel_offset = center_bottom - (self.img_x*0.75) 
            degree_per_pixel = 1 / self.img_x  # 640이면 1픽셀당 약 0.00156
            steer = 0.5 + (pixel_offset * degree_per_pixel) 
            if steer > 0.6 :
                steer = 0.8
        # if center[-1][0] > 0:
        #     center_bottom = center[-1][0] 
        #     img_center = self.img_x // 2
        #     pixel_offset = center_bottom - img_center
        #     degree_per_pixel = 1 / self.img_x  # 640이면 1픽셀당 약 0.00156
        #     steer = 0.5 + (pixel_offset * degree_per_pixel) * 2.0


        else:
            # 차선을 못 보면 중앙 유지
            center_bottom = self.img_x // 2
            pixel_offset = center_bottom - img_center
            steer = 0.5 + pixel_offset




        base_speed = 500  # 기본 속도

        # 조향 값이 커질수록 속도를 줄이기
        steer_strength = abs(steer - 0.5) * 2  # 0~1 범위로 정규화된 조향 강도
        speed_offset = int(steer_strength * 700)  # 감속량 최대 700까지

        speed = base_speed - speed_offset
        speed = max(1200, speed)  # 최소 속도 제한


        self.steer_pub.publish(steer)
        self.speed_pub.publish(speed)

        os.system("clear")
        print(f"center_bottom: {center_bottom}")
        print(f"img_center: {img_center}")
        print(f"pixel_offset: {pixel_offset}")
        print(f"steer: {steer:.3f}")
        print(f"speed: {speed}")


        sliding_window_msg = self.bridge.cv2_to_compressed_imgmsg(sliding_window_img)
        self.image_pub.publish(sliding_window_msg)

        cv2.namedWindow("Sliding Window", cv2.WINDOW_NORMAL)
        cv2.imshow("Sliding Window", sliding_window_img)
        cv2.imshow("image", img)
        cv2.waitKey(1)
    

    def calculate_curvature(self, fit, y_eval):
        A, B = fit[0], fit[1]
        return ((1 + (2 * A * y_eval + B)**2)**1.5) / abs(2 * A)

    def meter_per_pixel(self):
        # 가로 0.35m (차선 간 거리), 세로 1m (가정) 기준
        # self.img_x, self.img_y 는 이미지의 해상도
        # 예시 값: 이미지 폭 640, 높이 480 일 때
        meter_per_pix_x = 0.35 / 320
        meter_per_pix_y = 1.0 / 480
        return meter_per_pix_x, meter_per_pix_y


if __name__ == "__main__":
    sliding_window = SlidingWindow()
    rospy.spin()