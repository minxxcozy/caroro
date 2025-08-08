#!/usr/bin/env python3
# -*- coding:utf-8 -*-

import rospy
import cv2
from std_msgs.msg import Float64
from sensor_msgs.msg import CompressedImage
from cv_bridge import CvBridge
import numpy as np
import os

class PID:
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
        
        self.prev_error = error

        output = self.Kp * error + self.Ki * self.integral + self.Kd * derivative
        
        # 출력 정규화
        output = output / 640.0
        
        return output

class LaneFollower:
    def __init__(self):
        rospy.init_node("lane_follower_node")
        rospy.Subscriber("/image_jpeg/compressed", CompressedImage, self.cam_CB)
        
        self.speed_pub = rospy.Publisher("/lane_speed_cmd", Float64, queue_size=1)
        self.steer_pub = rospy.Publisher("/lane_steer_cmd", Float64, queue_size=1)
        
        self.bridge = CvBridge()
        self.pid = PID(0.7, 0.0, 0.2)
        
        self.steer_history = deque(maxlen=5)
        self.center_history = deque(maxlen=3)
        
        self.prev_steer = 0.5
        self.stable_count = 0

    def moving_average_filter(self, new_value, history_queue):
        """이동평균 필터"""
        history_queue.append(new_value)
        return sum(history_queue) / len(history_queue)
    
    def estimate_line_x(mask, threshold=100):
        coords = cv2.findNonZero(mask)
        if coords is not None and len(coords) > threshold:
            avg = np.mean(coords[:,0,0])
            return int(avg)
        else:
            return None
        
    def get_current_lane(left_yellow_mask, right_yellow_mask):
        left_yellow = cv2.countNonZero(left_yellow_mask)
        right_yellow = cv2.countNonZero(right_yellow_mask)
        if right_yellow > left_yellow:
            return 'left'
        elif left_yellow > right_yellow:
            return 'right'
        else:
            return 'unknown'

    def cam_CB(self, msg):
        os.system("clear")
        img = self.bridge.compressed_imgmsg_to_cv2(msg)
        y, x = img.shape[0:2]

        hsv = cv2.cvtColor(img, cv2.COLOR_BGR2HSV)
        h, s, v = cv2.split(hsv)

        # 퍼스펙티브 변환 (yellow_first.py의 좌표 반영)
        src_points = np.float32([
            [0, 420], [280, 260], [x - 280, 260], [x, 420]
        ])
        dst_points = np.float32([
            [x // 8, 480], [x // 8, 0], [x // 8 * 7, 0], [x // 8 * 7, 480]
        ])

        matrix = cv2.getPerspectiveTransform(src_points, dst_points)
        warped_img = cv2.warpPerspective(filtered_img, matrix, (x, y))  # 조감도 이미지

        height, width = warped_img.shape[:2]

        left_hsv = hsv[:, :width//2]
        right_hsv = hsv[:, width//2:]

        lower_white = np.array([0, 0, 220]) 
        upper_white = np.array([180, 30, 255])
        lower_yellow = np.array([15, 100, 100])
        upper_yellow = np.array([35, 255, 255])

        left_white = cv2.inRange(left_hsv, lower_white, upper_white)
        left_yellow = cv2.inRange(left_hsv, lower_yellow, upper_yellow)
        right_white = cv2.inRange(right_hsv, lower_white, upper_white)
        right_yellow = cv2.inRange(right_hsv, lower_yellow, upper_yellow)

        left_mask = cv2.bitwise_or(left_white, left_yellow)
        right_mask = cv2.bitwise_or(right_white, right_yellow)

        left_x = estimate_line_x(left_mask)
        right_x = estimate_line_x(right_mask)
    
        lane_width_guess = 160  

        if left_x is not None and right_x is not None:
            lane_center = (left_x + (right_x + width//2)) / 2
        elif left_x is not None:
            lane_center = left_x - lane_width_guess
        elif right_x is not None:
            lane_center = (right_x + width//2) + lane_width_guess
        else:
            lane_center = width / 2

        mid = width / 2

        error = lane_center - mid
        speed = 0
        angle = steer(error)
        drive(angle, speed)

        current_lane = get_current_lane(left_yellow, right_yellow)

        kernel = np.ones((3,3), np.uint8)
        yellow_range = cv2.morphologyEx(yellow_range, cv2.MORPH_OPEN, kernel)
        yellow_range = cv2.morphologyEx(yellow_range, cv2.MORPH_CLOSE, kernel)

        filtered_img = cv2.bitwise_and(img, img, mask=yellow_range)

        matrix = cv2.getPerspectiveTransform(src_points, dst_points)
        warped_img = cv2.warpPerspective(filtered_img, matrix, (x, y))

        grayed_img = cv2.cvtColor(warped_img, cv2.COLOR_BGR2GRAY)
        _, bin_img = cv2.threshold(grayed_img, 50, 255, cv2.THRESH_BINARY)

        # 히스토그램을 통한 차선 위치 분석
        roi_height = y // 3
        roi_bin_img = bin_img[y - roi_height:, :]
        histogram = np.sum(roi_bin_img, axis=0)
        
        histogram = cv2.GaussianBlur(histogram.astype(np.float32), (15, 1), 0).flatten()
        
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
            print(f"Both lanes: L={left_center}, R={right_center}, Center={center_index}")
            
        elif len(left_indices) > 0:
            left_center = (left_indices[0] + left_indices[-1]) // 2
            estimated_lane_width = x * 0.3
            center_index = left_center + estimated_lane_width // 2
            print(f"Left lane only: {left_center} -> Center: {center_index}")
            
        elif len(right_indices) > 0:
            right_center = (right_indices[0] + right_indices[-1]) // 2
            estimated_lane_width = x * 0.3
            center_index = right_center - estimated_lane_width // 2
            print(f"Right lane only: {right_center} -> Center: {center_index}")
            
        else:
            print("No lane detected")
            if len(self.center_history) > 0:
                center_index = int(np.mean(self.center_history))

        center_index = int(self.moving_average_filter(center_index, self.center_history))

        standard_line = x // 2
        error = center_index - standard_line
        
        pid_output = self.pid.pid_control(error)
        
        max_steer_change = 0.05
        steer = np.clip(pid_output, 0.2, 0.8)
        
        if abs(steer - self.prev_steer) > max_steer_change:
            if steer > self.prev_steer:
                steer = self.prev_steer + max_steer_change
            else:
                steer = self.prev_steer - max_steer_change

        steer = self.moving_average_filter(steer, self.steer_history)
        self.prev_steer = steer


        self.steer_pub.publish(Float64(steer))
        self.speed_pub.publish(Float64(speed))

        # 디버깅용 시각화
        cv2.line(warped_img, (standard_line, 0), (standard_line, y), (255, 0, 0), 2)
        cv2.line(warped_img, (center_index, 0), (center_index, y), (0, 0, 255), 2)
        
        cv2.imshow("img", img)
        cv2.imshow("warped_img", warped_img)
        cv2.waitKey(1)

def main():
    try:
        LaneFollower()
        rospy.spin()
    except rospy.ROSInterruptException:
        pass

if __name__ == "__main__":
    main()