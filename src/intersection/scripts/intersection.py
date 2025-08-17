# #!/usr/bin/env python3
# # -*- coding:utf-8 -*-

# # ROS와 OpenCV 관련 모듈 임포트
# import rospy
# import cv2
# from std_msgs.msg import Int32, Bool
# from sensor_msgs.msg import CompressedImage
# from cv_bridge import CvBridge
# import numpy as np

# class Lane_sub:
#     def __init__(self):
#         rospy.init_node("intersection_sub_node")
#         rospy.Subscriber("/image_jpeg/compressed", CompressedImage, self.cam_CB)
#         rospy.Subscriber("/first_arrived_flag", Bool, self.left_position)
        
#         # 메시지 초기화
#         self.image_msg = CompressedImage()
#         self.bridge = CvBridge()  # ROS 이미지 ↔ OpenCV 이미지 변환기
#         self.cross_flag = 0  # 허프라인이 감지된 횟수 (교차로 감지 용도 등)
#         self.count = 0
#         self.prev_if_state = False

#         self.is_left_position = False

#         self.intersection_yellow_pub = rospy.Publisher("intersection_yellow", Int32, queue_size=10)

#     # ----------- 콜백 함수 ------------
#     def left_position(self, msg):
#         self.is_left_position = msg.data
#         # rospy.loginfo(f"Received  left_position flag: {self.is_left_position}")

#     def cam_CB(self, msg):
#         img = self.bridge.compressed_imgmsg_to_cv2(msg)
#         y, x = img.shape[0:2]  # 이미지 크기 정보 추출(높이h, 너비w)

#         # HSV 색공간으로 변환
#         img_hsv = cv2.cvtColor(img, cv2.COLOR_BGR2HSV)
        
#         # ----------- HSV  ------------    
#         # H의 임계값 : 0~179 (색상무관)
#         # S의 임계값 : 0~64 (채도가 낮음)
#         # V의 임계값 : 200~255 (명도가 높음)

#         # 노란색 범위 마스크 생성
#         yellow_lower = np.array([18, 110, 150])
#         yellow_upper = np.array([40, 255, 255])
#         yellow_range = cv2.inRange(img_hsv, yellow_lower, yellow_upper)

#         # 마스크로 필터링된 이미지 생성
#         filtered_img = cv2.bitwise_and(img, img, mask=yellow_range)

#         # ---------------- 퍼스펙티브 변환 (조감도 시점 만들기)(Bird Width View) ----------------
#         src_points = np.float32([
#             [0, 310], [155, 270], [400, 270], [500, 320]
#         ])
#         dst_points = np.float32([
#             [x // 8, 480], [x // 8, 0], [x // 8 * 7, 0], [x // 8 * 7, 480]
#         ])

#         matrix = cv2.getPerspectiveTransform(src_points, dst_points)
#         warped_img = cv2.warpPerspective(filtered_img, matrix, (x, y))  # 조감도 이미지

#         # 흑백 및 이진화
#         grayed_img = cv2.cvtColor(warped_img, cv2.COLOR_BGR2GRAY)
#         _, bin_img = cv2.threshold(grayed_img, 50, 255, cv2.THRESH_BINARY) #픽셀 값이 임계값(여기서는 50)보다 크면 255(흰색), 작으면 0(검은색)으로 변환.

#         # 히스토그램을 통한 차선 위치 분석
#         histogram = np.sum(bin_img, axis=0) #세로 방향(높이 방향)으로 합산.
#         left_hist = histogram[0 : x // 2]
#         right_hist = histogram[x // 2 :]

#         left_indices = np.where(left_hist > 30)[0] #배열의 인덱스를 반환
#         right_indices = np.where(right_hist > 30)[0] + x // 2 # 오른쪽 히스토그램 인덱스는 x // 2를 더해 조정
        
#         indices = np.where(histogram > 30)[0]

#         # if self.is_left_position:
#         #     if len(left_indices) == 0 and len(right_indices) == 0:
#         #         if not self.prev_if_state:
#         #             self.count = self.count + 1
#         #             print(f"{self.count + 7}")
#         #             self.intersection_yellow_pub.publish(Int32(self.count + 7))
#         #         self.prev_if_state = True
#         #     else:
#         #         if self.prev_if_state:
#         #             self.count = self.count + 1
#         #             print(f"{self.count + 7}")
#         #             self.intersection_yellow_pub.publish(Int32(self.count + 7))
#         #         self.prev_if_state = False

#         if self.is_left_position:
#             if len(left_indices) == 0 and len(right_indices) == 0:
#                 if self.prev_if_state:
#                     self.count = self.count + 1
#                     self.prev_if_state = False
#                 print(f"{self.count + 7}")
#                 self.intersection_yellow_pub.publish(Int32(self.count + 7))
#             else:
#                 if not self.prev_if_state:
#                     self.count = self.count + 1
#                     self.prev_if_state = True
#                 print(f"{self.count + 7}")
#                 self.intersection_yellow_pub.publish(Int32(self.count + 7))

#         # ---------------- 허프 선 변환을 통한 선 검출 ----------------
#         lines = cv2.HoughLinesP(bin_img, 1, np.pi/180, 90, 50, 5) #입력 이미지, 거리 해상도, 각도 해상도, 직선인식 최소 임계값, 최소 직선 길이, 직선 사이 최대 간격
#         if lines is not None:
#             for line in lines:
#                 x1, y1, x2, y2 = line[0]
#                 cv2.line(warped_img, (x1, y1), (x2, y2), (0, 255, 0), 1)
#                 self.cross_flag += 1

#         # ---------------- 디버깅용 이미지 출력 ----------------
#         # cv2.imshow("warped_img", warped_img)
#         # cv2.imshow("img", img)
#         cv2.waitKey(1)

# def main():
#     try:
#         lane_sub = Lane_sub()  # 클래스 인스턴스 생성
#         rospy.spin()           # ROS 노드 계속 실행
#     except rospy.ROSInterruptException:
#         pass

# if __name__ == "__main__":
#     main()

#!/usr/bin/env python3
# -*- coding:utf-8 -*-

# ROS와 OpenCV 관련 모듈 임포트
import rospy
import cv2
from std_msgs.msg import Int32, Bool
from sensor_msgs.msg import CompressedImage
from cv_bridge import CvBridge
import numpy as np

class Lane_sub:
    def __init__(self):
        rospy.init_node("intersection_sub_node")
        rospy.Subscriber("/image_jpeg/compressed", CompressedImage, self.cam_CB)
        rospy.Subscriber("/first_arrived_flag", Bool, self.left_position)
        
        # 메시지 초기화
        self.image_msg = CompressedImage()
        self.bridge = CvBridge()  # ROS 이미지 ↔ OpenCV 이미지 변환기
        self.cross_flag = 0  # 허프라인이 감지된 횟수 (교차로 감지 용도 등)
        self.count = 0
        self.prev_if_state = False

        self.is_left_position = False

        self.intersection_yellow_pub = rospy.Publisher("intersection_yellow", Int32, queue_size=10)

    # ----------- 콜백 함수 ------------
    def left_position(self, msg):
        self.is_left_position = msg.data
        # rospy.loginfo(f"Received  left_position flag: {self.is_left_position}")

    def cam_CB(self, msg):
        img = self.bridge.compressed_imgmsg_to_cv2(msg)
        y, x = img.shape[0:2]  # 이미지 크기 정보 추출(높이h, 너비w)

        # HSV 색공간으로 변환
        img_hsv = cv2.cvtColor(img, cv2.COLOR_BGR2HSV)
        
        # ----------- HSV  ------------    
        # H의 임계값 : 0~179 (색상무관)
        # S의 임계값 : 0~64 (채도가 낮음)
        # V의 임계값 : 200~255 (명도가 높음)

        # 노란색 범위 마스크 생성
        yellow_lower = np.array([18, 110, 150])
        yellow_upper = np.array([40, 255, 255])
        yellow_range = cv2.inRange(img_hsv, yellow_lower, yellow_upper)

        # 마스크로 필터링된 이미지 생성
        filtered_img = cv2.bitwise_and(img, img, mask=yellow_range)

        # ---------------- 퍼스펙티브 변환 (조감도 시점 만들기)(Bird Width View) ----------------
        src_points = np.float32([
            [0, 310], [155, 270], [400, 270], [500, 320]
        ])
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

        # if self.is_left_position:
        #     if len(left_indices) == 0 and len(right_indices) == 0:
        #         if not self.prev_if_state:
        #             self.count = self.count + 1
        #             print(f"{self.count + 7}")
        #             self.intersection_yellow_pub.publish(Int32(self.count + 7))
        #         self.prev_if_state = True
        #     else:
        #         if self.prev_if_state:
        #             self.count = self.count + 1
        #             print(f"{self.count + 7}")
        #             self.intersection_yellow_pub.publish(Int32(self.count + 7))
        #         self.prev_if_state = False

        if self.is_left_position:
            if len(left_indices) == 0 and len(right_indices) == 0:
                if self.prev_if_state:
                    self.count = self.count + 1
                    self.prev_if_state = False
                print(f"{self.count + 7}")
                self.intersection_yellow_pub.publish(Int32(self.count + 7))
            else:
                if not self.prev_if_state:
                    self.count = self.count + 1
                    self.prev_if_state = True
                print(f"{self.count + 7}")
                self.intersection_yellow_pub.publish(Int32(self.count + 7))

        # ---------------- 허프 선 변환을 통한 선 검출 ----------------
        lines = cv2.HoughLinesP(bin_img, 1, np.pi/180, 90, 50, 5) #입력 이미지, 거리 해상도, 각도 해상도, 직선인식 최소 임계값, 최소 직선 길이, 직선 사이 최대 간격
        if lines is not None:
            for line in lines:
                x1, y1, x2, y2 = line[0]
                cv2.line(warped_img, (x1, y1), (x2, y2), (0, 255, 0), 1)
                self.cross_flag += 1

        # ---------------- 디버깅용 이미지 출력 ----------------
        # cv2.imshow("warped_img", warped_img)
        # cv2.imshow("img", img)
        cv2.waitKey(1)

def main():
    try:
        lane_sub = Lane_sub()  # 클래스 인스턴스 생성
        rospy.spin()           # ROS 노드 계속 실행
    except rospy.ROSInterruptException:
        pass

if __name__ == "__main__":
    main()