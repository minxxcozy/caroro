#!/usr/bin/env python3
#-*- coding:utf-8 -*-

# import rospy
# from sensor_msgs.msg import CompressedImage
# from cv_bridge import CvBridge
import cv2 
import numpy as np


# class Lane_sub:
#     def __init__(self):
#         rospy.init_node("lane_sub_node")
#         rospy.Subscriber("/image_jpeg/compressed", CompressedImage, self.cam_CB)
#         self.image_msg = CompressedImage()
#         self.bridge = CvBridge()
#         self.cross_flag = 0

printed = False
        
def stopline_detect(frame):
    global printed
    if not printed:
        print("stopline detect start!")
        printed = True
        
    img = frame
    y, x = img.shape[0:2]
    img_hsv = cv2.cvtColor(img, cv2.COLOR_BGR2HSV)
    yellow_lower = np.array([15, 128, 0])
    yellow_upper = np.array([40, 255, 255])
    yellow_range = cv2.inRange(img_hsv, yellow_lower, yellow_upper)
    
    white_lower = np.array([0, 0, 192])
    white_upper = np.array([179, 64, 255])
    white_range = cv2.inRange(img_hsv, white_lower, white_upper)
    
    combined_range = cv2.bitwise_or(yellow_range, white_range)
    
    filtered_img = cv2.bitwise_and(img, img, mask=combined_range)
    
    # Perspective Transform
    src_point1 = [0, 420]
    src_point2 = [280, 260]
    src_point3 = [x-280, 260]
    src_point4 = [x, 420]
    src_points = np.float32([src_point1, src_point2, src_point3, src_point4])
    
    dst_point1 = [x//8, 480]
    dst_point2 = [x//8, 0]
    dst_point3 = [x//8*7, 0]
    dst_point4 = [x//8*7, 480]
    dst_points = np.float32([dst_point1, dst_point2, dst_point3, dst_point4])
    
    matrix = cv2.getPerspectiveTransform(src_points, dst_points)
    warped_img = cv2.warpPerspective(filtered_img, matrix, (x,y))
    grayed_img = cv2.cvtColor(warped_img, cv2.COLOR_BGR2GRAY)
    bin_img = np.zeros_like(grayed_img)
    bin_img[grayed_img>200] = 255
    histogram_x = np.sum(bin_img, axis=0)
    histogram_y = np.sum(bin_img, axis=1)
    
    cross_indices = np.where(histogram_y>50000)[0]


    try:
        cross_threshold = 25
        cross_diff = cross_indices[-1] - cross_indices[0]

        if cross_threshold < cross_diff:
    # Hough Transform으로 직선 검출
            canny_img = cv2.Canny(bin_img, 50, 150)
            lines = cv2.HoughLinesP(canny_img, 1, np.pi / 180, threshold=50, minLineLength=50, maxLineGap=10)

            is_horizontal_line_detected = False
            min_horizontal_length = 100  # 가로 길이 제한

            if lines is not None:
                for line in lines:
                    x1, y1, x2, y2 = line[0]
                    dx = x2 - x1
                    dy = y2 - y1
                    if dx == 0:
                        continue  # 수직선은 제외
                    slope = dy / dx
                    line_length = np.sqrt(dx**2 + dy**2)
                    
                    if abs(slope) < 0.0001 and line_length >= min_horizontal_length:  # 거의 수평 (기울기 0도 근처)
                        is_horizontal_line_detected = True
                        break

            if is_horizontal_line_detected:
                cross_flag = True
                cv2.rectangle(warped_img, (0, cross_indices[0]), (x, cross_indices[-1]), (0, 255, 0), 3)
            else:
                cross_flag = False
        else:
            cross_flag = False

    except:
        cross_flag = False
        
    cv2.imshow("warped_img", warped_img)
    cv2.waitKey(1)
        
    return cross_flag


    # # imshow
    # cv2.imshow("img", img)
    # cv2.imshow("warped_img", warped_img)
    # cv2.imshow("bin_img", bin_img)
    # cv2.imshow("canny_img", canny_img)
    # cv2.imshow("filtered_img", filtered_img)
    # cv2.waitKey(1)
        
# def main():
#     try:
#         lane_sub = Lane_sub()
#         rospy.spin()
#     except rospy.ROSInterruptException:
#         pass
    
# if __name__ == '__main__':
#    main()