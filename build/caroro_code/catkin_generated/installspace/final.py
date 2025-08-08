#!/usr/bin/env python3

import sys
import os
sys.path.append(os.path.dirname(os.path.abspath(__file__)))

import rospy
from sensor_msgs.msg import Image
from std_msgs.msg import Int32
from cv_bridge import CvBridge
import cv2
from std_msgs.msg import Float64

from stopline_detect import stopline_detect
from sensor_msgs.msg import CompressedImage
# from obstacle_avoid import obstacle_avoid
import rotary
import traffic_light
from morai_msgs.msg import GetTrafficLightStatus

class MissionController:
    def __init__(self):
        rospy.init_node('caroro_mission_controller')
        rospy.Subscriber('/image_jpeg/compressed', CompressedImage, self.image_CB)
        rospy.loginfo("미션 통합 노드 실행")
        # rospy.Timer(rospy.Duration(0.1), self.publish_default_speed)
        traffic_light.init_traffic_light_node()   # 신호등 노드 초기화
        
        self.bridge = CvBridge()
        self.stop_line_count = 0   # 0: 장애물, 1: 로터리, 2: 신호등
        self.current_mission = 0
        self.prev_stop_detected = False
        self.printed = False
        self.last_detect_time = rospy.Time.now()
        self.stop_detect_delay = rospy.Duration(2)  # 1.5초 동안 무시
        # self.speed_pub = rospy.Publisher('/commands/motor/speed', Float64, queue_size=10)
        
        # main.py에 정지선 횟수 pub
        self.stop_count_pub = rospy.Publisher('/stop_line_count', Int32, queue_size=1)  
        
        # main.py에서 받은 속도, 조향 초기값
        self.current_speed = 0.0
        self.current_steer = 0.0
        
        # main.py에서 속도, 조향 sub
        self.cmd_speed_sub = rospy.Subscriber('/lane_speed_cmd', Float64, self.main_speed_callback)
        self.cmd_steer_sub = rospy.Subscriber('/lane_steer_cmd', Float64, self.main_steer_callback)

        # 최종 속도, 조향 pub
        self.motor_speed_pub = rospy.Publisher('/commands/motor/speed', Float64, queue_size=10)
        self.servo_pos_pub = rospy.Publisher('/commands/servo/position', Float64, queue_size=10)
        
        # 정동적 
        self.obstacle_avoid_pub = rospy.Publisher('obstacle_run_cmd', Float64, queue_size=10)
        
        self.is_stopped_temporarily = False  # 일시 정지 상태 여부
        self.rotary_stop_done = False
        self.traffic_status = False
        self.printed_traffic = False
        #self.default_speed = 3000
    
    def main_speed_callback(self, msg):
            self.current_speed = msg.data
            self.motor_speed_pub.publish(Float64(self.current_speed))

    def main_steer_callback(self, msg):
            self.current_steer = msg.data
            self.servo_pos_pub.publish(Float64(self.current_steer))

    def traffic_light(self, frame):
        
        if not self.printed:
            print("trffic light mission start!")
            self.printed = True
            
        if traffic_light.traffic_status:
            self.motor_speed_pub.publish(Float64(data=self.current_speed))
            print("출발~!~!")
        else:
            self.motor_speed_pub.publish(Float64(0.0))
            print("정지!")  
            
        
    def end_rotary_stop(self, event):
        rospy.loginfo("로터리 정지 종료, 속도 5.0으로 복귀")
        self.is_stopped_temporarily = False

     
    def image_CB(self, msg):
        frame = self.bridge.compressed_imgmsg_to_cv2(msg, "bgr8")
        
        # 정지선 인식 호출
        now = rospy.Time.now()
        stop_detected = stopline_detect(frame)
        
        if stop_detected and not self.prev_stop_detected and (now - self.last_detect_time > self.stop_detect_delay):
            self.stop_line_count += 1
            self.last_detect_time = now
            rospy.loginfo(f"정지선 누적 감지 수: {self.stop_line_count}")
        self.stop_count_pub.publish(Int32(self.stop_line_count))
        self.prev_stop_detected = stop_detected
        
        # 미션 선택 
        if self.stop_line_count >= 6:
            self.current_mission = 2  # 신호등
        elif self.stop_line_count == 5:
            if not self.rotary_stop_done:
                rospy.loginfo("로터리 진입 전 2초 정지")
                self.is_stopped_temporarily = True
                self.motor_speed_pub.publish(Float64(0.0))
                rospy.Timer(rospy.Duration(2.0), self.end_rotary_stop, oneshot=True)
                self.rotary_stop_done = True
            self.current_mission = 1  # 로터리
        else:
            self.current_mission = 0  # 정동적 장애물
            
        # 미션 수행
        if self.current_mission == 0: # 정동적
            self.obstacle_avoid_pub.publish(Float64(True))
            
        elif self.current_mission == 1: # 로터리
            rotary(frame)
            self.obstacle_avoid_pub.publish(Float64(False))
        elif self.current_mission == 2: # 신호등
            self.traffic_light(frame)
            self.obstacle_avoid_pub.publish(Float64(False))
        else:
            self.obstacle_avoid_pub.publish(Float64(False))
def main():
    controller = MissionController()
    rospy.spin()

if __name__ == '__main__':
    main()