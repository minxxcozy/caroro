# #!/usr/bin/env python3
# # -*- coding:utf-8 -*-

# import sys
# import os
# sys.path.insert(0, os.path.dirname(os.path.abspath(__file__)))

# import rospy
# from sensor_msgs.msg import CompressedImage
# from std_msgs.msg import Int32, Float64, Bool
# from cv_bridge import CvBridge
# import cv2
# from stopline_detect import stopline_detect


# class MissionController:
#     # 미션 종류
#     MISSION_OBSTACLE = 0         # 정적/동적 장애물 미션
#     MISSION_ROTARY = 1           # 로터리 미션
#     MISSION_TRAFFIC_LIGHT = 2    # 신호등 미션
    
#     # 장애물 회피
#     OBSTACLE_MIDDLE_TIME = 8
#     OBSTACLE_MANEUVER_TIME = 14  # 장애물 회피를 위한 조향 유지 시간 
#     OBSTACLE_STOP_TIME = 5       # 장애물 앞에서 일시 정지 시간 

#     # 로터리 미션 상태 
#     ROTARY_PHASE_WAIT = 0
#     ROTARY_PHASE_STOPPING = 1
#     ROTARY_PHASE_TURNING = 2
#     ROTARY_PHASE_NORMAL_DRIVING = 3
    
#     # 로터리 속도
#     ROTARY_ENTRY_SPEED = 800
    

#     def __init__(self):
#         rospy.init_node('caroro_mission_controller')
#         rospy.loginfo("미션 통합 노드 실행")
        
#         self.bridge = CvBridge()
        
#         # sub
#         self.image_sub = rospy.Subscriber('/image_jpeg/compressed', CompressedImage, self.image_CB)
#         rospy.Subscriber('/lane_speed_cmd', Float64, self.main_speed_callback)
#         rospy.Subscriber('/lane_steer_cmd', Float64, self.main_steer_callback)
#         rospy.Subscriber('/traffic_status', Bool, self.traffic_callback)
#         rospy.Subscriber("/obstacle_run_cmd", Float64, self.obstacle_callback)
#         rospy.Subscriber("/rotary_state", Bool, self.rotary_state_callback)
#         rospy.Subscriber("/line_cmd", Int32, self.rotary_lane_callback)
#         rospy.Subscriber("/arrival_flag", Bool, self.arrival_callback)
        
#         # SLAM
#         rospy.Subscriber("/slam/speed", Float64, self.slam_speed_callback)
#         rospy.Subscriber("/slam/position", Float64, self.slam_position_callback)
#         rospy.Subscriber('/lane_steer_cmd', Float64, self.lane_follow_steer_callback)   # 2_line.py
#         rospy.Subscriber("/rotary_stop_flag", Bool, self.rotary_stop_callback)          # pose stop flag 
#         rospy.Subscriber("/rotary_flag_num", Int32, self.rotary_flag_num_callback)      # pose num
#         rospy.Subscriber("/intersection_yellow", Int32, self.intersection_callback)
#         rospy.Subscriber("/curve_flag", Bool, self.corner_callback)
        
        
#         # pub
#         self.stop_count_pub = rospy.Publisher('/stop_line_count', Int32, queue_size=10)  
#         self.motor_speed_pub = rospy.Publisher('/commands/motor/speed', Float64, queue_size=10)
#         self.servo_pos_pub = rospy.Publisher('/commands/servo/position', Float64, queue_size=10)
#         self.line_pub = rospy.Publisher('/line_cmd', Int32, queue_size=1)
        
#         # 미션 상태 변수 초기화
#         self.stop_line_count = 0 
#         self.current_mission = self.MISSION_OBSTACLE
#         self.prev_stop_detected = False
#         self.last_detect_time = rospy.Time.now()
#         self.stop_detect_delay = rospy.Duration(2)

#         # 기본 차선 주행
#         self.current_speed = 0.0
#         self.current_steer = 0.5
        
#         # SLAM 명령 값들
#         self.slam_speed = 0.0
#         self.slam_position = 0.5
        
#         # 각 미션별 상태 변수
#         self.traffic_status = False
#         self.rotary_state = True
#         self.arrival_flag = False  
        
#         # 장애물 미션 상태
#         self.obstacle_run_cmd = None
#         self.obstacle_cnt = 0
#         self.obstacle_line = 2
        
#         # 로터리 미션 상태
#         self.rotary_phase = self.ROTARY_PHASE_WAIT
#         self.rotary_phase_start_time = None
#         self.rotary_stop_time = rospy.Duration(2.0)
#         self.rotary_lane = None     # 차선
#         self.rotary_stop = False

#         # 로터리 flag
#         self.rotary_flag_num = 0
#         self.intersection_num = 0
        
#         self.corner_flag = False


#     def main_speed_callback(self, msg):
#         self.current_speed = msg.data

#     def main_steer_callback(self, msg):
#         self.current_steer = msg.data
        
#     def lane_follow_steer_callback(self, msg):
#         self.lane_follow_steer = msg.data

#     def obstacle_callback(self, msg):
#         self.obstacle_run_cmd = msg.data

#     def traffic_callback(self, msg):
#         self.traffic_status = msg.data

#     def rotary_state_callback(self, msg):
#         self.rotary_state = msg.data 

#     def arrival_callback(self, msg):
#         self.arrival_flag = msg.data

#     def slam_speed_callback(self, msg):
#         self.slam_speed = msg.data

#     def slam_position_callback(self, msg):
#         self.slam_position = msg.data
        
#     def rotary_lane_callback(self, msg):
#         self.rotary_lane = msg.data
    
#     def intersection_callback(self, msg):
#         self.intersection_num = msg.data
    
#     def rotary_flag_num_callback(self, msg):
#         self.rotary_flag_num = msg.data
#         rospy.loginfo(f"[Rotary] FLAG NUM 수신: {self.rotary_flag_num}")
    
#     def rotary_stop_callback(self, msg): 
#         self.rotary_stop = msg.data

#     def corner_callback(self, msg):
#         self.corner_flag = msg.data

#     # 정지선 감지 및 미션 전환 결정
#     def image_CB(self, msg):
#         try:
#             frame = self.bridge.compressed_imgmsg_to_cv2(msg, "bgr8")
#             now = rospy.Time.now()

#             if not self.arrival_flag:
#                 self.prev_stop_detected = False
#                 return
            
#             elif self.rotary_phase == self.ROTARY_PHASE_NORMAL_DRIVING:
#                 stop_detected = stopline_detect(frame)
#                 print("신호등 인식")
#                 if stop_detected and not self.prev_stop_detected and (now - self.last_detect_time > self.stop_detect_delay):
#                     self.stop_line_count += 1
#                     self.last_detect_time = now
#                     rospy.loginfo(f"정지선 감지 누적 횟수 : {self.stop_line_count}")
#                 self.prev_stop_detected = stop_detected

#             else:
#                 stop_detected = False
#                 self.prev_stop_detected = False
                    
#             if self.stop_line_count == 0 : self.pre_state = self.traffic_status

#             self.update_mission()
#             self.stop_count_pub.publish(self.stop_line_count)

#         except Exception as e:
#             rospy.logerr(f"이미지 처리 중 오류 발생: {e}")


#     # 미션 전환
#     def update_mission(self):
#         prev_mission = self.current_mission

#         if self.stop_line_count == 1:
#             self.current_mission = self.MISSION_TRAFFIC_LIGHT
        
#         elif self.intersection_num > 10 and self.intersection_num < 13:
#             if self.rotary_phase == self.ROTARY_PHASE_WAIT:
#                 rospy.loginfo("로터리 미션 시작")
#                 self.rotary_phase = self.ROTARY_PHASE_STOPPING
#                 self.rotary_phase_start_time = None
#             self.current_mission = self.MISSION_ROTARY

#         elif self.rotary_stop == False:
#             self.current_mission = self.MISSION_OBSTACLE

#         if prev_mission != self.current_mission:
#             rospy.loginfo(f"미션 전환 : {prev_mission} -> {self.current_mission}")


#     # 장애물 회피
#     def run_obstacle_mission(self):
#         speed, steer = self.current_speed, self.current_steer
        
#         if self.intersection_num != 0:
#             self.line_pub.publish(self.obstacle_line)
#             return speed, steer
        
#         # 정적 장애물 감지 명령이 들어오면 회피 동작 시작
#         if self.obstacle_run_cmd == 2.0 and self.obstacle_cnt == 0:
#             self.obstacle_cnt = 1
            
#             if self.obstacle_line == 2: self.obstacle_line = 1
#             else: self.obstacle_line = 2
     
#             self.obstacle_run_cmd = 3.0       
        
#         if self.corner_flag :
#             # corner
#             if 0 < self.obstacle_cnt <= self.OBSTACLE_MANEUVER_TIME:
#                 self.obstacle_cnt += 1

#                 if 0 < self.obstacle_cnt <= self.OBSTACLE_MIDDLE_TIME:                
#                     if self.obstacle_line == 1:
#                         steer_offset = -0.4
#                         self.obstacle_line = 1
#                     else:
#                         steer_offset = 0.4
#                         self.obstacle_line = 2

#                 else:
#                     if self.obstacle_line == 1:
#                         steer_offset = self.current_steer + 0.5
#                         self.obstacle_line = 1
#                     else:
#                         steer_offset = self.current_steer + 0.5
#                         self.obstacle_line = 2
#         else :
#             # straight 
#             if 0 < self.obstacle_cnt <= self.OBSTACLE_MANEUVER_TIME:
#                 self.obstacle_cnt += 1

#                 if 0 < self.obstacle_cnt <= self.OBSTACLE_MIDDLE_TIME:                
#                     if self.obstacle_line == 1:
#                         steer_offset = -0.4
#                         self.obstacle_line = 1
#                     else:
#                         steer_offset = 0.4
#                         self.obstacle_line = 2

#                 else:
#                     if self.obstacle_line == 1:
#                         steer_offset = 0.42
#                         self.obstacle_line = 1
#                     else:
#                         steer_offset = -0.4
#                         self.obstacle_line = 2

#             steer = 0.5 + steer_offset
#             speed = 2000
            
#             if self.obstacle_cnt == self.OBSTACLE_MANEUVER_TIME:
#                 self.obstacle_cnt = 0
            
#         self.line_pub.publish(self.obstacle_line)
#         return speed, steer
    
    
#     # 로터리 미션
#     def run_rotary_mission(self):
#         speed, steer = self.current_speed, self.current_steer

#         # 1. 진입 전 정지 (rotary_stop 플래그 기준)
#         if self.rotary_phase == self.ROTARY_PHASE_STOPPING:
#             now = rospy.Time.now()
#             if self.rotary_phase_start_time is None:
#                 self.rotary_phase_start_time = now  # 정지 시작 시각 기록
#                 rospy.loginfo("[Rotary] 정지 시작")

#             elapsed = (now - self.rotary_phase_start_time).to_sec()

#             if elapsed < self.rotary_stop_time.to_sec():
#                 # flag가 True이면 2초 정지 유지
#                 speed = 0.0
#                 steer = self.current_steer
#             else:
#                 # 정지 완료 후 회전 단계로 전환
#                 rospy.loginfo("[Rotary] 정지 완료, 회전 단계로 전환")
#                 self.rotary_phase = self.ROTARY_PHASE_TURNING
#                 self.rotary_phase_start_time = None
#                 self.rotary_stop = False  # 정지 flag 해제

#         # 2. 꺾기 유지 (flag 기반, rotary_state 확인)
#         elif self.rotary_phase == self.ROTARY_PHASE_TURNING:
#             speed = self.ROTARY_ENTRY_SPEED

#             if not self.rotary_state:
#                 speed = 0.0
#                 # rospy.loginfo("[Rotary] 장애물!")
#             else:
#                 # 차선별 + flag별 steer 값 정의
#                 steer_table = {
#                     1: {  # 1차선
#                         0: 0.90,    # 기본
#                         1: 0.85,
#                         2: 0.60,
#                         3: 1.0
#                     },
#                     2: {  # 2차선
#                         0: 0.88,    # 기본 : 2차선은 0, 1 값을 동일하게 설정할 것!
#                         1: 0.88,
#                         2: 0.50,
#                         3: 1.0
#                     }
#                 }

#                 # 현재 차선과 flag에 맞는 steer 값 적용
#                 lane = self.rotary_lane if self.rotary_lane in steer_table else 1
#                 flag_num = self.rotary_flag_num if self.rotary_flag_num in steer_table[lane] else 0
#                 steer = steer_table[lane][flag_num]

#                 # flag_num 3이면 정상 주행으로 전환
#                 if flag_num == 3:
#                     rospy.loginfo("[Rotary] 세 번째 FLAG 적용, 정상 주행 전환")
#                     self.rotary_phase = self.ROTARY_PHASE_NORMAL_DRIVING

#         # 3. 정상 주행
#         elif self.rotary_phase == self.ROTARY_PHASE_NORMAL_DRIVING:
#             speed = self.current_speed
#             steer = self.lane_follow_steer

#         return speed, steer
    

#     # 신호등 미션
#     def run_traffic_light_mission(self):
#         if (self.pre_state == False and self.traffic_status == False):
#             speed = 0.0
#         else :
#             speed = self.current_speed 

#         steer = self.current_steer
#         return speed, steer


#     def main(self):
#         rate = rospy.Rate(20)
        
#         while not rospy.is_shutdown():
#             # rospy.loginfo_throttle(1.0, f"[DEBUG] arrival_flag 상태: {self.arrival_flag}")
            
#             if self.arrival_flag:
#                 speed, steer = self.current_speed, self.current_steer

#                 if self.current_mission == self.MISSION_OBSTACLE:           # 정/동적 장애물 미션
#                     speed, steer = self.run_obstacle_mission()
#                 elif self.current_mission == self.MISSION_ROTARY:           # 로터리 미션
#                     speed, steer = self.run_rotary_mission()
#                 elif self.current_mission == self.MISSION_TRAFFIC_LIGHT:    # 신호등 미션
#                     speed, steer = self.run_traffic_light_mission()

#             else:
#                 speed, steer = self.slam_speed, self.slam_position
#                 # rospy.loginfo_throttle(2.0, f"SLAM 모드 - 속도: {speed}, 조향: {steer}")
            
#             self.motor_speed_pub.publish(speed)
#             self.servo_pos_pub.publish(steer)
#             rate.sleep()
            
# def main():
#     try:
#         controller = MissionController()
#         controller.main()
#     except rospy.ROSInterruptException:
#         pass

# if __name__ == '__main__':
#     main()



#!/usr/bin/env python3
# -*- coding:utf-8 -*-

import sys
import os
sys.path.insert(0, os.path.dirname(os.path.abspath(__file__)))

import rospy
from sensor_msgs.msg import CompressedImage
from std_msgs.msg import Int32, Float64, Bool
from cv_bridge import CvBridge
import cv2
from stopline_detect import stopline_detect


class MissionController:
    # 미션 종류
    MISSION_OBSTACLE = 0         # 정적/동적 장애물 미션
    MISSION_ROTARY = 1           # 로터리 미션
    MISSION_TRAFFIC_LIGHT = 2    # 신호등 미션
    
    # 장애물 회피
    OBSTACLE_MIDDLE_TIME = 8
    OBSTACLE_MANEUVER_TIME = 14  # 장애물 회피를 위한 조향 유지 시간 
    OBSTACLE_STOP_TIME = 5       # 장애물 앞에서 일시 정지 시간 

    # 로터리 미션 상태 
    ROTARY_PHASE_WAIT = 0
    ROTARY_PHASE_STOPPING = 1
    ROTARY_PHASE_TURNING = 2
    ROTARY_PHASE_NORMAL_DRIVING = 3
    
    # 로터리 속도
    ROTARY_ENTRY_SPEED = 800
    

    def __init__(self):
        rospy.init_node('caroro_mission_controller')
        rospy.loginfo("미션 통합 노드 실행")
        
        self.bridge = CvBridge()
        
        # sub
        self.image_sub = rospy.Subscriber('/image_jpeg/compressed', CompressedImage, self.image_CB)
        rospy.Subscriber('/lane_speed_cmd', Float64, self.main_speed_callback)
        rospy.Subscriber('/lane_steer_cmd', Float64, self.main_steer_callback)
        rospy.Subscriber('/traffic_status', Bool, self.traffic_callback)
        rospy.Subscriber("/obstacle_run_cmd", Float64, self.obstacle_callback)
        rospy.Subscriber("/rotary_state", Bool, self.rotary_state_callback)
        rospy.Subscriber("/line_cmd", Int32, self.rotary_lane_callback)
        rospy.Subscriber("/arrival_flag", Bool, self.arrival_callback)
        
        # SLAM
        rospy.Subscriber("/slam/speed", Float64, self.slam_speed_callback)
        rospy.Subscriber("/slam/position", Float64, self.slam_position_callback)
        rospy.Subscriber('/lane_steer_cmd', Float64, self.lane_follow_steer_callback)   # 2_line.py
        rospy.Subscriber("/rotary_stop_flag", Bool, self.rotary_stop_callback)          # pose stop flag 
        rospy.Subscriber("/rotary_flag_num", Int32, self.rotary_flag_num_callback)      # pose num
        rospy.Subscriber("/intersection_yellow", Int32, self.intersection_callback)
        rospy.Subscriber("/curve_flag", Bool, self.corner_callback)
        
        
        # pub
        self.stop_count_pub = rospy.Publisher('/stop_line_count', Int32, queue_size=10)  
        self.motor_speed_pub = rospy.Publisher('/commands/motor/speed', Float64, queue_size=10)
        self.servo_pos_pub = rospy.Publisher('/commands/servo/position', Float64, queue_size=10)
        self.line_pub = rospy.Publisher('/line_cmd', Int32, queue_size=1)
        
        # 미션 상태 변수 초기화
        self.stop_line_count = 0 
        self.current_mission = self.MISSION_OBSTACLE
        self.prev_stop_detected = False
        self.last_detect_time = rospy.Time.now()
        self.stop_detect_delay = rospy.Duration(2)

        # 기본 차선 주행
        self.current_speed = 0.0
        self.current_steer = 0.5
        
        # SLAM 명령 값들
        self.slam_speed = 0.0
        self.slam_position = 0.5
        
        # 각 미션별 상태 변수
        self.traffic_status = False
        self.rotary_state = True
        self.arrival_flag = False  
        
        # 장애물 미션 상태
        self.obstacle_run_cmd = None
        self.obstacle_cnt = 0
        self.obstacle_line = 2
        
        # 로터리 미션 상태
        self.rotary_phase = self.ROTARY_PHASE_WAIT
        self.rotary_phase_start_time = None
        self.rotary_stop_time = rospy.Duration(2.0)
        self.rotary_lane = None     # 차선
        self.rotary_stop = False

        # 로터리 flag
        self.rotary_flag_num = 0
        self.intersection_num = 0
        
        self.corner_flag = False


    def main_speed_callback(self, msg):
        self.current_speed = msg.data

    def main_steer_callback(self, msg):
        self.current_steer = msg.data
        
    def lane_follow_steer_callback(self, msg):
        self.lane_follow_steer = msg.data

    def obstacle_callback(self, msg):
        self.obstacle_run_cmd = msg.data

    def traffic_callback(self, msg):
        self.traffic_status = msg.data

    def rotary_state_callback(self, msg):
        self.rotary_state = msg.data 

    def arrival_callback(self, msg):
        self.arrival_flag = msg.data

    def slam_speed_callback(self, msg):
        self.slam_speed = msg.data

    def slam_position_callback(self, msg):
        self.slam_position = msg.data
        
    def rotary_lane_callback(self, msg):
        self.rotary_lane = msg.data
    
    def intersection_callback(self, msg):
        self.intersection_num = msg.data
    
    def rotary_flag_num_callback(self, msg):
        self.rotary_flag_num = msg.data
        rospy.loginfo(f"[Rotary] FLAG NUM 수신: {self.rotary_flag_num}")
    
    def rotary_stop_callback(self, msg): 
        self.rotary_stop = msg.data

    def corner_callback(self, msg):
        self.corner_flag = msg.data

    # 정지선 감지 및 미션 전환 결정
    def image_CB(self, msg):
        try:
            frame = self.bridge.compressed_imgmsg_to_cv2(msg, "bgr8")
            now = rospy.Time.now()

            if not self.arrival_flag:
                self.prev_stop_detected = False
                return
            
            elif self.rotary_phase == self.ROTARY_PHASE_NORMAL_DRIVING:
                stop_detected = stopline_detect(frame)
                print("신호등 인식")
                if stop_detected and not self.prev_stop_detected and (now - self.last_detect_time > self.stop_detect_delay):
                    self.stop_line_count += 1
                    self.last_detect_time = now
                    rospy.loginfo(f"정지선 감지 누적 횟수 : {self.stop_line_count}")
                self.prev_stop_detected = stop_detected

            else:
                stop_detected = False
                self.prev_stop_detected = False
                    
            if self.stop_line_count == 0 : self.pre_state = self.traffic_status

            self.update_mission()
            self.stop_count_pub.publish(self.stop_line_count)

        except Exception as e:
            rospy.logerr(f"이미지 처리 중 오류 발생: {e}")


    # 미션 전환
    def update_mission(self):
        prev_mission = self.current_mission

        if self.stop_line_count == 1:
            self.current_mission = self.MISSION_TRAFFIC_LIGHT
        
        elif self.intersection_num > 10 and self.intersection_num < 13:
            if self.rotary_phase == self.ROTARY_PHASE_WAIT:
                rospy.loginfo("로터리 미션 시작")
                self.rotary_phase = self.ROTARY_PHASE_STOPPING
                self.rotary_phase_start_time = None
            self.current_mission = self.MISSION_ROTARY

        elif self.rotary_stop == False:
            self.current_mission = self.MISSION_OBSTACLE

        if prev_mission != self.current_mission:
            rospy.loginfo(f"미션 전환 : {prev_mission} -> {self.current_mission}")


    # 장애물 회피
    def run_obstacle_mission(self):
        speed, steer = self.current_speed, self.current_steer
        steer_offset = 0
        
        if self.intersection_num != 0:
            self.line_pub.publish(self.obstacle_line)
            return speed, steer
        
        # 정적 장애물 감지 명령이 들어오면 회피 동작 시작
        if self.obstacle_run_cmd == 2.0 and self.obstacle_cnt == 0:
            self.obstacle_cnt = 1
            
            if self.obstacle_line == 2: self.obstacle_line = 1
            else: self.obstacle_line = 2
     
            self.obstacle_run_cmd = 3.0       
        
        if self.corner_flag :
            # corner
            if 0 < self.obstacle_cnt <= self.OBSTACLE_MANEUVER_TIME:
                self.obstacle_cnt += 1

                if 0 < self.obstacle_cnt <= self.OBSTACLE_MIDDLE_TIME:                
                    if self.obstacle_line == 1:
                        steer_offset = -0.3
                        self.obstacle_line = 1
                    else:
                        steer_offset = 0.3
                        self.obstacle_line = 2

                else:
                    if self.obstacle_line == 1:
                        steer_offset = self.current_steer + 0.5
                        self.obstacle_line = 1
                    else:
                        steer_offset = self.current_steer + 0.5
                        self.obstacle_line = 2

                steer = 0.5 + steer_offset
                speed = 2000
        else :
            # straight 
            if 0 < self.obstacle_cnt <= self.OBSTACLE_MANEUVER_TIME:
                self.obstacle_cnt += 1

                if 0 < self.obstacle_cnt <= self.OBSTACLE_MIDDLE_TIME:                
                    if self.obstacle_line == 1:
                        steer_offset = -0.41
                        self.obstacle_line = 1
                    else:
                        steer_offset = 0.4
                        self.obstacle_line = 2

                else:
                    if self.obstacle_line == 1:
                        steer_offset = 0.42
                        self.obstacle_line = 1
                    else:
                        steer_offset = -0.4
                        self.obstacle_line = 2
                
                steer = 0.5 + steer_offset
                speed = 2000
            
            if self.obstacle_cnt == self.OBSTACLE_MANEUVER_TIME:
                self.obstacle_cnt = 0
            
        self.line_pub.publish(self.obstacle_line)
        return speed, steer
    
    
    # 로터리 미션
    def run_rotary_mission(self):
        speed, steer = self.current_speed, self.current_steer
        
        '''
        # 1. 진입 전 정지 (rotary_stop 플래그 기준)
        if self.rotary_phase == self.ROTARY_PHASE_STOPPING:
            now = rospy.Time.now()
            if self.rotary_phase_start_time is None:
                self.rotary_phase_start_time = now  # 정지 시작 시각 기록
                rospy.loginfo("[Rotary] 정지 시작")

            elapsed = (now - self.rotary_phase_start_time).to_sec()

            if elapsed < self.rotary_stop_time.to_sec():
                # flag가 True이면 2초 정지 유지
                speed = 0.0
                steer = self.current_steer
            else:
                # 정지 완료 후 회전 단계로 전환
                rospy.loginfo("[Rotary] 정지 완료, 회전 단계로 전환")
                self.rotary_phase = self.ROTARY_PHASE_TURNING
                self.rotary_phase_start_time = None
                self.rotary_stop = False  # 정지 flag 해제
        '''

        # 1. 진입 전 정지 제거, rotary_stop 플래그 읽으면 바로 회전 단계
        if self.rotary_phase == self.ROTARY_PHASE_STOPPING:
            rospy.loginfo("[Rotary] 정지 단계 건너뛰고 회전 단계로 전환")
            self.rotary_phase = self.ROTARY_PHASE_TURNING
            self.rotary_phase_start_time = None
            self.rotary_stop = False  # 정지 flag 해제

        # 2. 꺾기 유지 (flag 기반, rotary_state 확인)
        elif self.rotary_phase == self.ROTARY_PHASE_TURNING:
            speed = self.ROTARY_ENTRY_SPEED

            if not self.rotary_state:
                speed = 0.0
            else:
                # 차선별 + flag별 steer 값 정의
                steer_table = {
                    1: {    0: 0.90, 1: 0.85, 2: 0.60, 3: 1.0   },
                    2: {    0: 0.88, 1: 0.88, 2: 0.50, 3: 1.0   }
                }

                lane = self.rotary_lane if self.rotary_lane in steer_table else 1
                flag_num = self.rotary_flag_num if self.rotary_flag_num in steer_table[lane] else 0
                steer = steer_table[lane][flag_num]

                if flag_num == 3:
                    rospy.loginfo("[Rotary] 세 번째 FLAG 적용, 정상 주행 전환")
                    self.rotary_phase = self.ROTARY_PHASE_NORMAL_DRIVING
                    
        # 3. 정상 주행
        elif self.rotary_phase == self.ROTARY_PHASE_NORMAL_DRIVING:
            speed = self.current_speed
            steer = self.lane_follow_steer

        return speed, steer
    

    # 신호등 미션
    def run_traffic_light_mission(self):
        if (self.pre_state == False and self.traffic_status == False):
            speed = 0.0
        else :
            speed = self.current_speed 

        steer = self.current_steer
        return speed, steer


    def main(self):
        rate = rospy.Rate(20)
        
        while not rospy.is_shutdown():
            # rospy.loginfo_throttle(1.0, f"[DEBUG] arrival_flag 상태: {self.arrival_flag}")
            
            if self.arrival_flag:
                speed, steer = self.current_speed, self.current_steer

                if self.current_mission == self.MISSION_OBSTACLE:           # 정/동적 장애물 미션
                    speed, steer = self.run_obstacle_mission()
                elif self.current_mission == self.MISSION_ROTARY:           # 로터리 미션
                    speed, steer = self.run_rotary_mission()
                elif self.current_mission == self.MISSION_TRAFFIC_LIGHT:    # 신호등 미션
                    speed, steer = self.run_traffic_light_mission()

            else:
                speed, steer = self.slam_speed, self.slam_position
                # rospy.loginfo_throttle(2.0, f"SLAM 모드 - 속도: {speed}, 조향: {steer}")
            
            self.motor_speed_pub.publish(speed)
            self.servo_pos_pub.publish(steer)
            rate.sleep()
            
def main():
    try:
        controller = MissionController()
        controller.main()
    except rospy.ROSInterruptException:
        pass

if __name__ == '__main__':
    main()