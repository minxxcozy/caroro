#!/usr/bin/env python3
# -*- coding: utf-8 -*-

import subprocess
import rospy
from std_msgs.msg import Int32, Float64

class CamDriving:
    def __init__(self):
        rospy.init_node("main_controller_node")
        rospy.Subscriber("/obstacle_run_cmd", Float64, self.obstacle_callback)
        rospy.Subscriber("/line_cmd", Int32, self.line_cmd_callback)
        rospy.Subscriber("/stop_line_count", Int32, self.line_count_callback)
        rospy.Subscriber("/intersection_yellow", Int32, self.intersection_callback)


        self.current_mode = None
        self.process = None
        self.obs = None
        self.line = 2
        self.inter = 0

    def intersection_callback(self, msg) : 
        self.inter = msg.data

    def launch_script(self, mode_name, script_name, log_msg):
        if self.current_mode == mode_name:
            return  # 이미 실행 중

        # 이전 프로세스 종료
        if self.process and self.process.poll() is None:
            self.process.terminate()
            rospy.loginfo("이전 모드 프로세스 종료")

        rospy.loginfo(log_msg)
        self.process = subprocess.Popen(["rosrun", "cam_driving", script_name])
        self.current_mode = mode_name

    def line_cmd_callback(self, msg):
        self.line = msg.data


    def obstacle_callback(self, msg):
        self.obs = msg.data

    def line_count_callback(self, msg):
        mode = msg.data  # 먼저 선언

        if self.obs is None:
            rospy.logwarn("장애물 상태 수신 전 대기")
            return

        if self.inter is None:
            return
        else :
            inter = self.inter

        # 정상 모드 분기
        if inter == 0:
            #obstacle
            if self.obs == 1.0:
                self.launch_script("stop", "stop.py", "동적 장애물 정지 모드 실행")
            else : 
                if self.line == 1:
                    self.launch_script("1_line", "1_line.py", "1차선 실행")
                else :
                    self.launch_script("2_line", "2_line.py", "2차선 실행")

        elif inter in range(8,11):
            # left_turn
            if self.line == 1:
                self.launch_script("left_turn_1", "left_turn_1.py", "1-1 실행")
            else :
                self.launch_script("left_turn_2", "left_turn_2.py", "2-2 실행")

        elif (mode == 0 or mode == 1) and (inter in range(11,13)):
            #rotary
            self.launch_script("traffic_right_turn", "traffic_right_turn.py", "traffic_right_turn 실행")

        elif inter in range(13,15):
            #Traffic
            self.launch_script("traffic_right_turn", "traffic_right_turn.py", "traffic_turn 실행")

        elif inter == 14 or inter == 15 or inter == 16:
            #Traffic Light
            self.launch_script("traffic_right_turn", "traffic_right_turn.py", "right_turn 실행")

        elif inter in range(17,19):
            self.launch_script("right_turn_after", "right_turn_after.py", "right_turn_after 실행")
            
        elif inter == 19 :
            #finish
            self.launch_script("right_turn_after", "right_turn_after.py", "종료")
        else : 
            rospy.logwarn(f"알 수 없는 모드: {mode}, : {inter}")
            self.launch_script("2_line", "2_line.py", "2차선 실행")
 

if __name__ == '__main__':
    cam_driving = CamDriving()
    rospy.spin()