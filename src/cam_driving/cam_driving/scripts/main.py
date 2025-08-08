#!/usr/bin/env python3
# -*- coding: utf-8 -*-

import subprocess
import rospy
from std_msgs.msg import Int32, Float64

class CamDriving:
    def __init__(self):
        rospy.init_node("main_controller_node")
        rospy.Subscriber("/obstacle_run_cmd", Float64, self.obstable_callback)
        rospy.Subscriber("/stop_line_count", Int32, self.line_count_callback)

        self.current_mode = None
        self.process = None
        self.obs = None  # 초기값

    def launch_script(self, mode_name, script_name, log_msg):
        if self.current_mode == mode_name:
            return  # 같은 모드면 아무 것도 안 함

        # 기존 프로세스 종료
        if self.process and self.process.poll() is None:
            self.process.terminate()
            rospy.loginfo("기존 모드 종료됨")

        rospy.loginfo(log_msg)
        self.process = subprocess.Popen(["rosrun", "cam_driving", script_name])
        self.current_mode = mode_name

    def obstable_callback(self, msg):
        self.obs = msg.data

    def line_count_callback(self, msg):
        mode = msg.data

        # if self.obs is None:
        #     rospy.logwarn("장애물 상태(obs)가 아직 수신되지 않음")
        #     return

        if mode in [0, 2, 3, 4, 5]:
            if self.obs == 1.0:
                rospy.loginfo("동적 장애물 정지 모드")
                self.launch_script("stop", "stop.py", "정지 모드 시작")

            elif self.obs == 2.0:
                rospy.loginfo("정적 장애물 회피 모드")
                self.launch_script("yellow_to_white", "yellow_to_white.py", "2 모드 시작")

            else:
                # self.launch_script("white_to_yellow", "white_to_yellow.py", "1 모드 시작")
                self.launch_script("yellow_to_white", "yellow_to_white.py", "2 모드 시작")
                

                
        elif mode in [1]:
            self.launch_script("yellow_curve", "yellow_curve.py", "점선 좌회전 모드 시작")
        else:
            rospy.logwarn("유효하지 않은 모드: %s", mode)

if __name__ == '__main__':
    cam_driving = CamDriving()
    rospy.spin()
