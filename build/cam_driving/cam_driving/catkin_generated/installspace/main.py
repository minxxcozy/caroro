#!/usr/bin/env python3
# -*- coding: utf-8 -*-

import subprocess
import rospy
from std_msgs.msg import Int32


class CamDriving:
    def __init__(self):
        rospy.init_node("main_controller_node")
        rospy.Subscriber("/stop_line_count", Int32, self.line_count_callback)

        self.current_mode = None
        self.process = None

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

    def line_count_callback(self, msg):
        mode = msg.data

        if mode in [0, 4]:
            self.launch_script("yellow_curve", "yellow_curve.py", "노란색 1차선 추적 모드 시작")
        elif mode == 1:
            self.launch_script("white_second", "white_second.py", "흰색 2차선 추적 모드 시작")
        elif mode == 2:
            self.launch_script("white_to_yellow", "white_to_yellow.py", "흰색 → 노란색 전환 차선 추적 모드 시작")
        elif mode == 3:
            self.launch_script("yellow_curve", "yellow_curve.py", "점선 좌회전 모드 시작")
        else:
            rospy.logwarn("NONONONONONONONONO")



if __name__ == '__main__':
    cam_driving = CamDriving()
    rospy.spin()
