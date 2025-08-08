# #!/usr/bin/env python3
# # -*- coding: utf-8 -*-

# import subprocess
# import rospy
# from std_msgs.msg import Int32, Float64

# class CamDriving:
#     def __init__(self):
#         rospy.init_node("main_controller_node")
#         rospy.Subscriber("/obstacle_run_cmd", Float64, self.obstable_callback)
#         rospy.Subscriber("/stop_line_count", Int32, self.line_count_callback)

#         self.current_mode = None
#         self.process = None
#         self.obs = None  # 초기값

#     def launch_script(self, mode_name, script_name, log_msg):
#         if self.current_mode == mode_name:
#             return  # 같은 모드면 아무 것도 안 함

#         # 기존 프로세스 종료
#         if self.process and self.process.poll() is None:
#             self.process.terminate()
#             rospy.loginfo("기존 모드 종료됨")

#         rospy.loginfo(log_msg)
#         self.process = subprocess.Popen(["rosrun", "cam_driving", script_name])
#         self.current_mode = mode_name

#     def obstable_callback(self, msg):
#         self.obs = msg.data
#         # print("self.obs : " ,self.obs )

#     def line_count_callback(self, msg):
#         mode = msg.data

#         # if self.obs is None:
#         #     rospy.logwarn("장애물 상태(obs)가 아직 수신되지 않음")
#         #     return
#         print("self.obs22 : " ,self.obs )

#         if mode == 0:
#             if self.obs == 1.0:
#                 rospy.loginfo("동적 장애물 정지 모드")
#                 self.launch_script("stop", "stop.py", "정지 모드 시작")

#             elif self.obs == 2.0:
#                 rospy.loginfo("정적 장애물 회피 모드")
#                 self.launch_script("obs_1_line", "obs_1_line.py", "1 모드 시작")

#             else:
#                 self.launch_script("2_line", "2_line.py", "2 모드 시작")
        
#         elif mode in [1,2,3,4]:

#             if self.obs == 1.0:
#                 rospy.loginfo("동적 장애물 정지 모드")
#                 self.launch_script("stop", "stop.py", "정지 모드 시작")

#             elif self.obs == 2.0:
#                 rospy.loginfo("정적 장애물 회피 모드")
#                 self.launch_script("obs_1_line", "obs_1_line.py", "1 모드 시작")

#             else:
#                 self.launch_script("obs_2_line", "obs_2_line.py", "2 모드 시작")
                

                
#         elif mode in [10]:
#             self.launch_script("left_turn", "left_turn.py", "점선 좌회전 모드 시작")
#         else:
#             rospy.logwarn("유효하지 않은 모드: %s", mode)

# if __name__ == '__main__':
#     cam_driving = CamDriving()
#     rospy.spin()


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

        # 최신 상태값 저장
        self.obs = None
        self.latest_mode = None

        # 주기적으로 main() 실행 (0.1초마다)
        rospy.Timer(rospy.Duration(0.1), self.main_loop)

    def launch_script(self, mode_name, script_name, log_msg):
        if self.current_mode == mode_name:
            return  # 같은 모드면 재실행 방지

        # 기존 프로세스 종료
        if self.process and self.process.poll() is None:
            self.process.terminate()
            rospy.loginfo("기존 모드 종료됨")

        rospy.loginfo(log_msg)
        self.process = subprocess.Popen(["rosrun", "cam_driving", script_name])
        self.current_mode = mode_name

    # 콜백들은 값만 저장
    def obstable_callback(self, msg: Float64):
        self.obs = msg.data
        rospy.loginfo(f"[obs] {self.obs}")

    def line_count_callback(self, msg: Int32):
        self.latest_mode = msg.data
        rospy.loginfo(f"[mode] {self.latest_mode}")

    # 주기적으로 호출되는 메인 로직
    def main_loop(self, event):
        # 둘 다 수신되기 전이면 대기
        if self.obs is None or self.latest_mode is None:
            return

        mode = self.latest_mode

        # 좌회전 우선 처리


        # 일반 분기
        if mode == 0:
            if self.obs == 1.0:
                rospy.loginfo("동적 장애물 정지 모드")
                self.launch_script("stop", "stop.py", "정지 모드 시작")
            elif self.obs == 2.0:
                rospy.loginfo("정적 장애물 회피 모드")
                self.launch_script("obs_1_line", "obs_1_line.py", "1 모드 시작")
            else:
                self.launch_script("2_line", "2_line.py", "2 모드 시작")

        elif mode in [2, 3, 4]:
            if self.obs == 1.0:
                rospy.loginfo("동적 장애물 정지 모드")
                self.launch_script("stop", "stop.py", "정지 모드 시작")
            elif self.obs == 2.0:
                rospy.loginfo("정적 장애물 회피 모드")
                self.launch_script("obs_1_line", "obs_1_line.py", "1 모드 시작")
            else:
                self.launch_script("obs_2_line", "obs_2_line.py", "2 모드 시작")

        else:
            rospy.logwarn(f"유효하지 않은 모드: {mode}")

if __name__ == '__main__':
    CamDriving()
    rospy.spin()