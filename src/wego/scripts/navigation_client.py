#! /usr/bin/env python3

import rospy
import math

from move_base_msgs.msg import MoveBaseAction, MoveBaseGoal
from actionlib_msgs.msg import GoalStatus
import actionlib
from std_msgs.msg import Bool
from geometry_msgs.msg import PoseWithCovarianceStamped

class NavigationClient:
    def __init__(self):
        self.client = actionlib.SimpleActionClient('move_base', MoveBaseAction)
        self.flag_pub = rospy.Publisher('/arrival_flag', Bool, queue_size=1)
        self.client.wait_for_server()
        self.arrival_flag = False
        self.goal_list = list()
        
        # 도착 판정을 위한 오차 범위 (미터 단위)
        self.arrival_tolerance = 0.5  # 1미터 이내에 들어오면 도착으로 판정
        
        # 로봇의 현재 위치
        self.current_x = 0.0
        self.current_y = 0.0
        
        # AMCL pose 구독
        rospy.Subscriber('/amcl_pose', PoseWithCovarianceStamped, self.pose_callback)
        
        # 로봇 출발지
        self.start = MoveBaseGoal()
        self.start.target_pose.header.frame_id = 'map'
        self.start.target_pose.pose.position.x = 17.920871348030918
        self.start.target_pose.pose.position.y = -4.73296913383257
        self.start.target_pose.pose.orientation.z = 0.0020586990898896155
        self.start.target_pose.pose.orientation.w = 0.9999978808767833

        self.goal_list.append(self.start)




        self.goal1 = MoveBaseGoal()
        self.goal1.target_pose.header.frame_id = 'map'
        self.goal1.target_pose.pose.position.x = 15.823250114282638
        self.goal1.target_pose.pose.position.y = -9.978778437337722
        self.goal1.target_pose.pose.orientation.z = 0.0014885433036157358
        self.goal1.target_pose.pose.orientation.w = 0.9999988921188029

        self.goal_list.append(self.goal1)



        self.goal2 = MoveBaseGoal()
        self.goal2.target_pose.header.frame_id = 'map'
        self.goal2.target_pose.pose.position.x = 17.11269189446072
        self.goal2.target_pose.pose.position.y = -10.028219972621413
        self.goal2.target_pose.pose.orientation.z = 0.026512065169513717
        self.goal2.target_pose.pose.orientation.w = 0.9996484934217864

        self.goal_list.append(self.goal2)

        self.goal = MoveBaseGoal()
        self.goal.target_pose.header.frame_id = 'map'
        self.goal.target_pose.pose.position.x = 19.684298097937315
        self.goal.target_pose.pose.position.y = -10.029209137608639
        self.goal.target_pose.pose.orientation.z = 0.022758815101167416
        self.goal.target_pose.pose.orientation.w = 0.9997409846231127

        self.goal_list.append(self.goal)


        self.sequence = 0
        self.start_time = rospy.Time.now()
        self.goal_sent = False  # 목표가 전송되었는지 추적
    
    def pose_callback(self, msg):
        # AMCL에서 로봇의 현재 위치를 받아옴
        self.current_x = msg.pose.pose.position.x
        self.current_y = msg.pose.pose.position.y
        rospy.loginfo_throttle(5.0, f"현재 로봇 위치: ({self.current_x:.2f}, {self.current_y:.2f})")

    
    def calculate_distance_to_goal(self):
        # 현재 위치와 목표 위치 사이의 거리 계산
        if not self.goal_sent:
            return float('inf')  # 목표가 전송되지 않았으면 무한대 거리
            
        current_goal = self.goal_list[self.sequence]
        goal_x = current_goal.target_pose.pose.position.x
        goal_y = current_goal.target_pose.pose.position.y
        
        distance = math.sqrt((self.current_x - goal_x)**2 + (self.current_y - goal_y)**2)
        return distance
    
    def is_arrived(self):
        #오차 범위 내에 도착했는지 확인
        distance = self.calculate_distance_to_goal()
        return distance <= self.arrival_tolerance
    
    def run(self):
        current_state = self.client.get_state()
        
        # 상태 디버깅
        rospy.loginfo_throttle(3.0, f"현재 상태: {current_state}, arrival_flag: {self.arrival_flag}")
        
        if self.arrival_flag == False:
            # 첫 번째 목표 전송 또는 목표가 완료/실패된 경우
            if not self.goal_sent or current_state in [GoalStatus.SUCCEEDED, GoalStatus.ABORTED, GoalStatus.PREEMPTED]:
                self.start_time = rospy.Time.now()
                self.sequence = (self.sequence + 1) % len(self.goal_list)  # 전체 목표 수에 맞게 순환
                self.client.send_goal(self.goal_list[self.sequence])
                self.goal_sent = True
                self.flag_pub.publish(False)
                current_goal = self.goal_list[self.sequence]
                rospy.loginfo(f"새 목표 전송: sequence {self.sequence}/{len(self.goal_list)-1}, 목표: ({current_goal.target_pose.pose.position.x:.2f}, {current_goal.target_pose.pose.position.y:.2f})")
                
            # 목표로 이동 중인 경우
            elif current_state == GoalStatus.ACTIVE:
                # 거리 기반 도착 확인
                if self.is_arrived():
                    distance = self.calculate_distance_to_goal()
                    rospy.loginfo(f"목표 {self.sequence} 도달! 거리: {distance:.2f}m")
                    self.client.cancel_all_goals()
                    
                    # 마지막 목표(인덱스 3)에 도착했을 때만 arrival_flag=True
                    if self.sequence == len(self.goal_list) - 1:
                        rospy.loginfo("마지막 목표 도착!")
                        self.arrival_flag = True
                        self.flag_pub.publish(True)
                    else:
                        rospy.loginfo(f"중간 목표 도착")
                        self.goal_sent = False  # 다음 목표 전송을 위해 플래그 리셋
                        self.flag_pub.publish(False)
                    
                # 100초 타임아웃 체크
                elif (rospy.Time.now().to_sec() - self.start_time.to_sec()) > 100.0:
                    rospy.logwarn("목표 취소 후 재시도")
                    self.stop()
                    self.goal_sent = False  # 새 목표 전송을 위해 플래그 리셋
                    self.flag_pub.publish(False)
                    
                else:
                    # 정상 이동 중 - 거리 정보 표시
                    distance = self.calculate_distance_to_goal()
                    rospy.loginfo_throttle(2.0, f"목표까지 거리: {distance:.2f}m")
                    self.flag_pub.publish(False)
            else:
                self.flag_pub.publish(False)
        else:
            # arrival_flag가 True인 상태 유지
            self.flag_pub.publish(True)
            rospy.loginfo_throttle(5.0, "arrival_flag=True 상태 유지")
    
    def stop(self):
        self.client.cancel_all_goals()
        


def main():
    rospy.init_node('navigation_client')
    nc = NavigationClient()
    rate = rospy.Rate(10)

    while not rospy.is_shutdown():
        nc.run()
        rate.sleep()

if __name__ == '__main__':
    main()