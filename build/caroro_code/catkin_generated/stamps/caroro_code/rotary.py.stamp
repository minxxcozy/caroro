#!/usr/bin/env python3
#-*- coding:utf-8 -*-

import rospy
from sensor_msgs.msg import LaserScan
from std_msgs.msg import Bool  
from math import pi

class Rotary:
    def __init__(self):
        rospy.init_node("rotary_node")
        
        # /scan 토픽으로부터 LaserScan 메시지 구독
        rospy.Subscriber("/scan", LaserScan, self.lidar_CB)
        
        # /rotary_state 토픽에 Bool 메시지를 발행할 퍼블리셔 생성
        self.state_pub = rospy.Publisher("/rotary_state", Bool, queue_size=1)
        
        # 현재 차량의 상태를 추적하는 변수. None으로 초기화하여,
        # 첫 메시지 수신 시 반드시 상태가 변경되도록 함
        self.current_state = None

    def lidar_CB(self, msg):
        self.scan_msg = msg
        
        # LiDAR 데이터의 각도 정보를 도(degree) 단위로 변환
        degree_min = self.scan_msg.angle_min * 180/pi
        degree_angle_increment = self.scan_msg.angle_increment * 180/pi
        degrees = [degree_min + degree_angle_increment * index for index, value in enumerate(self.scan_msg.ranges)]

        # 정지 및 출발 조건 충족 여부를 확인하기 위한 플래그 변수
        stop_condition_met = False
        go_condition_met = False
        
        # LiDAR 데이터 배열을 순회하며 장애물 감지 로직 처리
        for i in range(len(self.scan_msg.ranges)):
            deg = degrees[i]
            dist = self.scan_msg.ranges[i]
            
            # 유효한 거리(0m 초과, 2m 미만)에서만 장애물 감지
            if 0 < dist < 2:
                # Stop 조건: -10도 ~ 40도 범위에 장애물이 감지되면 플래그를 True로 설정
                if -30 <= deg <= 40:
                    stop_condition_met = True
                
                # Go 조건: -40도 근처(오차범위 0.5도)에 장애물이 감지되면 플래그를 True로 설정
                if abs(deg + 40) < 0.5:
                    go_condition_met = True
        
        # 상태 전환 로직
        new_state = self.current_state
        
        # 1. 어떤 장애물도 감지되지 않으면 'moving' 상태로 전환
        if not stop_condition_met and not go_condition_met:
            new_state = "moving"
        # 2. 'stop' 조건이 충족되면 'stopping' 상태로 전환
        elif stop_condition_met:
            new_state = "stopping"
        # 3. 'go' 조건이 충족되면 'go' 상태로 전환
        elif go_condition_met:
            new_state = "go"
        
        # 이전 상태와 새로운 상태가 다를 경우에만 메시지 출력 및 토픽 발행
        if new_state != self.current_state:
            self.current_state = new_state
            
            state_msg = Bool()
            
            # 상태에 따라 메시지를 출력하고 Bool 값을 설정
            if self.current_state == "stopping":
                print("Stopping!")
                state_msg.data = False
            elif self.current_state == "go":
                print("Go!")
                state_msg.data = True
            elif self.current_state == "moving":
                print("Moving forward...")
                state_msg.data = True
            
            self.state_pub.publish(state_msg)
            
def main():
    try:
        rotary = Rotary()
        rospy.spin()
    except rospy.ROSInterruptException:
        pass

if __name__ == "__main__":
    main()

