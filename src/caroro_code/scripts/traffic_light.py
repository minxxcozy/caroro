#!/usr/bin/env python3
#-*- coding:utf-8 -*-

import rospy
from morai_msgs.msg import GetTrafficLightStatus

traffic_status = False
verbose_logging = False  # 출력 여부 제어
printed = False

def traffic_CB(msg):
    global traffic_status
    
    # 신호등 인덱스가 "SN000005"일 때만 처리
    if msg.trafficLightIndex == "SN000005":
        signal = msg.trafficLightStatus
            
        # 초록불(16) 또는 좌회전(33)일 때 출발 신호 (True)를 보냄
        if signal == 16 or signal == 33:
            traffic_status = True
            if verbose_logging :
                print("Traffic signal: Start (True)")
        # 빨간불(1) 또는 노란불(4), 그 외의 신호일 때 정지 신호 (False)를 보냄
        else :
            traffic_status = False
            if verbose_logging:
                print("Traffic signal: Stop (False)")

def init_traffic_light_node(verbose = False):
    global verbose_logging
    verbose_logging = verbose
    rospy.Subscriber("/GetTrafficLightStatus", GetTrafficLightStatus, traffic_CB)