#!/usr/bin/env python3
# -*- coding: utf-8 -*-

import rospy
from geometry_msgs.msg import PoseWithCovarianceStamped
from std_msgs.msg import Bool, Int32

class AreaChecker:
    def __init__(self):
        rospy.init_node("curve_detect_node")

        # first_curve 범위 꼭짓점 (x, y)
        self.first_curve_left_top = (39.720, -8.000)
        self.first_curve_left_bottom = (35.800, -8.000)
        self.first_curve_right_top = (39.720, -10.100)
        self.first_curve_right_bottom = (35.800, -10.100)

        # second_curve 범위 꼭짓점 (x, y)
        self.second_curve_left_top = (39.720, 1.090)
        self.second_curve_left_bottom = (36.510, 1.090)
        self.second_curve_right_top = (39.720, -1.620)
        self.second_curve_right_bottom = (36.510, -1.620)

        self.prev_state = False
        self.state = False
        self.num = 0
        self.prev_num = 0

        # publisher
        self.flag_pub = rospy.Publisher("/curve_flag", Bool, queue_size=1, latch=True)
        self.num_pub = rospy.Publisher("/curve_num_flag", Int32, queue_size=1, latch=True)

        # subscriber
        rospy.Subscriber("/amcl_pose", PoseWithCovarianceStamped, self.pose_callback)

        rospy.spin()

    def first_curve_rectangle(self, x, y):
        rect = [self.first_curve_left_top, self.first_curve_left_bottom, self.first_curve_right_bottom, self.first_curve_right_top]
        
        def cross(o, a, b):
            return (a[0]-o[0])*(b[1]-o[1]) - (a[1]-o[1])*(b[0]-o[0])

        inside = True
        for i in range(4):
            o = rect[i]
            a = rect[(i+1) % 4]
            if cross(o, a, (x, y)) < 0:  # 반시계 방향
                inside = False
                break
        return inside
    
    def second_curve_rectangle(self, x, y):
        rect = [self.second_curve_left_top, self.second_curve_left_bottom, self.second_curve_right_bottom, self.second_curve_right_top]
        
        def cross(o, a, b):
            return (a[0]-o[0])*(b[1]-o[1]) - (a[1]-o[1])*(b[0]-o[0])

        inside = True
        for i in range(4):
            o = rect[i]
            a = rect[(i+1) % 4]
            if cross(o, a, (x, y)) < 0:  # 반시계 방향
                inside = False
                break
        return inside

    def pose_callback(self, msg):
        x = msg.pose.pose.position.x
        y = msg.pose.pose.position.y

        first_curve_pose = self.first_curve_rectangle(x, y)
        second_curve_pose = self.second_curve_rectangle(x, y)

        if (first_curve_pose or second_curve_pose):
            self.state = True

            if first_curve_pose:
                self.num = 1
            elif second_curve_pose:
                self.num = 2

        else:
            self.state = False
            self.num = 0
        
        # 계속 pub
        self.flag_pub.publish(Bool(data=self.state))
        self.num_pub.publish(Int32(data=self.num))
        rospy.loginfo(f"Pos: ({x:.3f}, {y:.3f}) -> {self.num} -> {self.state}")

        # # 바뀔 때 pub
        # if (self.state != self.prev_state) or (self.num != self.prev_num):
        #     self.flag_pub.publish(Bool(data=self.state))
        #     self.num_pub.publish(Int32(data=self.num))
        #     rospy.loginfo(f"Pos: ({x:.3f}, {y:.3f}) -> {self.num} -> {self.state}")
        
        self.prev_state = self.state
        self.prev_num = self.num

if __name__ == "__main__":
    try:
        AreaChecker()
    except rospy.ROSInterruptException:
        pass