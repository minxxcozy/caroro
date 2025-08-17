#!/usr/bin/env python3
# -*- coding: utf-8 -*-

import rospy
from geometry_msgs.msg import PoseWithCovarianceStamped
from std_msgs.msg import Bool, Int32

class AreaChecker:
    def __init__(self):
        rospy.init_node("rotary_pose_node")

        # in 범위 꼭짓점 (x, y)
        self.in_left_top = (32.120, -2.800)
        self.in_left_bottom = (32.120, -2.610)
        self.in_right_top = (30.840, -2.900)
        self.in_right_bottom = (30.840, -2.710)

        # middle 범위 꼭짓점 (x, y)
        self.middle_left_top = (30.882, -3.617)
        self.middle_left_bottom = (30.582, -3.617)
        self.middle_right_top = (30.932, -3.806)
        self.middle_right_bottom = (30.612, -3.806)

        # out 범위 꼭짓점 (x, y)
        self.out_left_top = (31.643, -3.900)
        self.out_left_bottom = (31.817, -3.580)
        self.out_right_top = (30.328, -3.884)
        self.out_right_bottom = (30.254, -3.494)

        self.prev_state = False
        self.state = False
        self.num = 0
        self.prev_num = 0

        # publisher
        self.flag_pub = rospy.Publisher("/rotary_flag", Bool, queue_size=1, latch=True)
        self.num_pub = rospy.Publisher("/rotary_flag_num", Int32, queue_size=1, latch=True)

        # subscriber
        rospy.Subscriber("/amcl_pose", PoseWithCovarianceStamped, self.pose_callback)

        rospy.loginfo("Rotary Out Area Check Started")
        rospy.spin()

    def in_rectangle(self, x, y):
        rect = [self.in_left_top, self.in_left_bottom, self.in_right_bottom, self.in_right_top]
        
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
    
    def middle_rectangle(self, x, y):
        rect = [self.middle_left_top, self.middle_left_bottom, self.middle_right_bottom, self.middle_right_top]

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

    def out_rectangle(self, x, y):
        rect = [self.out_left_top, self.out_left_bottom, self.out_right_bottom, self.out_right_top]
        
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

        in_pose = self.in_rectangle(x, y)
        middle_pose = self.middle_rectangle(x, y)
        out_pose = self.out_rectangle(x, y)

        if (in_pose or middle_pose or out_pose):
            self.state = True

            if in_pose:
                self.num = 1
            elif middle_pose:
                self.num = 2
            elif out_pose:
                self.num = 3

        else:
            self.state = False
            self.num = 0
            
        if (self.state != self.prev_state) or (self.num != self.prev_num):
            self.flag_pub.publish(Bool(data=self.state))
            self.num_pub.publish(Int32(data=self.num))
            rospy.loginfo(f"Pos: ({x:.3f}, {y:.3f}) -> {self.num} -> {self.state}")
        
        self.prev_state = self.state
        self.prev_num = self.num

        # x = msg.pose.pose.position.x
        # y = msg.pose.pose.position.y

        # in_pose = self.in_rectangle(x, y)
        # middle_pose = self.middle_rectangle(x, y)
        # out_pose = self.out_rectangle(x, y)

        # if (in_pose or middle_pose or out_pose):
        #     self.state = True
        # else:
        #     self.state = False
            
        # self.pub.publish(Bool(data=self.state))
        # rospy.loginfo(f"Pos: ({x:.3f}, {y:.3f}) -> {self.state}")

if __name__ == "__main__":
    try:
        AreaChecker()
    except rospy.ROSInterruptException:
        pass