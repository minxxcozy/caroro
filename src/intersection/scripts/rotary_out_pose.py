#!/usr/bin/env python3
# -*- coding: utf-8 -*-

import rospy
from geometry_msgs.msg import PoseWithCovarianceStamped
from std_msgs.msg import Bool

class AreaChecker:
    def __init__(self):
        rospy.init_node("rotary_out_node")

        # 범위 꼭짓점 (x, y)
        self.left_top = (31.643, -3.900)
        self.left_bottom = (31.817, -3.580)
        self.right_top = (30.328, -3.884)
        self.right_bottom = (30.254, -3.494)

        self.state = False
        self.prev_state = False

        # publisher
        self.pub = rospy.Publisher("/rotary_out_flag", Bool, queue_size=5)

        # subscriber
        rospy.Subscriber("/amcl_pose", PoseWithCovarianceStamped, self.pose_callback)

        rospy.loginfo("Rotary Out Area Check Started")
        rospy.spin()

    def point_in_rectangle(self, x, y):
        rect = [self.left_top, self.left_bottom, self.right_bottom, self.right_top]
        
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

        in_area = self.point_in_rectangle(x, y)

        if in_area:
            self.state = True
        else:
            self.state = False

        if self.state != self.prev_state:
            self.pub.publish(Bool(data=self.state))
            rospy.loginfo(f"Pos: ({x:.3f}, {y:.3f}) -> {self.state}")
        
        self.prev_state = self.state

if __name__ == "__main__":
    try:
        AreaChecker()
    except rospy.ROSInterruptException:
        pass