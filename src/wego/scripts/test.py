#!/usr/bin/env python3
#-*- coding:utf-8 -*-

import rospy
import os
from sensor_msgs.msg import LaserScan
from std_msgs.msg import Float64
from math import *

class Distance:
    def __init__(self):
        rospy.init_node("distance_node")
        rospy.Subscriber("/scan", LaserScan, self.lidar_CB)
        self.pub = rospy.Publisher("/commands/motor/speed", Float64, queue_size=1)
        self.image_msg = LaserScan()
        self.speed_msg = Float64()
        
    def lidar_CB(self, msg):
        os.system("clear")
        self.scan_msg = msg
        degree_min = self.scan_msg.angle_min * 180/pi
        degree_max = self.scan_msg.angle_max * 180/pi
        degree_angle_increment = self.scan_msg.angle_increment * 180/pi
        # print(self.scan_msg)
        # print(degree_min)
        # print(degree_max)
        # print(degree_angle_increment)
        degrees = [degree_min + degree_angle_increment * index for index, value in enumerate(self.scan_msg.ranges)]
        # print(degrees)

        obstacle = 0
        num = 0
        for i, distance in enumerate(msg.ranges):
            if -30 < degrees[i] < 30 and 0 < distance < 2:
                print(f"Index: {i}, Angle: {degrees[i]:.3f}°, Distance: {distance:.3f} m")
                num += 1
            else:
                pass
    
        print(num)
        
        if num != 0:
            self.speed_msg.data = 0
        else:
            self.speed_msg.data = 1200
        self.pub.publish(self.speed_msg)   
        
def main():
    try:
        distance = Distance()
        rospy.spin()
    except rospy.ROSInterruptException:
        pass

if __name__ == "__main__":
    main()