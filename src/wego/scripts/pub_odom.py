#! /usr/bin/env python3

import rospy
import tf2_ros
from geometry_msgs.msg import TransformStamped
from nav_msgs.msg import Odometry

class PubTF:
    def __init__(self):
        rospy.init_node('odom_tf_broadcast', anonymous=True)
        self.br=tf2_ros.TransformBroadcaster()
        self.transformstamped = TransformStamped()
        self.transformstamped.header.frame_id = "odom"
        rospy.Subscriber('odom', Odometry, self.callback)
    
    def callback(self, msg):
        self.transformstamped.header.stamp = rospy.Time.now()
        self.transformstamped.child_frame_id = msg.child_frame_id

        self.transformstamped.transform.translation.x = msg.pose.pose.position.x
        self.transformstamped.transform.translation.y = msg.pose.pose.position.y
        self.transformstamped.transform.translation.z = msg.pose.pose.position.z
        
        self.transformstamped.transform.rotation = msg.pose.pose.orientation
        self.br.sendTransform(self.transformstamped)

def main():
    try:
        pub_tf = PubTF()
        rospy.spin()
    except rospy.ROSInterruptException:
        pass

if __name__ == "__main__":
    main()