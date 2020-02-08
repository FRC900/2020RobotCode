#!/usr/bin/env python

import rospy
from as726x_msgs.msg import AS726xCalibratedChannelData as ccd
from std_msgs.msg import Int8

def publisher():
    pub = rospy.Publisher("chatter", Int8, queue_size=10)
    rospy.init_node("talker", ccd, anonymous=True)
    rate = rospy.Rate(10)
    while not rospy.is_shutdown():
        rospy.loginfo(1)
        pub.publish()
        rate.sleep()

if __name__ == "__main__":
    try:
        publisher()
    except rospy.ROSInterruptExecution:
        pass
