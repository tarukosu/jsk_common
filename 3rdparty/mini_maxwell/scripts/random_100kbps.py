#!/usr/bin/env python
import roslib
roslib.load_manifest('mini_maxwell')
import rospy
import sys

import random

from std_msgs.msg import Int32
import time


rospy.init_node("random_connection")
pub = rospy.Publisher("/rate_limit", Int32)

def setConnection(event):
    if random.randint(1, 9) == 1:
        pub.publish(Int32(128))
        time.sleep(5)
    a = random.randint(1, 3)
    if a == 1:
        pub.publish(Int32(50000))
    elif a==2:
        pub.publish(Int32(100000))
    elif a==3:
        pub.publish(Int32(200000))

rospy.Timer(rospy.Duration(20), setConnection)
rospy.spin()
