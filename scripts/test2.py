#!/usr/bin/python3
# -*- coding: utf-8 -*-
import rospy
from std_msgs.msg import String

def callback(data):
    rospy.loginfo(f"Received: {data.data}")

def subscriber_node():
    rospy.init_node('subscriber_node', anonymous=True)
    rospy.Subscriber('chatter', String, callback)

    rospy.spin()

if __name__ == '__main__':
    try:
        subscriber_node()
    except rospy.ROSInterruptException:
        pass