#!/usr/bin/python3
# -*- coding: utf-8 -*-
import rospy
import random
from std_msgs.msg import Float32MultiArray

def random_node_connecter():
    # Initialize the ROS node
    rospy.init_node('random_node_connecter', anonymous=True)

    # Create a publisher for /node_connecter topic
    pub = rospy.Publisher('/node_connecter', Float32MultiArray, queue_size=10)

    # Set publishing rate
    rate = rospy.Rate(10)

    while not rospy.is_shutdown():
        # Generate exactly 6 random float values for node statuses
        node_status = Float32MultiArray()
        node_status.data = [
            random.uniform(0.0, 1.0),
            random.uniform(0.0, 1.0),
            random.uniform(0.0, 1.0),
            random.uniform(0.0, 1.0),  # GPS Fix
            random.uniform(0.0, 1.0),  # GPS Velocity
            random.uniform(0.0, 1.0),  # IMU
            random.uniform(0.0, 1.0),  # ZED Camera
            random.uniform(0.0, 1.0)   # Camera
        ]

        # Log and publish the message
        rospy.loginfo(f"Publishing to /node_connecter: {node_status.data}")
        pub.publish(node_status)

        # Sleep to maintain the publishing rate
        rate.sleep()

if __name__ == '__main__':
    try:
        random_node_connecter()
    except rospy.ROSInterruptException:
        pass
