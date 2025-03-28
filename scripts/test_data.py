#!/usr/bin/env python3
import rospy
from std_msgs.msg import Float32MultiArray, String
import json

def total_resource_callback(msg):
    print("Total Resource:")
    print(type(msg.data))

def topics_hzbw_callback(msg):
    data = json.loads(msg.data)
    print("\nTopics Hz/BW:")
    print(type(data))
    for topic_info in data:
        print(type(topic_info))

def nodes_resource_callback(msg):
    data = json.loads(msg.data)
    print("\nNode Resource Usage:")
    print(type(data))
    for node_info in data:
        print(type(node_info))

def listener():
    rospy.init_node('data_printer', anonymous=True)

    rospy.Subscriber("/total_resource", Float32MultiArray, total_resource_callback)
    rospy.Subscriber("/topics_hzbw", String, topics_hzbw_callback)
    rospy.Subscriber("/nodes_resource", String, nodes_resource_callback)

    rospy.spin()

if __name__ == '__main__':
    listener()
