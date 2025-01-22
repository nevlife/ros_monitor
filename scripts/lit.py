#!/usr/bin/env python3

import rospy
from std_msgs.msg import Float32, UInt64

class NodeResourceSubscriber:
    def __init__(self):
        # ROS 노드 초기화
        rospy.init_node("node_resource_subscriber")

        # 구독할 토픽 리스트
        self.topics = [
            "/cpu_monitor/cpu_monitor/cpu",
            "/cpu_monitor/cpu_monitor/mem",
            "/cpu_monitor/play_1737537775123604652/cpu",
            "/cpu_monitor/play_1737537775123604652/mem",
            "/cpu_monitor/rosout/cpu",
            "/cpu_monitor/rosout/mem"
        ]

        # 구독 설정
        self.subscribers = []
        for topic in self.topics:
            if topic.endswith("cpu"):  # 토픽 이름이 "cpu"로 끝나면
                rospy.loginfo(f"Subscribing to CPU topic: {topic}")
                self.subscribers.append(rospy.Subscriber(topic, Float32, self.cpu_callback, callback_args=topic))
            elif topic.endswith("mem"):  # 토픽 이름이 "mem"으로 끝나면
                rospy.loginfo(f"Subscribing to Memory topic: {topic}")
                self.subscribers.append(rospy.Subscriber(topic, UInt64, self.mem_callback, callback_args=topic))

    def cpu_callback(self, msg, topic):
        rospy.loginfo(f"[{topic}] CPU Usage: {msg.data:.2f}%")

    def mem_callback(self, msg, topic):
        rospy.loginfo(f"[{topic}] Memory Usage: {msg.data} bytes")

if __name__ == "__main__":
    try:
        NodeResourceSubscriber()
        rospy.spin()
    except rospy.ROSInterruptException:
        pass
