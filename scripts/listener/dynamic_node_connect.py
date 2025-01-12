#!/usr/bin/python3
# -*- coding: utf-8 -*-
import rospy
import rosgraph
import rosnode
from rosgraph.masterapi import Master
from std_msgs.msg import String
from threading import Lock


class DynamicSubscriber:
    def __init__(self):
        rospy.init_node("dynamic_node_connecter", anonymous=True)
        self.master = Master('/ros_master')
        self.subscribers = {}  # 토픽별 구독자 저장
        self.lock = Lock()  # 동기화 처리
        self.rate = rospy.Rate(1)  # 1Hz
        rospy.Timer(rospy.Duration(5.0), self.update_topics)
        rospy.loginfo("DynamicSubscriber initialized.")

    def update_topics(self, event):
        """ROS Master에서 현재 토픽 목록 가져오기 및 구독"""
        try:
            topic_list = self.master.getPublishedTopics('/')
            rospy.loginfo(f"Found {len(topic_list)} topics.")
            with self.lock:
                for topic, msg_type in topic_list:
                    if topic not in self.subscribers:
                        self.subscribe_to_topic(topic, msg_type)
        except Exception as e:
            rospy.logwarn(f"Failed to fetch topics: {e}")

    def subscribe_to_topic(self, topic, msg_type):
        """주어진 토픽에 대한 구독 설정"""
        rospy.loginfo(f"Subscribing to {topic} with type {msg_type}.")
        try:
            # 메시지 타입을 로드
            msg_class = self.load_message_class(msg_type)
            if msg_class:
                self.subscribers[topic] = rospy.Subscriber(
                    topic, msg_class, self.generic_callback, callback_args=topic
                )
            else:
                rospy.logwarn(f"Failed to load message class for {msg_type}.")
        except Exception as e:
            rospy.logwarn(f"Failed to subscribe to {topic}: {e}")

    @staticmethod
    def load_message_class(msg_type):
        """메시지 타입 문자열로부터 메시지 클래스 로드"""
        try:
            pkg, msg = msg_type.split('/')
            module = __import__(f"{pkg}.msg", fromlist=[msg])
            return getattr(module, msg)
        except Exception as e:
            rospy.logwarn(f"Error loading message class {msg_type}: {e}")
            return None

    def generic_callback(self, msg, topic):
        """모든 구독 콜백의 공통 처리"""
        rospy.loginfo(f"Received on {topic}: {msg}")

    def spin(self):
        """ROS 스핀"""
        rospy.spin()

def main():
    try:
        subscriber = DynamicSubscriber()
        subscriber.spin()
    except rospy.ROSInterruptException:
        pass
    
if __name__ == "__main__":
    main()
