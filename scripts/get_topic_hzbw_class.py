#!/usr/bin/python3
# -*- coding: utf-8 -*-
import rospy
import rosgraph.masterapi
import json
import time
from collections import deque
from std_msgs.msg import String

class TopicMonitor:
    def __init__(self, topic):
        self.topic = topic
        self.timestamps = deque(maxlen=500)
        self.byte_sizes = deque(maxlen=500)
        self.subscriber = rospy.Subscriber(topic, rospy.AnyMsg, self.callback)

    def callback(self, msg):
        now = time.perf_counter()
        msg_size = len(msg._buff) if hasattr(msg, '_buff') else len(str(msg))
        self.timestamps.append(now)
        self.byte_sizes.append(msg_size)

    def get_hz(self):
        now = time.perf_counter()
        valid_timestamps = [t for t in self.timestamps if now - t <= 1]
        elapsed_time = valid_timestamps[-1] - valid_timestamps[0] if len(valid_timestamps) > 1 else 1
        return len(valid_timestamps) / elapsed_time if elapsed_time > 0 else 0

    def get_bw(self):
        now = time.perf_counter()
        valid_byte_sizes = [size for t, size in zip(self.timestamps, self.byte_sizes) if now - t <= 1]
        return sum(valid_byte_sizes) / 1024.0 if valid_byte_sizes else 0.0  # KB 변환


class ROSTopicMonitor:
    def __init__(self):
        rospy.init_node("ros_topic_monitor", anonymous=True)
        self.master = rosgraph.masterapi.Master('/roscore')
        self.topic_monitors = {}  # 각 토픽별 객체 저장
        self.publisher = rospy.Publisher("/topic_metrics", String, queue_size=100)

    def update_subscriptions(self):
        active_topics = {t[0] for t in self.master.getSystemState()[0]}  # 현재 실행 중인 토픽 목록
        new_topics = active_topics - set(self.topic_monitors.keys())

        # 새로운 토픽을 객체로 추가
        for topic in new_topics:
            self.topic_monitors[topic] = TopicMonitor(topic)

        # 존재하지 않는 토픽 삭제
        removed_topics = set(self.topic_monitors.keys()) - active_topics
        for topic in removed_topics:
            del self.topic_monitors[topic]

    def calculate_metrics(self):
        topic_metrics_list = []
        for topic, monitor in self.topic_monitors.items():
            topic_metrics = {
                "topic": topic,
                "hz": round(monitor.get_hz(), 2),
                "bw": round(monitor.get_bw(), 2)
            }
            print(topic_metrics)
            topic_metrics_list.append(topic_metrics)

        if topic_metrics_list:
            self.publisher.publish(json.dumps(topic_metrics_list))

    def run_monitoring(self):
        rate = rospy.Rate(1)  # 1Hz
        while not rospy.is_shutdown():
            self.update_subscriptions()
            self.calculate_metrics()
            rate.sleep()


if __name__ == "__main__":
    monitor = ROSTopicMonitor()
    monitor.run_monitoring()
