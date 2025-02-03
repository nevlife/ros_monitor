#!/usr/bin/python3
# -*- coding: utf-8 -*-
import rospy
import rosgraph
import time
import os
import yaml
from std_msgs.msg import String
from threading import Lock
from collections import deque
import json  


class ROSTopicMetrics:
    def __init__(self, topic, window_size=10):
        self.topic = topic
        self.hz_lock = Lock()
        self.bandwidth_lock = Lock()

        self.times = deque(maxlen=window_size)
        self.bytes_received = 0
        self.start_time = None
        self.end_time = None

        self.subscriber = rospy.Subscriber(topic, rospy.AnyMsg, self.callback)

    def callback(self, msg):
        now = time.perf_counter()
        with self.hz_lock:
            self.times.append(now)
        
        with self.bandwidth_lock:
            self.start_time = self.start_time or now
            self.end_time = now
            self.bytes_received += len(msg._buff) if hasattr(msg, '_buff') else len(str(msg))

    def get_hz(self):
        with self.hz_lock:
            if len(self.times) < 2:
                return None

            deltas = [self.times[i + 1] - self.times[i] for i in range(len(self.times) - 1)]
            return 1.0 / (sum(deltas) / len(deltas)) if deltas else -1


    def get_bandwidth(self):
        with self.bandwidth_lock:
            if not self.start_time or not self.end_time:
                return -1
            duration = self.end_time - self.start_time
            return self.bytes_received / duration if duration > 0 else -1

    def reset(self):
        with self.hz_lock, self.bandwidth_lock:
            self.times.clear()
            self.bytes_received = 0
            self.start_time = None
            self.end_time = None


class MetricsManager:
    def __init__(self):
        self.yaml_file = os.path.join(os.path.dirname(os.path.abspath(__file__)), '../cfg/topic_lst.yaml')
        self.monitors = {}
        self.initialize_monitors()

    def initialize_monitors(self):
        try:
            with open(self.yaml_file, 'r') as file:
                topic_list = file.read().splitlines()
        except Exception as e:
            rospy.logerr(f"Failed to read YAML file: {e}")
            topic_list = []

        try:
            master = rosgraph.Master('/rostopic')
            published_topics = {t[0] for t in master.getPublishedTopics('')}
            filtered_topics = [topic for topic in topic_list if topic in published_topics]
        except Exception as e:
            rospy.logerr(f"Failed to retrieve published topics: {e}")
            filtered_topics = []

        rospy.loginfo(f"Initializing monitors for {len(filtered_topics)} topics...")
        self.monitors = {topic: ROSTopicMetrics(topic) for topic in filtered_topics}

    def get_metrics(self):
        return {
            topic: {
                'hz': monitor.get_hz(),
                'bw': monitor.get_bandwidth()
            }
            for topic, monitor in self.monitors.items()
        }


def main():
    rospy.init_node('topic_hzbw', anonymous=True)
    topic_hzbw_pub = rospy.Publisher('/topic_hzbw', String, queue_size=100)

    manager = MetricsManager()
    rate = rospy.Rate(1)  
    try:
        while not rospy.is_shutdown():
            metrics = manager.get_metrics()
            json_data = json.dumps([
                {'topic': topic, 'hz': metric['hz'], 'bw': metric['bw']}
                for topic, metric in metrics.items()
            ])
            rospy.loginfo(json_data)
            topic_hzbw_pub.publish(json_data)
            rate.sleep()
    except rospy.ROSInterruptException:
        rospy.loginfo("Shutting down topic_hzbw node.")


if __name__ == '__main__':
    main()
