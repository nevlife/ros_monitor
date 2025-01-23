#!/usr/bin/python3
# -*- coding: utf-8 -*-
import rospy
import rosgraph
import time
import os
from std_msgs.msg import String
from threading import Lock
from collections import deque
import json  # JSON 형식으로 노드 데이터 직렬화


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
        '''Callback to collect data for Hz and Bandwidth calculations.'''
        now = time.perf_counter()

        # Update Hz
        with self.hz_lock:
            self.times.append(now)

        # Update Bandwidth
        with self.bandwidth_lock:
            if self.start_time is None:
                self.start_time = now
            self.end_time = now

            if hasattr(msg, '_buff'):
                msg_size = len(msg._buff)
            else:
                msg_size = len(str(msg))

            self.bytes_received += msg_size

    def get_hz(self):
        '''Calculate and return Hz.'''
        with self.hz_lock:
            if len(self.times) < 2:
                return None

            deltas = [self.times[i + 1] - self.times[i] for i in range(len(self.times) - 1)]
            if deltas:
                return 1.0 / (sum(deltas) / len(deltas))
            else:
                return None

    def get_bandwidth(self):
        '''Calculate and return Bandwidth.'''
        with self.bandwidth_lock:
            if self.start_time is None or self.end_time is None:
                return None

            duration = self.end_time - self.start_time
            if duration == 0:
                return None

            return self.bytes_received / duration

    def reset(self):
        '''Reset metrics data.'''
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
        '''Initialize topic monitors from YAML file.'''
        try:
            with open(self.yaml_file, 'r') as file:
                topic_list = file.read().splitlines()
        except Exception as e:
            rospy.logerr(f"Failed to read YAML file: {e}")
            topic_list = []

        master = rosgraph.Master('/rostopic')
        published_topics = master.getPublishedTopics('')

        topic_names = [t[0] for t in published_topics]
        filtered_topic_list = [topic for topic in topic_list if topic in topic_names]

        rospy.loginfo(f'Init monitors for {len(filtered_topic_list)} topics...')

        for topic in filtered_topic_list:
            self.monitors[topic] = ROSTopicMetrics(topic)

    def get_metrics(self):
        '''Gather metrics from all monitored topics.'''
        results = {}
        for topic, monitor in self.monitors.items():
            hz = monitor.get_hz()
            bandwidth = monitor.get_bandwidth()

            results[topic] = {
                'hz': hz if hz is not None else 0.0,
                'bw': bandwidth if bandwidth is not None else 0.0
            }
        return results


def main():
    try:
        rospy.init_node('topic_hzbw', anonymous=True)
        topic_hzbw_pub = rospy.Publisher('/topic_hzbw', String, queue_size=100)

        manager = MetricsManager()
        rate = rospy.Rate(1)  # Publish at 1 Hz

        while not rospy.is_shutdown():
            metrics = manager.get_metrics()

            # Prepare JSON data for publishing
            data = []
            for topic, metric in metrics.items():
                hz = metric['hz']
                bw = metric['bw']
                print(f"Topic: {topic}, Hz: {hz:.2f}, BW: {bw:.2f} bytes/sec")
                data.append({
                    'topic': topic,
                    'hz': hz,
                    'bw': bw
                })

            # Serialize the data to JSON and publish it
            json_data = json.dumps(data)
            print(json_data)
            topic_hzbw_pub.publish(json_data)

            rate.sleep()
    except rospy.ROSInterruptException:
        rospy.loginfo("Shutting down topic_hzbw node.")


if __name__ == '__main__':
    main()
