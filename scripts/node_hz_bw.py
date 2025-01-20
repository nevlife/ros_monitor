#!/usr/bin/python3
# -*- coding: utf-8 -*-
import rospy
import rosgraph
import time
import os
import psutil  # For system metrics
from collections import deque
from threading import Lock


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
        '''data for hz and bandwidth calculations'''
        now = time.perf_counter()

        # hz
        with self.hz_lock:
            self.times.append(now)

        # bandwidth
        with self.bandwidth_lock:
            if self.start_time is None:
                self.start_time = now
            self.end_time = now

            if hasattr(msg, '_buff'):
                msg_size = len(msg._buff)
            else:
                # _buff 속성이 없을 때 메시지를 문자열로 변환한 후 길이 계산
                msg_size = len(str(msg))

            self.bytes_received += msg_size

    def get_hz(self):
        """return Hz"""
        with self.hz_lock:
            if len(self.times) < 2:
                return None
            deltas = []
            timestamps = list(self.times)

            for i in range(len(timestamps) - 1):
                t1 = timestamps[i]
                t2 = timestamps[i + 1]
                delta = t2 - t1
                deltas.append(delta)

            if deltas:
                hz = 1.0 / (sum(deltas) / len(deltas))
                return hz
            else:
                return None

    def get_bandwidth(self):
        """return bandwidth"""
        with self.bandwidth_lock:

            if self.start_time is None or self.end_time is None:
                return None

            duration = self.end_time - self.start_time

            if duration == 0:
                return None

            return self.bytes_received / duration

    def reset(self):
        """init data"""
        with self.hz_lock, self.bandwidth_lock:
            self.times.clear()
            self.bytes_received = 0
            self.start_time = None
            self.end_time = None


class MetricsManager:
    def __init__(self, yaml_file=None):
        if yaml_file is None:
            base_path = os.path.dirname(os.path.abspath(__file__))  # 현재 파일의 디렉터리 경로
            yaml_file = os.path.join(base_path, "../cfg/topic_lst.yaml")  # YAML 파일 절대 경로
        self.monitors = {}
        self.yaml_file = yaml_file
        self.initialize_monitors()

    def initialize_monitors(self):
        # YAML 파일에서 토픽 리스트 읽기
        try:
            with open(self.yaml_file, 'r') as file:
                topic_list = file.read().splitlines()
        except Exception as e:
            rospy.logerr(f"Failed to read YAML file: {e}")
            topic_list = []

        # ROS 마스터에서 퍼블리시된 토픽 가져오기
        master = rosgraph.Master('/rostopic')
        published_topics = master.getPublishedTopics('')

        # 지정된 토픽만 필터링
        filtered_topic_list = [topic for topic in topic_list if topic in [t[0] for t in published_topics]]

        rospy.loginfo(f'Init monitors for {len(filtered_topic_list)} topics...')

        for topic in filtered_topic_list:
            self.monitors[topic] = ROSTopicMetrics(topic)

    def get_node_system_metrics(self):
        """Retrieve system metrics (CPU, RAM) for the current node."""
        try:
            process = psutil.Process(os.getpid())
            cpu_usage = process.cpu_percent(interval=0.1) / psutil.cpu_count()
            memory_usage = process.memory_info().rss / (1024 * 1024)  # Convert to MB
            return {
                "cpu": f"{cpu_usage:.2f}%",
                "memory": f"{memory_usage:.2f} MB"
            }
        except Exception as e:
            rospy.logerr(f"Failed to get system metrics: {e}")
            return {
                "cpu": "N/A",
                "memory": "N/A"
            }

    def get_metrics(self):
        results = {}
        for topic, monitor in self.monitors.items():
            hz = monitor.get_hz()
            bandwidth = monitor.get_bandwidth()

            if hz is not None:
                hz_val = hz
            else:
                hz_val = 'N/A'

            if bandwidth is not None:
                bandwidth_val = bandwidth
            else:
                bandwidth_val = 'N/A'

            results[topic] = {
                'hz': hz_val,
                'bandwidth': bandwidth_val
            }

        # Add system metrics for the current node
        system_metrics = self.get_node_system_metrics()
        results["node_metrics"] = system_metrics
    
        return results

if __name__ == '__main__':
    try:
        manager = MetricsManager()
        rate = rospy.Rate(1)  # 1 Hz
        while not rospy.is_shutdown():
            metrics = manager.get_metrics()
            for topic, data in metrics.items():
                if topic == "node_metrics":
                    rospy.loginfo(f"Node CPU = {data['cpu']}, Memory = {data['memory']}")
                else:
                    rospy.loginfo(f"{topic}: Hz = {data['hz']}, Bandwidth = {data['bandwidth']} bytes/sec")
            rate.sleep()
    except rospy.ROSInterruptException:
        pass
