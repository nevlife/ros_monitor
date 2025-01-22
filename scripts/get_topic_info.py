#!/usr/bin/python3
# -*- coding: utf-8 -*-
import rospy
import rosgraph
import rosnode
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
        '''return hz'''
        
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
        '''init data'''
        with self.hz_lock, self.bandwidth_lock:
            self.times.clear()
            self.bytes_received = 0
            self.start_time = None
            self.end_time = None
    

class MetricsManager:
    def __init__(self):
        self.yaml_file = os.path.join(os.path.dirname(os.path.abspath(__file__)), '../cfg/topic_lst.yaml')
        self.monitors = {}
        self.topic_to_node_map = {}
        self.initialize_monitors()
        
    def initialize_monitors(self):
        #read yaml file
        try:
            with open(self.yaml_file, 'r') as file:
                topic_list = file.read().splitlines()
        except Exception as e:
            rospy.logerr(f"Failed to read YAML file: {e}")
            topic_list = []

        # ROS 마스터에서 퍼블리시된 토픽 가져오기
        master = rosgraph.Master('/rostopic')
        published_topics = master.getPublishedTopics('')
        
        topic_names = []
        filtered_topic_list = []
        
        for t in published_topics:
            topic_names.append(t[0])
            
        for topic in topic_list:
            if topic in topic_names:
                filtered_topic_list.append(topic)
        
        
        rospy.loginfo(f'Init monitors for {len(filtered_topic_list)} topics...')

        for topic in filtered_topic_list:
            self.monitors[topic] = ROSTopicMetrics(topic)

    def get_metrics(self):
        results = {}
        for topic, monitor in self.monitors.items():
            hz = monitor.get_hz()
            bandwidth = monitor.get_bandwidth()
            
            if hz is not None:
                hz_val = f'{hz:.2f}'
            else:
                hz_val = 'N/A'

            if bandwidth is not None:
                bandwidth_val = f'{bandwidth:.2f}'
            else:
                bandwidth_val = 'N/A'
                
            results[topic] = {
                'hz': hz_val,
                'bw': bandwidth_val
            }

        return results
