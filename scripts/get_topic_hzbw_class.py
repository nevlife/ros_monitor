#!/usr/bin/python3
# -*- coding: utf-8 -*-
import os
import rospy
import rosgraph.masterapi
from std_msgs.msg import String

import csv
import json

import threading

import time
from collections import deque

class TopicMonitor(threading.Thread):
    def __init__(self, topic):
        super().__init__()
        self.topic = topic
        self.timestamps = deque(maxlen=500)
        self.byte_sizes = deque(maxlen=500)
        self.subscriber = rospy.Subscriber(topic, rospy.AnyMsg, self.callback)
        
        self.running = True
        
    def callback(self, msg):
        now = time.time()
        msg_size = len(msg._buff) if hasattr(msg, '_buff') else len(str(msg))
        self.timestamps.append(now)
        self.byte_sizes.append(msg_size)

    def get_hz_and_bw(self):
        now = time.time()
        
        #valid_timestamps = [t for t in self.timestamps if now - t <= 1]
        while self.timestamps and now - self.timestamps[0] > 1:
            self.timestamps.popleft()
            self.byte_sizes.popleft()

        
        n = len(self.timestamps)
        
        if n <= 1:
            return 0, 0; #1hz 미만이면 0리턴
        
        elapsed_time = max(self.timestamps[-1] - self.timestamps[0], 1e-6) 
        
        # if elapsed_time <= 0:
        #     return 0; #멀티 스레딩으로 인한 시간 역전 방지
        
        return (n -1) / elapsed_time , sum(self.byte_sizes) / 1024.0 if self.byte_sizes else 0.0

    # def get_bw(self):
    #     now = time.time()
        
    #     #valid_byte_sizes = [size for t, size in zip(self.timestamps, self.byte_sizes) if now - t <= 1]
    #     while self.timestamps and now - self.timestamps[0] > 1:
    #         self.timestamps.popleft()
    #         self.byte_sizes.popleft()
            
    #     return sum(self.byte_sizes) / 1024.0 if self.byte_sizes else 0.0  #convert kb
    
    def run(self):
        while self.running and not rospy.is_shutdown():
            time.sleep(0.1)  # 불필요한 CPU 사용을 줄이기 위해 sleep
        
    def stop(self):
        self.running = False

class ROSTopicMonitor:
    def __init__(self):
        rospy.init_node("ros_topic_monitor", anonymous=True)
        
        #self.master = rosgraph.masterapi.Master('/roscore')
        try:
            self.master = rosgraph.Master('/rostopic')
        except Exception as e:
            rospy.logerr(f"[monitor] Failed to retrieve published topics: {e}")    
                
        self.topic_monitors = {} #토픽 객체
        self.publisher = rospy.Publisher("/topics_hzbw", String, queue_size=100)

        self.lock = threading.Lock()
        self.monitor_thread = threading.Thread(target=self.run_monitoring)
        self.monitor_thread.start()
        
    def update_subscriptions(self):
        active_topics = {t[0] for t in self.master.getSystemState()[0]}  #현재 실행 중인 토픽 목록
        new_topics = active_topics - set(self.topic_monitors.keys())

        #새 토픽 추가하고 스레드 실행
        for topic in new_topics:
            monitor = TopicMonitor(topic)
            self.topic_monitors[topic] = monitor
            monitor.start()
            
        #토픽 삭제
        removed_topics = set(self.topic_monitors.keys()) - active_topics
        
        for topic in removed_topics:
            self.topic_monitors[topic].stop()
            self.topic_monitors[topic].join()
            del self.topic_monitors[topic]

    def calculate_metrics(self):
        topic_metrics_list = []
        for topic, monitor in self.topic_monitors.items():
            hz , bw = monitor.get_hz_and_bw()
            topic_metrics = {
                "topic": topic,
                "hz": hz,
                "bw": bw,
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
    try:
        rospy.spin()
    except KeyboardInterrupt:
        for topic_monitor in monitor.topic_monitors.values():
            topic_monitor.stop()
            topic_monitor.join()
