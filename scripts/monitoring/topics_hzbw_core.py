#!/usr/bin/python3
# -*- coding: utf-8 -*-
import os
import sys
import rospy
import rosgraph.masterapi
from std_msgs.msg import String

import json

import threading

import time
from collections import deque

# core.py 모듈 import를 위한 경로 설정
scripts_dir = os.path.dirname(os.path.dirname(os.path.abspath(__file__)))
sys.path.append(scripts_dir)

# core.py에서 Monitor 클래스 import
from core.core import Monitor, AggregationStrategies

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
        
        while self.timestamps and now - self.timestamps[0] > 1:
            self.timestamps.popleft()
            self.byte_sizes.popleft()
        
        n_1s = len(self.timestamps)
        bw = sum(self.byte_sizes)
        
        return n_1s, bw
    
    def run(self):
        while self.running and not rospy.is_shutdown():
            time.sleep(0.1)  # 불필요한 CPU 사용을 줄이기 위해 sleep
        
    def stop(self):
        self.running = False

class ROSTopicMonitor:
    def __init__(self):
        self.yaml_path = '/home/pgw/catkin_ws/src/ros_monitor/cfg/topic_lst.yaml'
        self.monitored_topics = self.load_yaml()
        
        rospy.init_node('topics_hzbw_node', anonymous=True)
        
        try:
            self.master = rosgraph.Master('/rostopic')
        except Exception as e:
            rospy.logerr(f'[monitor] Failed to retrieve published topics: {e}')    
                
        self.topic_monitors = {} # 토픽 객체
        self.previous_topics = set()  # 이전 실행된 토픽 목록 저장

        # 기존 String 메시지 발행자
        #self.publisher = rospy.Publisher('/topics_hzbw', String, queue_size=100)
        
        # Monitor 객체 생성 - '/monitoring' 토픽으로 발행
        self.monitor = Monitor(monitorDescription="Topic Frequency and Bandwidth Monitor", monitoring_type="topic_hzbw")

        self.lock = threading.Lock()
        self.yaml_timer = time.time()
        self.monitor_thread = threading.Thread(target=self.run_monitoring)
        self.monitor_thread.start()
    
    def load_yaml(self):
        if not os.path.exists(self.yaml_path):
            rospy.logerr(f'[monitor] yaml file not found: {self.yaml_path}')
            return {}
        
        topics = {}
        try:
            with open(self.yaml_path, 'r') as f:
                for l in f:
                    s = l.strip()
                    if not s:
                        continue
                    
                    if ':' in s:
                        t, v = s.split(':', 1)
                        t = t.strip()
                        v = v.strip()
                        
                    else:
                        t = s
                        v = -1
                    topics[t] = v
                    
        except Exception as e:
            rospy.logerr(f'[monitor] Error loading yaml file: {e}')
            return {}
        
        return topics

    def update_subscriptions(self):
        active_topics = {t[0] for t in self.master.getSystemState()[0]}  # 현재 실행 중인 토픽 목록
        
        new_topics = active_topics - set(self.topic_monitors.keys())  # 새로 추가된 토픽들
        
        if self.monitored_topics:
            active_topics &= set(self.monitored_topics.keys())
            new_topics &= set(self.monitored_topics.keys())
            
        removed_topics = set(self.topic_monitors.keys()) - active_topics  # 삭제된 토픽들

        # 노드 목록이 변경된 경우에만 정렬 수행
        if new_topics or removed_topics:
            sorted_topics = sorted(active_topics)  # 이름 기준 오름차순 정렬

            # 기존 토픽 리스트 업데이트
            self.previous_topics = set(sorted_topics)

            # 새 토픽 추가
            for topic in new_topics:
                monitor = TopicMonitor(topic)
                self.topic_monitors[topic] = monitor
                monitor.start()

            # 삭제된 토픽 정리
            for topic in removed_topics:
                self.topic_monitors[topic].stop()
                self.topic_monitors[topic].join()
                del self.topic_monitors[topic]

    def calculate_metrics(self):
        topic_metrics_list = []
        for topic in sorted(self.topic_monitors.keys()):
            monitor = self.topic_monitors[topic]
            hz, bw = monitor.get_hz_and_bw()

            target_hz = self.monitored_topics.get(topic)
            error_level = 0.0  # 기본 오류 수준
            # 목표 주파수가 설정된 경우 오류 수준 계산
            if target_hz is not None and target_hz != "None":
                try:
                    target_hz_float = float(target_hz)
                    # 주파수가 목표의 80% 미만이면 경고, 50% 미만이면 심각
                    if hz < target_hz_float * 0.5:
                        error_level = 2.0  # 심각
                    elif hz < target_hz_float * 0.8:
                        error_level = 1.0  # 경고
                    else:
                        error_level = 0.0  # 정상
                except ValueError:
                    error_level = 0.0  # 목표 주파수를 숫자로 변환할 수 없는 경우
            else:
                error_level = 0.0  # 목표 주파수가 설정되지 않은 경우
            
            # 모니터링 객체에 값 추가
            #topic_name_safe = topic.replace('/', '_').lstrip('_')  # 슬래시를 언더스코어로 변환
            self.monitor.addValue(f"hz_{topic}", hz, "Hz", target_hz, error_level)
            self.monitor.addValue(f"bw_{topic}", bw, "B/s", 0.0, 0.0)
            
            topic_metrics = {
                'topic': topic,
                'hz': hz,
                'bw': bw,
                'target_hz': target_hz
            }
            topic_metrics_list.append(topic_metrics)
        #print(topic_metrics_list)

        # if topic_metrics_list:
        #     self.publisher.publish(json.dumps(topic_metrics_list))
            
    def run_monitoring(self):
        rate = rospy.Rate(1)  # 1Hz
        while not rospy.is_shutdown():
            curr_time = time.time()
            if curr_time - self.yaml_timer >= 5:
                updated_yaml = self.load_yaml()
                if updated_yaml:
                    self.monitored_topics = updated_yaml
                self.yaml_timer = curr_time
                
            self.update_subscriptions()
            self.calculate_metrics()
            rate.sleep()


if __name__ == '__main__':
    monitor = ROSTopicMonitor()
    try:
        rospy.spin()
    except KeyboardInterrupt:
        for topic_monitor in monitor.topic_monitors.values():
            topic_monitor.stop()
            topic_monitor.join()