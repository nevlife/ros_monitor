#!/usr/bin/python3
# -*- coding: utf-8 -*-
import rospy
import rosgraph.masterapi
import json
import time
from collections import deque
from std_msgs.msg import String

class ROSTopicMonitor:
    def __init__(self):
        rospy.init_node("ros_topic_monitor", anonymous=True)
        
        #rosmaster가 활성화 될때까지 대기
        
        self.master = rosgraph.masterapi.Master('/roscore')
        self.topic_data = {}  # 각 토픽의 Hz 및 BW 데이터 저장
        self.subscribers = {}  # 현재 구독 중인 토픽 저장
        self.update_interval = 1  # 토픽 목록 갱신 주기 (초)
        self.publisher = rospy.Publisher("/topic_metrics", String, queue_size=100)  # JSON 퍼블리시
        self.last_active_topics = set()  # 마지막으로 감지한 활성 토픽 목록

    def get_active_topics(self):
        """ 현재 실행 중인 모든 퍼블리싱 토픽을 가져옴 """
        state = self.master.getSystemState()
        return {t[0] for t in state[0]}  # 퍼블리셔 목록을 집합(set)으로 반환

    def topic_callback(self, msg, topic):
        """ 메시지 수신 시 Hz 및 BW 계산을 위해 데이터 저장 """
        now = time.perf_counter()
        msg_size = len(msg._buff) if hasattr(msg, '_buff') else len(str(msg))

        if topic not in self.topic_data:
            self.topic_data[topic] = {"timestamps": deque(maxlen=200), "byte_sizes": deque(maxlen=5000)}
            
        self.topic_data[topic]["timestamps"].append(now)
        self.topic_data[topic]["byte_sizes"].append(msg_size)

    def calculate_metrics(self):
        """ 5초 동안의 데이터로 Hz & BW 계산, 1초마다 퍼블리시 """
        now = time.perf_counter()
        topic_metrics_list = []
        
        #딕셔너리 크기 변경 오류 방지지
        for topic, data in self.topic_data.copy().items():
            timestamps = data["timestamps"]
            byte_sizes = data["byte_sizes"]
            # 최근 5초 이내의 메시지만 유지
            # valid_timestamps = [t for t in timestamps if now - t <= 1]
            # valid_byte_sizes = [size for t, size in zip(timestamps, byte_sizes) if now - t <= 1]

            # # 슬라이딩 윈도우를 사용한 Hz 계산
            # elapsed_time = valid_timestamps[-1] - valid_timestamps[0] if len(valid_timestamps) > 1 else 1
            # hz = len(valid_timestamps) / elapsed_time if elapsed_time > 0 else 0

            # # BW 계산 (KB 단위 변환)
            # bw = sum(valid_byte_sizes) / 1024.0 if valid_byte_sizes else 0.0  

            valid_timestamps = []
            valid_byte_sizes = []
            # 최근 1초 이내의 메시지만 유지
            for i in range(len(timestamps)):
                if now - timestamps[i] <= 1:
                    valid_timestamps.append(timestamps[i])
                    valid_byte_sizes.append(byte_sizes[i])

            # 슬라이딩 윈도우를 사용한 Hz 계산
            if len(valid_timestamps) > 1:
                elapsed_time = valid_timestamps[-1] - valid_timestamps[0]
            else:
                elapsed_time = 1  # 최소한 1초로 설정하여 0으로 나누는 오류 방지

            hz = len(valid_timestamps) / elapsed_time if elapsed_time > 0 else 0

            # BW 계산 (KB 단위 변환)
            if valid_byte_sizes:
                bw = sum(valid_byte_sizes) / 1024.0
            else:
                bw = 0.0

            # JSON 데이터 생성
            topic_metrics = {
                "topic": topic,
                "hz": round(hz, 2),
                "bw": round(bw, 2)
            }
            topic_metrics_list.append(topic_metrics)
        # 1초마다 모든 토픽 데이터 퍼블리시
        if topic_metrics_list:
            self.publisher.publish(json.dumps(topic_metrics_list))

    def update_subscriptions(self):
        """ 실행 중인 토픽 목록을 확인하여 새로운 토픽을 추가하고, 종료된 토픽은 구독 취소 """
        active_topics = self.get_active_topics()

        # 새로운 토픽 추가
        new_topics = active_topics - self.last_active_topics
        for topic in new_topics:
            self.subscribers[topic] = rospy.Subscriber(topic, rospy.AnyMsg, self.topic_callback, callback_args=topic)

        # 종료된 토픽 제거
        removed_topics = self.last_active_topics - active_topics
        for topic in removed_topics:
            rospy.loginfo(f"[모니터링 종료] {topic}")
            self.subscribers[topic].unregister()
            del self.subscribers[topic]
            del self.topic_data[topic]  # 데이터 삭제

        # 현재 활성화된 토픽 목록 업데이트
        self.last_active_topics = active_topics

    def run_monitoring(self):
        """ 실행 중인 모든 토픽을 주기적으로 재구독 및 Hz/BW 계산 """
        rospy.loginfo("모니터링 시작")
        rate = rospy.Rate(1)  # 1Hz (1초 주기)
        while not rospy.is_shutdown():
            self.update_subscriptions()  # 토픽 목록 갱신
            self.calculate_metrics()  # Hz & BW 계산 및 퍼블리시
            rate.sleep()  # 정확한 주기 유지

if __name__ == "__main__":
    monitor = ROSTopicMonitor()
    monitor.run_monitoring()
