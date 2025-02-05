#!/usr/bin/python3
# -*- coding: utf-8 -*-
import rospy
import rosgraph.masterapi
import json
import time
import threading
from collections import deque
from std_msgs.msg import String

class ROSTopicMonitor:
    def __init__(self, update_interval=5):
        rospy.init_node("ros_topic_monitor", anonymous=True)
        self.master = rosgraph.masterapi.Master('/roscore')
        self.topic_data = {}  # 각 토픽의 Hz 및 BW를 저장
        self.subscribers = {}  # 현재 구독 중인 토픽 저장
        self.update_interval = update_interval  # 토픽 목록 갱신 주기 (초)
        self.publisher = rospy.Publisher("/topic_metrics", String, queue_size=10)  # JSON 데이터 퍼블리시
        self.lock = threading.Lock()  # 데이터 동기화용 Lock

    def get_active_topics(self):
        """ 현재 실행 중인 모든 퍼블리싱 토픽을 가져옴 """
        state = self.master.getSystemState()
        return {t[0] for t in state[0]}  # 퍼블리셔 목록을 집합(set)으로 반환

    def topic_callback(self, msg, topic):
        """ 메시지 수신 시 Hz 및 BW 계산을 위해 데이터 저장 """
        now = time.time()
        msg_size = len(msg._buff) if hasattr(msg, '_buff') else len(str(msg))

        with self.lock:
            if topic not in self.topic_data:
                self.topic_data[topic] = {"timestamps": deque(maxlen=100), "bytes": 0, "flag": 1}

            self.topic_data[topic]["timestamps"].append(now)
            self.topic_data[topic]["bytes"] += msg_size

    def calculate_metrics(self, topic):
        """ Hz 및 BW 계산 (멀티스레딩 사용) """
        while not rospy.is_shutdown():
            with self.lock:
                if topic not in self.topic_data:
                    continue

                timestamps = self.topic_data[topic]["timestamps"]
                byte_count = self.topic_data[topic]["bytes"]

                # Hz 계산 (최근 1초 동안 받은 메시지 개수)
                now = time.time()
                valid_timestamps = [t for t in timestamps if now - t <= 1]
                hz = len(valid_timestamps)

                # BW 계산 (KB 단위)
                bw = byte_count / 1024.0 if byte_count > 0 else 0.0

                # flag 설정 (토픽이 활성화된 경우 1, 아니면 0)
                flag = 1 if hz > 0 else 0

                # JSON 데이터 생성
                topic_metrics = {
                    "topic": topic,
                    "flag": flag,
                    "hz": hz,
                    "bw": bw
                }

                # ROS 퍼블리시
                self.publisher.publish(json.dumps(topic_metrics))

            time.sleep(1)  # 1초마다 업데이트

    def update_subscriptions(self):
        """ N초마다 실행 중인 토픽 목록을 확인하여 새로운 토픽을 추가하고, 종료된 토픽은 구독 취소 """
        active_topics = self.get_active_topics()

        # 새로운 토픽 구독 추가
        for topic in active_topics - self.subscribers.keys():
            rospy.loginfo(f"[모니터링 추가] {topic}")
            self.subscribers[topic] = rospy.Subscriber(topic, rospy.AnyMsg, self.topic_callback, callback_args=topic)

            # 멀티스레딩으로 각 토픽 Hz & BW 계산
            thread = threading.Thread(target=self.calculate_metrics, args=(topic,))
            thread.daemon = True
            thread.start()

        # 종료된 토픽 구독 해제
        for topic in list(self.subscribers.keys() - active_topics):
            rospy.loginfo(f"[모니터링 종료] {topic}")
            self.subscribers[topic].unregister()
            del self.subscribers[topic]
            del self.topic_data[topic]  # 데이터 삭제

    def run_monitoring(self):
        """ 실행 중인 모든 토픽을 주기적으로 재구독하며 모니터링 시작 """
        rospy.loginfo("모니터링 시작")
        while not rospy.is_shutdown():
            self.update_subscriptions()
            time.sleep(self.update_interval)  # N초마다 토픽 업데이트

if __name__ == "__main__":
    monitor = ROSTopicMonitor(update_interval=5)
    monitor.run_monitoring()
