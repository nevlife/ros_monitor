#!/usr/bin/python3
# -*- coding: utf-8 -*-
import rospy
import rosgraph
import time
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

        # Subscriber 설정
        self.subscriber = rospy.Subscriber(topic, rospy.AnyMsg, self.callback)

    def callback(self, msg):
        '''Hz, 대역폭, 쓰루풋 계산을 위한 데이터터'''
        now = time.monotonic()

        #Hz
        with self.hz_lock:
            self.times.append(now)

        #대역폭 및 쓰루풋 계산용용
        with self.bandwidth_lock:
            if self.start_time is None:
                self.start_time = now
            self.end_time = now

            if hasattr(msg, '_buff'):
                #_buff 속성이 있을때
                msg_size = len(msg._buff)
            else:
                #_buff 속성이 없으때 메시지를 문자열로 변환한 후 길이 계산
                msg_size = len(str(msg))
                
            self.bytes_received += msg_size

    def get_hz(self):
        """Hz 계산"""
        with self.hz_lock:
            if len(self.times) < 2:
                return None  #데이터 부족 시 None 반환
            
            deltas = []
            timestamps = list(self.times)
            
            for i in range(len(timestamps) - 1):
                t1 = timestamps[i]
                t2 = timestamps[i+1]
                delta = t2 - t1
                deltas.append(delta)
                
            if deltas:
                hz = 1.0 / (sum(deltas)/len(deltas))
                return hz
            else:
                return None
            
    def get_bandwidth(self):
        '''return bandwidth'''
        with self.bandwidth_lock:

            if self.start_time is None or self.end_time is None:
                return None  #데이터 부족 시 None 반환
            
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


def monitor_metrics():
    rospy.init_node("topic_metrics_monitor", anonymous=True)

    # ROS 마스터에서 모든 토픽 가져오기
    master = rosgraph.Master('/rostopic')
    published_topics = master.getPublishedTopics('')
    topic_list = [topic[0] for topic in published_topics]

    # 필요 없는 토픽 제외
    ignore_topics = ['/clock', '/rosout', '/rosout_agg']
    topic_list = [topic for topic in topic_list if topic not in ignore_topics]

    # 각 토픽별 Metrics 모니터 생성
    monitors = {topic: ROSTopicMetrics(topic) for topic in topic_list}

    rospy.loginfo(f"Monitoring metrics for {len(monitors)} topics...")

    rate = rospy.Rate(1)  # 1Hz (1초마다 업데이트)
    try:
        while not rospy.is_shutdown():
            metrics_results = []

            # 모든 토픽에 대한 메트릭 계산
            for topic, monitor in monitors.items():
                hz = monitor.get_hz()
                bandwidth = monitor.get_bandwidth()

                metrics_results.append({
                    "topic": topic,
                    "hz": hz if hz is not None else "N/A",
                    "bandwidth": f"{bandwidth / 1024:.2f} KB/s" if bandwidth is not None else "N/A",
                })

                # 데이터 초기화
                monitor.reset()

            # 결과 출력 (또는 전송)
            rospy.loginfo(metrics_results)

            rate.sleep()
    except rospy.ROSInterruptException:
        rospy.loginfo("Shutting down metrics monitor.")


if __name__ == "__main__":
    monitor_metrics()
