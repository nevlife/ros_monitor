import rospy
import rosgraph
import time
import os
import psutil  # For system metrics
from collections import deque
from threading import Lock
import subprocess


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
                msg_size = len(str(msg))

            self.bytes_received += msg_size

    def get_hz(self):
        with self.hz_lock:
            if len(self.times) < 2:
                return None
            deltas = [self.times[i + 1] - self.times[i] for i in range(len(self.times) - 1)]
            if deltas:
                return 1.0 / (sum(deltas) / len(deltas))
            return None

    def get_bandwidth(self):
        with self.bandwidth_lock:
            if self.start_time is None or self.end_time is None:
                return None
            duration = self.end_time - self.start_time
            if duration == 0:
                return None
            return self.bytes_received / duration


class MetricsManager:
    def __init__(self, yaml_file=None):
        if yaml_file is None:
            base_path = os.path.dirname(os.path.abspath(__file__))
            yaml_file = os.path.join(base_path, "../cfg/topic_lst.yaml")
        self.monitors = {}
        self.yaml_file = yaml_file
        self.node_pids = {}
        self.initialize_monitors()

    def initialize_monitors(self):
        try:
            with open(self.yaml_file, 'r') as file:
                topic_list = file.read().splitlines()
        except Exception as e:
            rospy.logerr(f"Failed to read YAML file: {e}")
            topic_list = []

        master = rosgraph.Master('/rostopic')
        published_topics = master.getPublishedTopics('')

        filtered_topic_list = [topic for topic in topic_list if topic in [t[0] for t in published_topics]]

        rospy.loginfo(f'Init monitors for {len(filtered_topic_list)} topics...')
        for topic in filtered_topic_list:
            self.monitors[topic] = ROSTopicMetrics(topic)

        # 노드별 PID 가져오기
        self.node_pids = self.get_node_pids()

    def get_node_pids(self):
        node_pids = {}
        for node in rosgraph.Master('/rostopic').getSystemState()[0]:  # 모든 퍼블리셔 노드 가져오기
            try:
                node_info = subprocess.run(['rosnode', 'info', node], stdout=subprocess.PIPE, text=True)
                for line in node_info.stdout.splitlines():
                    if 'Pid:' in line:
                        pid = int(line.split(':')[-1].strip())
                        node_pids[node] = pid
            except Exception as e:
                rospy.logwarn(f"Failed to get PID for node {node}: {e}")
        return node_pids

    def get_metrics(self):
        results = {}
        for topic, monitor in self.monitors.items():
            hz = monitor.get_hz()
            bandwidth = monitor.get_bandwidth()

            hz_val = f'{hz:.2f}' if hz is not None else 'N/A'
            bandwidth_val = f'{bandwidth:.2f}' if bandwidth is not None else 'N/A'

            # PID로 메모리 및 CPU 사용량 가져오기
            node_name = next((n for n in self.node_pids if topic in n), None)
            if node_name and node_name in self.node_pids:
                pid = self.node_pids[node_name]
                try:
                    proc = psutil.Process(pid)
                    cpu_usage = f'{proc.cpu_percent(interval=0.1) / psutil.cpu_count():.2f}'
                    memory_usage = f'{proc.memory_info().rss / (1024*1024):.2f}'
                except Exception as e:
                    cpu_usage = 'N/A'
                    memory_usage = 'N/A'
            else:
                cpu_usage = 'N/A'
                memory_usage = 'N/A'

            results[topic] = {
                'hz': hz_val,
                'bw': bandwidth_val,
                'cpu_usage': cpu_usage,
                'memory': memory_usage,
            }

        return results
