#!/usr/bin/env python3
import sys
import rospy
import json
import os
import time
from std_msgs.msg import Float32MultiArray, String

class RosMonitorDataLogger:
    def __init__(self):
        rospy.init_node("ros_monitor_logger", anonymous=True)

        # 데이터와 타임스탬프 관리
        self.data = {
            "total_resource": {},
            "topics_hzbw": {},
            "node_resource_usage": {},
            "gpu_pmon": {}
        }
        # 각 데이터가 마지막으로 업데이트된 시간 기록
        self.last_update = {
            "total_resource": 0,
            "topics_hzbw": 0,
            "node_resource_usage": 0,
            "gpu_pmon": 0
        }

        timestamp = time.strftime("%Y%m%d_%H%M%S")
        self.file_path = os.path.expanduser(f'~/catkin_ws/src/ros_monitor/data/diag_{timestamp}.json')

        rospy.Subscriber("/total_resource", Float32MultiArray, self.total_resource_callback)
        rospy.Subscriber("/topics_hzbw", String, self.topics_hzbw_callback)
        rospy.Subscriber("/nodes_resource", String, self.nodes_resource_callback)
        rospy.Subscriber("/gpu_pmon", String, self.gpu_pmon_callback)

        self.run()

    def total_resource_callback(self, msg):
        keys = [
            'cpu_user', 'cpu_nice', 'cpu_system', 'cpu_idle', 'cpu_iowait',
            'cpu_irq', 'cpu_softirq', 'cpu_steal', 'cpu_guest', 'cpu_guest_nice',
            'cpu_usage_percent', 'cpu_temp', 'cpu_load_1min', 'cpu_load_5min', 'cpu_load_15min',
            'mem_used', 'mem_total', 'mem_usage_percent',
            'gpu_usage_percent', 'gpu_mem_used', 'gpu_mem_total', 'gpu_mem_usage', 'gpu_temp'
        ]
        self.data["total_resource"] = dict(zip(keys, msg.data))
        self.last_update["total_resource"] = time.time()

    def topics_hzbw_callback(self, msg):
        topics_list = json.loads(msg.data)
        self.data["topics_hzbw"] = {
            topic_info["topic"]: {
                "hz": topic_info["hz"],
                "bw": topic_info["bw"]
            } for topic_info in topics_list
        }
        self.last_update["topics_hzbw"] = time.time()

    def nodes_resource_callback(self, msg):
        nodes_list = json.loads(msg.data)
        self.data["node_resource_usage"] = {
            node_info["node"]: {
                "cpu": node_info["cpu"],
                "mem": node_info["mem"]
            } for node_info in nodes_list
        }
        self.last_update["node_resource_usage"] = time.time()

    def gpu_pmon_callback(self, msg):
        self.data["gpu_pmon"] = json.loads(msg.data)
        self.last_update["gpu_pmon"] = time.time()

    def run(self):
        rate = rospy.Rate(1)  # 1Hz
        while not rospy.is_shutdown():
            self.clear_old_data()  # 추가: 오래된 데이터 정리
            self.save_json()
            rate.sleep()

    def save_json(self):
        with open(self.file_path, 'a') as f:
            json.dump(self.data, f)
            f.write('\n')
        print("Data saved:", self.data)

    def clear_old_data(self):
        current_time = time.time()
        # 데이터를 받은 지 2초가 넘었다면 빈 값으로 초기화
        for key in self.data:
            if current_time - self.last_update[key] > 2:
                self.data[key] = {}

if __name__ == "__main__":
    try:
        RosMonitorDataLogger()
    except rospy.ROSInterruptException:
        pass
