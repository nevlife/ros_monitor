#!/usr/bin/env python3
import rospy
import json
import os
import time
from std_msgs.msg import Float32MultiArray, String
from copy import deepcopy

class RosMonitorDataLogger:
    DATA_TIMEOUT = 2  # 데이터 유효 시간(초)

    def __init__(self):
        rospy.init_node("ros_monitor_logger", anonymous=True)

        # 데이터 및 타임스탬프 관리
        self.data = {
            "total_resource": {},
            "topics_hzbw": {},
            "node_resource_usage": {},
            "gpu_pmon": {}
        }

        self.last_data = deepcopy(self.data)  # 이전 데이터를 저장해 중복 방지
        self.last_update = {key: 0 for key in self.data}

        timestamp = time.strftime("%Y%m%d_%H%M%S")
        self.file_path = os.path.expanduser(f'~/catkin_ws/src/ros_monitor/data/diag_{timestamp}.json')

        rospy.Subscriber("/total_resource", Float32MultiArray, self.total_resource_callback)
        rospy.Subscriber("/topics_hzbw", String, self.topics_hzbw_callback)
        rospy.Subscriber("/nodes_resource", String, self.nodes_resource_callback)
        rospy.Subscriber("/gpu_pmon", String, self.gpu_pmon_callback)

        self.run()

    # Callbacks
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
            topic["topic"]: {"hz": topic["hz"], "bw": topic["bw"]} 
            for topic in topics_list
        }
        self.last_update["topics_hzbw"] = time.time()

    def nodes_resource_callback(self, msg):
        nodes_list = json.loads(msg.data)
        self.data["node_resource_usage"] = {
            node["node"]: {"cpu": node["cpu"], "mem": node["mem"]} 
            for node in nodes_list
        }
        self.last_update["node_resource_usage"] = time.time()

    def gpu_pmon_callback(self, msg):
        self.data["gpu_pmon"] = json.loads(msg.data)
        self.last_update["gpu_pmon"] = time.time()

    # 데이터 관리
    def clear_old_data(self):
        current_time = time.time()
        for key in self.data:
            if current_time - self.last_update[key] > self.DATA_TIMEOUT:
                self.data[key] = {}

    def data_changed(self):
        return self.data != self.last_data

    def save_json_if_changed(self):
        if self.data_changed():
            with open(self.file_path, 'a') as f:
                json.dump(self.data, f)
                f.write('\n')
            self.last_data = deepcopy(self.data)
            rospy.loginfo("Data updated and saved.")
        else:
            rospy.loginfo("No data change; skip saving.")

    def run(self):
        rate = rospy.Rate(1)  # 1Hz
        while not rospy.is_shutdown():
            self.clear_old_data()            # 오래된 데이터 초기화
            self.save_json_if_changed()      # 변경됐을 때만 저장
            rate.sleep()

if __name__ == "__main__":
    try:
        RosMonitorDataLogger()
    except rospy.ROSInterruptException:
        pass
