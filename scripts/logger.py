#!/usr/bin/env python3
import rospy
import json
import os
import time
import signal
import fcntl
from std_msgs.msg import Float32MultiArray, String

class RosMonitorDataLogger:
    DATA_TIMEOUT = 2  # 데이터 유효 시간(초)

    def __init__(self):
        rospy.init_node("ros_monitor_logger", anonymous=True)

        self.data = {
            "total_resource": {},
            "topics_hzbw": {},
            "node_resource_usage": {},
            "gpu_pmon": {}
        }
        self.last_update = {key: 0 for key in self.data}

        self.current_date = time.strftime("%Y%m%d")
        self.file_path = self.get_file_path(self.current_date)
        self.file = self.safe_open(self.file_path)

        # 파일 잠금 (중복 실행 방지)
        try:
            fcntl.flock(self.file, fcntl.LOCK_EX | fcntl.LOCK_NB)
        except BlockingIOError:
            rospy.logerr("Another instance is already running. Exiting.")
            self.file.close()
            exit(1)

        # 종료 시그널 처리
        signal.signal(signal.SIGINT, self.handle_exit)
        signal.signal(signal.SIGTERM, self.handle_exit)

        # ROS Subscribers
        rospy.Subscriber("/total_resource", Float32MultiArray, self.total_resource_callback)
        rospy.Subscriber("/topics_hzbw", String, self.topics_hzbw_callback)
        rospy.Subscriber("/nodes_resource", String, self.nodes_resource_callback)
        rospy.Subscriber("/gpu_pmon", String, self.gpu_pmon_callback)

        self.run()

    def get_file_path(self, date_str):
        dir_path = os.path.expanduser('~/catkin_ws/src/ros_monitor/data')
        os.makedirs(dir_path, exist_ok=True)
        return os.path.join(dir_path, f'diag_{date_str}.json')

    def safe_open(self, path):
        try:
            return open(path, 'a')
        except Exception as e:
            rospy.logfatal(f"Failed to open log file: {e}")
            exit(1)

    def handle_exit(self, signum, frame):
        if hasattr(self, 'file') and not self.file.closed:
            self.file.close()
            rospy.loginfo("File closed safely on shutdown.")
        rospy.signal_shutdown("Terminated by signal")

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
        try:
            topics_list = json.loads(msg.data)
            self.data["topics_hzbw"] = {
                topic["topic"]: {"hz": topic["hz"], "bw": topic["bw"]}
                for topic in topics_list
            }
            self.last_update["topics_hzbw"] = time.time()
        except Exception as e:
            rospy.logwarn(f"Failed to parse topics_hzbw: {e}")

    def nodes_resource_callback(self, msg):
        try:
            nodes_list = json.loads(msg.data)
            self.data["node_resource_usage"] = {
                node["node"]: {"cpu": node["cpu"], "mem": node["mem"]}
                for node in nodes_list
            }
            self.last_update["node_resource_usage"] = time.time()
        except Exception as e:
            rospy.logwarn(f"Failed to parse node_resource: {e}")

    def gpu_pmon_callback(self, msg):
        try:
            self.data["gpu_pmon"] = json.loads(msg.data)
            self.last_update["gpu_pmon"] = time.time()
        except Exception as e:
            rospy.logwarn(f"Failed to parse gpu_pmon: {e}")

    def clear_old_data(self):
        current_time = time.time()
        for key in self.data:
            if current_time - self.last_update[key] > self.DATA_TIMEOUT:
                self.data[key] = {}

    def save_json(self):
        timestamp = time.strftime("%Y-%m-%d %H:%M:%S")
        output_data = {"timestamp": timestamp, **self.data}

        current_date = time.strftime("%Y%m%d")
        if current_date != self.current_date:
            self.file.close()
            self.current_date = current_date
            self.file_path = self.get_file_path(current_date)
            self.file = self.safe_open(self.file_path)

        try:
            json.dump(output_data, self.file)
            self.file.write('\n')
            self.file.flush()
            rospy.loginfo(f"Data saved at {timestamp}")
        except Exception as e:
            rospy.logerr(f"Failed to write to file: {e}")

    def run(self):
        rate = rospy.Rate(1)  # 1Hz
        while not rospy.is_shutdown():
            self.clear_old_data()
            self.save_json()
            rate.sleep()

    def __del__(self):
        if hasattr(self, 'file') and not self.file.closed:
            self.file.close()
            rospy.loginfo("File closed on __del__.")

if __name__ == "__main__":
    try:
        RosMonitorDataLogger()
    except rospy.ROSInterruptException:
        pass
