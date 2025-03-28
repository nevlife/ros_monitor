#!/usr/bin/python3
# -*- coding: utf-8 -*-


from datetime import datetime
import sqlite3
import json
import rospy
from std_msgs.msg import Float32MultiArray, String
import psutil

logical_cores = psutil.cpu_count(logical=True)

class AnalyzerDB:
    def __init__(self):
        db_path = "/home/pgw/catkin_ws/src/ros_monitor/data/data.db"
        # isolation_level=None 추가 (autocommit 모드로 설정)
        self.conn = sqlite3.connect(db_path, check_same_thread=False, isolation_level=None)
        self.conn.row_factory = sqlite3.Row

    def insert_total_resource(self, timestamp, data):
        self.conn.execute("""
            INSERT OR REPLACE INTO total_resource (
                timestamp, cpu_usage_percent, cpu_temp,
                mem_used, mem_total, mem_usage_percent,
                gpu_usage_percent, gpu_temp, gpu_mem_usage
            ) VALUES (?, ?, ?, ?, ?, ?, ?, ?, ?)
        """, (
            timestamp,
            data.get('cpu_usage_percent'),
            data.get('cpu_temp'),
            data.get('mem_used'),
            data.get('mem_total'),
            data.get('mem_usage_percent'),
            data.get('gpu_usage_percent'),
            data.get('gpu_temp'),
            data.get('gpu_mem_usage'),
        ))

    def insert_node_resource(self, timestamp, node_name, cpu, mem):
        self.conn.execute("""
            INSERT INTO node_resource (timestamp, node_name, cpu, mem)
            VALUES (?, ?, ?, ?)
        """, (timestamp, node_name, cpu, mem))

    def insert_topic_hzbw(self, timestamp, topic_name, hz, bw):
        self.conn.execute("""
            INSERT INTO topic_hzbw (timestamp, topic_name, hz, bw)
            VALUES (?, ?, ?, ?)
        """, (timestamp, topic_name, hz, bw))

    def insert_gpu_pmon(self, timestamp, pid, type_, sm, mem, cmd):
        self.conn.execute("""
            INSERT INTO gpu_pmon (timestamp, pid, type, sm_usage, mem_usage, command)
            VALUES (?, ?, ?, ?, ?, ?)
        """, (timestamp, pid, type_, sm, mem, cmd))

    def insert_alert(self, level, source, message):
        timestamp = rospy.get_time()
        self.conn.execute("""
            INSERT INTO alert_log (timestamp, level, source, message)
            VALUES (?, ?, ?, ?)
        """, (timestamp, level, source, message))





db = AnalyzerDB()

def total_resource_callback(msg):
    keys = [
        'cpu_user', 'cpu_nice', 'cpu_system', 'cpu_idle', 'cpu_iowait',
        'cpu_irq', 'cpu_softirq', 'cpu_steal', 'cpu_guest', 'cpu_guest_nice',
        'cpu_usage_percent', 'cpu_temp', 'cpu_load_1min', 'cpu_load_5min', 'cpu_load_15min',
        'mem_used', 'mem_total', 'mem_usage_percent',
        'gpu_usage_percent', 'gpu_mem_used', 'gpu_mem_total', 'gpu_mem_usage', 'gpu_temp'
    ]
    values = msg.data
    data = dict(zip(keys, values))
    timestamp = rospy.get_time()
    db.insert_total_resource(timestamp, data)

    if data['cpu_usage_percent'] > 90:
        db.insert_alert("WARNING", "total_resource", f"CPU usage high: {data['cpu_usage_percent']}%")

def topics_hzbw_callback(msg):
    try:
        data = json.loads(msg.data)
        timestamp = rospy.get_time()
        for topic in data:
            if "topic" in topic:
                db.insert_topic_hzbw(timestamp, topic["topic"], topic["hz"], topic["bw"])
                if topic["hz"] < 2:
                    db.insert_alert("WARNING", "topics_hzbw", f"{topic['topic']} Hz too low: {topic['hz']:.2f}")
    except Exception as e:
        rospy.logwarn(f"topics_hzbw_callback error: {e}")

def nodes_resource_callback(msg):
    try:
        data = json.loads(msg.data)
        timestamp = rospy.get_time()
        for node in data:
            cpu = round(node["cpu"] / logical_cores, 3)
            db.insert_node_resource(timestamp, node["node"], cpu, node["mem"])
            if cpu > 0.9:
                db.insert_alert("WARNING", "node_resource", f"{node['node']} CPU high: {cpu * 100:.1f}%")
    except Exception as e:
        rospy.logwarn(f"nodes_resource_callback error: {e}")

def gpu_pmon_callback(msg):
    try:
        data = json.loads(msg.data)
        timestamp = rospy.get_time()
        for proc in data:
            db.insert_gpu_pmon(
                timestamp,
                proc.get("pid", "-"),
                proc.get("type", "-"),
                proc.get("sm_usage", 0.0),
                proc.get("mem_usage", 0.0),
                proc.get("command", "-")
            )
            if proc.get("sm_usage", 0.0) > 90:
                db.insert_alert("WARNING", "gpu_pmon", f"GPU SM high: PID {proc.get('pid')} {proc.get('sm_usage')}%")
    except Exception as e:
        rospy.logwarn(f"gpu_pmon_callback error: {e}")

def main():
    rospy.init_node("analyzer_node")
    rospy.Subscriber("/total_resource", Float32MultiArray, total_resource_callback)
    rospy.Subscriber("/topics_hzbw", String, topics_hzbw_callback)
    rospy.Subscriber("/nodes_resource", String, nodes_resource_callback)
    rospy.Subscriber("/gpu_pmon", String, gpu_pmon_callback)
    rospy.spin()

if __name__ == "__main__":
    main()
