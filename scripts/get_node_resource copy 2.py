#!/usr/bin/python3
# -*- coding: utf-8 -*-
import rospy
import rosnode
import rosgraph
import psutil
import json
import subprocess
import time
try:
    from xmlrpc.client import ServerProxy
except ImportError:
    from xmlrpclib import ServerProxy

from std_msgs.msg import String

class NodeResourceMonitor:
    def __init__(self):
        rospy.init_node("all_nodes_resource_monitor", anonymous=True)
        self.resource_pub = rospy.Publisher("/all_nodes_resource", String, queue_size=10)
        self.master = rosgraph.Master('/rosnode')
        self.node_info = {}  # {node_name: {"api": node_api, "pid": pid, "last_checked": timestamp}}
        self.rate = rospy.Rate(1)  # 1초마다 실행
        self.last_update_time = time.time()  # 전체 노드 목록 업데이트 주기 (예: 10초마다)
    
    def update_node_list(self):
        """ 현재 실행 중인 ROS 노드 목록을 가져와, 새로운 노드는 추가하고 종료된 노드는 제거 """
        current_nodes = set(rosnode.get_node_names())
        existing_nodes = set(self.node_info.keys())

        # 새로운 노드 추가
        new_nodes = current_nodes - existing_nodes
        for node in new_nodes:
            self.node_info[node] = self.get_node_info(node)

        # 종료된 노드 제거
        removed_nodes = existing_nodes - current_nodes
        for node in removed_nodes:
            rospy.logwarn(f"Node {node} has been removed.")
            del self.node_info[node]

    def get_node_info(self, node_name):
        """ 노드의 API URI와 PID를 한 번만 가져오고 캐싱 """
        try:
            node_api = self.master.lookupNode(node_name)  # API URI 가져오기
            resp = ServerProxy(node_api).getPid('/NODEINFO')  # XML-RPC 요청으로 PID 가져오기
            pid = resp[2]  # PID 값
            return {"api": node_api, "pid": pid, "last_checked": time.time()}
        except Exception as e:
            rospy.logwarn(f"Failed to get API URL & PID for node {node_name}: {e}")
            return {"api": None, "pid": None, "last_checked": time.time()}

    def refresh_node_info(self, node_name):
        """ 일정 시간마다 (예: 10초) API URI & PID를 다시 조회 """
        node_data = self.node_info.get(node_name)
        if node_data:
            last_checked = node_data.get("last_checked", 0)
            if time.time() - last_checked > 10:  # 10초마다 업데이트
                rospy.loginfo(f"Refreshing API URI & PID for node {node_name}")
                self.node_info[node_name] = self.get_node_info(node_name)

    def get_node_cpu_ram_usage(self, pid):
        """ 주어진 PID의 CPU 및 RAM 사용량을 반환 """
        if not pid or not psutil.pid_exists(pid):
            return None, None
        process = psutil.Process(pid)
        return process.cpu_percent(interval=1), process.memory_info().rss / (1024 * 1024)

    def monitor_nodes(self):
        """ 모든 실행 중인 ROS 노드의 PID, CPU, RAM 사용량을 하나의 JSON 형태로 발행 """
        while not rospy.is_shutdown():
            start_time = time.perf_counter_ns()  # 실행 시작 시간 측정

            # 일정 시간마다 노드 목록 갱신 (예: 10초마다)
            if time.time() - self.last_update_time > 10:
                self.update_node_list()
                self.last_update_time = time.time()

            node_usage = {}

            for node, info in self.node_info.items():
                pid = info["pid"]
                
                # PID가 존재하지 않으면 다시 조회
                if not psutil.pid_exists(pid):
                    rospy.logwarn(f"PID for {node} is no longer valid. Refreshing...")
                    self.refresh_node_info(node)
                    pid = self.node_info[node]["pid"]

                # CPU & RAM 사용량 가져오기
                if pid:
                    cpu, mem = self.get_node_cpu_ram_usage(pid)
                    if cpu is not None and mem is not None:
                        node_usage[node] = {"cpu": cpu, "mem": mem}

            # JSON 형태로 변환 후 ROS 토픽 발행
            usage_json = json.dumps(node_usage)
            self.resource_pub.publish(String(usage_json))

            end_time = time.perf_counter_ns()  # 실행 종료 시간 측정
            elapsed_time_ns = end_time - start_time
            rospy.loginfo(f"Loop execution time: {elapsed_time_ns} ns")

            self.rate.sleep()

if __name__ == "__main__":
    monitor = NodeResourceMonitor()
    monitor.monitor_nodes()